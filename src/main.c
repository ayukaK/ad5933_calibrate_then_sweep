/*
 * main.c — AD5933 calibration + measurement (log sweep) with per-frequency gain + phase correction
 *
 * Hardware note (AD5933 typical hookup):
 *   - DUT between VOUT and VIN
 *   - RFB between VOUT and VIN (feedback for the internal I/V stage)
 *   - Optional CFB in parallel with RFB for stability if DUT is capacitive
 *
 * Quick RFB rule of thumb:
 *   VIN_AC ≈ VEXC_AC * (RFB / |Z_DUT|)
 *   Pick RFB so VIN_AC stays comfortably below full-scale (and below saturation with PGA).
 *   For |Z| ~ 10–20 kΩ, RFB ~ 10 kΩ is a reasonable starting point.
 *
 * This file:
 *   1) Builds a log-spaced frequency list (AD5933 only does linear sweeps natively; we reprogram each point).
 *   2) Runs CAL on a known impedance model: Rseries + (Rpar || C)
 *   3) Stores GainFactor[f] and PhaseOffset[f]
 *   4) Runs MEAS on DUT and applies per-frequency correction
 *
 * Recommended prj.conf (USB CDC logging can choke if you print too fast):
 *   CONFIG_LOG=y
 *   CONFIG_LOG_MODE_DEFERRED=y
 *   CONFIG_LOG_BUFFER_SIZE=32768
 *   CONFIG_LOG_DEFAULT_LEVEL=3
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>

LOG_MODULE_REGISTER(ad5933_cal, LOG_LEVEL_INF);

/* ---------------- AD5933 config ---------------- */
#define AD5933_ADDR        0x0D

/* IMPORTANT: set this to the REAL clock driving AD5933 MCLK pin.
 * If you're using the internal oscillator (no external MCLK), leave at 16000000.
 */
#define AD5933_MCLK_HZ     16000000u

/* Registers */
#define REG_CTRL_HB        0x80
#define REG_CTRL_LB        0x81

#define REG_START_FREQ_2   0x82 /* 0x82..0x84 */
#define REG_INC_FREQ_2     0x85 /* 0x85..0x87 */
#define REG_NUM_INC_2      0x88 /* 0x88..0x89 */
#define REG_SETTLE_2       0x8A /* 0x8A..0x8B */

#define REG_STATUS         0x8F
#define REG_REAL_1         0x94 /* 0x94..0x95 */
#define REG_IMAG_1         0x96 /* 0x96..0x97 */

/* Status bits */
#define STATUS_DATA_VAL    0x02 /* D1 */

/* Control LB bits */
#define CTRL_LB_INT_CLK    0x00
#define CTRL_LB_EXT_CLK    0x08
#define CTRL_LB_RESET      0x10

/* Control HB function codes (D15..D12) */
#define HB_INIT_FREQ       0x10
#define HB_START_SWEEP     0x20
#define HB_INC_FREQ        0x30
#define HB_REPEAT_FREQ     0x40
#define HB_STANDBY         0xB0

/* Output range bits (HB D10..D9). Common mapping (double-check your datasheet):
 * 00: 2.0 Vpp, 01: 1.0 Vpp, 10: 400 mVpp, 11: 200 mVpp
 */
#define HB_RANGE_2VPP      0x00
#define HB_RANGE_1VPP      0x02
#define HB_RANGE_400MVPP   0x04
#define HB_RANGE_200MVPP   0x06

/* PGA bit (HB D8) */
#define HB_PGA_X5          0x00
#define HB_PGA_X1          0x01

/* ---------------- Sweep settings ---------------- */
/* Log sweep endpoints */
#define SWEEP_START_HZ     1u
#define SWEEP_STOP_HZ      200000u
#define SWEEP_POINTS       70u

/* Settling cycles: keep small at first; increase if you see unstable readings after frequency changes */
#define SETTLING_TOTAL_CYCLES    0x0002u


/*
 * AD5933 settling time register is 16-bit:
 *   [15:14] multiplier (00=x1, 01=x2, 10=x4)
 *   [13:0]  number of settling cycles
 */
#define SETTLING_MUL_MASK          0xC000u
#define SETTLING_CYCLES_MASK       0x3FFFu
#define SETTLE_MUL_X1              0x0000u
#define SETTLE_MUL_X2              0x4000u
#define SETTLE_MUL_X4              0x8000u
#define SETTLING_MAX_CYCLES        0x3FFFu
#define SETTLING_MAX_TOTAL_CYCLES  (4u * SETTLING_MAX_CYCLES)

/* Print decimation: print every Nth line to reduce volume */
#define PRINT_EVERY_N      1u

/* Small delay between lines so host can drain (helps on USB CDC) */
#define LINE_DELAY_MS      1u

/* Status polling */
#define POLL_MS            2u
#define DATA_TIMEOUT_MS    2000u  /* for mid/high freq. very low freq can take much longer */

/* ---------------- Known calibration network ----------------
 * Z_known = Rseries + (Rpar || 1/(j*w*C))
 */
#define R_SERIES_OHM       9830.0
#define R_PAR_OHM          5540.0
#define C_PAR_F            1e-6

#define PI_D               3.1415926535897932384626433832795

static const struct device *i2c_dev;

/* ---------------- Helpers ---------------- */

static inline double clamp_pos(double x, double minv)
{
    return (x < minv) ? minv : x;
}

/* Wrap angle to (-pi, +pi] */
static double wrap_pi(double a)
{
    while (a <= -PI_D) a += 2.0 * PI_D;
    while (a >  PI_D)  a -= 2.0 * PI_D;
    return a;
}

/* Frequency tuning word
 *
 * NOTE: many AD5933 examples use:
 *   code = f_out * 2^27 / (MCLK/4)
 *
 * Which is algebraically:
 *   code = f_out * 2^29 / MCLK
 *
 * This function implements: code = f * 2^29 / MCLK.
 */
static uint32_t freq_to_word(uint32_t freq_hz)
{
    uint64_t num = (uint64_t)freq_hz << 29; /* f * 2^29 */
    uint32_t code = (uint32_t)(num / (uint64_t)AD5933_MCLK_HZ);

    /* AD5933 start/inc registers are 3 bytes (24 bits). */
    return code & 0xFFFFFFu;
}

/* ---------------- Low-level I2C helpers ---------------- */

static int write8(uint8_t reg, uint8_t v)
{
    int ret = i2c_reg_write_byte(i2c_dev, AD5933_ADDR, reg, v);
    if (ret) {
        LOG_ERR("W FAIL reg 0x%02X <- 0x%02X (ret=%d)", reg, v, ret);
    }
    return ret;
}

static int read8(uint8_t reg, uint8_t *v)
{
    int ret = i2c_reg_read_byte(i2c_dev, AD5933_ADDR, reg, v);
    if (ret) {
        LOG_ERR("R FAIL reg 0x%02X (ret=%d)", reg, ret);
    }
    return ret;
}

static int wb_write8_strict(uint8_t reg, uint8_t v)
{
    uint8_t rb = 0;
    int ret = write8(reg, v);
    if (ret) return ret;

    ret = read8(reg, &rb);
    if (ret) return ret;

    if (rb != v) {
        LOG_ERR("WB MISMATCH reg 0x%02X: wrote 0x%02X read 0x%02X", reg, v, rb);
        return -EIO;
    }
    return 0;
}

static int wb_write8_masked(uint8_t reg, uint8_t v, uint8_t mask)
{
    uint8_t rb = 0;
    int ret = write8(reg, v);
    if (ret) return ret;

    ret = read8(reg, &rb);
    if (ret) return ret;

    if ((rb & mask) != (v & mask)) {
        LOG_ERR("WB MISMATCH reg 0x%02X (mask 0x%02X): wrote 0x%02X read 0x%02X",
                reg, mask, v, rb);
        return -EIO;
    }
    return 0;
}

static int wb_write16_strict(uint8_t reg_hi, uint16_t v)
{
    int ret = wb_write8_strict(reg_hi, (uint8_t)((v >> 8) & 0xFF));
    if (ret) return ret;
    return wb_write8_strict((uint8_t)(reg_hi + 1), (uint8_t)(v & 0xFF));
}

static int wb_write24_strict(uint8_t reg_msb, uint32_t v24)
{
    int ret = wb_write8_strict(reg_msb,     (uint8_t)((v24 >> 16) & 0xFF));
    if (ret) return ret;
    ret = wb_write8_strict((uint8_t)(reg_msb + 1), (uint8_t)((v24 >> 8) & 0xFF));
    if (ret) return ret;
    return wb_write8_strict((uint8_t)(reg_msb + 2), (uint8_t)(v24 & 0xFF));
}

/* Control write: HB strict; LB masked (ignore RESET self-clear) */
static int ad5933_set_control(uint8_t hb, uint8_t lb)
{
    int ret = wb_write8_strict(REG_CTRL_HB, hb);
    if (ret) return ret;

    /* Verify only clock select bit (0x08); ignore reset bit (0x10) */
    return wb_write8_masked(REG_CTRL_LB, lb, 0x08);
}

static void ad5933_dump_ctrl(void)
{
    uint8_t hb = 0, lb = 0;
    if (rb_read8_strict(REG_CTRL_HB, &hb) || rb_read8_strict(REG_CTRL_LB, &lb)) {
        LOG_WRN("CTRL read failed");
        return;
    }

    const uint8_t func = (uint8_t)((hb >> 4) & 0x0Fu);
    const uint8_t range = (uint8_t)(hb & 0x03u);
    const bool pga_x5 = ((hb & HB_PGA_X5) != 0u);

    const bool int_clk = ((lb & CTRL_LB_INT_CLK) != 0u);

    LOG_INF("CTRL_HB=0x%02X CTRL_LB=0x%02X | func=0x%X range=%s PGA=%s clk=%s",
            hb, lb, func,
            (range == 0u) ? "2.0Vpp" : (range == 1u) ? "1.0Vpp" : (range == 2u) ? "0.4Vpp" : "0.2Vpp",
            pga_x5 ? "x5" : "x1",
            int_clk ? "INT" : "EXT");
}


static int read_real_imag(int16_t *real, int16_t *imag)
{
    uint8_t r_buf[2] = {0};
    uint8_t i_buf[2] = {0};

    int rr = i2c_burst_read(i2c_dev, AD5933_ADDR, REG_REAL_1, r_buf, 2);
    int ir = i2c_burst_read(i2c_dev, AD5933_ADDR, REG_IMAG_1, i_buf, 2);
    if (rr || ir) return rr ? rr : ir;

    *real = (int16_t)((r_buf[0] << 8) | r_buf[1]);
    *imag = (int16_t)((i_buf[0] << 8) | i_buf[1]);
    return 0;
}

static int wait_data_valid(uint32_t f_hz, uint16_t settling_total_cycles,
                          uint32_t timeout_floor_ms, uint8_t *last_status)
{
    /*
     * AD5933 settling is expressed in OUTPUT cycles. Minimum wait scales as cycles/f.
     * We add margin for DFT + I2C polling.
     */
    if (f_hz == 0U) {
        f_hz = 1U;
    }

    double settle_ms_d = 1000.0 * ((double)settling_total_cycles / (double)f_hz);
    uint32_t timeout_ms = timeout_floor_ms;
    uint32_t needed_ms = (uint32_t)(settle_ms_d + 750.0);
    if (needed_ms > timeout_ms) {
        timeout_ms = needed_ms;
    }
    if (timeout_ms > 30000U) {
        timeout_ms = 30000U;
    }

    int64_t t0 = k_uptime_get();
    uint8_t st = 0;

    while ((uint32_t)(k_uptime_get() - t0) < timeout_ms) {
        if (read8(REG_STATUS, &st) != 0) return -EIO;
        if (st & STATUS_DATA_VAL) {
            if (last_status) *last_status = st;
            return 0;
        }
        k_msleep(POLL_MS);
    }

    if (last_status) *last_status = st;
    return -ETIMEDOUT;
}


/* ---------------- Known impedance math ---------------- */
static void z_known_network(double freq_hz, double *mag_out, double *phase_out_rad)
{
    const double w = 2.0 * PI_D * freq_hz;

    /* Y = G + jB where:
     *  G = 1/R
     *  B = wC
     */
    const double G = 1.0 / R_PAR_OHM;
    const double B = w * C_PAR_F;

    const double denom = (G * G) + (B * B);

    /* Zpar = 1/Y = (G - jB)/(G^2+B^2) */
    const double zpar_re =  G / denom;
    const double zpar_im = -B / denom;

    const double z_re = R_SERIES_OHM + zpar_re;
    const double z_im = zpar_im;

    *mag_out = sqrt((z_re * z_re) + (z_im * z_im));
    *phase_out_rad = atan2(z_im, z_re);
}

/* ---------------- Frequency list (log spaced) ---------------- */
static void build_log_sweep(uint32_t *freqs, uint16_t n, uint32_t f_start, uint32_t f_stop)
{
    if (n == 0) return;

    /* Avoid log(0) */
    const double fs = clamp_pos((double)f_start, 1.0);
    const double fe = clamp_pos((double)f_stop,  1.0);

    if (n == 1) {
        freqs[0] = (uint32_t)llround(fs);
        return;
    }

    const double log_fs = log(fs);
    const double log_fe = log(fe);
    const double step = (log_fe - log_fs) / (double)(n - 1);

    for (uint16_t i = 0; i < n; i++) {
        const double f = exp(log_fs + step * (double)i);
        uint32_t fi = (uint32_t)llround(f);
        if (fi < 1u) fi = 1u;
        freqs[i] = fi;
    }
}

/* ---------------- Single-frequency measurement ---------------- */

static uint16_t settling_pack(uint32_t total_cycles_effective)
{
    if (total_cycles_effective == 0u) {
        total_cycles_effective = 1u;
    }

    uint16_t mul_bits = SETTLE_MUL_X1;
    uint32_t div = 1u;

    if (total_cycles_effective <= SETTLING_MAX_CYCLES) {
        mul_bits = SETTLE_MUL_X1;
        div = 1u;
    } else if (total_cycles_effective <= (2u * SETTLING_MAX_CYCLES)) {
        mul_bits = SETTLE_MUL_X2;
        div = 2u;
    } else if (total_cycles_effective <= (4u * SETTLING_MAX_CYCLES)) {
        mul_bits = SETTLE_MUL_X4;
        div = 4u;
    } else {
        /* Clamp to max representable. */
        mul_bits = SETTLE_MUL_X4;
        div = 4u;
        total_cycles_effective = SETTLING_MAX_TOTAL_CYCLES;
    }

    /* Round up so we never undershoot. */
    uint32_t cycles_reg = (total_cycles_effective + (div - 1u)) / div;
    if (cycles_reg > SETTLING_MAX_CYCLES) {
        cycles_reg = SETTLING_MAX_CYCLES;
    }

    return (uint16_t)(mul_bits | (uint16_t)(cycles_reg & SETTLING_CYCLES_MASK));
}


static int ad5933_measure_one(uint32_t freq_hz,
                             uint16_t settling_total_cycles,
                             uint8_t clk_sel,
                             uint8_t hb_range_pga,
                             int16_t *real, int16_t *imag,
                             uint8_t *status_out)
{
    const uint32_t code = freq_to_word(freq_hz);

    /* Program settling time. AD5933 uses a 16-bit register: [15:14] mul, [13:0] cycles. */
const uint16_t settle_reg = settling_pack(settling_total_cycles);

/* CTRL_LB: clock select bit only (settling is NOT in CTRL_LB). */
const uint8_t ctrl_lb = (uint8_t)(clk_sel);

/* Put in standby, program registers for "single point" */
if (ad5933_set_control(HB_STANDBY | hb_range_pga, ctrl_lb) != 0) return -EIO;
k_msleep(2);

if (wb_write24_strict(REG_START_FREQ_2, code) != 0) return -EIO;

/* No increment, 0 increments */
if (wb_write24_strict(REG_INC_FREQ_2, 0) != 0) return -EIO;
if (wb_write16_strict(REG_NUM_INC_2, 0) != 0) return -EIO;
if (wb_write16_strict(REG_SETTLE_2, settle_reg) != 0) return -EIO;


    /* Init frequency, then start sweep (measures first/only point) */
    if (ad5933_set_control(HB_INIT_FREQ | hb_range_pga, ctrl_lb) != 0) return -EIO;
    k_msleep(2);

    if (ad5933_set_control(HB_START_SWEEP | hb_range_pga, ctrl_lb) != 0) return -EIO;

    /* Timeout floor: wait_data_valid() expands this using settling_total_cycles/freq_hz */
    uint32_t timeout_floor = DATA_TIMEOUT_MS;
    if (freq_hz < 20u) {
        timeout_floor = 15000u;
    } else if (freq_hz < 100u) {
        timeout_floor = 4000u;
    }

    uint8_t st = 0;
    int ret = wait_data_valid(freq_hz, settling_total_cycles, timeout_floor, &st);
    if (ret) {
        LOG_WRN("[MEAS] f=%u Hz: timeout (last STATUS=0x%02X)", freq_hz, st);
        if (status_out) *status_out = st;
        return ret;
    }

    if (read_real_imag(real, imag) != 0) return -EIO;
    if (status_out) *status_out = st;

    return 0;
}

/* ---------------- Main ---------------- */

int main(void)
{
<<<<<<< HEAD
        return 4;
=======
    LOG_INF("=== AD5933 CALIBRATION THEN MEASURE (log sweep) ===");
    LOG_INF("Build: %s %s", __DATE__, __TIME__);

    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C not ready");
        return 0;
    }
    LOG_INF("Using I2C device: %s", i2c_dev->name);

    /* Range/PGA selection */
    const uint8_t hb_range_pga = (HB_RANGE_2VPP | HB_PGA_X5);

    /* External clock? */
    const uint8_t clk_sel = CTRL_LB_INT_CLK;
    const uint8_t ctrl_lb_init = (uint8_t)(clk_sel);

    /* Reset AD5933 (self-clearing reset bit) */
    (void)ad5933_set_control(HB_STANDBY | hb_range_pga, (uint8_t)(ctrl_lb_init | CTRL_LB_RESET));
    k_msleep(10);
    (void)ad5933_set_control(HB_STANDBY | hb_range_pga, ctrl_lb_init);
    k_msleep(5);

    LOG_INF("CTRL before start:");
    ad5933_dump_ctrl();

    /* Build log frequency list */
    static uint32_t freqs[SWEEP_POINTS];
    build_log_sweep(freqs, SWEEP_POINTS, SWEEP_START_HZ, SWEEP_STOP_HZ);

    /* Allocate calibration arrays */
    static double gain_factor[SWEEP_POINTS];
    static double phase_sys[SWEEP_POINTS];

    LOG_INF("[CAL] Connect known: Rseries + (Rpar||C)");
    LOG_INF("[CAL] Rseries=%.0f, Rpar=%.0f, C=%.9f", (double)R_SERIES_OHM, (double)R_PAR_OHM, (double)C_PAR_F);
    LOG_INF("[CAL] Log points=%u start=%uHz stop=%uHz", (unsigned)SWEEP_POINTS, (unsigned)SWEEP_START_HZ, (unsigned)SWEEP_STOP_HZ);
    LOG_INF("[CAL] idx f(Hz) REAL IMAG |Z_known|(mΩ) GainFactor(n) phase_off(mdeg)");

    for (uint16_t i = 0; i < SWEEP_POINTS; i++) {
        const uint32_t f = freqs[i];

        int16_t re = 0, im = 0;
        uint8_t st = 0;

        int ret = ad5933_measure_one(f, SETTLING_TOTAL_CYCLES, clk_sel, hb_range_pga, &re, &im, &st);
        if (ret) {
            gain_factor[i] = NAN;
            phase_sys[i] = NAN;
            continue;
        }

        const double d_re = (double)re;
        const double d_im = (double)im;
        const double mag_dft = sqrt((d_re * d_re) + (d_im * d_im));
        const double phi_dft = atan2(d_im, d_re);

        double zmag = 0.0, zphi = 0.0;
        z_known_network((double)f, &zmag, &zphi);

        /* Gain factor per datasheet approach: Gain = 1 / (|Z_known| * |DFT|) */
        const double gf = (mag_dft > 0.0) ? (1.0 / (zmag * mag_dft)) : NAN;

        /* System phase offset: phi_sys = phi_dft - phi_known */
        const double ph_sys = wrap_pi(phi_dft - zphi);

        gain_factor[i] = gf;
        phase_sys[i] = ph_sys;

        if ((i % PRINT_EVERY_N) == 0u) {
            LOG_INF("%3u %7u %6d %6d %12lld %12.3e %10lld",
                    (unsigned)i,
                    (unsigned)f,
                    (int)re,
                    (int)im,
                    (long long)llround(zmag * 1000.0),
                    gf,
                    (long long)llround(ph_sys * 1000.0 * 180.0 / PI_D));
            if (LINE_DELAY_MS) k_msleep(LINE_DELAY_MS);
        }
    }

    LOG_INF("[CAL] Done. Swap to UNKNOWN DUT.");
    k_msleep(200);

    LOG_INF("[LOG] points=%u start=%uHz stop=%uHz", (unsigned)SWEEP_POINTS, (unsigned)SWEEP_START_HZ, (unsigned)SWEEP_STOP_HZ);
    LOG_INF("[LOG] idx f(Hz) REAL IMAG |DFT| phase(deg) |Z|(mΩ) Zphase(deg)");

    for (uint16_t i = 0; i < SWEEP_POINTS; i++) {
        const uint32_t f = freqs[i];
        const double gf = gain_factor[i];
        const double ph_sys = phase_sys[i];

        if (!isfinite(gf) || !isfinite(ph_sys)) {
            continue;
        }

        int16_t re = 0, im = 0;
        uint8_t st = 0;

        int ret = ad5933_measure_one(f, SETTLING_TOTAL_CYCLES, clk_sel, hb_range_pga, &re, &im, &st);
        if (ret) continue;

        const double d_re = (double)re;
        const double d_im = (double)im;
        const double mag_dft = sqrt((d_re * d_re) + (d_im * d_im));
        const double phi_dft = atan2(d_im, d_re);

        const double zmag = (mag_dft > 0.0) ? (1.0 / (gf * mag_dft)) : NAN;
        const double zphi = wrap_pi(phi_dft - ph_sys);

        if ((i % PRINT_EVERY_N) == 0u) {
            LOG_INF("%3u %7u %6d %6d %6lld %10lld %12lld %10lld",
                    (unsigned)i,
                    (unsigned)f,
                    (int)re,
                    (int)im,
                    (long long)llround(mag_dft),
                    (long long)llround(phi_dft * 180.0 / PI_D),
                    (long long)llround(zmag * 1000.0),
                    (long long)llround(zphi * 180.0 / PI_D));
            if (LINE_DELAY_MS) k_msleep(LINE_DELAY_MS);
        }
    }

    LOG_INF("Done.");
    return 0;
>>>>>>> 3f6773a (Actual Commit)
}
