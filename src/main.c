/*
 * Minimal AD5933 bring-up (Zephyr)
 *
 * What this does:
 *  1) Reset AD5933
 *  2) Configure output range + PGA, select internal clock
 *  3) Program ONE frequency (single-point "sweep")
 *  4) Start measurement
 *  5) Wait for DATA_VALID
 *  6) Read REAL/IMAG and print them
 *
 * Wiring (typical):
 *  - VOUT -> one side of DUT
 *  - VIN  -> other side of DUT (through the AD5933 input network / RFB)
 *  - RFB between VIN and VOUT (feedback resistor for internal I/V)
 *  - I2C SDA/SCL + power/ground
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <stdint.h>

LOG_MODULE_REGISTER(ad5933_min, LOG_LEVEL_INF);

/* -------- AD5933 I2C address -------- */
#define AD5933_ADDR 0x0D

/* -------- Registers -------- */
#define REG_CTRL_HB      0x80
#define REG_CTRL_LB      0x81
#define REG_START_FREQ   0x82 /* 0x82..0x84 */
#define REG_INC_FREQ     0x85 /* 0x85..0x87 */
#define REG_NUM_INC      0x88 /* 0x88..0x89 */
#define REG_SETTLE       0x8A /* 0x8A..0x8B */
#define REG_STATUS       0x8F
#define REG_REAL         0x94 /* 0x94..0x95 */
#define REG_IMAG         0x96 /* 0x96..0x97 */

/* -------- Status bits -------- */
#define STATUS_DATA_VALID 0x02  /* D1 */

/* -------- CTRL_LB bits --------
 * INT clock is "bit cleared" (0x00). EXT clock sets bit 0x08.
 * Reset is bit 0x10 (self-clears).
 */
#define CTRL_LB_INT_CLK  0x00
#define CTRL_LB_EXT_CLK  0x08
#define CTRL_LB_RESET    0x10

/* -------- CTRL_HB function codes (upper nibble) -------- */
#define HB_INIT_FREQ     0x10
#define HB_START_SWEEP   0x20
#define HB_STANDBY       0xB0

/* -------- Output range + PGA bits (common mapping) --------
 * Verify with your datasheet if needed.
 */
#define HB_RANGE_2VPP    0x00
#define HB_RANGE_1VPP    0x02
#define HB_RANGE_400MVPP 0x04
#define HB_RANGE_200MVPP 0x06

#define HB_PGA_X5        0x00
#define HB_PGA_X1        0x01

/* -------- Clock (MCLK) used by AD5933 --------
 * If you are NOT feeding an external MCLK pin, AD5933 uses internal oscillator.
 * Many designs assume ~16 MHz internal.
 */
#define AD5933_MCLK_HZ   16000000u

/* -------- Demo frequency -------- */
#define DEMO_FREQ_HZ     10000u   /* 10 kHz is a good “quick test” frequency */

/* -------- Settling cycles --------
 * 16-bit: [15:14] multiplier, [13:0] cycles
 * We'll just do x1 and a small cycle count.
 */
#define SETTLE_X1        0x0000
#define SETTLE_CYCLES    0x0002  /* 2 cycles (minimal). Increase if noisy. */

static const struct device *i2c_dev;

/* ---- Convert frequency to AD5933 tuning word (24-bit) ----
 * code = f * 2^29 / MCLK   (common AD5933 formula)
 */
static uint32_t freq_to_word(uint32_t f_hz)
{
    uint64_t num = ((uint64_t)f_hz) << 29;
    uint32_t code = (uint32_t)(num / (uint64_t)AD5933_MCLK_HZ);
    return code & 0xFFFFFFu;
}

/* ---- I2C helpers ---- */
static int w8(uint8_t reg, uint8_t v)
{
    return i2c_reg_write_byte(i2c_dev, AD5933_ADDR, reg, v);
}

static int r8(uint8_t reg, uint8_t *v)
{
    return i2c_reg_read_byte(i2c_dev, AD5933_ADDR, reg, v);
}

static int w16(uint8_t reg_hi, uint16_t v)
{
    int ret = w8(reg_hi, (uint8_t)((v >> 8) & 0xFF));
    if (ret) return ret;
    return w8(reg_hi + 1, (uint8_t)(v & 0xFF));
}

static int w24(uint8_t reg_msb, uint32_t v24)
{
    int ret = w8(reg_msb, (uint8_t)((v24 >> 16) & 0xFF));
    if (ret) return ret;
    ret = w8(reg_msb + 1, (uint8_t)((v24 >> 8) & 0xFF));
    if (ret) return ret;
    return w8(reg_msb + 2, (uint8_t)(v24 & 0xFF));
}

/* ---- Write control register (HB then LB) ---- */
static int set_ctrl(uint8_t hb, uint8_t lb)
{
    int ret = w8(REG_CTRL_HB, hb);
    if (ret) return ret;
    return w8(REG_CTRL_LB, lb);
}

/* ---- Read REAL/IMAG (signed 16-bit) ---- */
static int read_real_imag(int16_t *real, int16_t *imag)
{
    uint8_t rb[2], ib[2];

    int ret = i2c_burst_read(i2c_dev, AD5933_ADDR, REG_REAL, rb, 2);
    if (ret) return ret;

    ret = i2c_burst_read(i2c_dev, AD5933_ADDR, REG_IMAG, ib, 2);
    if (ret) return ret;

    *real = (int16_t)((rb[0] << 8) | rb[1]);
    *imag = (int16_t)((ib[0] << 8) | ib[1]);
    return 0;
}

int main(void)
{
    LOG_INF("AD5933 minimal test start");

    /* 1) Get I2C device (assumes you are using i2c0 in your DTS) */
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("i2c0 not ready");
        return 0;
    }
    LOG_INF("Using I2C: %s", i2c_dev->name);

    /* Choose output range + PGA */
    const uint8_t hb_cfg = HB_RANGE_2VPP | HB_PGA_X5;

    /* 2) Reset AD5933 (reset bit self-clears) */
    (void)set_ctrl(HB_STANDBY | hb_cfg, CTRL_LB_INT_CLK | CTRL_LB_RESET);
    k_msleep(10);
    (void)set_ctrl(HB_STANDBY | hb_cfg, CTRL_LB_INT_CLK);
    k_msleep(5);

    /* 3) Program single-point measurement (start freq = DEMO, inc=0, num_inc=0) */
    const uint32_t code = freq_to_word(DEMO_FREQ_HZ);
    const uint16_t settle = (uint16_t)(SETTLE_X1 | (SETTLE_CYCLES & 0x3FFF));

    if (w24(REG_START_FREQ, code) ||
        w24(REG_INC_FREQ, 0) ||
        w16(REG_NUM_INC, 0) ||
        w16(REG_SETTLE, settle)) {
        LOG_ERR("Failed writing AD5933 config regs");
        return 0;
    }

    /* 4) INIT_FREQ then START_SWEEP to trigger measurement */
    if (set_ctrl(HB_INIT_FREQ | hb_cfg, CTRL_LB_INT_CLK)) {
        LOG_ERR("INIT_FREQ failed");
        return 0;
    }
    k_msleep(2);

    if (set_ctrl(HB_START_SWEEP | hb_cfg, CTRL_LB_INT_CLK)) {
        LOG_ERR("START_SWEEP failed");
        return 0;
    }

    /* 5) Poll until DATA_VALID */
    uint8_t st = 0;
    int64_t t0 = k_uptime_get();
    while ((k_uptime_get() - t0) < 2000) { /* 2s timeout */
        if (r8(REG_STATUS, &st)) {
            LOG_ERR("STATUS read failed");
            return 0;
        }
        if (st & STATUS_DATA_VALID) {
            break;
        }
        k_msleep(5);
    }

    if (!(st & STATUS_DATA_VALID)) {
        LOG_ERR("Timeout waiting for DATA_VALID (STATUS=0x%02X)", st);
        return 0;
    }

    /* 6) Read result */
    int16_t re = 0, im = 0;
    if (read_real_imag(&re, &im)) {
        LOG_ERR("Failed reading REAL/IMAG");
        return 0;
    }

    LOG_INF("f=%u Hz | REAL=%d IMAG=%d", (unsigned)DEMO_FREQ_HZ, (int)re, (int)im);
    LOG_INF("Done.");

    return 0;
}
