/*
 * src/main.c — AD5933 sweep (Zephyr / nRF52840 dongle)
 *
 * What it does:
 *  - Scans I2C bus for AD5933 (0x0D or 0x0C) and selects it
 *  - Initializes AD5933
 *  - Performs a linear frequency sweep
 *  - Prints: f_hz, Re, Im, |DFT|, phase_deg
 *
 * Optional:
 *  - If you calibrate with a known resistor (RCAL_OHM), it prints |Z| too.
 *
 * Hardware basics:
 *  - nRF52840 SDA/SCL must match overlay pins
 *  - SDA/SCL need pull-ups to 3.3V (4.7k is good)
 *  - Common GND between dongle and AD5933 board
 *  - AD5933 powered correctly (3.3V if your module supports it)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

#include <stdint.h>

LOG_MODULE_REGISTER(ad5933, LOG_LEVEL_INF);

/* ---------------- User sweep settings ---------------- */
#define SWEEP_START_HZ   1000u
#define SWEEP_STEP_HZ    1000u
#define SWEEP_POINTS     100u

/* Settling cycles: increase if your readings are unstable */
#define SETTLING_CYCLES  100u

/* If you do a calibration run with a known resistor (recommended),
 * set its value here (ohms). If you don't calibrate, leave 0.
 */
#define RCAL_OHM         10000.0

/* ---------------- AD5933 registers ---------------- */
#define REG_CTRL_HB      0x80
#define REG_CTRL_LB      0x81
#define REG_START_FREQ_2 0x82 /* 0x82..0x84 */
#define REG_INC_FREQ_2   0x85 /* 0x85..0x87 */
#define REG_NUM_INC_2    0x88 /* 0x88..0x89 */
#define REG_SETTLE_2     0x8A /* 0x8A..0x8B */
#define REG_STATUS       0x8F
#define REG_REAL_1       0x94 /* 0x94..0x95 */
#define REG_IMAG_1       0x96 /* 0x96..0x97 */

/* Status bits */
#define STATUS_DATA_VALID 0x02

/* Control LB */
#define CTRL_LB_INT_CLK   0x00
#define CTRL_LB_EXT_CLK   0x08
#define CTRL_LB_RESET     0x10

/* Control HB function codes (D15..D12) */
#define HB_INIT_FREQ      0x10
#define HB_START_SWEEP    0x20
#define HB_INC_FREQ       0x30
#define HB_REPEAT_FREQ    0x40
#define HB_STANDBY        0xB0

/* Output range + PGA (module dependent; these are common defaults) */
#define HB_RANGE_2VPP     0x00
#define HB_PGA_X5         0x00
#define HB_PGA_X1         0x01

/* IMPORTANT: AD5933 clock.
 * Many boards use ~16MHz or 16.776MHz. If you know your board’s oscillator,
 * change this. If unsure, start with 16MHz and focus on getting ACK + stable data.
 */
#define AD5933_MCLK_HZ    16000000u

static const struct device *i2c_dev;
static uint8_t ad5933_addr = 0;

/* ---------- helpers ---------- */

static uint32_t freq_to_word(uint32_t f_hz)
{
	/* AD5933: code = f * 2^27 / (MCLK/4) = f * 2^29 / MCLK */
	uint64_t num = (uint64_t)f_hz << 29;
	uint32_t code = (uint32_t)(num / (uint64_t)AD5933_MCLK_HZ);
	return code & 0xFFFFFFu;
}

static int w8(uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte(i2c_dev, ad5933_addr, reg, val);
}

static int r8(uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte(i2c_dev, ad5933_addr, reg, val);
}

static int w16(uint8_t reg_hi, uint16_t v)
{
	int ret = w8(reg_hi, (uint8_t)(v >> 8));
	if (ret) return ret;
	return w8((uint8_t)(reg_hi + 1), (uint8_t)(v & 0xFF));
}

static int w24(uint8_t reg_msb, uint32_t v24)
{
	int ret = w8(reg_msb, (uint8_t)((v24 >> 16) & 0xFF));
	if (ret) return ret;
	ret = w8((uint8_t)(reg_msb + 1), (uint8_t)((v24 >> 8) & 0xFF));
	if (ret) return ret;
	return w8((uint8_t)(reg_msb + 2), (uint8_t)(v24 & 0xFF));
}

static int read_ri(int16_t *re, int16_t *im)
{
	uint8_t rb[2], ib[2];
	int ret = i2c_burst_read(i2c_dev, ad5933_addr, REG_REAL_1, rb, 2);
	if (ret) return ret;
	ret = i2c_burst_read(i2c_dev, ad5933_addr, REG_IMAG_1, ib, 2);
	if (ret) return ret;

	*re = (int16_t)((rb[0] << 8) | rb[1]);
	*im = (int16_t)((ib[0] << 8) | ib[1]);
	return 0;
}

static int wait_data_valid(uint32_t timeout_ms)
{
	int64_t t0 = k_uptime_get();
	uint8_t st = 0;

	while ((uint32_t)(k_uptime_get() - t0) < timeout_ms) {
		if (r8(REG_STATUS, &st) != 0) {
			return -EIO;
		}
		if (st & STATUS_DATA_VALID) {
			return 0;
		}
		k_msleep(2);
	}
	return -ETIMEDOUT;
}

/* ---------- I2C scan / detect ---------- */

static bool i2c_addr_acks(uint8_t addr)
{
	/* Probe with a register read of STATUS (most meaningful) */
	uint8_t st = 0;
	int ret = i2c_reg_read_byte(i2c_dev, addr, REG_STATUS, &st);
	return (ret == 0);
}

static bool ad5933_detect(void)
{
	/* Common AD5933 7-bit addresses depending on ADDR strap */
	const uint8_t candidates[] = { 0x0D, 0x0C };

	for (size_t i = 0; i < sizeof(candidates); i++) {
		uint8_t a = candidates[i];
		if (i2c_addr_acks(a)) {
			ad5933_addr = a;
			uint8_t st = 0;
			(void)r8(REG_STATUS, &st);
			LOG_INF("AD5933 ACK at 0x%02X (STATUS=0x%02X)", ad5933_addr, st);
			return true;
		}
	}

	LOG_ERR("No AD5933 ACK at 0x0D/0x0C. If you still get -EIO/-5:");
	LOG_ERR("1) overlay pins don't match wiring, 2) no pullups to 3.3V, 3) no common GND, 4) not powered.");
	return false;
}

/* ---------- AD5933 init/sweep ---------- */

static int ad5933_reset(uint8_t hb_range_pga, uint8_t clk_sel)
{
	/* Put in standby, then toggle RESET bit in LB */
	int ret = w8(REG_CTRL_HB, (uint8_t)(HB_STANDBY | hb_range_pga));
	if (ret) return ret;

	ret = w8(REG_CTRL_LB, (uint8_t)(clk_sel | CTRL_LB_RESET));
	if (ret) return ret;

	k_msleep(5);

	ret = w8(REG_CTRL_LB, clk_sel);
	if (ret) return ret;

	k_msleep(5);
	return 0;
}

static int ad5933_program_sweep(uint32_t start_hz, uint32_t step_hz, uint16_t points, uint16_t settling_cycles)
{
	/* AD5933 wants NUM_INCR = points-1 */
	if (points < 2) return -EINVAL;

	uint32_t start_code = freq_to_word(start_hz);
	uint32_t inc_code   = freq_to_word(step_hz);
	uint16_t num_incr   = (uint16_t)(points - 1);

	int ret = w24(REG_START_FREQ_2, start_code);
	if (ret) return ret;

	ret = w24(REG_INC_FREQ_2, inc_code);
	if (ret) return ret;

	ret = w16(REG_NUM_INC_2, num_incr);
	if (ret) return ret;

	/* Settling: simplest form (no multiplier bits), just cycles in 14-bit field */
	if (settling_cycles > 0x3FFF) settling_cycles = 0x3FFF;
	ret = w16(REG_SETTLE_2, settling_cycles);
	if (ret) return ret;

	return 0;
}

static int ad5933_cmd(uint8_t hb_func, uint8_t hb_range_pga, uint8_t clk_sel)
{
	int ret = w8(REG_CTRL_HB, (uint8_t)(hb_func | hb_range_pga));
	if (ret) return ret;
	return w8(REG_CTRL_LB, clk_sel);
}

/* ---------- main ---------- */

int main(void)
{
	LOG_INF("AD5933 sweep starting...");

	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("i2c0 not ready");
		return 0;
	}
	LOG_INF("Using I2C device: %s", i2c_dev->name);

	if (!ad5933_detect()) {
		while (1) k_sleep(K_SECONDS(1));
	}

	/* Setup: range and PGA */
	uint8_t hb_range_pga = (uint8_t)(HB_RANGE_2VPP | HB_PGA_X5);

	/* Clock select: start with INT clock unless you know you’re providing EXT */
	uint8_t clk_sel = CTRL_LB_INT_CLK;

	int ret = ad5933_reset(hb_range_pga, clk_sel);
	if (ret) {
		LOG_ERR("reset failed: %d", ret);
		while (1) k_sleep(K_SECONDS(1));
	}

	ret = ad5933_program_sweep(SWEEP_START_HZ, SWEEP_STEP_HZ, SWEEP_POINTS, SETTLING_CYCLES);
	if (ret) {
		LOG_ERR("program_sweep failed: %d", ret);
		while (1) k_sleep(K_SECONDS(1));
	}

	/* Start sequence */
	ret = ad5933_cmd(HB_INIT_FREQ, hb_range_pga, clk_sel);
	if (ret) {
		LOG_ERR("INIT failed: %d", ret);
		while (1) k_sleep(K_SECONDS(1));
	}
	k_msleep(2);

	ret = ad5933_cmd(HB_START_SWEEP, hb_range_pga, clk_sel);
	if (ret) {
		LOG_ERR("START failed: %d", ret);
		while (1) k_sleep(K_SECONDS(1));
	}

	/* Optional calibration:
	 * If RCAL_OHM > 0, we treat the first sweep as calibration and compute gain per point.
	 * Then you can re-run a second sweep for your unknown (R||C) without reflashing
	 * (just reset/power-cycle after swapping DUT).
	 */
	static double gain[SWEEP_POINTS];
	for (uint16_t i = 0; i < SWEEP_POINTS; i++) gain[i] = NAN;

	LOG_INF("Format:");
	if (RCAL_OHM > 0.0) {
		LOG_INF("idx,f_hz,Re,Im,mag,phase_deg,Z_ohm   (CAL mode: connect RCAL=%.2f ohm)", RCAL_OHM);
	} else {
		LOG_INF("idx,f_hz,Re,Im,mag,phase_deg        (RAW mode: no impedance calc)");
	}

	for (uint16_t i = 0; i < SWEEP_POINTS; i++) {

		/* Wait for data */
		ret = wait_data_valid(2000);
		if (ret) {
			LOG_ERR("Timeout waiting data at i=%u (ret=%d)", i, ret);
			break;
		}

		int16_t re = 0, im = 0;
		ret = read_ri(&re, &im);
		if (ret) {
			LOG_ERR("read_ri failed at i=%u (ret=%d)", i, ret);
			break;
		}

		double dre = (double)re;
		double dim = (double)im;
		double mag = sqrt((dre * dre) + (dim * dim));
		double ph_deg = atan2(dim, dre) * (180.0 / PI);


		uint32_t f = SWEEP_START_HZ + (uint32_t)i * SWEEP_STEP_HZ;

		if (RCAL_OHM > 0.0) {
			/* Gain factor = 1 / (RCAL * mag) */
			double g = (mag > 0.0) ? (1.0 / (RCAL_OHM * mag)) : (double)NAN;

			gain[i] = g;

			/* In calibration mode, the impedance would be RCAL (sanity check) */
			double z = (mag > 0.0) ? (1.0 / (g * mag)) : NAN;

			LOG_INF("%u,%u,%d,%d,%.3f,%.2f,%.2f",
				(unsigned)i, (unsigned)f, (int)re, (int)im, mag, ph_deg, z);
		} else {
			LOG_INF("%u,%u,%d,%d,%.3f,%.2f",
				(unsigned)i, (unsigned)f, (int)re, (int)im, mag, ph_deg);
		}

		/* Step frequency (except after last point) */
		if (i + 1 < SWEEP_POINTS) {
			ret = ad5933_cmd(HB_INC_FREQ, hb_range_pga, clk_sel);
			if (ret) {
				LOG_ERR("INC failed at i=%u (ret=%d)", i, ret);
				break;
			}
		}
	}

	LOG_INF("Sweep done.");
	while (1) k_sleep(K_SECONDS(1));
}
