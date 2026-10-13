/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "as7343.h"

#include <lib/mathlib/mathlib.h>
#include <parameters/param.h>

AS7343::AS7343(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors")),
	_saturated(perf_alloc(PC_COUNT, MODULE_NAME": saturated"))
{
	_sensor_spectral.device_id = get_device_id();
}

AS7343::~AS7343()
{
	ScheduleClear();
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_saturated);
}

int AS7343::probe()
{
	if (set_register_bank(true) != PX4_OK) {
		return PX4_ERROR;
	}

	uint8_t device_id = 0;
	const int ret = read_reg(AS7343_REG_ID, device_id);

	if (set_register_bank(false) != PX4_OK || ret != PX4_OK) {
		return PX4_ERROR;
	}

	if (device_id != AS7343_DEVICE_ID) {
		PX4_DEBUG("unexpected device ID 0x%02X", device_id);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int AS7343::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_DEBUG("I2C init failed");
		return ret;
	}

	int32_t again = _again;
	param_get(param_find("SENS_AS7343_GAIN"), &again);
	_again = math::constrain(again, (int32_t)0, (int32_t)AS7343_AGAIN_MAX);

	ret = configure();

	if (ret != PX4_OK) {
		PX4_ERR("configuration failed");
		return ret;
	}

	_sensor_spectral_pub.advertise();

	// A full 18 channel measurement takes three SMUX cycles, poll for completion faster than that
	ScheduleOnInterval(50_ms);
	return PX4_OK;
}

int AS7343::configure()
{
	// Power on with measurements disabled while configuring
	int ret = write_reg(AS7343_REG_ENABLE, AS7343_ENABLE_PON);

	uint8_t cfg20 = 0;
	ret |= read_reg(AS7343_REG_CFG20, cfg20);
	ret |= write_reg(AS7343_REG_CFG20, (cfg20 & ~(3 << 5)) | AS7343_CFG20_AUTO_SMUX_18CH);

	ret |= write_reg(AS7343_REG_ATIME, AS7343_ATIME);
	ret |= write_reg(AS7343_REG_ASTEP_L, AS7343_ASTEP & 0xFF);
	ret |= write_reg(AS7343_REG_ASTEP_L + 1, AS7343_ASTEP >> 8);

	uint8_t cfg1 = 0;
	ret |= read_reg(AS7343_REG_CFG1, cfg1);
	ret |= write_reg(AS7343_REG_CFG1, (cfg1 & ~0x1F) | _again);

	// Make sure the on-board LED driver is off
	ret |= write_reg(AS7343_REG_LED, 0);

	ret |= write_reg(AS7343_REG_ENABLE, AS7343_ENABLE_PON | AS7343_ENABLE_SP_EN);

	if (ret != PX4_OK) {
		return PX4_ERROR;
	}

	_sensor_spectral.gain = 0.5f * (1 << _again);
	_sensor_spectral.integration_time = AS7343_INTEGRATION_TIME_S;

	return PX4_OK;
}

void AS7343::RunImpl()
{
	perf_begin(_sample_perf);

	if (collect() != PX4_OK) {
		perf_count(_comms_errors);
	}

	perf_end(_sample_perf);
}

int AS7343::collect()
{
	uint8_t status2 = 0;

	if (read_reg(AS7343_REG_STATUS2, status2) != PX4_OK) {
		return PX4_ERROR;
	}

	if ((status2 & AS7343_STATUS2_AVALID) == 0) {
		// measurement still in progress
		return PX4_OK;
	}

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// ASTATUS followed by 18 little-endian 16 bit channel values
	uint8_t reg = AS7343_REG_ASTATUS;
	uint8_t buffer[1 + 2 * NUM_CHANNELS] {};

	if (transfer(&reg, 1, buffer, sizeof(buffer)) != PX4_OK) {
		return PX4_ERROR;
	}

	_sensor_spectral.saturated = (buffer[0] & AS7343_ASTATUS_ASAT) != 0;

	if (_sensor_spectral.saturated) {
		perf_count(_saturated);
	}

	for (int i = 0; i < NUM_CHANNELS; i++) {
		_sensor_spectral.counts[i] = buffer[1 + 2 * i] | (buffer[2 + 2 * i] << 8);
	}

	_sensor_spectral.timestamp_sample = timestamp_sample;
	_sensor_spectral.timestamp = hrt_absolute_time();
	_sensor_spectral_pub.publish(_sensor_spectral);

	return PX4_OK;
}

int AS7343::set_register_bank(bool bank1)
{
	uint8_t cfg0 = 0;

	if (read_reg(AS7343_REG_CFG0, cfg0) != PX4_OK) {
		return PX4_ERROR;
	}

	if (bank1) {
		cfg0 |= AS7343_CFG0_REG_BANK;

	} else {
		cfg0 &= ~AS7343_CFG0_REG_BANK;
	}

	return write_reg(AS7343_REG_CFG0, cfg0);
}

int AS7343::read_reg(uint8_t address, uint8_t &data)
{
	return transfer(&address, 1, &data, 1);
}

int AS7343::write_reg(uint8_t address, uint8_t value)
{
	uint8_t buf[2] = {address, value};
	return transfer(buf, sizeof(buf), nullptr, 0);
}
