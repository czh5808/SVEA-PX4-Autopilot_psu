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

/**
 * @file as7343.h
 *
 * Driver for the ams OSRAM AS7343 14-channel multispectral sensor connected via I2C.
 * Register map based on the SparkFun AS7343 Arduino and Python libraries.
 */

#pragma once

#include <stdint.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_spectral.h>

using namespace time_literals;

static constexpr uint8_t AS7343_I2C_ADDRESS = 0x39;
static constexpr uint8_t AS7343_DEVICE_ID = 0x81;

// Bank 1 registers (0x58 - 0x7F), only accessible with CFG0.REG_BANK set
static constexpr uint8_t AS7343_REG_ID = 0x5A;

// Bank 0 registers (0x80 and above)
static constexpr uint8_t AS7343_REG_ENABLE = 0x80;
static constexpr uint8_t AS7343_REG_ATIME = 0x81;
static constexpr uint8_t AS7343_REG_STATUS2 = 0x90;
static constexpr uint8_t AS7343_REG_ASTATUS = 0x94; // reading ASTATUS latches all DATA registers
static constexpr uint8_t AS7343_REG_CFG0 = 0xBF;
static constexpr uint8_t AS7343_REG_CFG1 = 0xC6;
static constexpr uint8_t AS7343_REG_LED = 0xCD;
static constexpr uint8_t AS7343_REG_ASTEP_L = 0xD4;
static constexpr uint8_t AS7343_REG_CFG20 = 0xD6;

static constexpr uint8_t AS7343_ENABLE_PON = (1 << 0);
static constexpr uint8_t AS7343_ENABLE_SP_EN = (1 << 1);
static constexpr uint8_t AS7343_STATUS2_AVALID = (1 << 6);
static constexpr uint8_t AS7343_ASTATUS_ASAT = (1 << 7);
static constexpr uint8_t AS7343_CFG0_REG_BANK = (1 << 4);
static constexpr uint8_t AS7343_CFG20_AUTO_SMUX_18CH = (3 << 5);

static constexpr uint8_t AS7343_AGAIN_MAX = 12; // 2048x

// Integration time per SMUX cycle = (ATIME + 1) * (ASTEP + 1) * 2.78 us ~= 50 ms
static constexpr uint8_t AS7343_ATIME = 29;
static constexpr uint16_t AS7343_ASTEP = 599;
static constexpr float AS7343_INTEGRATION_TIME_S = (AS7343_ATIME + 1) * (AS7343_ASTEP + 1) * 2.78e-6f;

class AS7343 : public device::I2C, public I2CSPIDriver<AS7343>
{
public:
	AS7343(const I2CSPIDriverConfig &config);
	~AS7343() override;

	int init() override;
	int probe() override;
	void RunImpl();
	static void print_usage();

protected:
	void print_status() override;

private:
	static constexpr int NUM_CHANNELS = sensor_spectral_s::NUM_CHANNELS;

	int configure();
	int collect();

	int set_register_bank(bool bank1);
	int read_reg(uint8_t address, uint8_t &data);
	int write_reg(uint8_t address, uint8_t value);

	uORB::PublicationMulti<sensor_spectral_s> _sensor_spectral_pub{ORB_ID(sensor_spectral)};

	sensor_spectral_s _sensor_spectral{};
	uint8_t _again{9};

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;
	perf_counter_t _saturated;
};
