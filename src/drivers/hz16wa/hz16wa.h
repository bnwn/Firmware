/****************************************************************************
 *
 *   Copyright (c) 2016 Enigma Development Team. All rights reserved.
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
 * @file hz16wa.h
 * @author zhouxiaoming
 *
 * Driver for the Flowmeter hz16wa connected via PWM.
 *
 * This driver accesses the pwm_input published by the pwm_input driver.
 */
#pragma once

#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_device.h>
#include <drivers/drv_flowmeter.h>
#include <drivers/drv_pwm_input.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>
#include <systemlib/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/flowmeter_sensor.h>

#include <board_config.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <cstdlib>
#include <string.h>
#include <stdio.h>


#define HZ16WA_DEVICE_PATH "/dev/hz16wa"

/* Device limits */
#define HZ16WA_MIN_FLOWRATE
#define HZ16WA_MAX_FLOWRATE

/* default conversion wait time */
#define HZ16WA_CONVERSION_INTERVAL 50*1000UL /* 50ms */

#define HZ16WA_EMPTY_TIMEOUT       1000*1000UL /* 1s */

/* maximun time to wait for a conversion to complete */
#define HZ16WA_CONVERSION_TIMEOUT  1000*1000UL /* 1s */


class HZ16WA : public device::CDev
{
public:
	HZ16WA(const char *path);
        virtual ~HZ16WA();

	int init() override;

	ssize_t read(struct file *filp, char *buffer, size_t buflen) override;
	int	ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* @brief
	*   Diagnostics - print some basic information about the driver.
	*/
    void print_info();

	/**
	 * @brief
	 *   print registers to console
	 */
    void print_registers();

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg        Instance pointer for the driver that is polling.
	*/
    static void cycle_trampoline(void *arg);

protected:
	virtual int probe();

    /**
    * Set the min and max distance thresholds if you want the end points of the sensors
    * range to be brought in at all, otherwise it will use the defaults LL40LS_MIN_DISTANCE
    * and LL40LS_MAX_DISTANCE
    */
    void                set_minimum_flowrate(const float min);
    void                set_maximum_flowrate(const float max);
    float               get_minimum_flowrate() const;
    float               get_maximum_flowrate() const;

private:
	work_s			_work;
    ringbuffer::RingBuffer	*_reports;
    int                     _class_instance;
    int                     _orb_class_instance;
	int			_pwmSub;
    struct pwm_input_s	_pwm;
	orb_advert_t	        _flowmeter_sensor_topic;
	struct flowmeter_sensor_s _flowmeter;
    float                   _max_flowrate;
    float                   _min_flowrate;

	perf_counter_t	        _sample_perf;
	perf_counter_t	        _read_errors;
    perf_counter_t	        _buffer_overflows;
    perf_counter_t	        _sensor_zero_resets;

    int                     _measure_ticks;
	/**
	 * Start automatic measurement
	 */
    void start();

	/**
	 * Stop automatic measurement
	 */
    void stop();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void cycle();

	/**
	 * Fetch measurements from the sensor and update the report ring.
	 */
    int measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
    int collect();

	/**
	 *  Reset the driver.
	 */
    int reset();
};
