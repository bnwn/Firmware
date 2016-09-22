/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file pointatob.h
 *
 * Navigator mode to access pointatob
 *
 * @author Enigma
 */

#ifndef NAVIGATOR_POINTATOB_H
#define NAVIGATOR_POINTATOB_H

#include <drivers/drv_hrt.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <dataman/dataman.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>

#include "navigator_mode.h"
#include "mission_block.h"
#include "mission_feasibility_checker.h"

class Navigator;

class PointAToB : public MissionBlock
{
public:
    PointAToB(Navigator *navigator, const char *name);

    virtual ~PointAToB();

	virtual void on_inactive();

	virtual void on_activation();

    virtual void on_active();

    const unsigned DM_POINT_COUNT = 4;

private:
    /**
     * Check whether the point item is reached
     */
    bool is_point_item_reached(struct pointatob_item_s point_item);

	/**
     * Move on to next point item or switch to loiter
	 */
    void advance_point();

	/**
     * Set new point items
	 */
    void set_point_items();

	/**
	 * Returns true if we need to do a takeoff at the current state
	 */
	bool do_need_takeoff();

	/**
     * Calculate takeoff height for point item considering ground clearance
     */
    float calculate_takeoff_altitude(struct pointatob_item_s *point_item);

    float get_absolute_altitude_for_item(struct pointatob_item_s &point_item);

	/**
     * Save current point state to dataman
     */
    void save_current_state();

	/**
     * Check whether a point is ready to go
	 */
    bool check_point_valid(bool force);

    /**
     * update point item
     */
    void update_point_item();

	/**
     * Reset point item
	 */
    void reset_point_item();

    /**
     * Returns true if we need to reset the point
     */
    bool need_to_reset_point(bool active);

    /**
     * current point set to position setpoint triplet
     */
    void point_item_to_position_setpoint(const struct pointatob_item_s *item, struct position_setpoint_s *sp);

    control::BlockParamFloat _param_takeoff_alt;
	control::BlockParamInt _param_yawmode;
	control::BlockParamInt _param_force_vtol;
    control::BlockParamFloat _param_fw_climbout_diff;
    control::BlockParamInt _param_turn_direction;
    control::BlockParamInt _param_interval_distance;
    control::BlockParamInt _param_flight_altitude;
    control::BlockParamInt _param_need_to_takeoff;

    pointatob_item_s _point_item, _point_item_a, _point_item_b, _point_item_current, _point_item_next;
	bool _need_takeoff;					/**< if true, then takeoff must be performed before going to the first waypoint (if needed) */
    bool _point_updated;
	bool _inited;
    bool _home_inited;
    bool _need_point_reset;

    enum work_item_type {
        WORK_ITEM_TYPE_DEFAULT,		/**< default mission item */
        WORK_ITEM_TYPE_TAKEOFF,		/**< takeoff before moving to waypoint */
        WORK_ITEM_TYPE_MOVE_TO_LAND,	/**< move to land waypoint before descent */
		WORK_ITEM_TYPE_ALIGN,		/**< align for next waypoint */
		WORK_ITEM_TYPE_CMD_BEFORE_MOVE,	/**<  */
		WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF,	/**<  */
		WORK_ITEM_TYPE_TRANSITON_BEFORE_LAND,	/**<  */
		WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION	/**<  */
	} _work_item_type;	/**< current type of work to do (sub mission item) */

};

#endif
