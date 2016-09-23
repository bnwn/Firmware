/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file navigator_pointatob.cpp
 *
 * Helper class to access pointatob
 *
 * @author Enigma
 */

#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <float.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <drivers/drv_hrt.h>

#include <dataman/dataman.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <navigator/navigation.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

#include "pointatob.h"
#include "navigator.h"

PointAToB::PointAToB(Navigator *navigator, const char *name) :
    MissionBlock(navigator, name),
    _param_takeoff_alt(this, "MIS_TAKEOFF_ALT", false),
	_param_yawmode(this, "MIS_YAWMODE", false),
	_param_force_vtol(this, "VT_NAV_FORCE_VT", false),
    _param_fw_climbout_diff(this, "FW_CLMBOUT_DIFF", false),
    _param_turn_direction(this, "ATOB_TURN_DIR", false),
    _param_interval_distance(this, "ATOB_INTERVAL_D", false),
    _param_flight_altitude(this, "ATOB_FLIGHT_ALT", false),
    _param_need_to_takeoff(this, "TAKEOFF_SWITCH", false),
    _point_item_a{},
    _point_item_b{},
    _point_item_current{},
    _point_item_next{},
    _need_takeoff(true),
    _point_updated(false),
	_inited(false),
	_home_inited(false),
    _need_point_reset(false),
	_work_item_type(WORK_ITEM_TYPE_DEFAULT)
{
	/* load initial params */
	updateParams();
}

PointAToB::~PointAToB()
{
}

void
PointAToB::on_inactive()
{
	/* Without home a mission can't be valid yet anyway, let's wait. */
	if (!_navigator->home_position_valid()) {
		return;
    }

    /* Reset first point to not arried */
    _starting_point_reached = false;

    if (_inited) {

		/* reset the current offboard mission if needed */
        if (need_to_reset_point(false)) {
            reset_point_item();
            update_point_item();
		}

	} else {

        /* On init let's check the point, maybe there is already one available. */
        if (check_point_valid(false)) {
            update_point_item();
            _inited = true;
        }
	}

	/* require takeoff after non-loiter or landing */
	if (!_navigator->get_can_loiter_at_sp() || _navigator->get_land_detected()->landed) {
		_need_takeoff = true;

		/* Reset work item type to default if auto take-off has been paused or aborted,
		   and we landed in manual mode. */
		if (_work_item_type == WORK_ITEM_TYPE_TAKEOFF) {
			_work_item_type = WORK_ITEM_TYPE_DEFAULT;
		}
	}
}

void
PointAToB::on_activation()
{
    set_point_items();
}

void
PointAToB::on_active()
{
    check_point_valid(false);

	/* reset the current offboard mission if needed */
    if (need_to_reset_point(true)) {
        reset_point_item();
        update_point_item();
        _point_updated = true;
	}

	/* reset mission items if needed */
    if (_point_updated) {
        _point_updated = false;
        set_point_items();
	}

	/* lets check if we reached the current mission item */
    if (is_point_item_reached(_point_item)) {
        /* switch to next waypoint */
        advance_point();
        set_point_items();
    }
}

bool
PointAToB::is_point_item_reached(struct pointatob_item_s point_item)
{
    hrt_abstime now = hrt_absolute_time();

    if ((_navigator->get_land_detected()->landed == false)
        && !_waypoint_position_reached) {

        float dist = -1.0f;
        float dist_xy = -1.0f;
        float dist_z = -1.0f;

        float altitude_amsl = point_item.altitude_is_relative
                    ? point_item.altitude + _navigator->get_home_position()->alt
                    : point_item.altitude;

        dist = get_distance_to_point_global_wgs84(point_item.lat, point_item.lon, altitude_amsl,
                    _navigator->get_global_position()->lat,
                    _navigator->get_global_position()->lon,
                    _navigator->get_global_position()->alt,
                    &dist_xy, &dist_z);

        /* for normal mission items used their acceptance radius */
        float mission_acceptance_radius = _navigator->get_acceptance_radius(point_item.acceptance_radius);

        /* if set to zero use the default instead */
        if (mission_acceptance_radius < NAV_EPSILON_POSITION) {
            mission_acceptance_radius = _navigator->get_acceptance_radius();
        }

        if (dist >= 0.0f && dist <= mission_acceptance_radius
            && dist_z <= _navigator->get_altitude_acceptance_radius()) {
            _waypoint_position_reached = true;
        }


        if (_waypoint_position_reached) {
            // reached just now
            _time_wp_reached = now;
        }
    }

    /* Check if the waypoint and the requested yaw setpoint. */

    if (_waypoint_position_reached && !_waypoint_yaw_reached) {

        if ((_navigator->get_vstatus()->is_rotary_wing
            || point_item.force_heading)
            && PX4_ISFINITE(point_item.yaw)) {

            /* check yaw if defined only for rotary wing except takeoff */
            float yaw_err = _wrap_pi(point_item.yaw - _navigator->get_global_position()->yaw);

            /* accept yaw if reached or if timeout is set in which case we ignore not forced headings */
            if (fabsf(yaw_err) < math::radians(_param_yaw_err.get())
                || (_param_yaw_timeout.get() >= FLT_EPSILON && !point_item.force_heading)) {

                _waypoint_yaw_reached = true;
            }

            /* if heading needs to be reached, the timeout is enabled and we don't make it, abort mission */
            if (!_waypoint_yaw_reached && point_item.force_heading &&
                (_param_yaw_timeout.get() >= FLT_EPSILON) &&
                (now - _time_wp_reached >= (hrt_abstime)_param_yaw_timeout.get() * 1e6f)) {

                _navigator->set_mission_failure("unable to reach heading within timeout");
            }

        } else {
            _waypoint_yaw_reached = true;
        }
    }

    /* Once the waypoint and yaw setpoint have been reached we can start the loiter time countdown */
    if (_waypoint_position_reached && _waypoint_yaw_reached) {

        if (_time_first_inside_orbit == 0) {
            _time_first_inside_orbit = now;
        }

        if (!_starting_point_reached) {
            _starting_point_reached = true;
            start_pump();
        }

        /* check if the MAV was long enough inside the waypoint orbit */
        if (now - _time_first_inside_orbit >= (hrt_abstime)point_item.time_inside * 1e6f) {
            return true;
        }
    }

    // all acceptance criteria must be met in the same iteration
    _waypoint_position_reached = false;
    _waypoint_yaw_reached = false;
    return false;
}

void
PointAToB::advance_point()
{
    /* do not advance mission item if we're processing sub mission work items */
    if (_work_item_type != WORK_ITEM_TYPE_DEFAULT) {
        return;
    }

    /* set next point to current */
    _point_item_current = _point_item_next;

    /* generate next point */
    _point_item_next.current_seq = _point_item_current.current_seq ^ 1;

    /* next point on the same side with current point */
    if (1 == _point_item_current.current_seq) {
        _point_item_next.distance_multiple = _point_item_current.distance_multiple + 1;

        /* next point on the offside with current point */
    } else {
        _point_item_next.distance_multiple = _point_item_current.distance_multiple;
    }

    struct pointatob_item_s point_item_tmp;

    /* estimate whether the current on the same side with point A or B */
    if (_point_item_current.distance_multiple % 2 == _point_item_current.current_seq) {
        /* next point on the same side with point A */
        point_item_tmp = _point_item_a;

        /* next point on the same side with point B */
    } else {
        point_item_tmp = _point_item_b;
    }

    waypoint_from_heading_and_distance(point_item_tmp.lat, point_item_tmp.lon, point_item_tmp.turn_bearing, \
                                       (float)(_point_item_next.distance_multiple * _param_interval_distance.get()), &_point_item_next.lat, &_point_item_next.lon);

    /* update to datamanger */
    save_current_state();
}

float
PointAToB::get_absolute_altitude_for_item(struct pointatob_item_s &point_item)
{
    if (point_item.altitude_is_relative) {
        return point_item.altitude + _navigator->get_home_position()->alt;

	} else {
        return point_item.altitude;
	}
}

void
PointAToB::set_point_items()
{
	/* make sure param is up to date */
    updateParams();

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	work_item_type new_work_item_type = WORK_ITEM_TYPE_DEFAULT;

    /* get point item */
    _point_item = _point_item_current;

    /*********************************** handle mission item *********************************************/

    /* we have a new position item so set previous position setpoint to current */
    set_previous_pos_setpoint();

    /* do takeoff before going to setpoint if needed and not already in takeoff */
    if (do_need_takeoff() && _work_item_type != WORK_ITEM_TYPE_TAKEOFF) {

        new_work_item_type = WORK_ITEM_TYPE_TAKEOFF;

        float takeoff_alt = calculate_takeoff_altitude(&_point_item_current);

        mavlink_log_info(_navigator->get_mavlink_log_pub(), "takeoff to %.1f meters above home",
                 (double)(takeoff_alt - _navigator->get_home_position()->alt));

        _point_item.lat = _navigator->get_global_position()->lat;
        _point_item.lon = _navigator->get_global_position()->lon;
        _point_item.altitude = takeoff_alt;
        _point_item.altitude_is_relative = false;
        /* ignore yaw for takeoff items */
        _point_item.yaw = NAN;
        _point_item.time_inside = 0;
    }

	/*********************************** set setpoints and check next *********************************************/

    /* set current position setpoint from point item (is protected against non-position items) */
    point_item_to_position_setpoint(&_point_item, &pos_sp_triplet->current);

	/* set current work item type */
	_work_item_type = new_work_item_type;

	/* require takeoff after landing or idle */
	if (pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LAND
	    || pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {

		_need_takeoff = true;
	}

    _navigator->set_can_loiter_at_sp(false);

    if (_point_item_current.time_inside <= 0.001f) {
        /* try to process next mission item */
        if (_work_item_type == WORK_ITEM_TYPE_DEFAULT) {
            point_item_to_position_setpoint(&_point_item_next, &pos_sp_triplet->next);

        } else {
            point_item_to_position_setpoint(&_point_item_current, &pos_sp_triplet->next);
        }

	} else {
		/* vehicle will be paused on current waypoint, don't set next item */
		pos_sp_triplet->next.valid = false;
    }

	_navigator->set_position_setpoint_triplet_updated();
}

bool
PointAToB::do_need_takeoff()
{
	if (_navigator->get_vstatus()->is_rotary_wing) {

        float takeoff_alt = calculate_takeoff_altitude(&_point_item_current);

		/* force takeoff if landed (additional protection) */
		if (_navigator->get_land_detected()->landed) {
			_need_takeoff = true;

			/* if in-air and already above takeoff height, don't do takeoff */

		} else if (_navigator->get_global_position()->alt > takeoff_alt) {
			_need_takeoff = false;
		}

		/* check if current mission item is one that requires takeoff before */
        if (_need_takeoff && (1 == _param_need_to_takeoff.get())) {

			_need_takeoff = false;
			return true;
		}
	}

	return false;
}

float
PointAToB::calculate_takeoff_altitude(struct pointatob_item_s *point_item)
{
	/* calculate takeoff altitude */
    float takeoff_alt = get_absolute_altitude_for_item(*point_item);

	/* takeoff to at least MIS_TAKEOFF_ALT above home/ground, even if first waypoint is lower */
	if (_navigator->get_land_detected()->landed) {
		takeoff_alt = fmaxf(takeoff_alt, _navigator->get_global_position()->alt + _param_takeoff_alt.get());

	} else {
		takeoff_alt = fmaxf(takeoff_alt, _navigator->get_home_position()->alt + _param_takeoff_alt.get());
	}

	return takeoff_alt;
}

void
PointAToB::save_current_state()
{
    const size_t len = sizeof(struct pointatob_item_s);

    if (dm_write(DM_KEY_POINTATOB, DM_KEY_POINT_CURRENT, DM_PERSIST_POWER_ON_RESET, &_point_item_current, len) != len) {
        mavlink_log_critical(_navigator->get_mavlink_log_pub(), "write current point error.");
    }

    if (dm_write(DM_KEY_POINTATOB, DM_KEY_POINT_NEXT, DM_PERSIST_POWER_ON_RESET, &_point_item_next, len) != len) {
        mavlink_log_critical(_navigator->get_mavlink_log_pub(), "write next point error.");
    }
}

bool
PointAToB::check_point_valid(bool force)
{
    if ((!_home_inited && _navigator->home_position_valid()) || force) {

        /* Check if all all waypoints are above the home altitude, only return false if bool throw_error = true */
        for (size_t i = 0; i < DM_POINT_COUNT; i++) {
            struct pointatob_item_s point_item_tmp;
            const size_t len = sizeof(struct pointatob_item_s);

            if (dm_read(DM_KEY_POINTATOB, i, &point_item_tmp, len) != len) {
                /* not supposed to happen unless the datamanager can't access the SD card, etc. */
                return false;
            }

            /* calculate the global waypoint altitude */
            float wp_alt = (point_item_tmp.altitude_is_relative) ? point_item_tmp.altitude + _navigator->get_home_position()->alt : point_item_tmp.altitude;

            if (_navigator->get_home_position()->alt > wp_alt) {
                mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Rejecting point A to B: Waypoint %d below home", i+1);
                return false;
            }
        }

        _home_inited = _navigator->home_position_valid();

        return true;

        /* check simple, paticular reference function _missionFeasibilityChecker.checkMissionFeasible */
	}

    return false;
}

void PointAToB::update_point_item()
{
    const size_t len = sizeof(struct pointatob_item_s);

    if (dm_read(DM_KEY_POINTATOB, DM_KEY_POINT_A, &_point_item_a, len) == len) {
        if (dm_read(DM_KEY_POINTATOB, DM_KEY_POINT_B, &_point_item_b, len) == len) {
            if (dm_read(DM_KEY_POINTATOB, DM_KEY_POINT_CURRENT, &_point_item_current, len) == len) {
                if (dm_read(DM_KEY_POINTATOB, DM_KEY_POINT_NEXT, &_point_item_next, len) == len) {

                    /* set altitude as param from ground control if param larger than 1 meters */
                    if (_param_flight_altitude.get() > 1) {
                        _point_item_current.altitude = _point_item_next.altitude = _param_flight_altitude.get();
                        _point_item_next.altitude_is_relative = _point_item_current.altitude_is_relative = true;
                    }

                    _point_item_current.acceptance_radius = _point_item_next.acceptance_radius = _navigator->get_acceptance_radius();
                    _point_item_current.force_heading = _point_item_next.force_heading = false;

                    return;
                    /* not supposed to happen unless datamannger can't access the SD card */
                } else {
                    mavlink_and_console_log_critical(_navigator->get_mavlink_log_pub(), "next point read error.");
                }

            } else {
               mavlink_and_console_log_critical(_navigator->get_mavlink_log_pub(),"current point read error.");
            }

        } else {
            mavlink_and_console_log_critical(_navigator->get_mavlink_log_pub(), "point B read error.");
        }

    } else {
        mavlink_and_console_log_critical(_navigator->get_mavlink_log_pub(), "point A read error.");
    }
}

void
PointAToB::reset_point_item()
{
    const size_t len = sizeof(struct pointatob_item_s);
    struct pointatob_item_s point_item_a, point_item_b;

    if (dm_read(DM_KEY_POINTATOB, DM_KEY_POINT_A, &point_item_a, len) == len) {
        if (dm_read(DM_KEY_POINTATOB, DM_KEY_POINT_B, &point_item_b, len) == len) {
            if (dm_write(DM_KEY_POINTATOB, DM_KEY_POINT_CURRENT, DM_PERSIST_POWER_ON_RESET, &point_item_a, len) == len) {
                if (dm_write(DM_KEY_POINTATOB, DM_KEY_POINT_NEXT, DM_PERSIST_POWER_ON_RESET, &point_item_b, len) == len) {
                    return;
                    /* not supposed to happen unless datamannger can't access the SD card */
                } else {
                    mavlink_and_console_log_critical(_navigator->get_mavlink_log_pub(), "next point reset failed.");
                }

            } else {
               mavlink_and_console_log_critical(_navigator->get_mavlink_log_pub(),"current point reset failed.");
            }

        } else {
            mavlink_and_console_log_critical(_navigator->get_mavlink_log_pub(), "point B read error when reset.");
        }

    } else {
        mavlink_and_console_log_critical(_navigator->get_mavlink_log_pub(), "point A read error when reset.");
    }
}

bool
PointAToB::need_to_reset_point(bool active)
{
    /* always saved in datamanger */

    return false;
}

void
PointAToB::point_item_to_position_setpoint(const struct pointatob_item_s *item, struct position_setpoint_s *sp)
{
    sp->lat = item->lat;
    sp->lon = item->lon;
    sp->alt = item->altitude_is_relative ? item->altitude + _navigator->get_home_position()->alt : item->altitude;
    sp->yaw = item->yaw;
    sp->loiter_radius = _navigator->get_loiter_radius();
    sp->loiter_direction = 1;
    sp->pitch_min = item->pitch_min;
    sp->acceptance_radius = item->acceptance_radius;
    sp->disable_mc_yaw_control = false;
    sp->cruising_speed = _navigator->get_cruising_speed();
    sp->cruising_throttle = _navigator->get_cruising_throttle();
    sp->type = position_setpoint_s::SETPOINT_TYPE_POSITION;

    sp->valid = true;
}
