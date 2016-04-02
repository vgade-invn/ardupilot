#ifndef DEEPSTALL_H
#define DEEPSTALL_H

#include "pidcontroller.h"
#include <stdint.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_Param/AP_Param.h>

enum STAGE {
    DEEPSTALL_FLY_TO_LOITER = 0,
    DEEPSTALL_LOITER,
    DEEPSTALL_APPROACH,
    DEEPSTALL_LAND,
};


class DeepStall
{
	public:
		DeepStall();
		void setYRCParams(float _Kyr, float _yrLimit, float Kp, float Ki, float Kd, float ilim);
		void land(float track, float yawrate, Location current_loc, float deepstal_l1, float yawlimit);
		STAGE getApproachWaypoint(Location &target, Location &land_loc, Location &current_loc, Vector3f _wind, float v_d, float deltah, float vspeed, int32_t heading_cd, AP_Navigation *nav_controller, float loiter_radius, float heading);
		float getRudderNorm();
		
		void computeApproachPath(Vector3f wind, float loiterRadius, float d_s, float v_d, float deltah, float vspeed, Location &landing, float heading);
		
		void abort();
		
		void setTargetHeading(float hdg, bool constrain);
		PIDController *YawRateController;
		
		float targetTrack;
		STAGE stage; // 1 - arc point, 2 - course intercept, 3 - deep stall entry point, 4 - stall
		bool ready;
                int32_t deepstall_start_time;
                Location loiter;
                Location loiter_exit;

                static const struct AP_Param::GroupInfo var_info[];
                AP_Float vf_a;
                AP_Float vf_b;
                AP_Float tcon;
                AP_Float ds_a;
                AP_Float ds_b;

	private:
                float predictDistanceTraveled(Vector3f wind, float altitude, float vspeed);
                bool verify_loiter_breakout(Location &current_loc, int32_t heading_cd);

                Location extended_approach;
                Location landing_point;

                int32_t loiter_sum_cd;
                int32_t old_target_bearing_cd;

		float rCmd;
		float targetHeading;
		float ds;
		float yrLimit;
		uint32_t _last_t;
		
		// Approach parameters
		float d_predict;
};

#endif
