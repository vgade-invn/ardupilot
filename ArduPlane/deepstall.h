#ifndef DEEPSTALL_H
#define DEEPSTALL_H

#include "pidcontroller.h"
#include <stdint.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

enum STAGE {
	FLY_TO_ARC = 0,
	COURSE_INTERCEPT,
	DEEPSTALL_ENTRY,
	DEEPSTALL_LAND
};

class DeepStall
{
	public:
		DeepStall();
		void setTarget(float lat, float lon);
		void setYRCParams(float _Kyr, float _yrLimit, float Kp, float Ki, float Kd, float ilim);
		void land(float track, float yawrate, float lat, float lon, float deepstal_l1);
		bool getApproachWaypoint(Location &target, Location &land_loc, Location &current, Vector3f _wind, float v_d, float deltah, float vspeed);
		float getRudderNorm();
		
		void computeApproachPath(Vector3f wind, float loiterRadius, float d_s, float v_d, float deltah, float vspeed, float lat, float lon);
		
		void abort();
		
		void setTargetHeading(float hdg);
		PIDController *YawRateController;
		
		float targetTrack;
		int stage; // 1 - arc point, 2 - course intercept, 3 - deep stall entry point, 4 - stall
		bool ready;
                int32_t deepstall_start_time;
	
	private:
		float land_lat;
		float land_lon;
		float rCmd;
		float targetHeading;
		float lradius;
		float ds;
		float Kyr;
		float yrLimit;
		uint32_t _last_t;
		
		// Approach parameters
		float d_predict;
		// Deepstall entry point
		float lat_e;
		float lon_e;
		// Course intercept point
		float lat_i;
		float lon_i;
		// Pre-final loiter
		float lat_l;
		float lon_l;
		// Projected entry point
		float lat_p;
		float lon_p;
		
                Location loc_entry;
                Location _land_loc;
};

#endif
