/* 
	Simulator connector for Airsim: https://github.com/Microsoft/AirSim
*/

#pragma once

#include <AP_HAL/utility/Socket.h>
#include "SIM_Aircraft.h"

namespace SITL {

/* 
	Airsim Simulator
*/

class AirSim : public Aircraft {
public:
	AirSim(const char *home_str, const char *frame_str);

	/* update model by one time step */
	void update(const struct sitl_input &input) override;

	/* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new AirSim(home_str, frame_str);
    }

private:
#ifdef __linux__
	struct __attribute__((__packed__)) fdm_packet {
#else
#pragma pack(push,1)
	struct fdm_packet {
#endif
		// this is the packet sent by the simulator
		// to the APM executable to update the simulator state
		// All values are little-endian
		uint64_t timestamp;
		double latitude, longitude; // degrees
		double altitude;  // MSL
		double heading;   // degrees
		double speedN, speedE, speedD; // m/s
		double xAccel, yAccel, zAccel;       // m/s/s in body frame
		double rollRate, pitchRate, yawRate; // degrees/s/s in earth frame
		double rollDeg, pitchDeg, yawDeg;    // euler angles, degrees
		double airspeed; // m/s
		uint32_t magic; // 0x4c56414f
	};
#ifndef __linux__
#pragma pack(pop)
#endif

	/*
		packet sent to Ardupilot
		Taken from Airsim ArduCopter Solo connector
	*/
	// No idea why it's 11, HIL_ACTUATOR_CONTROLS message takes 16 servo values
	// This can possibly be changed in Airsim
	// Actually, AirSim is only taking in 8 PWM values, need to look into this
	// It's supports both HIL_CONTROLS & HIL_ACTUATOR_CONTROLS message but uses only 8 values
	static const int kArduCopterRotorControlCount = 11;

	struct servo_packet {
		// ArduPilot Solo rotor control datagram format
		uint16_t pwm[kArduCopterRotorControlCount];
		uint16_t speed, direction, turbulance;
	};

	// default connection_info_.ip_address
	const char *airsim_ip = "127.0.0.1";

	// default connection_info_.ip_port
	uint16_t airsim_sensor_port = 14560;

	// default connection_info_.sitl_ip_port
	uint16_t airsim_control_port = 14556;

	SocketAPM sock;
	bool opened_socket;
	double last_timestamp;

    fdm_packet last_pkt;

	bool bind_socket(void);
	void send_servos(const struct sitl_input &input);
	void recv_fdm(const struct sitl_input &input);
};

}
