/* 
	Simulator Connector for AirSim
*/

#include "SIM_AirSim.h"

#include <stdio.h>
#include <arpa/inet.h>
#include <errno.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

AirSim::AirSim(const char *home_str, const char *frame_str) :
	Aircraft(home_str, frame_str),
	sock(true),
	last_timestamp(0),
	opened_socket(false)
{
	printf("Starting SITL Airsim\n");

    const char *colon = strchr(frame_str, ':');
    if (colon) {
        airsim_ip = colon+1;
    }
    
	AP_Param::set_default_by_name("AHRS_EKF_TYPE", 10);
    AP_Param::set_default_by_name("INS_GYR_CAL", 0);
}

/*
	Bind in/out socket
*/
bool AirSim::bind_socket(void)
{
	if (opened_socket) {
		return true;
	}

    if (!sock.bind("0.0.0.0", airsim_sensor_port)) {
		printf("Unable to bind Airsim sensor in socket at port %u - Error: %s\n",
				 airsim_sensor_port, strerror(errno));
		return false;
	}
	printf("Bind SITL sensor input at %s:%u\n", "127.0.0.1", airsim_sensor_port);
	printf("AirSim control interface set to %s:%u\n", airsim_ip, airsim_control_port);
	sock.set_blocking(false);
	sock.reuseaddress();
	opened_socket = true;

	return true;
}

/*
	Decode and send servos
*/
void AirSim::send_servos(const struct sitl_input &input)
{
	servo_packet pkt{0};

	for (uint8_t i=0; i<kArduCopterRotorControlCount; i++) {
		// No need to normalize since Airsim does that in ArduCopterSoloApi
		// Range 1000-2000
		pkt.pwm[i] = input.servos[i];
	}
	pkt.speed = input.wind.speed;
	pkt.direction = input.wind.direction;
	pkt.turbulance = input.wind.turbulence;

	ssize_t send_ret = sock.sendto(&pkt, sizeof(pkt), airsim_ip, airsim_control_port);
	if (send_ret != sizeof(pkt)) {
		if (send_ret <= 0) {
			printf("Unable to send servo output to %s:%u - Error: %s, Return value: %ld\n",
					 airsim_ip, airsim_control_port, strerror(errno), send_ret);
		} else {
			printf("Sent %ld bytes instead of %ld bytes\n", send_ret, sizeof(pkt));
		}
	}
}

/*
	Receive new sensor data from simulator
	This is a blocking function
*/
void AirSim::recv_fdm(const struct sitl_input &input)
{
	// Resend servo packet every 0.1 second till we get data
	// We do get the timestamp in the packet, sync can be achieved in a similar manner as JSBSim
    fdm_packet pkt{0};

    ssize_t recv_ret = sock.recv(&pkt, sizeof(pkt), 100);
	
	while (recv_ret != sizeof(pkt)) {
		if (recv_ret <= 0) {
			printf("Error while receiving sensor data at port %u - Error: %s, Received value: %ld\n",
					 airsim_sensor_port, strerror(errno), recv_ret);
		} else {
			printf("Received %ld bytes instead of %ld bytes\n", recv_ret, sizeof(pkt));
		}

		send_servos(input);
		recv_ret = sock.recv(&pkt, sizeof(pkt), 100);
	}
	// NOTE: Airsim sends data in little-endian

	// Taken from CRRCSim.cpp
	accel_body = Vector3f(pkt.xAccel, pkt.yAccel, pkt.zAccel);
    gyro = Vector3f(radians(pkt.rollRate), radians(pkt.pitchRate), radians(pkt.yawRate));
    velocity_ef = Vector3f(pkt.speedN, pkt.speedE, pkt.speedD);

    location.lat = pkt.latitude * 1.0e7;
    location.lng = pkt.longitude * 1.0e7;
    location.alt = pkt.altitude * 100.0f;
    // printf("Location: %d, %d, %d\n", location.lat, location.lng, location.alt);

    // position.x = pkt.latitude;
    // position.y = pkt.longitude;
    // position.z = pkt.altitude;

    airspeed = pkt.airspeed;
    airspeed_pitot = pkt.airspeed;

    dcm.from_euler(radians(pkt.rollDeg), radians(pkt.pitchDeg), radians(pkt.yawDeg));

    // Airsim is taking 3ms between each message
    if (last_pkt.timestamp) {
        time_now_us += pkt.timestamp - last_pkt.timestamp;
    }

    AP::logger().Write("ASM1", "TimeUS,TUS,R,P,Y,GX,GY,GZ",
                       "QQffffff",
                       AP_HAL::micros64(),
                       pkt.timestamp,
                       pkt.rollDeg,
                       pkt.pitchDeg,
                       pkt.yawDeg,
                       degrees(gyro.x),
                       degrees(gyro.y),
                       degrees(gyro.z));

    Vector3f velocity_bf = dcm.transposed() * velocity_ef;
    position = home.get_distance_NED(location);

    AP::logger().Write("ASM2", "TimeUS,AX,AY,AZ,VX,VY,VZ,PX,PY,PZ,Alt,SD",
                       "Qfffffffffff",
                       AP_HAL::micros64(),
                       accel_body.x,
                       accel_body.y,
                       accel_body.z,
                       velocity_bf.x,
                       velocity_bf.y,
                       velocity_bf.z,
                       position.x,
                       position.y,
                       position.z,
                       pkt.altitude,
                       pkt.speedD);

    last_pkt = pkt;
}

/*
  update the AirSim simulation by one time step
*/
void AirSim::update(const struct sitl_input &input)
{
	if (!bind_socket()) {
		return;
	}
	
	send_servos(input);
    recv_fdm(input);
    adjust_frame_time(1.0e6/3000);
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
