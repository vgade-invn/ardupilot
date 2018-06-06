/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  BalanceBot simulator class
*/

#include "SIM_BalanceBot.h"

#include <string.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

BalanceBot::BalanceBot(const char *home_str, const char *frame_str) :
    SimRover(home_str, frame_str)
{
    dcm.from_euler(0,radians(0),0);
}

/*
  update the BalanceBot simulation by one time step
 */
void BalanceBot::update(const struct sitl_input &input)
{
    const float length = 1.0f; //m length of pendulum rod

    const float mass_cart = 1.0f; // kg
    const float mass_rod = 1.0f; //kg

    const float max_force = 5.0f; //N

    const float I_rod = (mass_rod*4*length*length)/12.0f;

    const float damping_constant = 0.7;

    float steering,throttle;

    float motor1 = 2*((input.servos[0]-1000)/1000.0f - 0.5f);
    float motor2 = 2*((input.servos[2]-1000)/1000.0f - 0.5f);
    steering = motor1 - motor2;
    throttle = 0.5*(motor1 + motor2);

    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    // speed along x axis, +ve is forward
    float speed = velocity_ef.x;

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    // yaw rate in degrees/s
    float yaw_rate = calc_yaw_rate(steering, velocity_body.x);

    //input force to the cart
    float force_on_body = throttle * max_force; //N

    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    float theta = p;

    float ang_vel = gyro.y;
    
    // temp variables to hold updated theta and angular rate
    float new_ang_vel = ang_vel;
    float new_theta = theta;

    float accel = (force_on_body - (damping_constant*speed) - mass_rod*length*ang_vel*ang_vel*sin(theta)
    + (3.0f/4.0f)*mass_rod*GRAVITY_MSS*sin(theta)*cos(theta))
            / (mass_cart + mass_rod - (3.0f/4.0f)*mass_rod*cos(theta)*cos(theta));

    float angular_accel = mass_rod*length*(GRAVITY_MSS*sin(theta) + accel*cos(theta))
        /(I_rod + mass_rod*length*length);

    new_ang_vel += angular_accel * delta_time;
    new_theta += new_ang_vel * delta_time;

    ang_vel= new_ang_vel;
    theta = fmod(new_theta,radians(360));

    gyro = Vector3f(0,ang_vel,radians(yaw_rate));

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // accel in body frame due to motor
    accel_body = Vector3f(0, 0, 0);

    // add in accel due to direction change
    accel_body.y += radians(yaw_rate) * speed;

    // now in earth frame
    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(accel, 0, GRAVITY_MSS);

    // we are on the ground, so our vertical accel is zero
    accel_earth.z = 0;

    if (!hal.util->get_soft_armed()) {
        // reset to vertical when not armed for faster testing
        accel_earth.zero();
        velocity_ef.zero();
        dcm.identity();
        gyro.zero();
        theta = 0;
        ang_vel = 0;
        angular_accel = 0;
    }
    
    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    position += (velocity_ef * delta_time);

//    ::printf("acc:%f speed:%f theta: %d\ ang_vel %d\n",throttle, velocity_ef.x,(int)degrees(theta),(int)degrees(ang_vel));

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

}// namespace SITL
