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
  rover simulator class
*/

#include "SIM_BalanceBot.h"

#include <string.h>
#include <stdio.h>

namespace SITL {

BalanceBot::BalanceBot(const char *home_str, const char *frame_str) :
    SimRover(home_str, frame_str)
{
    dcm.from_euler(0,radians(0),0);
}

/*
  update the rover simulation by one time step
 */
void BalanceBot::update(const struct sitl_input &input)
{
//    const float delta_time = frame_time_us * 1.0e-6f;
//
//    Matrix3f body_to_ned;
//    _states.attitude.rotation_matrix(body_to_ned);
//    Vector3f euler312 = body_to_ned.to_euler312();
//    float yaw = euler312.z;
//    float pitch = euler312.y;
//    const Matrix3f ned_to_horizon_frame = Matrix3f(Vector3f(cos(yaw), sin(yaw), 0),
//                                                   Vector3f(-sin(yaw), cos(yaw), 0),
//                                                   Vector3f(0,0,1));
//
//    const float motor_R = 10.0f;
//    const float motor_Km = 0.308f;
//    const float batt_voltage = 3.7f*4.0f;
//
//    const float stalk_len = 1.8f;
//    Vector3f pos_cart_bf = Vector3f(0,0,0); // m
//    Vector3f pos_head_bf = Vector3f(0,0,-stalk_len); // m
//    Vector3f pos_stalk_bf = Vector3f(0,0,-stalk_len*0.5f); // m
//    Vector3f pos_Lwheel = Vector3f(0,-0.15f, 0); // m
//    Vector3f pos_Rwheel = Vector3f(0, 0.15f, 0); // m
//    const float mass_head = .6f; // kg
//    const float mass_stalk = .27f; // kg
//    const float mass_cart = 1.6f; // kg
//    const float mass_wheel = .3f; // kg
//    const float total_mass = mass_head+mass_stalk+mass_cart+mass_wheel*2.0f;
//    //::printf("%f\n", total_mass);
//    const Vector3f pos_cg = (pos_head_bf * mass_head + pos_stalk_bf * mass_stalk + pos_cart_bf * mass_cart + pos_Lwheel * mass_wheel + pos_Rwheel * mass_wheel) / total_mass; // m
//    pos_cart_bf -= pos_cg;
//    pos_head_bf -= pos_cg;
//    pos_stalk_bf -= pos_cg;
//    pos_Lwheel -= pos_cg;
//    pos_Rwheel -= pos_cg;
//
//    const float radius_wheel = .095f; // m
//
//    const float floor_pos_down = pos_Lwheel.z+radius_wheel;
//
//    const float Im_wheel_y = 0.5f*mass_wheel*radius_wheel*radius_wheel; // kg m^2
//
//    const float Im_xy = mass_cart*sq(pos_cart_bf.z) + mass_head * sq(pos_head_bf.z) + mass_stalk*sq(stalk_len)/12.0f;
//    const float Im_z = (mass_cart+mass_wheel*2.0f)*sq(0.3f)/12.0f;
//
//    const float friction_coefficient = 1.0f;
//
//    const float wheel_damping_ratio = 0.2f;
//    const float wheel_freq = 10.0f; // Hz
//    const float wheel_stiffness = 0.5f*total_mass*sq(wheel_freq*2.0f*M_PI); // N/m
//    const float wheel_damping = 2*wheel_damping_ratio*sqrt(wheel_stiffness*0.5f*total_mass); // N/m/s

    //float motorL_tdem = -torque_out;
    //float motorR_tdem = -torque_out;

    const float length = 1.0f; //m length of pendulum rod
    const float wheel_radius = .095f; //m

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

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    // speed along x axis, +ve is forward
    float speed = velocity_body.x;

    // yaw rate in degrees/s
    float yaw_rate = calc_yaw_rate(steering, speed);

    //input force to the cart
    float force_on_body = throttle * max_force; //N

    // temp variables to hold updated theta and angular rate
    float new_ang_vel = ang_vel;
    float new_theta = theta;

    float accel = (force_on_body - (damping_constant*x_speed) - mass_rod*length*ang_vel*ang_vel*sin(theta)
    + (3.0f/4.0f)*mass_rod*GRAVITY_MSS*sin(theta)*cos(theta))
            / (mass_cart + mass_rod - (3.0f/4.0f)*mass_rod*cos(theta)*cos(theta));

    angular_accel = mass_rod*length*(GRAVITY_MSS*sin(theta) + accel*cos(theta))
            /(I_rod + mass_rod*length*length);

    x_speed+= accel*delta_time;
    new_ang_vel += angular_accel * delta_time;
    new_theta += new_ang_vel * delta_time;

    // training wheels
//    if (training_wheels && !(fabsf(new_theta)<=radians(10))){
//        ang_vel = 0;
//    }
//    else {
        ang_vel= new_ang_vel;
        theta = fmod(new_theta,radians(360));
//    }

    gyro = Vector3f(0,(ang_vel),radians(yaw_rate));

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

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    position += (velocity_ef * delta_time);

    ::printf("acc:%f speed:%f theta: %d\ e_speed: %f ang_vel %d\n",accel, x_speed,(int)degrees(theta),velocity_ef.x,(int)degrees(ang_vel));

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
    

//    SimRover::update(input);

    //float motorL_tdem = input.hydra0_torque/(2000.0f/0.065f);
    //float motorR_tdem = input.hydra1_torque/(2000.0f/0.065f);

//    float motorL_tdem = motor1*0.001;
//    float motorR_tdem = motor2*0.001;
//
//    //::printf("%d ", (int16_t)input.hydra0_torque);
//
//    //::printf("%f %f\n",motorL_tdem, dbg1);
//
//    float motorL_angvel = _states.Lwheel_ang_vel_y-_states.ang_vel.y;
//    float motorR_angvel = _states.Rwheel_ang_vel_y-_states.ang_vel.y;
//
//    //::printf("%f %f\n",motorL_tdem, motorR_tdem);
//
//    float motorL_Vemf = motorL_angvel*motor_Km;
//    float motorR_Vemf = motorR_angvel*motor_Km;
//
//    float motorL_Vdem = motor_R*motorL_tdem/motor_Km;
//    float motorR_Vdem = motor_R*motorR_tdem/motor_Km;
//
//    float motorL_Vin = constrain_float(motorL_Vdem-motorL_Vemf, -batt_voltage, batt_voltage);
//    float motorR_Vin = constrain_float(motorR_Vdem-motorR_Vemf, -batt_voltage, batt_voltage);
//
//    dbg1 = (ned_to_horizon_frame*_states.vel_ned).x;
//
//    float motorL_torque = motor_Km*(motorL_Vin+motorL_Vemf)/motor_R;
//    float motorR_torque = motor_Km*(motorR_Vin+motorR_Vemf)/motor_R;
//
//    ::printf("%f %f\n",motorL_torque, motorR_torque);
//
//    Vector3f contact_Rwheel_bf = Vector3f(radius_wheel * -sin(pitch), 0, radius_wheel*cos(pitch)) + pos_Rwheel;
//    float Rwheel_compression = (body_to_ned*contact_Rwheel_bf + _states.pos_ned).z-floor_pos_down;
//
//    Vector3f contact_Lwheel_bf = Vector3f(radius_wheel * -sin(pitch), 0, radius_wheel*cos(pitch)) + pos_Lwheel;
//    float Lwheel_compression = (body_to_ned*contact_Lwheel_bf + _states.pos_ned).z-floor_pos_down;
//
//    Vector3f Rwheel_force_bf = Vector3f(0,0,0);
//    Vector3f Lwheel_force_bf = Vector3f(0,0,0);
//
//    float Rwheel_total_torque = motorR_torque;
//    float Lwheel_total_torque = motorL_torque;
//
//    if (Rwheel_compression > 0) {
//        Vector3f Rwheel_force_hf = Vector3f(0,0,0);
//        Rwheel_force_hf.z = -wheel_stiffness * Rwheel_compression;
//        Vector3f Rwheel_contact_velocity = body_to_ned * (_states.ang_vel % contact_Rwheel_bf) + _states.vel_ned;
//        Rwheel_force_hf.z -= Rwheel_contact_velocity.z * wheel_damping;
//
//        float horizontal_force_limit = fabs(Rwheel_force_hf.z * friction_coefficient);
//
//        float longitudinal_velocity = (ned_to_horizon_frame * Rwheel_contact_velocity).x;
//        float wheel_velocity = _states.Rwheel_ang_vel_y * radius_wheel;
//        Rwheel_force_hf.x = constrain_float(-(wheel_velocity+longitudinal_velocity) * 200.0f, -horizontal_force_limit, horizontal_force_limit);
//
//        Rwheel_total_torque += Rwheel_force_hf.x * radius_wheel;
//
//        float lateral_velocity = (ned_to_horizon_frame * Rwheel_contact_velocity).y;
//        Rwheel_force_hf.y = constrain_float(-lateral_velocity * 200.0f, -horizontal_force_limit, horizontal_force_limit);
//
//        Rwheel_force_bf = body_to_ned.transposed() * ned_to_horizon_frame.transposed() * Rwheel_force_hf;
//    }
//
//    if (Lwheel_compression > 0) {
//        Vector3f Lwheel_force_hf = Vector3f(0,0,0);
//        Lwheel_force_hf.z = -wheel_stiffness * Lwheel_compression;
//        Vector3f Lwheel_contact_velocity = body_to_ned * (_states.ang_vel % (contact_Lwheel_bf)) + _states.vel_ned;
//        Lwheel_force_hf.z -= Lwheel_contact_velocity.z * wheel_damping;
//
//        float horizontal_force_limit = fabs(Lwheel_force_hf.z * friction_coefficient);
//
//        float longitudinal_velocity = (ned_to_horizon_frame * Lwheel_contact_velocity).x;
//        float wheel_velocity = _states.Lwheel_ang_vel_y * radius_wheel;
//        Lwheel_force_hf.x = constrain_float(-(wheel_velocity+longitudinal_velocity) * 200.0f, -horizontal_force_limit, horizontal_force_limit);
//
//        Lwheel_total_torque += Lwheel_force_hf.x * radius_wheel;
//
//        float lateral_velocity = (ned_to_horizon_frame * Lwheel_contact_velocity).y;
//        Lwheel_force_hf.y = constrain_float(-lateral_velocity * 200.0f, -horizontal_force_limit, horizontal_force_limit);
//
//        Lwheel_force_bf = body_to_ned.transposed() * ned_to_horizon_frame.transposed() * Lwheel_force_hf;
//    }
//
//    Vector3f gravity_ned = Vector3f(0,0,9.8065);
//    Vector3f gravity_bf = body_to_ned.mul_transpose(gravity_ned);
//
//    Vector3f total_force_bf = Lwheel_force_bf + Rwheel_force_bf + gravity_bf*total_mass;
//    Vector3f total_moment_bf = contact_Lwheel_bf % Lwheel_force_bf + contact_Rwheel_bf % Rwheel_force_bf;
//    total_moment_bf.y -= Lwheel_total_torque;
//    total_moment_bf.y -= Rwheel_total_torque;
//
//    Vector3f coordinate_accel_bf = total_force_bf / total_mass;
//
//    Vector3f angular_acceleration_bf;
//    angular_acceleration_bf.x = total_moment_bf.x / Im_xy;
//    angular_acceleration_bf.y = total_moment_bf.y / Im_xy;
//    angular_acceleration_bf.z = total_moment_bf.z / Im_z;
//
//    // update states
//    _states.Lwheel_ang_vel_y += Lwheel_total_torque / Im_wheel_y * dt;
//    _states.Rwheel_ang_vel_y += Rwheel_total_torque / Im_wheel_y * dt;
//    _states.Lwheel_ang_pos_y = wrap_2PI(_states.Lwheel_ang_pos_y + motorL_angvel * dt);
//    _states.Rwheel_ang_pos_y = wrap_2PI(_states.Rwheel_ang_pos_y + motorR_angvel * dt);
//    _states.proper_accel = coordinate_accel_bf - gravity_bf;
//    _states.ang_vel += angular_acceleration_bf * dt;
//    _states.vel_ned += (body_to_ned * coordinate_accel_bf) * dt;
//    _states.pos_ned += _states.vel_ned * dt;
//    _states.attitude.rotate(_states.ang_vel * dt);
//    _states.attitude.normalize();
//
//    // training wheels ...
//    float r, p, y;
//    const float training_limit = radians(10);
//    _states.attitude.to_euler(r, p, y);
//    if (r >= training_limit) {
//        r = training_limit;
//        _states.ang_vel.x = MIN(_states.ang_vel.x, 0);
//    }
//    if (r <= -training_limit) {
//        r = -training_limit;
//        _states.ang_vel.x = MAX(_states.ang_vel.x, 0);
//    }
//    if (p >= training_limit) {
//        p = training_limit;
//        _states.ang_vel.y = MIN(_states.ang_vel.y, 0);
//    }
//    if (p <= -training_limit) {
//        p = -training_limit;
//        _states.ang_vel.y = MAX(_states.ang_vel.y, 0);
//    }
//    _states.attitude.from_euler(r, p, y);
//
//    //::printf("tru % .2f ", _states.Lwheel_ang_vel_y);
//
//    //::printf("wdot % .6f % .6f % .6f w % .6f % .6f % .6f v % .6f % .6f % .6f\n", angular_acceleration_bf.x,angular_acceleration_bf.y,angular_acceleration_bf.z, _states.ang_vel.x,_states.ang_vel.y,_states.ang_vel.z);
//
//    // update output
//    position = _states.pos_ned + body_to_ned*pos_cart_bf;
//    velocity_ef = _states.vel_ned + body_to_ned * (_states.ang_vel % pos_cart_bf);
//    accel_body = _states.proper_accel + angular_acceleration_bf % pos_cart_bf + _states.ang_vel % (_states.ang_vel % pos_cart_bf);
//
//    accel_body.x = constrain_float(accel_body.x, -16*GRAVITY_MSS, 16*GRAVITY_MSS);
//    accel_body.y = constrain_float(accel_body.y, -16*GRAVITY_MSS, 16*GRAVITY_MSS);
//    accel_body.z = constrain_float(accel_body.z, -16*GRAVITY_MSS, 16*GRAVITY_MSS);
//
//    gyro = _states.ang_vel;
//
//    gyro.x = constrain_float(gyro.x, -radians(2000), radians(2000));
//    gyro.y = constrain_float(gyro.y, -radians(2000), radians(2000));
//    gyro.z = constrain_float(gyro.z, -radians(2000), radians(2000));
//    _states.ang_vel = gyro;
//
//    _states.attitude.rotation_matrix(dcm);
//
//    float predicted_Lwheel_ang_vel = (-(ned_to_horizon_frame*velocity_ef).x + (body_to_ned*gyro).z*(.295/2.0f))/radius_wheel;
//    //dbg2 = _states.Lwheel_ang_vel_y;
//    //::printf("% .2f % .2f\n", predicted_Lwheel_ang_vel, _states.Lwheel_ang_vel_y);
//
//    // update lat/lon/altitude
//    update_position();
}

} // namespace SITL
