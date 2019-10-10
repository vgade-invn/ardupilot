#include <AP_HAL/AP_HAL.h>
#include "AC_Circle.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Scheduler/AP_Scheduler.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_Circle::var_info[] = {
    // @Param: RADIUS
    // @DisplayName: Circle Radius
    // @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
    // @Units: cm
    // @Range: 0 10000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("RADIUS",  0,  AC_Circle, _radius, AC_CIRCLE_RADIUS_DEFAULT),

    // @Param: RATE
    // @DisplayName: Circle rate
    // @Description: Circle mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
    // @Units: deg/s
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE",    1, AC_Circle, _rate,    AC_CIRCLE_RATE_DEFAULT),

    // @Param: PATH_TYPE
    // @DisplayName: Type of path to follow
    // @Description: Selects what type of path to follow
    // @Values: 0:Circle
    // @User: Standard
    AP_GROUPINFO("PATH_TYPE",  2, AC_Circle, _path_type, int(PathType::CIRCLE)),
    
    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_Circle::AC_Circle(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.panorama = false;
}

/*
  get path position rotated by a yaw
 */
Vector2f AC_Circle::PathFunction::get_relpos_rotated(float t, float yaw_rad) const
{
    Vector2f v = get_relpos(t) * radius;
    Vector2f v2;
    const float cos_yaw = cosf(yaw_rad);
    const float sin_yaw = sinf(yaw_rad);
    v2.x = v.x * cos_yaw - v.y * sin_yaw;
    v2.y = v.x * sin_yaw + v.y * cos_yaw;
    return v2;
}

// equation for a circle
Vector2f AC_Circle::PathCircle::get_relpos(float t) const
{
    t *= M_PI * 2;
    return Vector2f(-cosf(t), -sinf(t));
}

// yaw for a circle
float AC_Circle::PathCircle::get_yaw(float t) const
{
    t *= M_PI * 2;
    return wrap_PI(t);
}

// equation for a figure 8
Vector2f AC_Circle::PathFigureEight::get_relpos(float t) const
{
    t *= M_PI * 2;
    return Vector2f(sinf(t), sinf(t)*cosf(t));
}

// equation for a Lissajour curve (ABC logo)
Vector2f AC_Circle::PathLissajous::get_relpos(float t) const
{
    // with thanks to https://www.grant-trebbin.com/2017/10/abc-logo-lissajous-curve.html
    t *= M_PI * 2;
    return -Vector2f(cosf(t), cosf(3*t + M_PI*0.5));
}

/// init - initialise circle controller setting center specifically
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
bool AC_Circle::init(const Vector3f& center)
{
    _center = center;

    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.set_desired_accel_xy(0.0f,0.0f);
    _pos_control.set_desired_velocity_xy(0.0f,0.0f);
    _pos_control.init_xy_controller();

    // set initial position target to reasonable stopping point
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();

    if (!is_positive(_radius)) {
        return false;
    }

    if (path_function) {
        delete path_function;
        path_function = nullptr;
    }

    _initial_yaw = _ahrs.yaw;

    switch ((PathType)_path_type.get()) {
    case PathType::CIRCLE:
        path_function = new PathCircle(_radius.get(), _initial_yaw);
        break;

    case PathType::FIGURE_EIGHT:
        path_function = new PathFigureEight(_radius.get(), _initial_yaw);
        break;

    case PathType::FIGURE_LISSAJOUS:
        path_function = new PathLissajous(_radius.get(), _initial_yaw);
        break;
        
    default:
        return false;
    }

    _time_s = 0;
    _rate_now = 0;
    _last_vel.zero();
    _first_pos = path_function->get_relpos_rotated(0, _initial_yaw);
    for (uint8_t i=0; i<ARRAY_SIZE(_last_pos); i++) {
        _last_pos[i] = _first_pos;
    }
    _loop_time = AP::scheduler().get_filtered_loop_time();

    // calculate velocities
    calc_velocities(true);

    // set start angle from position
    init_start_angle(false);

    return true;
}

/// init - initialise circle controller setting center using stopping point and projecting out based on the copter's heading
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
bool AC_Circle::init()
{
    // set initial position target to reasonable stopping point
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();

    // get stopping point
    _center = _pos_control.get_pos_target();

    return init(_center);
}

/// set_circle_rate - set circle rate in degrees per second
void AC_Circle::set_rate(float deg_per_sec)
{
    if (!is_equal(deg_per_sec, _rate.get())) {
        _rate = deg_per_sec;
        calc_velocities(false);
    }
}

/// update - update circle controller
void AC_Circle::update()
{
    // calculate dt
    float dt = _loop_time;

    if (_rate > _rate_now + 0.5 || _rate < _rate_now - 0.5) {
        _rate_now += (_rate - _rate_now) * 0.0005;
    }

    const float rate_scale = _rate_now/360.0;
    float dt_scaled = dt * rate_scale;

    _time_s += dt_scaled;

    // we use a multi-element delay line, to allow the velocity and
    // acceleration to be computed ahead of the current point in time,
    // which allows for them to be smoothed
    Vector2f pos1 = _last_pos[0];
    const uint8_t alen = ARRAY_SIZE(_last_pos);
    for (uint8_t i=0; i<alen-1; i++) {
        _last_pos[i] = _last_pos[i+1];
    }
    _last_pos[alen-1] = path_function->get_relpos_rotated(_time_s + dt_scaled*alen, _initial_yaw);
    Vector2f relpos = (pos1 - _first_pos);
    Vector2f dpos = _last_pos[alen-1] - _last_pos[alen-2];

    // calculate target position
    Vector3f pos = _center;
    pos.x += relpos.x;
    pos.y += relpos.y;
    pos.z = _pos_control.get_alt_target();

    Vector2f vel = dpos / dt;

    // smooth velocity so that we don't introduce as much
    // accel noise with the numerical derivative
    vel = _last_vel * 0.8 + vel * 0.2;

    Vector2f accel = (vel - _last_vel) / dt;

    // smooth acceleration
    accel = _last_accel * 0.8 + accel * 0.2;

    // update position controller target
    _pos_control.set_xy_target(pos.x, pos.y);
    _pos_control.set_desired_velocity_xy(vel.x, vel.y);
    _pos_control.set_desired_accel_xy(accel.x, accel.y);

    AP::logger().Write("CIRC", "TimeUS,PX,PY,VX,VY,AX,AY,dt", "Qfffffff",
                       AP_HAL::micros64(),
                       pos.x, pos.y,
                       vel.x, vel.y,
                       accel.x, accel.y,
                       dt);

    const float yaw_lag = _attitude_control.get_yaw_time_constant() * rate_scale * 0.3;
    _yaw = degrees(path_function->get_yaw(_time_s + yaw_lag)) * 100;

    // update position controller
    _pos_control.update_xy_controller();

    _last_vel = vel;
    _last_accel = accel;
}

// get_closest_point_on_circle - returns closest point on the circle
//  circle's center should already have been set
//  closest point on the circle will be placed in result
//  result's altitude (i.e. z) will be set to the circle_center's altitude
//  if vehicle is at the center of the circle, the edge directly behind vehicle will be returned
void AC_Circle::get_closest_point_on_circle(Vector3f &result)
{
    // return center if radius is zero
    if (_radius <= 0) {
        result = _center;
        return;
    }

    // get current position
    Vector3f stopping_point;
    _pos_control.get_stopping_point_xy(stopping_point);

    // calc vector from stopping point to circle center
    Vector2f vec;   // vector from circle center to current location
    vec.x = (stopping_point.x - _center.x);
    vec.y = (stopping_point.y - _center.y);
    float dist = norm(vec.x, vec.y);

    // if current location is exactly at the center of the circle return edge directly behind vehicle
    if (is_zero(dist)) {
        result.x = _center.x - _radius * _ahrs.cos_yaw();
        result.y = _center.y - _radius * _ahrs.sin_yaw();
        result.z = _center.z;
        return;
    }

    // calculate closest point on edge of circle
    result.x = _center.x + vec.x / dist * _radius;
    result.y = _center.y + vec.y / dist * _radius;
    result.z = _center.z;
}

// calc_velocities - calculate angular velocity max and acceleration based on radius and rate
//      this should be called whenever the radius or rate are changed
//      initialises the yaw and current position around the circle
void AC_Circle::calc_velocities(bool init_velocity)
{
    // if we are doing a panorama set the circle_angle to the current heading

    // circle mode angular velocity
    _angular_vel_max = radians(_rate);
    // limit to max slew rate angular velocity
    const float yaw_slew_limit = _attitude_control.get_slew_yaw_rads();
    if (yaw_slew_limit < _angular_vel_max) {
        _angular_vel_max = yaw_slew_limit;
        gcs().send_text(MAV_SEVERITY_INFO, "Circle: limited yaw rate to %.1fdps", degrees(yaw_slew_limit));
    }

    // maximum acceleration set to leave 25% for corrections
    _accel_max = 0.75 * _pos_control.get_max_accel_xy();

    // maximum velocity to achieve angular velocity
    float _velocity_max = _angular_vel_max * _radius;

    // limit velocity by maximum allowable velocity
    const float max_speed_xy = _pos_control.get_max_speed_xy();
    if (max_speed_xy < _velocity_max) {
        _velocity_max = max_speed_xy;
        gcs().send_text(MAV_SEVERITY_INFO, "Circle: xy speed to %.1fcm/s", max_speed_xy);
    }

    // limit velocity by maximum allowable radial acceleration
    const float vel_limit = safe_sqrt(_accel_max*_radius);
    if (vel_limit < _velocity_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Circle: vel limited to %.1fcm/s", vel_limit);
        _velocity_max = vel_limit;
    }

    // maximum jerk based on 50% roll or pitch angular acceleration
    const float accel_roll_limit = _attitude_control.get_accel_roll_max_radss();
    const float accel_pitch_limit = _attitude_control.get_accel_pitch_max_radss();
    const float accel_yaw_limit = _attitude_control.get_accel_yaw_max_radss();

    _jerk_max = 0.5f * MIN(accel_roll_limit, accel_pitch_limit) * GRAVITY_MSS * 100.0f; // cm/s/s

    // limit maximum acceleration by maximum radial jerk
    const float accel_limit = _jerk_max * _radius / _velocity_max;
    if (accel_limit < _accel_max) {
        _accel_max = accel_limit;
        gcs().send_text(MAV_SEVERITY_INFO, "Circle: accel limited to %.1fm/s/s", _accel_max);
    }

    // limit jerk to ensure maximum acceleration is not exceeded
    _jerk_max = MIN(_jerk_max, sq(_accel_max)/_velocity_max);

    // limit jerk by maximum yaw angular acceleration
    _jerk_max = MIN(_jerk_max, 0.25f * accel_yaw_limit * _radius);

    gcs().send_text(MAV_SEVERITY_INFO, "Circle: r=%.1fm rate=%.1fdeg/s vel=%.1fm/s accel=%.1fm/s/s",
                    _radius.get(), degrees(_angular_vel_max), _velocity_max*0.01, _accel_max*0.01);


    // initialise angular velocity
    if (init_velocity) {
        _angular_vel = 0.0f;
    }
}

// init_start_angle - sets the starting angle around the circle and initialises the angle_total
//  if use_heading is true the vehicle's heading will be used to init the angle causing minimum yaw movement
//  if use_heading is false the vehicle's position from the center will be used to initialise the angle
void AC_Circle::init_start_angle(bool use_heading)
{
    // initialise angle total
    _angle_total = 0;

    // if the radius is zero we are doing panorama so init angle to the current heading
    if (_radius <= 0) {
        _angle = _ahrs.yaw;
        return;
    }

    // if use_heading is true
    if (use_heading) {
        _angle = wrap_PI(_ahrs.yaw-M_PI);
    } else {
        // if we are exactly at the center of the circle, init angle to directly behind vehicle (so vehicle will backup but not change heading)
        const Vector3f &curr_pos = _inav.get_position();
        if (is_equal(curr_pos.x,_center.x) && is_equal(curr_pos.y,_center.y)) {
            _angle = wrap_PI(_ahrs.yaw-M_PI);
        } else {
            // get bearing from circle center to vehicle in radians
            float bearing_rad = atan2f(curr_pos.y-_center.y,curr_pos.x-_center.x);
            _angle = wrap_PI(bearing_rad);
        }
    }
}
