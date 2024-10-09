/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "odometry/odometry_differential.h"

namespace sixtron {

OdometryDifferential::OdometryDifferential(
        float rate_hz, float motor_resolution, float motor_wheel_radius, float enc_wheels_distance):
        _tick_theta(0.0f), _d_theta(0.0f), _tick_x(0.0f), _tick_y(0.0f), _robot_distance(0.0f) {

    // Rate will be used for v_lin and v_ang
    _odom_rate_hz = rate_hz;

    // Save basic encoders datas
    _motor_resolution = motor_resolution;
    _motor_wheel_radius = motor_wheel_radius;
    _motor_wheels_distance = enc_wheels_distance;

    // Basic calculs
    _wheel_perimeter = (2.0f * float(M_PI) * _motor_wheel_radius);
    _tick_per_meters = ((1.0f / (_wheel_perimeter)) * _motor_resolution);
    _meters_per_robot_revolution = (float(M_PI) * _motor_wheels_distance);
    _ticks_per_robot_revolution = meters2Ticks(_meters_per_robot_revolution);
}

OdometryDifferential::~OdometryDifferential() = default;

void OdometryDifferential::compute(int64_t encL, int64_t encR) {

    // compute curvilinear distance
    float new_distance = float(encL + encR) / 2.0f;
    float delta_distance = new_distance - _robot_distance;

    // compute new angle value
    float new_angle = float(encR - encL) / 2.0f;

    _d_theta = new_angle - _tick_theta;

    // compute X/Y coordinates
    float mid_angle = ticks2Rads(_tick_theta + (_d_theta / 2.0f));
    float d_x = delta_distance * cosf(mid_angle);
    float d_y = delta_distance * sinf(mid_angle);
    _tick_x += d_x;
    _tick_y += d_y;

    // Update values in meters
    _odometry_position.x = ticks2Meters(_tick_x);
    _odometry_position.y = ticks2Meters(_tick_y);

    // update global values
    _tick_theta += _d_theta;
    _odometry_position.theta = ticks2Rads(_tick_theta);

    _robot_distance += delta_distance;
    _odometry_speeds.x = ticks2Meters(delta_distance) * float(_odom_rate_hz);
    _odometry_speeds.theta = ticks2Rads(_d_theta) * float(_odom_rate_hz);
}

void OdometryDifferential::setPos(position pos) {
    // ! refactor: See issue #1
    // _x = pos.x;
    // _y = pos.y;
    // _theta = pos.theta;
}

}
