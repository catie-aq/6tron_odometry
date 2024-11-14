/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_ODOMETRY_DIFFERENTIAL_H
#define CATIE_SIXTRON_ODOMETRY_DIFFERENTIAL_H

#include "odometry/odometry.h"

namespace sixtron {

class OdometryDifferential: public Odometry {

public:
    OdometryDifferential(float rate_hz, // in Hertz
            float motor_resolution, // in ticks
            float motor_wheel_radius, // in meters
            float enc_wheels_distance); // in meters

    ~OdometryDifferential() override;

    void setPos(position pos) override;

protected:
    /**
     * @brief Compute the odometry from 2 encoders values (set up in differential).
     * @param encL Left encoder counter
     * @param encR Right encoder counter
     */
    void compute(int64_t encL, int64_t encR);

    inline float ticks2Meters(float ticks) const {
        return (ticks * (1.0f / _tick_per_meters));
    }

    inline float meters2Ticks(float meters) const {
        return (meters * _tick_per_meters);
    }

    inline float ticks2Rads(float ticks) const {
        return ((ticks * M_PI_F) / (_ticks_per_robot_revolution / 2.0f));
    }

private:
    float _motor_resolution, _motor_wheel_radius;
    float _motor_wheels_distance; // If external encoders: distance between the two encoder wheels.
    float _wheel_perimeter, _tick_per_meters;
    float _meters_per_robot_revolution, _ticks_per_robot_revolution;

    float _tick_theta, _d_theta;
    float _tick_x, _tick_y;
    float _robot_distance;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_ODOMETRY_DIFFERENTIAL_H
