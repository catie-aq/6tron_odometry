/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_ODOMETRY_TWO_ENCODERS_H
#define CATIE_SIXTRON_ODOMETRY_TWO_ENCODERS_H

#include "odometry/odometry.h"

namespace sixtron {

class OdometryTwoEncoders: public Odometry {

public:
    OdometryTwoEncoders(float rate_hz, // in Hertz
            float motor_resolution, // in ticks
            float motor_wheel_radius, // in meters
            float enc_wheels_distance); // in meters

    ~OdometryTwoEncoders() override;

    void setPos(position pos) override;

protected:
    /**
     * @brief Compute the odometry from 2 encoders values (set up in differential).
     * @param encL Left encoder counter
     * @param encR Right encoder counter
     */
    void compute(int64_t encL, int64_t encR);

    inline float ticks2Meters(float ticks) const {
        return (ticks * (1.0f / _tickPerMeters));
    }

    inline float meters2Ticks(float meters) const {
        return (meters * _tickPerMeters);
    }

    inline float ticks2Rads(float ticks) const {
        return ((ticks * float(M_PI)) / (_ticksPerRobotRevolution / 2.0f));
    }

private:
    float _motorResolution, _motorWheelRadius;
    float _motorWheelsDistance; // If external encoders: distance between the two encoder wheels. If
                                // not, distance between the two motor wheels.
    float _wheelPerimeter, _tickPerMeters;
    float _metersPerRobotRevolution, _ticksPerRobotRevolution;

    float _odomRateHz;

    float _tickTheta, _dTheta;
    float _tickX, _tickY;
    float _robot_distance;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_ODOMETRY_TWO_ENCODERS_H
