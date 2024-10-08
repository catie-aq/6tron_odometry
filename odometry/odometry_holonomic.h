/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_ODOMETRY_HOLONOMIC_H
#define CATIE_SIXTRON_ODOMETRY_HOLONOMIC_H

#include "math_utils/math_utils.h"
#include "odometry/odometry.h"

namespace sixtron {
class OdometryHolonomic: public Odometry {

public:
    OdometryHolonomic(float rate_hz, int number_of_wheels, float distance_to_center, float offset);

    ~OdometryHolonomic();

    void setPos(position pos) override;

    position getPos() override;

    float getSpeedLin() override; // correspond to the X axis of the mobile base

    float getSpeedTan() override; // correspond to the Y axis of the mobile base

    float getSpeedAng() override;

    float getTheta() override;

    float getX() override;

    float getY() override;

protected:
    /**
     * @brief Compute the odometry (_vRobotLin, _vRobotAng, _theta, x, y) from the values of the
     * encoders.
     * @param encN The matrix of N encoders
     */
    void compute(Matrix *encN);

    void calculInit();

    void updateRotation(float newAngle);

private:
    float _distance_to_center; // Distance between the contact point of the wheel driving the
                               // encoder and the center of the robot
    int _number_of_wheels;

    float _odom_rate_hz;
    float _vRobotLin, _vRobotAng, _vRobotTan;
    float _x_global, _y_global, _theta_global;
    float _offset_angle;
    Matrix _previous_odometry;
    Matrix _delta_vector;
    Matrix _rotation;
    Matrix _reverse;
    Matrix _global_position;
};
}

#endif // CATIE_SIXTRON_ODOMETRY_HOLONOMIC_H
