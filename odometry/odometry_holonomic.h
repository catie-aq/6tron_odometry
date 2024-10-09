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

    ~OdometryHolonomic() override;

    void setPos(position pos) override;

protected:
    /**
     * @brief Compute the odometry from 3 (or more) encoders values.
     * @param encN The matrix of N encoders
     */
    void compute(Matrix *encN);

    void calculInit();

    void updateRotation(float newAngle);

private:
    float _distance_to_center; // Distance between the contact point of the wheel driving the
                               // encoder and the center of the robot
    int _number_of_wheels;

    float _offset_angle;
    Matrix _previous_odometry;
    Matrix _delta_vector;
    Matrix _rotation;
    Matrix _reverse;
    Matrix _global_position;
};
}

#endif // CATIE_SIXTRON_ODOMETRY_HOLONOMIC_H
