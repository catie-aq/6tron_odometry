/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "odometry/odometry_holonomic.h"

namespace sixtron {
OdometryHolonomic::OdometryHolonomic(
        float rate_hz, int number_of_wheels, float distance_to_center, float offset):
        _number_of_wheels(number_of_wheels),
        _offset_angle(offset)

{
    // Rate will be used for v_lin, v_tan and v_ang
    _odom_rate_hz = rate_hz;
    _distance_to_center = distance_to_center;
}

OdometryHolonomic::~OdometryHolonomic() = default;

// todo: should be done in init() ?
void OdometryHolonomic::calculInit() {

    // creates the temporary direct matrix (Vx Vy Omega -> Vmotors) for inversion
    Matrix directMatrix = newMatrix(_number_of_wheels, 3);

    for (int line = 0; line < directMatrix.line; line++) {
        directMatrix.data[line][0] = -sinf(float(line * 2 * M_PI / _number_of_wheels));
        directMatrix.data[line][1] = cosf(float(line * 2 * M_PI / _number_of_wheels));
        directMatrix.data[line][2] = _distance_to_center;
    }

    // The following takes the command matrix (VxVyOmega -> Vmotors) and inverses it to calculate
    // the odometry of the mobile base (Xmotors -> XYTheta)
    _reverse = multiply(
            inverse(multiply(transpose(directMatrix), directMatrix)), transpose(directMatrix));

    // Rotation matrix initialized
    _rotation = newMatrix(3, 3);

    //... Except the last
    _rotation.data[_rotation.line - 1][_rotation.column - 1] = 1.0f;

    // Memory vector initialized
    _previous_odometry = newMatrix(3, 1);

    // Delta vector initialized
    _delta_vector = newMatrix(3, 1);

    // Global position vector initialized
    _global_position = newMatrix(3, 1);
}

void OdometryHolonomic::compute(Matrix *vectorEncoders) {

    // this matrix multiplication gives a 3x1 vectors containing X,Y and theta in the base's
    // referential
    Matrix XYThetaFromNEncoders = multiply(_reverse, *vectorEncoders);

    // delta vector updated
    for (int i = 0; i < _delta_vector.line; i++) {
        _delta_vector.data[i][0] = XYThetaFromNEncoders.data[i][0] - _previous_odometry.data[i][0];
    }

    // rotation matrix updated
    updateRotation(XYThetaFromNEncoders.data[2][0] - _odometry_offset.theta);

    // position in the global referential
    _global_position = multiply(_rotation, _delta_vector);

    // updating the position of the center of the base in the global referential
    _odometry_position.x += _global_position.data[0][0];
    _odometry_position.y += _global_position.data[1][0];
    _odometry_position.theta += _global_position.data[2][0];

    // stores the local position for the next call
    _previous_odometry = XYThetaFromNEncoders;
}

void OdometryHolonomic::updateRotation(float newAngle) // updates the rotation matrix
{
    _rotation.data[0][0] = cosf(newAngle + _offset_angle); // pour xg mult x
    _rotation.data[0][1] = -sinf(newAngle + _offset_angle); // pour xg mult y
    _rotation.data[1][0] = sinf(newAngle + _offset_angle); // pour yg mult x
    _rotation.data[1][1] = cosf(newAngle + _offset_angle); // pour yg mult y
}

}
