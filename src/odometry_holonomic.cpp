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
    // ? this should not exist anymore
    // set current position (at the time the function is called) as the origin of the global
    // referential
    // _x_global = 0.0f;
    // _y_global = 0.0f;
    // _theta_global = 0.0f;

    // creates the temporary direct matrix (Vx Vy Omega -> Vmotors) for inversion
    Matrix directMatrix;
    directMatrix.Matrice
            = std::vector<std::vector<float>>(_number_of_wheels, std::vector<float>(3));
    directMatrix.line = _number_of_wheels;
    directMatrix.column = 3;
    for (int line = 0; line < directMatrix.line; line++) {
        directMatrix.Matrice[line][0] = -sinf(float(line * 2 * M_PI / _number_of_wheels));
        directMatrix.Matrice[line][1] = cosf(float(line * 2 * M_PI / _number_of_wheels));
        directMatrix.Matrice[line][2] = _distance_to_center;
    }

    // The following takes the command matrix (VxVyOmega -> Vmotors) and inverses it to calculate
    // the odometry of the mobile base (Xmotors -> XYTheta)
    _reverse = multiply(
            inverse(multiply(transpose(directMatrix), directMatrix)), transpose(directMatrix));

    // todo : matrix initialisation methods should be done in math utils ...
    // Rotation matrix initialized
    _rotation.line = 3;
    _rotation.column = 3;
    _rotation.Matrice = std::vector<std::vector<float>>(3, std::vector<float>(3));

    // every component of the rotation matrix is initialized at 0
    for (int i = 0; i < _rotation.line; i++) {
        for (int j = 0; j < _rotation.column; j++) {
            _rotation.Matrice[i][j] = 0.0f;
        }
    }
    //... Except the last
    _rotation.Matrice[_rotation.line - 1][_rotation.column - 1] = 1.0f;

    // Memory vector initialized
    _previous_odometry.line = 3;
    _previous_odometry.column = 1;
    _previous_odometry.Matrice = std::vector<std::vector<float>>(3, std::vector<float>(1));

    // Delta vector initialized
    _delta_vector.line = 3;
    _delta_vector.column = 1;
    _delta_vector.Matrice = std::vector<std::vector<float>>(3, std::vector<float>(1));

    // Global position vector initialized
    _global_position.line = 3;
    _global_position.column = 1;
    _global_position.Matrice = std::vector<std::vector<float>>(3, std::vector<float>(1));
    for (int i = 0; i < _previous_odometry.line; i++) {
        _previous_odometry.Matrice[i][0] = 0;
        _global_position.Matrice[i][0] = 0;
    }
}

void OdometryHolonomic::compute(Matrix *vectorEncoders) {

    // this matrix multiplication gives a 3x1 vectors containing X,Y and theta in the base's
    // referential
    Matrix XYThetaFromNEncoders = multiply(_reverse, *vectorEncoders);

    // delta vector updated
    for (int i = 0; i < _delta_vector.line; i++) {
        _delta_vector.Matrice[i][0]
                = XYThetaFromNEncoders.Matrice[i][0] - _previous_odometry.Matrice[i][0];
    }

    // rotation matrix updated
    updateRotation(XYThetaFromNEncoders.Matrice[2][0]);

    // position in the global referential
    _global_position = multiply(_rotation, _delta_vector);

    // updating the position of the center of the base in the global referential
    _odometry_position.x += _global_position.Matrice[0][0];
    _odometry_position.y += _global_position.Matrice[1][0];
    _odometry_position.theta += _global_position.Matrice[2][0];

    // stores the local position for the next call
    _previous_odometry = XYThetaFromNEncoders;
}

void OdometryHolonomic::setPos(position pos) {
    // ! refactor: See issue #1
    // _x_global = pos.x;
    // _y_global = pos.y;
    // _theta_global = pos.theta;
}

void OdometryHolonomic::updateRotation(float newAngle) // updates the rotation matrix
{
    _rotation.Matrice[0][0] = cosf(newAngle + _offset_angle); // pour xg mult x
    _rotation.Matrice[0][1] = -sinf(newAngle + _offset_angle); // pour xg mult y
    _rotation.Matrice[1][0] = sinf(newAngle + _offset_angle); // pour yg mult x
    _rotation.Matrice[1][1] = cosf(newAngle + _offset_angle); // pour yg mult y
}

}
