/*
 * Copyright (c) 2022, CATIE, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Maintainer : ACH
 * Creation : 07/2022
 */

#include "odometry/odometry.h"

namespace sixtron {

    Odometry::Odometry() : _vRobotLin(0.0f), _vRobotAng(0.0f), _theta(0.0f), _tickTheta(0.0f),
                           _dTheta(0.0f), _x(0.0f), _tickX(0.0f), _y(0.0f), _tickY(0.0f), _robot_distance(0.0f) {

    }

    void Odometry::init(float rate_hz, float enc_resolution, float enc_wheel_radius, float enc_wheels_distance) {

        // Rate will be used for v_lin and v_ang
        _odomRateHz = rate_hz;

        // Save basic encoders datas
        _encResolution = enc_resolution;
        _encWheelRadius = enc_wheel_radius;
        _encWheelsDistance = enc_wheels_distance;

        // Basic calculs
        _wheelPerimeter = (2.0f * float(M_PI) * _encWheelRadius);
        _tickPerMeters = ((1.0f / (_wheelPerimeter)) * _encResolution);
        _metersPerRobotRevolution = (float(M_PI) * _encWheelsDistance);
        _ticksPerRobotRevolution = meters2Ticks(_metersPerRobotRevolution);
    }

    Odometry::~Odometry() = default;

    void Odometry::compute(int64_t encL, int64_t encR) {

        //compute curvilinear distance
        float newDistance = float(encL + encR) / 2.0f;
        float deltaDistance = newDistance - _robot_distance;

        //compute new angle value
        float newAngle = float(encR - encL) / 2.0f;

        _dTheta = newAngle - _tickTheta;

        //compute X/Y coordinates
        float midAngle = ticks2Rads(_tickTheta + (_dTheta / 2.0f));
        float dx = deltaDistance * cosf(midAngle);
        float dy = deltaDistance * sinf(midAngle);
        _tickX += dx;
        _tickY += dy;

        // Update values in meters
        _x = ticks2Meters(_tickX);
        _y = ticks2Meters(_tickY);

        //update global values
        _tickTheta += _dTheta;
        _theta = ticks2Rads(_tickTheta);

        _robot_distance += deltaDistance;
        _vRobotLin = ticks2Meters(deltaDistance) * float(_odomRateHz);
        _vRobotAng = ticks2Rads(_dTheta) * float(_odomRateHz);
    }

    float Odometry::getVRobotLin() const {
        return _vRobotLin;
    }

    float Odometry::getVRobotAng() const {
        return _vRobotAng;
    }

    float Odometry::getTheta() const {
        return _theta;
    }

    float Odometry::getX() const {
        return _x;
    }

    float Odometry::getY() const {
        return _y;
    }

}