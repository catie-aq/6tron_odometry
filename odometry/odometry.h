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
#ifndef CATIE_SIXTRON_ODOMETRY_H_
#define CATIE_SIXTRON_ODOMETRY_H_

#include <stdint.h>
#include <math.h>

namespace sixtron {

    class Odometry {

    public:

        Odometry();

        ~Odometry();

        void init(float rate_hz,
                  float enc_resolution,
                  float enc_wheel_radius,
                  float enc_wheels_distance);

        /**
         * @brief Compute the odometry (_vRobotLin, _vRobotAng, _theta, x, y) from the values of the encoders.
         * @param encL Left encoder counter
         * @param encR Right encoder counter
         */
        void compute(int64_t encL, int64_t encR);

        float getVRobotLin() const;

        float getVRobotAng() const;

        float getTheta() const;

        float getX() const;

        float getY() const;

        inline float ticks2Meters(float ticks) const { return (ticks * (1.0f / _tickPerMeters)); }

        inline float meters2Ticks(float meters) const { return (meters * _tickPerMeters); }

        inline float ticks2Rads(float ticks) const {
            return ((ticks * float(M_PI)) / (_ticksPerRobotRevolution / 2.0f));
        }

        inline float rads2meters(float rads) const { return (rads * (_encWheelsDistance / 2.0f)); }

    private:

        float _encResolution, _encWheelRadius, _encWheelsDistance;
        float _wheelPerimeter, _tickPerMeters;
        float _metersPerRobotRevolution, _ticksPerRobotRevolution;

        float _odomRateHz;
        float _vRobotLin, _vRobotAng;
        float _theta, _tickTheta, _dTheta;
        float _x, _tickX;
        float _y, _tickY;
        float _robot_distance;
    };


} // namespace sixtron

#endif // CATIE_SIXTRON_ODOMETRY_H_
