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
 * Creation : 01/2023
 */
#ifndef CATIE_SIXTRON_ODOMETRY_H
#define CATIE_SIXTRON_ODOMETRY_H

#include <math.h>
#include <stdint.h>

namespace sixtron {

typedef struct position position;

struct position {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
};

class Odometry {

public:
    virtual void init() = 0;

    virtual void update() = 0; // Update Odometry. Implementation depending on the type of odometry.

    virtual void setPos(position pos)
            = 0; // Bypass odometry by setting a new pos, useful when re-calibrate.

    virtual float getX() = 0;

    virtual float getY() = 0;

    virtual float getTheta() = 0;

    virtual position getPos() = 0;

    virtual float getSpeedLin() = 0;

    virtual float getSpeedAng() = 0;
};
} // namespace sixtron

#endif // CATIE_SIXTRON_ODOMETRY_H
