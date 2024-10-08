/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
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

    virtual float getSpeedTan() = 0;

    virtual float getSpeedAng() = 0;
};
} // namespace sixtron

#endif // CATIE_SIXTRON_ODOMETRY_H
