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
    virtual ~Odometry() = default;

    virtual void init() = 0;

    virtual void update() = 0; // Update Odometry. Implementation depending on the type of odometry.

    virtual void setPos(position pos)
            = 0; // Bypass odometry by setting a new pos, useful when re-calibrate.

    float getX() const {
        return _odometry_position.x;
    }

    float getY() const {
        return _odometry_position.y;
    }

    float getTheta() const {
        return _odometry_position.theta;
    }

    position getPos() const {
        return _odometry_position;
    }

    float getSpeedLin() const {
        return _odometry_speeds.x;
    }

    float getSpeedTan() const {
        return _odometry_speeds.y;
    }

    float getSpeedAng() const {
        return _odometry_speeds.theta;
    }

protected:
    float _odom_rate_hz = 0.0f;
    position _odometry_position;
    position _odometry_speeds; // todo: refactor, should be lin/tan/rot
};
} // namespace sixtron

#endif // CATIE_SIXTRON_ODOMETRY_H
