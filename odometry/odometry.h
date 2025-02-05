/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_ODOMETRY_H
#define CATIE_SIXTRON_ODOMETRY_H

#include "math_utils/math_utils.h"
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

    void setPos(const float x, const float y, const float theta) {
        position pos;
        pos.x = x;
        pos.y = y;
        pos.theta = theta;
        setPos(pos);
    }

    void setPos(const position &pos) {
        _odometry_offset.x = _odometry_position.x - pos.x;
        _odometry_offset.y = _odometry_position.y - pos.y;
        _odometry_offset.theta = _odometry_position.theta - pos.theta;
    }

    float getX() const {
        return _odometry_position.x - _odometry_offset.x;
    }

    float getY() const {
        return _odometry_position.y - _odometry_offset.y;
    }

    float getTheta() const {
        return _odometry_position.theta - _odometry_offset.theta;
    }

    position getPos() const {
        position pos;
        pos.x = getX();
        pos.y = getY();
        pos.theta = getTheta();
        return pos;
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
    position _odometry_position, _odometry_offset;
    position _odometry_speeds; // todo: refactor, should be lin/tan/rot
};
} // namespace sixtron

#endif // CATIE_SIXTRON_ODOMETRY_H
