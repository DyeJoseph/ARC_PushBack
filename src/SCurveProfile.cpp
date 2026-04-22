#include "SCurveProfile.h"
#include <cmath>
#include <algorithm>

// Returns the distance covered during the acceleration half (phases 1–3)
// for a given cruise velocity, without modifying any object state.
static float halfDist(float v, float aMax, float jMax) {
    float T1, T2;
    if (v * jMax <= aMax * aMax) {
        T1 = std::sqrt(v / jMax);
        T2 = 0.0f;
    } else {
        T1 = aMax / jMax;
        T2 = v / aMax - T1;
    }
    float pos = 0.0f, vel = 0.0f, acc = 0.0f;
    auto step = [&](float dur, float j) {
        pos += vel*dur + 0.5f*acc*dur*dur + (1.0f/6.0f)*j*dur*dur*dur;
        vel += acc*dur + 0.5f*j*dur*dur;
        acc += j*dur;
    };
    step(T1,  jMax);
    step(T2,  0.0f);
    step(T1, -jMax);
    return pos;
}

SCurveProfile::SCurveProfile(float distance, float maxVel, float maxAccel, float maxJerk) {
    sign = (distance >= 0.0f) ? 1 : -1;
    dist = std::fabs(distance);
    jMax = maxJerk;

    // Binary-search for the highest cruise velocity achievable within this distance.
    // (When distance is short, the robot can't reach maxVel before it must decelerate.)
    if (halfDist(maxVel, maxAccel, maxJerk) * 2.0f <= dist) {
        vCruise = maxVel;
    } else {
        float lo = 0.0f, hi = maxVel;
        for (int i = 0; i < 32; i++) {
            float mid = 0.5f * (lo + hi);
            (halfDist(mid, maxAccel, maxJerk) * 2.0f <= dist ? lo : hi) = mid;
        }
        vCruise = lo;
    }

    // Phase durations
    if (vCruise * jMax <= maxAccel * maxAccel) {
        T1 = std::sqrt(vCruise / jMax);
        T2 = 0.0f;
    } else {
        T1 = maxAccel / jMax;
        T2 = vCruise / maxAccel - T1;
    }

    float accelDist = halfDist(vCruise, maxAccel, jMax);
    T4 = std::max(0.0f, (dist - 2.0f * accelDist) / vCruise);
    tTotal = 4.0f * T1 + 2.0f * T2 + T4;
}

SCurveProfile::State SCurveProfile::getState(float t) const {
    if (t <= 0.0f) return {0.0f, 0.0f};
    if (t >= tTotal) return {(float)sign * dist, 0.0f};

    float pos = 0.0f, vel = 0.0f, acc = 0.0f;

    // Each call to advance() simulates one phase and consumes 't'.
    auto advance = [&](float duration, float j) {
        if (t <= 0.0f) return;
        float dt = (t < duration) ? t : duration;
        pos += vel*dt + 0.5f*acc*dt*dt + (1.0f/6.0f)*j*dt*dt*dt;
        vel += acc*dt + 0.5f*j*dt*dt;
        acc += j*dt;
        t -= dt;
    };

    advance(T1,  jMax);   // 1: accel ramps up
    advance(T2,  0.0f);   // 2: constant accel
    advance(T1, -jMax);   // 3: accel ramps down → reach cruise
    advance(T4,  0.0f);   // 4: cruise
    advance(T1, -jMax);   // 5: decel ramps up
    advance(T2,  0.0f);   // 6: constant decel
    advance(T1,  jMax);   // 7: decel ramps down → stop

    return {(float)sign * pos, (float)sign * vel};
}
