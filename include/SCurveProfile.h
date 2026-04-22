#pragma once

class SCurveProfile {
public:
    struct State {
        float position;  // inches from start (signed)
        float velocity;  // inches/sec (signed)
    };

    // distance  - signed inches to travel
    // maxVel    - peak cruise speed (in/s)
    // maxAccel  - peak acceleration (in/s^2)
    // maxJerk   - peak jerk (in/s^3)  — the main knob for wheel slip
    SCurveProfile(float distance, float maxVel, float maxAccel, float maxJerk);

    State getState(float t) const;
    float totalTime() const { return tTotal; }

private:
    float dist, jMax;
    float T1, T2, T4;   // jerk-phase, constant-accel-phase, cruise-phase durations
    float vCruise;
    float tTotal;
    int   sign;
};
