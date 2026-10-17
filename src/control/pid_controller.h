#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController
{
public:
    PIDController(float kp = 0.0f,
                  float ki = 0.0f,
                  float kd = 0.0f,
                  float integLimit = 0.0f,
                  float outputLimit = 0.0f,
                  float dFilterAlpha = 1.0f);

    void setGains(float kp, float ki, float kd);
    void setLimits(float integLimit, float outputLimit);
    void setDFilterAlpha(float alpha);
    void getGains(float &kp, float &ki, float &kd) const;

    void reset();

    float update(float target, float measurement, float dt, bool isYaw = false, bool derivativeOnMeasurement = true);

private:
    float Kp;
    float Ki;
    float Kd;

    float integrator;
    float prevError;
    float prevMeasurement;
    float dFiltered;

    float integLimit;
    float outputLimit;
    float dFilterAlpha;

    bool firstRun;

    float dt_max = 0.1f;

    void lpf_step(float &current, float target, float alpha);
    float wrap180_deg(float angle);
};

#endif
