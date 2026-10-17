#include "control/pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd, float integLimit, float outputLimit, float dFilterAlpha)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;

    this->integLimit = fabsf(integLimit);
    this->outputLimit = fabsf(outputLimit);
    this->dFilterAlpha = constrain(dFilterAlpha, 0.0f, 1.0f);

    reset();
}

void PIDController::setGains(float kp, float ki, float kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void PIDController::setLimits(float integLimit, float outputLimit)
{
    this->integLimit = fabsf(integLimit);
    this->outputLimit = fabsf(outputLimit);
}

void PIDController::setDFilterAlpha(float alpha)
{
    dFilterAlpha = constrain(alpha, 0.0f, 1.0f);
}

void PIDController::getGains(float &kp, float &ki, float &kd) const
{
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

void PIDController::lpf_step(float &current, float target, float alpha)
{
    current = current + (alpha * (target - current));
}

void PIDController::reset()
{
    integrator = 0.0f;
    prevError = 0.0f;
    prevMeasurement = 0.0f;
    dFiltered = 0.0f;
    firstRun = true;
}

float PIDController::update(float target, float measurement, float dt, bool is_yaw, bool derivativeOnMeasurement)
{
    if ((dt <= 0.0f) || (dt > dt_max))
    {
        reset();
        return 0.0f;
    }

    float error = target - measurement;

    if (is_yaw == true)
    {
        error = wrap180_deg(error);
    }

    float derivative = 0.0f;

    if (firstRun == true)
    {
        prevError = error;
        prevMeasurement = measurement;
        dFiltered = 0.0f;
        firstRun = false;
    }
    else
    {
        if (derivativeOnMeasurement == true)
        {
            float measurementDelta = measurement - prevMeasurement;

            if (is_yaw == true)
            {
                measurementDelta = wrap180_deg(measurementDelta);
            }

            derivative = -measurementDelta / dt;
        }
        else
        {
            derivative = (error - prevError) / dt;
        }

        lpf_step(dFiltered, derivative, dFilterAlpha);
    }

    integrator += error * dt;

    if (integLimit > 0.0f)
    {
        integrator = constrain(integrator, -integLimit, integLimit);
    }

    float output = Kp * error + Ki * integrator + Kd * dFiltered;

    if (outputLimit > 0.0f)
    {
        output = constrain(output, -outputLimit, outputLimit);
    }

    prevError = error;
    prevMeasurement = measurement;

    return output;
}

float PIDController::wrap180_deg(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}
