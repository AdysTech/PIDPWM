//Copyright @ Adys Tech
//Author : mvadu@adystech.com

#include "PidPwm.hpp"
#include <Arduino.h>
#include <driver/gpio.h>

PidPwm::PidPwm(uint8_t pwmPin, uint8_t channel, double freq, uint8_t resolution_bits, double criticalMax, PidParam param, CurrentValueFunction onCompute)
{
    _pwmChannel = channel;
    _pwmRes = resolution_bits;
    _pwmFrequency = freq;
    _pin = (gpio_num_t)pwmPin;
    _sp = NAN;
    _curValFun = onCompute;
    _cMax = criticalMax;
    _outPwm = _minOut = 0;
    _maxOut = (((uint32_t)1) << _pwmRes) - 1;
    _running = false;
    setTuningParams(param);
}

bool PidPwm::begin()
{
    if (isnan(_sp))
        return false;
    if (!isRunning())
    {

        gpio_pad_select_gpio(_pin);
        pinMode(_pin, OUTPUT);
        ledcAttachPin(_pin, _pwmChannel);
        ledcSetup(_pwmChannel, _pwmFrequency, _pwmRes);

        _computeTimer = xTimerCreate(
            "pidComputeTimer",
            pdMS_TO_TICKS(_computeInterval),
            pdTRUE,
            (void *)this,
            computeCallback);

        //store current value
        _lastVal = _curValFun();
        xTimerStart(_computeTimer, 100);
        _running = true;
        _lastValTs = millis();
        _intgErrSum = 0;
        _outPwm = 0;
    }
    return true;
}

void PidPwm::shutdown()
{
    if (isRunning())
    {
        xTimerStop(_computeTimer, 0);
        //ensure incase next timer callback gets called its not going to flip the pin
        if (millis() > _invocationTime)
        {
            delay(_computeInterval - (millis() - _invocationTime));
        }
        else
        {
            delay(_computeInterval);
        }
        ledcWrite(_pwmChannel, _minOut);
        ledcDetachPin(_pin);
        gpio_reset_pin(_pin);
        gpio_set_level(_pin, 0);
        _running = false;
    }
}

void PidPwm::computeCallback(TimerHandle_t xTimer)
{
    yield();
    PidPwm *pid = (PidPwm *)pvTimerGetTimerID(xTimer);

    if (!pid->isRunning())
        return;

    pid->_invocationTime = millis();
    double current = pid->_curValFun();
    //make sure the current value function is fast enough. If not expand the interval with 100ms buffer.
    if (millis() > pid->_invocationTime && millis() - pid->_invocationTime > pid->_computeInterval)
    {
        pid->setComputeInterval((millis() - pid->_invocationTime) + 100);
    }
    if (!isnan(current))
    {
        double dVal = (current - pid->_lastVal);
        double error = pid->_sp - current;

        pid->_lastVal = current;
        //over time accumulated error
        pid->_intgErrSum += pid->_ki * error;
        pid->_intgErrSum -= pid->_kp * dVal;

        //restrict the error to be within the maximum swing we can have with PWM out
        if (pid->_intgErrSum > pid->_maxOut)
        {
            pid->_intgErrSum = pid->_maxOut;
        }
        else if (pid->_intgErrSum < pid->_minOut)
            pid->_intgErrSum = pid->_minOut;

        /*Compute PID Output*/
        pid->_outPwm = pid->_intgErrSum - pid->_kd * dVal;

        //store the last successful outcome of output
        if (dVal != 0)
        {
            pid->_lastValTs = pid->_invocationTime;
        }
        //restrict the output to be within the maximum swing we can have with PWM out
        if (pid->_outPwm > pid->_maxOut)
            pid->_outPwm = pid->_maxOut;
        else if (pid->_outPwm < pid->_minOut)
            pid->_outPwm = pid->_minOut;
        //detect stall or sensor failure conditions.
        //with at least 10% output, if no change is detected in current value over 10 sample periods shut down the output
        if (pid->getOutputDS() > 0.1 && pid->_invocationTime - pid->_lastValTs > 100 * pid->getSamplePeriod())
        {
            pid->_outPwm = pid->_minOut;
        }
    }
    //just to be safe, if we can't measure current value, shut off output
    else
    {
        pid->_outPwm = pid->_minOut;
    }
    ledcWrite(pid->_pwmChannel, pid->_outPwm);
}

bool PidPwm::setTarget(double target)
{
    if (target < _cMax)
    {
        _sp = target;
        return true;
    }
    return false;
}

PidPwm::PidParam PidPwm::getTuningParams()
{
    return _param;
}

void PidPwm::setTuningParams(PidParam param)
{
    double SampleTimeInSec = ((double)_computeInterval) / 1000;
    _param = param;
    _kp = param.proportionalGain;
    _ki = param.integralGain * SampleTimeInSec;
    _kd = param.derivativeGain / SampleTimeInSec;
}

void PidPwm::setLimits(uint32_t min, uint32_t max)
{
    _minOut = min;
    _maxOut = max;
}

void PidPwm::setLimitsPercentage(uint8_t minDS, uint8_t maxDS)
{
    uint32_t max_possible = (((uint32_t)1) << _pwmRes) - 1;
    _minOut = minDS / 100.0 * max_possible;
    _maxOut = maxDS / 100.0 * max_possible;
}

uint32_t PidPwm::getSamplePeriod()
{
    return _computeInterval;
}

double PidPwm::getTarget()
{
    return _sp;
}

double PidPwm::getCurrent()
{
    return _lastVal;
}

uint32_t PidPwm::getOutput()
{
    return _outPwm;
}

double PidPwm::getOutputDS()
{
    return (_outPwm * 100.0 / ((((uint32_t)1) << _pwmRes) - 1));
}

void PidPwm::setComputeInterval(uint32_t ts)
{
    if (ts > 0)
    {
        double ratio = (double)ts / (double)_computeInterval;
        _ki *= ratio;
        _kd /= ratio;
        _computeInterval = ts;
        if (isRunning())
        {
            xTimerChangePeriod(_computeTimer,
                               pdMS_TO_TICKS(_computeInterval),
                               10);
        }
    }
}

bool PidPwm::isRunning()
{
    return _running;
}
//////////////////////////////////////////////////////////////