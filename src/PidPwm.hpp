//Copyright @ Adys Tech
//Author : mvadu@adystech.com
//Based on http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
#ifndef _PID_PWM_
#define _PID_PWM_

#include <functional>
#include <Arduino.h>

typedef std::function<double(void)> CurrentValueFunction;

class PidPwm
{
public:
  struct PidParam
  {
    double proportionalGain;
    double integralGain;
    double derivativeGain;
  };

  PidPwm(uint8_t pwmPin, uint8_t channel, double freq, uint8_t resolution_bits, double criticalMax, PidParam param, CurrentValueFunction onCompute);
  bool setTarget(double target);
  void setTuningParams(PidParam param);
  PidParam getTuningParams();
  
  //set the min and max duty cycle either in percentage (0-100 scale) or in raw PWM register values
  void setLimitsPercentage(uint8_t minDS, uint8_t maxDS);
  void setLimits(uint32_t min, uint32_t max);

  double getTarget();
  double getCurrent();
  uint32_t getOutput();
  double getOutputDS();
  uint32_t getSamplePeriod();
  void setComputeInterval(uint32_t ts);
  void shutdown();
  bool begin();
  bool isRunning();

private:
  bool _running;
  gpio_num_t _pin;
  uint8_t _pwmChannel, _pwmRes;
  uint32_t _outPwm, _maxOut, _minOut,_computeInterval = 500,_lastValTs;
  double _kp, _ki, _kd, _sp, _lastVal, _intgErrSum,_cMax,_pwmFrequency;
  CurrentValueFunction _curValFun;
  TimerHandle_t _computeTimer;
  PidParam _param;
  static void computeCallback(TimerHandle_t xTimer);
};

#endif //_PIDPWM_