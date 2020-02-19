### PIDPwm

Arduino already has an excellent [PID library](https://playground.arduino.cc/Code/PIDLibrary/), but its not written to make use of Esp32 RTOS architecture. Esp32 has second core which can take care of running the timer routines, and we can get much better timing control by utilizing RTOS timers. This library uses [RTOS xTimer](https://www.freertos.org/FreeRTOS-timers-xTimerCreate.html). 

This library is not designed to provide direct output value as a computed output which then can be consumed for any other calculations. Its for simple PWM controller which controls the duty cycle based on some measurements of the result of the PWM (e.g. temperature, RPM etc). 

Another feature of this library is it lets the control function as a callback, which gets invoked every x milliseconds, and returns the current value representing the effect of PID controllers output. In essence if the PIDPwm is controlling a heater, this function will return the current temperature. If its controlling a motor trying to accelerate to x RPM, this function will return the current measured RPM.
The library will then use the PID algorithm (pretty much as [described by Brett Beauregard](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)) to update teh PWM output. 

The library uses Esp32's built in PWM controller to generate the PWM and can be tied to any pins. Once setup the library takes care of running its own timers, and no special code (e.g. calling `compute` in every loop) is not needed to get the controller going.



##### example code to be part of setup portion of main.cpp
```
    PidPwm::PidParam param = {2, 5, 0.5};
    pid = new PidPwm(26, 2, 24000, 10, 100, param, [&]() -> double {
        //limit the temp to 100°C, if its more than that report NAN, PidPwm should cut off output bringing down the temp
        double t = ds->getTemperature();
        if (t > 100)
            return NAN;
        //hard off if we are already at the set Target
        if (isnan(targetTemp) || t > targetTemp)
            return NAN;
        return t;
    });
    pid->setTarget(targetTemp);
    pid->setComputeInterval(1000);
```

Reference: [Arduino PID library](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)
