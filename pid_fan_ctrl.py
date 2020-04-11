#!/usr/bin/python
# -*- coding: utf-8 -*-

# This python program reads the CPU temperature of the Raspberry Pi and adjusts the
# cooling fan speed accordingly, using a PID (Proportional-Integral-Derivative)
# algorithm.

# This file is heavily based on the one by Aerandir14 on
# https://www.instructables.com/id/PWM-Regulated-Fan-Based-on-CPU-Temperature-for-Ras/

import RPi.GPIO as GPIO
import time
import sys

# Configuration
FAN_PIN  = 17  # BCM pin used to drive transistor's base
dt       =  1  # [s] Time to wait between each refresh
PWM_FREQ = 25  # [Hz] Change this value if fan has strange behavior
setpoint = 50  # Setpoint (desired) temperature

# Setup GPIO pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_PIN, GPIO.OUT, initial=GPIO.LOW)
fan = GPIO.PWM(FAN_PIN, PWM_FREQ)
fan.start(0)

i = 0
cpuTemp     = 0
fanSpeed    = 0
cpuTempOld  = 0
fanSpeedOld = 0
integral    = 0
errorOld    = 0
output      = 0

# You will probably need to tweak these constants to suit your needs
Kp = 10
Ki = 0.1
Kd = 1

try:
    while 1:
        # Read CPU temperature
        cpuTempFile = open("/sys/class/thermal/thermal_zone0/temp", "r")
        cpuTemp = float(cpuTempFile.read()) / 1000
        cpuTempFile.close()

        # Perform PID routine - taken almost verbatim from https://en.wikipedia.org/wiki/PID_controller#Pseudocode
        error = setpoint - cpuTemp
        
        if 0<output<100: # Integral should not build up outside of the controllable range
          integral   = integral + error * dt
        
        derivative = (error - errorOld) / dt
        output     = int(min(100,max(0,-(Kp*error + Ki*integral + Kd*derivative))))
        errorOld   = error

        fan.ChangeDutyCycle(output)

        # Store current fan setting in text file for sending to data handler (not necessary for fan to function)
        file=open("fan.txt","w")
        file.write("%s" % output + "\n")
        file.close

        # Log CPU temp, error and output to log file - mostly for debugging purposes
        file=open("fan.log","a")
        file.write("%.2f" % cpuTemp + ", %.2f" % error + ", %s" % output + "\n")
        file.close

        time.sleep(dt)


# If a keyboard interrupt occurs (ctrl + c), the GPIO is set to 0 and the program exits.
except KeyboardInterrupt:
    print("Fan ctrl interrupted by keyboard")
    GPIO.cleanup()
    sys.exit()
