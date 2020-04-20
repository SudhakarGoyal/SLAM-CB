# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
file = open("/home/sudhakar/Downloads/SLAM_lec/Unit_A/robot4_motors.txt")
'''
Sudhakar's code'
'''
left_motor_ticks = []
right_motor_ticks = []
for line in file:
    sp = line.split()
    left_motor_ticks.append(int(sp[2]))
    right_motor_ticks.append(int(sp[6]))
    

left_motor_ticks_movement = []
right_motor_ticks_movement = []

left_motor_ticks_movement.append(0)
right_motor_ticks_movement.append(0)
for i in range(1, len(left_motor_ticks)):
    left_motor_ticks_movement.append(left_motor_ticks[i] - left_motor_ticks[i-1])
    right_motor_ticks_movement.append(right_motor_ticks[i] - right_motor_ticks[i-1])

plt.plot(left_motor_ticks_movement)    
plt.plot(right_motor_ticks_movement)



'''
code of the prof
'''
from lego_robot import LegoLogfile
logfile = LegoLogfile()
logfile.read("robot4_motors.txt")
plt.plot(logfile.motor_ticks)
