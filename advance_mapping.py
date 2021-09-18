import picar_4wd as fc

from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic 
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
import time
import numpy as np
import math
import sys
import matplotlib.pyplot as plt
# import termplotlib as tpl

np.set_printoptions(threshold=sys.maxsize)

speed = 30
ultrasonic_servo_offset = 0
servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)
us = Ultrasonic(Pin('D8'), Pin('D9'))


def scan():
    R = 100
    C = 101
    mat = np.zeros((R, C))
    # print(mat)
    current_angle = 0
    STEP = 5
    us_step = STEP
    dist = []
    servo.set_angle(current_angle)
    ANGLE_RANGE = 180
    sleep_time = .1
    # global current_angle, us_step, STEP
    
    # while(True):
    for i in range(2*int(ANGLE_RANGE/us_step)):
        if current_angle >= 90:
            current_angle = 90
            us_step = -STEP
        elif current_angle <= -90:
            current_angle = -90
            us_step = STEP
        current_angle = current_angle + us_step
        servo.set_angle(current_angle)
        time.sleep(sleep_time)
        distance = us.get_distance()
        dist.append([current_angle,distance])
        # print(current_angle, distance)
    # print(dist)

    coord = []
    xx = []
    yy = []
    for a, d in dist:
        x = int((-1) * np.sin(a*np.pi/180) * d)
        y = abs(int(np.cos(a*np.pi/180) * d))
        if y <= 100 and abs(x) <= 50: 
            xx.append(x)
            yy.append(y)
            print(x,y)
            mat[R-y-1][int(C/2)+x] = 1
        else: 
            continue
   
    print(xx)
    print(yy)
   
    plt.figure(figsize=(8, 6), dpi=80)
    
    
    plt.ylim([0,100])
    plt.xlim([-50,50])
    # fig.show()
    print(mat)
    plt.scatter(np.array(xx), np.array(yy))
    plt.show(block = True)



if __name__ == "__main__":
    try: 
        scan()
    finally: 
        fc.stop()
    
    # scan()