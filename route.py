import picar_4wd as fc

from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic 
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
from picar_4wd.speed import Speed
import time
import numpy as np
import math
import matplotlib.pyplot as plt

# speed = 2
power = 1
ultrasonic_servo_offset = 0
servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)
us = Ultrasonic(Pin('D8'), Pin('D9'))

speed4 = Speed(25)
speed4.start()

def get_block(dist, ref=0):
    x = 0
    block = [0]*5
    for a, d in dist:
        if d >= ref:
            x = 2
        if a == -30:
            block[0] = x
                
        elif a == -15:
            block[1] = x
        elif a == 0:
            block[2] = x
        elif a == 15:
            block[3] = x
        else:
            block[4] = x

    return block

def get_coord(dist):
    # print(dist)
    coord = []
    xx = []
    yy = []
    for a, d in dist:
        x = int((-1) * np.sin(a*np.pi/180) * d)
        y = abs(int(np.cos(a*np.pi/180) * d))
        coord.append([x,y])
        # yy.append(y)
    # coord.append([np.array(xx),np.array(yy)])
    # coord.append(xx)

    return coord
        # coord.append([int(x),int(y)])

    # print(coord)
    # plt.scatter(np.array(xx), np.array(yy))
    # plt.show(block = True)

def static_scan(ref):
    R = 30
    C = 61
    mat = np.zeros((R, C))
    # print(mat)
    current_angle = 0
    STEP = 45
    us_step = STEP
    dist = []
    servo.set_angle(current_angle)
    ANGLE_RANGE = 180
    MAX_ANGLE = 90
    sleep_time = .05

    for i in range(2* int(ANGLE_RANGE/us_step)):
        if current_angle >= ANGLE_RANGE/2:
            current_angle = MAX_ANGLE
            us_step = -STEP
        elif current_angle <= -MAX_ANGLE:
            current_angle = -MAX_ANGLE
            us_step = STEP
        current_angle = current_angle + us_step
        servo.set_angle(current_angle)
        time.sleep(sleep_time)
        distance = us.get_distance()
        dist.append([current_angle,distance])
        # print(current_angle, distance)
    coord = get_coord(dist)
    return coord

def scan(ref):
    # print(mat)
    current_angle = 0
    STEP = 15
    us_step = STEP
    dist = []
    servo.set_angle(current_angle)
    ANGLE_RANGE = 60
    MAX_ANGLE = 30
    sleep_time = .01

    for i in range(2* int(ANGLE_RANGE/us_step)):
        if current_angle >= ANGLE_RANGE/2:
            current_angle = MAX_ANGLE
            us_step = -STEP
        elif current_angle <= -MAX_ANGLE:
            current_angle = -MAX_ANGLE
            us_step = STEP
        current_angle = current_angle + us_step
        servo.set_angle(current_angle)
        time.sleep(sleep_time)
        distance = us.get_distance()
        dist.append([current_angle,distance])
        # print(current_angle, distance)
    block = get_block(dist, ref)
    coord = get_coord(dist)
    return block, coord

def get_new_orientation(old_orientation):
    if old_orientation == 'F':
            return 'R'
    elif old_orientation == 'R':
            return 'B'
    elif old_orientation == 'B':
            return 'L'
    elif old_orientation == 'L':
            return 'F'

def get_cx_cy(new_orientation, current_x, current_y, x, y, distance):

    if new_orientation == 'F':
        cx = current_x - y
        cy = current_y + x

    elif new_orientation == 'R':
        # current_x = current_x + x 
        cx = current_x + x        
        cy = current_y + y
       
    elif new_orientation == 'B':
        
        cx = current_x + y
        cy = current_y - x
       
    elif new_orientation == 'L':
       
        cx = current_x - x
        cy = current_y - y
        
    return cx, cy

def get_current(new_orientation, current_x, current_y,  distance):

    if new_orientation == 'F':
        current_x = current_x - distance

    elif new_orientation == 'R':
        current_y = current_y + distance

    elif new_orientation == 'B':
        current_x = current_x + distance
    elif new_orientation == 'L':
        # current_x = current_x + x 
        current_y = current_y - distance
    return current_x, current_y

def main():
    R = 201
    C = 201
    current_x = int(R/2)
    current_y = int(C/2)
    cx, cy = 0, 0
    
    path = np.zeros((R, C))
    v = 0
    t0 = 0
    new_orientation = 'F'
    old_orientation = None
    while True: 
        scan_list, _ = scan(30)
        if not scan_list:
            continue
        if t0 ==0:
            t0 = time.time()
        
        if scan_list != [2,2,2,2,2]:
            v = speed4()
            fc.stop()
            t1 = time.time()
            distance = int(v * (t1-t0))
            print("Distance: ",distance, t1-t0)
            current_x, current_y = get_current(new_orientation, current_x, current_y, distance)
            coord = static_scan(30)
            for x,y in coord:
                cx, cy = get_cx_cy(new_orientation, current_x, current_y, x, y, distance)
                print(f"scanlist: {scan_list}, x: {x}, y:{y}, cux:{current_x}, cuy:{current_y}, no:{new_orientation}, oo:{old_orientation}")
                path[cx ][cy] = 1
            print(coord)
     
            old_orientation = new_orientation
            time.sleep(1)

            
            fc.turn_right(power)
            time.sleep(.8)
            new_orientation = get_new_orientation(old_orientation)
            t0 = time.time()
        else:
            fc.forward(power)

    
    plt.imshow(path, interpolation='none')
    plt.show()

if __name__ == "__main__":
    try: 
        main()
        # scan(0)
    finally: 
        # print(path)
        fc.stop()