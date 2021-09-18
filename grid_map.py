import picar_4wd as fc

from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic 
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
from picar_4wd.speed import Speed
import time
import numpy as np
import math


power = 5
ultrasonic_servo_offset = 0
servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)
us = Ultrasonic(Pin('D8'), Pin('D9'))


def move(p, c):
    c = list(np.array(c)- np.array(p))
    if c == [0,1]:
        fc.turn_left(power)
        time.sleep(1.1)        
        fc.forward(power)
        time.sleep(1.1)
        fc.turn_right(power)
        time.sleep(1)
        fc.stop()
    elif c == [1,0]:
        fc.forward(power)
        time.sleep(1.1)
        fc.stop()
    elif c == [-1,0]:
        fc.backward(power)
        time.sleep(1)
    elif c == [0,-1]:
        fc.turn_right(power)
        time.sleep(1.1)
        fc.forward(power)
        time.sleep(1.1)
        fc.turn_left(power)
        time.sleep(1.3)
        fc.stop()
    # time.sleep(1)
    fc.stop()


def scan(ref):
    # print(mat)
    current_angle = 0
    STEP = 90
    us_step = STEP
    dist = []
    servo.set_angle(current_angle)
    ANGLE_RANGE = 180
    MAX_ANGLE = 90
    sleep_time = .5

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
    print(dist)
    block = get_block(dist, ref)
    return block

def get_block(dist, ref):
    c = None
    for a,d in dist:
        if d == -2:
            continue
        elif a == 90 and d < ref:
            c = [0,1]
            break
        elif a== -90 and d < ref:
            c = [0, -1]
            break
        elif a == 0 and d < ref:
            c = [1, 0]
            break
        else: 
            continue
    return c

def not_visited(visited):
    return True

def search(maze, cur):
    checks = [[1,0], [0,1], [0,-1], [-1,0]] #front, left, right, back
    for c in checks:
        new_x = cur[0] + c[0]
        new_y = cur[1] + c[1]
        if maze[new_x][new_y] in [1,-1]:
            continue

        else: 
            return [new_x, new_y]
    return [-1,-1]


def map(maze, start, ref, visited):
    cur = start
    while (not_visited(visited)):
        
        obs = scan(ref)
        print("Obstacle", obs)
        if obs:
            x = cur[0]+ obs[0]
            y = cur[1] + obs[1]
            # print("X and Y", x,y)
            if maze[x][y] not in [1,-1]:
                maze[x][y] = 1
            # print(maze)
        new_loc = search(maze, cur)
        if new_loc == [-1.-1]:
            print('NOT POSSIBLE')
        # print("New Path: ", path)
        prev = cur
        cur = new_loc
        # cur = [int(cur[0]), int(cur[1])]
        print("Current location: ", prev)
        print("Next location to Go: ", cur)
        move(prev, cur)
        print(maze)
    # return maze, cur
    fc.stop()


if __name__ == '__main__':

    n = 9
    maze = visited = np.zeros((n,n))

    maze[0][:] = visited[0][:] = -1
    maze[n-1]= visited[n-1] = -1
    maze[:,0] = visited[:,0] = -1
    maze[:,n-1] = visited[:,n-1] = -1

    try:
        start = [1,1]
        # end = [2,2]
        stop = False
        ref = 30
        map(maze, start, ref, visited)
    finally:
        fc.stop()