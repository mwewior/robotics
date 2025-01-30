#!/usr/bin/env python3
import ev3dev.ev3 as ev3
from ev3dev2.motor import MoveTank, MediumMotor
from ev3dev2.sensor.lego import TouchSensor
from time import sleep


def scale(readings):
    return (int(100 * readings[0] / 65), int(100 * readings[1] / 80))


def minmax(val):
    if val <= -100:
        return -100
    elif val >= 100:
        return 100
    else: return val


s1 = ev3.ColorSensor('in1')
s2 = ev3.ColorSensor('in4')
tank = MoveTank('outB', 'outD')
lever = MediumMotor('outC')
payload_detected = False
backing_with_payload = False
on_entry = False
double_trouble_r = 0
double_trouble_b = 0
w_payload = False
state = 0
side = ""
twice_check = False
states = {
    0: "line following before pickup",
    1: "turning for approaching pickup trajectory",
    2: "trajectory for pickup",
    3: "pickup",
    4: "backing for the track",
    5: "entering back the loop",
    6: "line following after pickup",
    7: "turning for approaching dropoff trajectory",
    8: "trajectory for dropoff",
    9: "dropoff"
}

# payload approach loop
while True:
    print(states[state])
    reading = scale((s1.reflected_light_intensity, s2.reflected_light_intensity))
    error = reading[0] - reading[1]
    tank.on(minmax(15 + 0.5 * error), minmax(15 - 0.5 * error))
    col1 = s1.color
    col2 = s2.color

    if not (col1 == col2 == 5):
        double_trouble_r = 0

    if double_trouble_r > 5:
        double_trouble_r = 0
        state = 3


    # payload approach state
    if col1 == 5 and col2 == 5 and (state == 2 or state == 3):
        double_trouble_r += 1
        if state == 3:
            tank.off()
            sleep(0.5)
            # back off
            tank.on_for_rotations(-50, -50, 2)
            # turn
            tank.on_for_rotations(-30, 30, 1.55)
            # back up
            tank.on_for_rotations(-50, -50, 1.3)
            # pick up
            lever.on_for_rotations(30, 4)
            tank.on_for_rotations(50, 50, 0.4)
            # exit state
            state = 4

    # entering approching trajectory state
    elif col1 == 5 and state == 0:
        state = 1
        side = "l"
        print(states[state])
        tank.off()
        tank.on_for_rotations(30, 30, 0.2)
        tank.on_for_rotations(-30, 30, 0.7)
        state = 2
    elif col2 == 5 and state == 0:
        state = 1
        side = "r"
        print(states[state])
        tank.off()
        tank.on_for_rotations(30, 30, 0.2)
        tank.on_for_rotations(30, -30, 0.7)
        state = 2
    # going back on track
    elif ((col1 == 1 and col2 == 1) or (col1 == 1 and col2 == 5) or (col1 == 5 and col2 == 1)) and state == 4:
        state = 5
        print(states[state])
        if side == "l":
            tank.on_for_rotations(-30, 30, 0.7)
        elif side == "r":
            tank.on_for_rotations(30, -30, 0.7)
        state = 6
        break



# payload deposition loop
while True:
    print(states[state])
    reading = scale((s1.reflected_light_intensity, s2.reflected_light_intensity))
    error = reading[0] - reading[1]
    tank.on(minmax(15 + 0.5 * error), minmax(15 - 0.5 * error))
    col1 = s1.color
    col2 = s2.color

    if not (col1 == col2 == 2):
        double_trouble_b = 0

    if double_trouble_b > 5:
        double_trouble_b = 0
        state = 9

    # entering approching trajectory state
    if col1 == 2 and state == 6 and twice_check:
        twice_check = False
        state = 7
        print(states[state])
        tank.off()
        tank.on_for_rotations(30, 30, 0.2)
        tank.on_for_rotations(-30, 30, 0.7)
        state = 8

    elif col1 == 2 and state == 6 and not twice_check:
        tank.on_for_degrees(10, 10, 1)
        col1 = s1.color
        if col1 == 2:
            twice_check = True
    elif col2 == 2 and state == 6 and twice_check:
        twice_check = False
        state = 7
        print(states[state])
        tank.off()
        tank.on_for_rotations(30, 30, 0.2)
        tank.on_for_rotations(30, -30, 0.7)
        state = 8
    elif col2 == 2 and state == 6 and not twice_check:
        tank.on_for_degrees(10, 10, 1)
        col2 = s1.color
        if col2 == 2:
            twice_check = True

    # payload approach state
    elif col1 == 2 and col2 == 2:
        double_trouble_b += 1
        if state == 9:
            tank.off()
            sleep(0.5)
            # back off
            tank.on_for_rotations(-50, -50, 2)
            # turn
            tank.on_for_rotations(-30, 30, 1.6)
            # back up
            tank.on_for_rotations(-50, -50, 1.3)
            # leave
            lever.on_for_rotations(-30, 4.5)
            tank.on_for_rotations(50, 50, 1.3)
            break