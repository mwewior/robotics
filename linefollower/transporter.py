#!/usr/bin/env python3
from ev3dev2.motor import MoveTank, LargeMotor, MediumMotor
from ev3dev2.sensor.lego import ColorSensor, TouchSensor, InfraredSensor
from ev3dev2.sound import Sound
from time import sleep


l_motor = LargeMotor('outB')
r_motor = LargeMotor('outD')
move = MoveTank('outB', 'outD')
servo = MediumMotor('outC')

touch = TouchSensor('in1')
l_color = ColorSensor('in2')
r_color = ColorSensor('in3')
infra = InfraredSensor('in4')

sound = Sound()


def speed_follow():
    Kp = 0.22
    Ki = 0.06
    Kd = 1.3

    max_speed = 100
    min_speed = 0
    set_speed = 40
    temp = True

    integ = 0
    last_error = 0

    while temp is True:

        if touch.is_pressed:
            temp = stopping()
        else:
            left_value = l_color.reflected_light_intensity + 5
            right_value = r_color.reflected_light_intensity
            error = left_value - right_value
            if error < 5:
                integ = 0
            else:
                integ += (last_error + error)

            deriv = (error - last_error)
            correction = (error * Kp) + (integ * Ki) + (deriv * Kd)

            velo_minus = constrain(set_speed - correction, min_speed, max_speed)
            velo_plus = constrain(set_speed + correction, min_speed, max_speed)

            if(correction < -5):
                move.on(-0.4*velo_minus, velo_minus)
            elif(correction > 5):
                move.on(velo_plus, -0.4*velo_plus)
            else:
                move.on(set_speed, set_speed)

            last_error = error
    return False


def dist():
    if infra.proximity == 1:
        move.off()
        l_motor.off()
        r_motor.off()
        return True


def holder(sign=0):
    if sign < 0:
        sign = -1
    else:
        sign = 1
    sleep(1)
    servo.on_for_degrees(20, sign*110)
    sleep(1)
    move.on(set_speed, set_speed)
    return False


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


def stopping():
    move.off()
    l_motor.off()
    r_motor.off()
    servo.off()
    sound.play_song((
        ('E5', 'e'), ('C5', 'e')
    ))
    return False


def entry(side):
    if side == 'R':
        L = 1
        R = -1
    if side == 'L':
        L = -1
        R = 1
    move.off()
    speed = 10
    move.on_for_seconds(speed, speed, 1)
    move.on_for_degrees(-100*L, -100*R, 348)
    temp = True
    temp2 = True
    while temp:
        move.on(-1*speed, -1*speed)
        if dist():
            move.on_for_degrees(-0.5*speed, -0.5*speed, 90)
            holder()
            temp = False
    while temp2:
        move.on(speed, speed)
        if ((l_color.reflected_light_intensity < 20 and r_color.reflected_light_intensity < 20) or
            (l_color.reflected_light_intensity < 20 and r_color.color == 5) or
            (l_color.color == 5 and r_color.reflected_light_intensity < 20)):
            move.on_for_seconds(speed, speed, 1)
            move.on_for_degrees(100*L, 100*R, 348)
            temp2 = False
    return 1


def entry_with_package(side):
    print("entry with pack")
    if side == 'R':
        L = 1
        R = -1
    if side == 'L':
        L = -1
        R = 1
    move.off()
    speed = 10
    move.on_for_seconds(speed, speed, 1)
    move.on_for_degrees(-10*L, -10*R, 348)
    move.on_for_seconds(-1*speed, -1*speed, 5)
    move.on_for_degrees(-0.5*speed, -0.5*speed, 90)
    holder(-1)
    servo.off()
    temp2 = True
    move.on_for_seconds(speed, speed, 4)
    sound.play_song((
        ('E5', 'e'), ('C5', 'e'),
        ('C5', 'e'), ('G5', 'e')
    ))
    while temp2:
        move.on(speed, speed)
        if l_color.reflected_light_intensity < 20 and r_color.reflected_light_intensity < 20:
            move.on_for_seconds(speed, speed, 1)
            move.on_for_degrees(10*L, 10*R, 348)
            temp2 = False
            speed_follow()
    return 0


Kp = 0.33
Ki = 0.06
Kd = 0.40

max_speed = 100
min_speed = 0
set_speed = 18
temp = True

Kp_p = 0.26
Ki_p = 0.05
Kd_p = 0.4
package_speed = 12

integ = 0
deriv = 0
last_error = 0
package = 0

sound.speak('engine start')


def pack_on(temp, package, last_error, integ, deriv, min_speed, max_speed):
    wjazd = 0
    while temp is True and package == 1:

        if touch.is_pressed:
            temp = stopping()
        else:

            left_value = l_color.reflected_light_intensity + 3
            right_value = r_color.reflected_light_intensity

            error = left_value - right_value
            if error < 5:
                integ = 0
            else:
                integ += (last_error + error)

            deriv = (error - last_error)
            correction = (error * Kp_p) + (integ * Ki_p) + (deriv * Kd_p)
            if wjazd == 1:
                correction = 0.5 * correction

            velo_minus = constrain(package_speed - correction, min_speed, max_speed)
            velo_plus = constrain(package_speed + correction, min_speed, max_speed)

            if(correction < -5):
                move.on(-0.35*velo_minus, velo_minus)
            elif(correction > 5):
                move.on(velo_plus, -0.35*velo_plus)
            else:
                move.on(package_speed, package_speed)

            last_error = error
            wjazd = 0
            if (60 < r_color.rgb[2] and r_color.rgb[2] < 160) and (60 < r_color.rgb[1] and r_color.rgb[1] < 90):
                wjazd = 1
                move.off()
                sleep(1)
                move.off()
                if (r_color.color == 2 or r_color.color == 3) and r_color.color != 1 and r_color.color != 6:
                    package = entry_with_package('R')
                    temp = True
                    package = 0
                    servo.off()
                else:
                    pack_on(temp, package, 0, 0, 0, min_speed, max_speed)

            if (25 < l_color.rgb[2] and l_color.rgb[2] < 130) and (20 < l_color.rgb[1] and l_color.rgb[1] < 50):
                wjazd = 1
                move.off()
                sleep(1)
                move.off()
                if (l_color.color == 2 or l_color.color == 3) and l_color != 1 and l_color != 6:
                    package = entry_with_package('L')
                    temp = True
                    package = 0
                else:
                    pack_on(temp, package, 0, 0, 0, min_speed, max_speed)
    return package


while temp is True and package == 0:

    if touch.is_pressed:
        temp = stopping()
    else:
        left_value = l_color.reflected_light_intensity + 3
        right_value = r_color.reflected_light_intensity

        error = left_value - right_value
        if error < 5:
            integ = 0
        else:
            integ += (last_error + error)

        deriv = (error - last_error)
        correction = (error * Kp) + (integ * Ki) + (deriv * Kd)

        velo_minus = constrain(set_speed - correction, min_speed, max_speed)
        velo_plus = constrain(set_speed + correction, min_speed, max_speed)

        if(correction < -5):
            move.on(-0.35*velo_minus, velo_minus)
        elif(correction > 5):
            move.on(velo_plus, -0.35*velo_plus)
        else:
            move.on(set_speed, set_speed)

        last_error = error

        if r_color.color == 5:
            package = entry('R')
            package = pack_on(temp, package, 0, 0, 0, min_speed, max_speed)

        if l_color.color == 5:
            package = entry('L')
            package = pack_on(temp, package, 0, 0, 0, min_speed, max_speed)
