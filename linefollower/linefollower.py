#!/usr/bin/env python3

from ev3dev2.motor import MoveTank, LargeMotor
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sound import Sound


sound = Sound()

# ev3 outputs
l_motor = LargeMotor('outB')
r_motor = LargeMotor('outD')
move = MoveTank('outB', 'outD')

# ev3 inputs
l_color = ColorSensor('in2')
r_color = ColorSensor('in3')
touch = TouchSensor('in1')


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


def stopping():
    l_motor.off()
    r_motor.off()
    move.off()
    sound.play_song((
        ('E5', 'e'), ('E5', 'e'), ('C5', 'e')
    ))
    return False


Kp = 0.22
Ki = 0.06
Kd = 1.3

max_speed = 100
min_speed = 0
set_speed = 40
temp = True

integ = 0
last_error = 0


sound.speak('engine start')
sound.play_song((
    ('F#4', 'e3'), ('G4', 'e3'), ('G#4', 'e3'),
    ('A4', 'q'), ('E4', 'q'), ('A3', 'q'),
), tempo=240)

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
