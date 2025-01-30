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

l_color = ColorSensor('in2')
r_color = ColorSensor('in3')


while True:
    print("lewy: ", l_color.reflected_light_intensity, "\t|\tprawy: ", r_color.reflected_light_intensity)
    print("lewy: ", l_color.color, "\t|\tprawy: ", r_color.color)
    rli = (l_color.reflected_light_intensity, r_color.reflected_light_intensity)
    print("lewy: ", l_color.rgb, "\t|\tprawy: ", r_color.rgb)
    print('\n\n\n')
    sleep(2)

    # # lewe silniki

    # if l_color.color == 5:
    #     sound.speak('red')
    #     sound.speak('left')

    # if l_color.color == 2:
    #     sound.speak('blue')
    #     sound.speak('left')

    # if l_color.color == 3:
    #     sound.speak('green')
    #     sound.speak('left')

    # # prawe silniki

    # if r_color.color == 5:
    #     sound.speak('red')
    #     sound.speak('right')

    # if r_color.color == 2:
    #     sound.speak('blue')
    #     sound.speak('right')

    # if r_color.color == 3:
    #     sound.speak('green')
    #     sound.speak('right')
