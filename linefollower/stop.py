#!/usr/bin/env python3
import ev3dev.ev3 as ev3
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.sound import Sound

m1 = ev3.LargeMotor('outD')
m2 = ev3.LargeMotor('outB')
touch = TouchSensor('in1')
sound = Sound()

m1.stop()
m2.stop()
# sound.play_song((
#         ('E5', 'e'), ('E5', 'e'), ('C5', 'e'),
#         ('A4', 'q'), ('A4', 'q'),
#         ('D5', 'q'), ('D5', 'q'), ('D5', 'e'),
#         ('F#5', 'e'), ('F#5', 'e'), ('G5', 'e'), ('A5', 'e'),
#     ), tempo=180)
sound.play_song((
    ('E5', 'e')
    ))
