#!/usr/bin/env python3
import ev3dev.ev3 as ev3


m1 = ev3.LargeMotor('outD')
m1.run_timed(time_sp=3000, speed_sp=1000)

m2 = ev3.LargeMotor('outB')
m2.run_timed(time_sp=3000, speed_sp=1000)
