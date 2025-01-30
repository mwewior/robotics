from ev3dev2.motor import MoveSteering, MoveTank, LargeMotor, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor

# ev3 outputs
l_motor = LargeMotor(OUTPUT_B)
r_motor = LargeMotor(OUTPUT_C)
move = MoveTank(OUTPUT_B, OUTPUT_C)

# ev3 inputs
l_color = ColorSensor(INPUT_2)
r_color = ColorSensor(INPUT_3)


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


def stopping():
    l_motor.off()
    r_motor.off()
    move.off()
    return False


# Constants:
power = 0.3
max_speed = 100     # moze ? SpeedPercent(100)
min_speed = 0
set_speed = 40
power = 0.5
k = 0.5
last_error = 0
temp = True


while temp is True:
    L_rli = l_color.reflected_light_intensity
    R_rli = r_color.reflected_light_intensity
    error = L_rli - R_rli

    # if abs(error) < 10:
    #     last_error = 0

    # last_error += error
    # correction = error * power + k * last_error

    correction = error * power

    velo_plus = constrain(set_speed + correction, min_speed, max_speed)
    velo_minus = constrain(set_speed - correction, min_speed, max_speed)

    if(correction < -5):
        move.on(-velo_minus, velo_minus)
    elif(correction > 5):
        move.on(velo_plus, -velo_plus)
    else:
        move.on(set_speed, set_speed)
    # last_error = error


# def pid(er, prev_er, dt, h, power, Ti, Td):
#     integ = (er + prev_er) * dt / 2
#     deriv = (er - prev_er) / h
#     return (power * (er + integ/Ti + Td*deriv))
