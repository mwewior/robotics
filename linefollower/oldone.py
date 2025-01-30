
# from ev3dev2.motor import MoveTank, LargeMotor  # , SpeedPercent
# from ev3dev2.sensor.lego import ColorSensor, TouchSensor
# from ev3dev2.sound import Sound


# sound = Sound()

# # ev3 outputs
# l_motor = LargeMotor('outB')
# r_motor = LargeMotor('outD')
# move = MoveTank('outB', 'outD')

# # ev3 inputs
# l_color = ColorSensor('in2')
# r_color = ColorSensor('in3')
# touch = TouchSensor('in1')


# def stopping():
#     l_motor.off()
#     r_motor.off()
#     move.off()
#     sound.play_song((
#         ('E5', 'e'), ('E5', 'e'), ('C5', 'e'),
#         # ('A4', 'q'), ('A4', 'q'),
#         # ('D5', 'q'), ('D5', 'q'), ('D5', 'e'),
#         # ('F#5', 'e'), ('F#5', 'e'), ('G5', 'e'), ('A5', 'e'),
#     ))
#     return False
#

# def constrain(val, min_val, max_val):
#     return min(max_val, max(min_val, val))


# # moze powinien tu byc jakis cypis?
# sound.play_song((
#     ('F#4', 'e3'), ('G4', 'e3'), ('G#4', 'e3'),
#     ('A4', 'q'), ('E4', 'q'), ('A3', 'q'),
# ), tempo=180)
# sound.speak('engine start')

# """
# # Constants:
# power = 0.5
# max_speed = 100     # moze ? SpeedPercent(100)
# min_speed = 0
# set_speed = 20
# power = 0.5
# k = 0.5
# last_error = 0
# temp = True

# while temp is True:
#     # zatrzymanie programu po wcisnieciu przycisku
#     if touch.is_pressed:
#         stopping()
#         temp = False

#     else:
#         L_rli = l_color.reflected_light_intensity
#         R_rli = r_color.reflected_light_intensity
#         error = L_rli - R_rli

#         if abs(error) < 10:
#             last_error = 0

#         last_error += error
#         correction = error * power + k * last_error

#         # lines 66-70 del
#         # ??? correction = error-last * power

#         velo_plus = constrain(set_speed + correction, min_speed, max_speed)
#         velo_minus = constrain(set_speed - correction, min_speed, max_speed)

#         if(correction < -5):
#             move.on(0, velo_minus)
#         elif(correction > 5):
#             move.on(velo_plus, 0)
#         else:
#             move.on(set_speed, set_speed)
#         # last_error = error
# """

# """
# ########################################################################################################################
# ########################################################################################################################
# ########################################################################################################################
#                                             P I D   R O B O T
# ########################################################################################################################
# ########################################################################################################################
# ########################################################################################################################
# """

# # def pid(er, prev_er, dt, h, Kp, Ki, Kd):
# #     integ = (er + prev_er) * dt / 2
# #     deriv = (er - prev_er) / h
# #     correction = error * Kp + integ * Ki + deriv * Kd

# """
# Próba PID
# """


# def set_da_speed(velo, correction):
#     if velo + correction < 0:
#         return 0
#     elif velo + correction > 100:
#         return 100
#     else:
#         return velo + correction


# # Constants:
# max_speed = 100     # moze ? SpeedPercent(100)
# min_speed = 0
# base_speed = 30

# temp = True

# Kp = 0.3
# Ki = 0
# Kd = 0

# dt = 0.1
# dx = 0.1

# error = 0
# last_error = 0
# integ = 0
# deriv = 0


# while True:
#     #  zatrzymanie programu po wcisnieciu przycisku
#     if touch.is_pressed:
#         l_motor.off()
#         r_motor.off()
#         move.off()
#         temp = stopping()
#         temp = False

#     L_rli = l_color.reflected_light_intensity
#     R_rli = r_color.reflected_light_intensity

#     error = L_rli - R_rli

#     integ = (error + integ)*dt/2
#     deriv = (error - last_error)/dx

#     correction = error * Kp
#     # + integ * Ki + deriv * Kd

#     # last_error = error

#     velo_plus = constrain(base_speed + correction, min_speed, max_speed)
#     velo_minus = constrain(base_speed - correction, min_speed, max_speed)

#     # zamiast constraina i velo plus/minus jest funkcja set_da_speed

#     # velocity = set_da_speed(base_speed, correction)

#     if(correction < -5):
#         move.on(-1*0.7*velo_minus, velo_minus)
#     elif(correction > 5):
#         move.on(velo_plus, -1*0.7*velo_plus)
#     else:
#         move.on(base_speed, base_speed)
