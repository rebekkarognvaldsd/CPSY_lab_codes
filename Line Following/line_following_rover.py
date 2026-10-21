#! /usr/bin/env python3
# Simple line follower with PID wheel-speed control.
# Black line: drive straight. White: turn on the spot until the line is found.
# If it gets whiter while turning, turn the other way.
# Run: python3 line_follow.py     Stop with Ctrl+C.

import time
import board
import busio
import adafruit_tcs34725
from gpiozero import DigitalOutputDevice, Motor
from encoders import Encoders

# ---------------- Settings ----------------
SPEED = 60            # rpm when driving straight
TURN = 30             # rpm of the outer wheel when turning (the inner wheel stops)
KP, KI, KD = 0.0055, 0.006, 0.00001
MAX_OUT = 0.3         # motor output limit
DT = 0.05             # loop period (s)
THRESHOLD = 4975      # C below = black, above = white
GROW = 500            # C must grow by more than this between two readings to switch direction
                      # (0 = any increase; white is noisy, so 0 makes it switch very often)
# ------------------------------------------

PPR = 700
SIDES = ("left", "right")

slp = DigitalOutputDevice("GPIO26")
motor = {"left": Motor("GPIO18", "GPIO12"), "right": Motor("GPIO13", "GPIO19")}
encoders = Encoders()
color = adafruit_tcs34725.TCS34725(busio.I2C(board.SCL, board.SDA))
color.integration_time = 50
color.gain = 4

integral = {"left": 0.0, "right": 0.0}
prev_error = {"left": 0.0, "right": 0.0}
prev_steps = {"left": encoders.left, "right": encoders.right}
prev_t_enc = encoders.t

on_line = True
turn_dir = 1          # +1 = right, -1 = left (kept between turns)
prev_c = 0

try:
    slp.on()
    start = last = time.time()
    while True:
        time.sleep(max(0.0, DT - (time.time() - last)))
        now = time.time()
        dt, last = now - last, now

        # 1. Measure wheel speeds (rpm)
        t_enc = encoders.t
        dt_enc, prev_t_enc = t_enc - prev_t_enc, t_enc
        speed = {}
        for side in SIDES:
            s = encoders.left if side == "left" else encoders.right
            speed[side] = (s - prev_steps[side]) / PPR / dt_enc * 60 if dt_enc > 0 else 0.0
            prev_steps[side] = s

        # 2. Line: black = straight, white = turn, whiter = turn the other way
        c = color.color_raw[3]
        new_on_line = c < THRESHOLD
        if not new_on_line and not on_line:   # was already turning
            if c > prev_c + GROW:             # getting whiter: wrong way, turn the other way
                turn_dir = -turn_dir
        prev_c = c
        if new_on_line != on_line:
            integral = {"left": 0.0, "right": 0.0}   # fresh PID when switching
        on_line = new_on_line

        if on_line:
            target = {"left": SPEED, "right": SPEED}
        else:
            # turn forward: outer wheel drives, inner wheel stops (right turn = left wheel drives)
            target = {"left": TURN if turn_dir > 0 else 0, "right": 0 if turn_dir > 0 else TURN}

        # 3. PID on each wheel (ramp up over the first second)
        ramp = min(1.0, now - start)
        for side in SIDES:
            error = target[side] * ramp - speed[side]
            integral[side] += error * dt
            derivative = (error - prev_error[side]) / dt
            prev_error[side] = error
            out = KP * error + KI * integral[side] + KD * derivative
            motor[side].value = max(0.0, min(MAX_OUT, out))   # never negative = never backward

        print(f"C={c:5d}  {'LINE' if on_line else 'TURN ' + ('R' if turn_dir > 0 else 'L')}  "
              f"L={speed['left']:6.1f}  R={speed['right']:6.1f} rpm  dt={dt*1000:.0f}ms")
except KeyboardInterrupt:
    pass
finally:
    for m in motor.values():
        m.stop()
    slp.off()
    encoders.close()
    print("Stopped.") 