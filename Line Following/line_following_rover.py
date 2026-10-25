#! /usr/bin/env python3
# Line follower: follows the EDGE of a black line on white with one colour sensor.
# The sensor sits half on the line, half on white. Grey = on the edge (drive straight),
# whiter = drifted off (steer toward the line), darker = drifted onto it (steer away).
# The steering is proportional to how far off it is, so it curves smoothly through turns.
# Lost (white for longer than LOST_TIME, e.g. it crossed the line in a sharp turn):
# sweep the other way, then back further, each sweep twice as long, until it sees the line.
#
# Run:        python3 line_following_rover.py
# Calibrate:  python3 line_following_rover.py cal   (motors off, prints C; set BLACK and WHITE)
# Stop with Ctrl+C (not Ctrl+Z, which leaves the pins busy).

import math
import sys
import time
import board
import busio
import adafruit_tcs34725
from gpiozero import DigitalOutputDevice, Motor
from encoders import Encoders

# ---------------- Settings ----------------
BLACK = 509          # C reading on the black line   (measure with "cal")
WHITE = 4013          # C reading on the white track  (measure with "cal")
EDGE = 1              # +1 = line is LEFT of the sensor, -1 = line is RIGHT of the sensor

SPEED = 30            # rpm on a straight
MIN_SPEED = 10        # rpm in the sharpest turn (slows down when far off the edge)
STEER_KP = 60         # rpm wheel difference per unit of error (error is -0.5 .. +0.5)
STEER_KD = 2.0        # damping on the steering (reduces wobble)

LOST_POS = 0.95       # pos above this = sensor sees only white
LOST_TIME = 0.5       # s of only white before it starts sweeping the other way
SEARCH = 25           # rpm of the driving wheel while sweeping (the other wheel stops)

KP, KI, KD = 0.0055, 0.006, 0.00001   # wheel-speed PID (from speed_pid.py)
MAX_OUT = 0.3         # motor output limit
BRAKE = 0.15          # reverse output allowed to slow a wheel down fast (targets are never backward)
DT = 0.040            # loop period (s): two decoder samples per loop, sensor has a fresh reading
# ------------------------------------------

PPR = 700
SIDES = ("left", "right")

color = adafruit_tcs34725.TCS34725(busio.I2C(board.SCL, board.SDA))
color.integration_time = 24   # ms; shorter than DT so every loop gets a new reading
color.gain = 4

if len(sys.argv) > 1 and sys.argv[1] == "cal":
    print("Hold the sensor over black, then white, then the edge. Ctrl+C to stop.")
    try:
        while True:
            print(f"C = {color.color_raw[3]:5d}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        sys.exit()

slp = DigitalOutputDevice("GPIO26")
motor = {"left": Motor("GPIO18", "GPIO12"), "right": Motor("GPIO13", "GPIO19")}
encoders = Encoders()
time.sleep(0.1)       # let the decoder send its first line

integral = {"left": 0.0, "right": 0.0}
prev_error = {"left": 0.0, "right": 0.0}
speed = {"left": 0.0, "right": 0.0}
l0, r0, prev_t_enc = encoders.sample
prev_steps = {"left": l0, "right": r0}
prev_line_err = 0.0
white_since = None    # time it last saw anything but white (None = sees the line)

try:
    slp.on()
    start = last = time.monotonic()
    while True:
        time.sleep(max(0.0, DT - (time.monotonic() - last)))
        now = time.monotonic()
        dt, last = now - last, now

        # 1. Wheel speeds (rpm). Counts and time come from the same decoder line.
        #    No new line since last loop: keep the previous speed instead of reading 0.
        l, r, t_enc = encoders.sample
        dt_enc = t_enc - prev_t_enc
        if dt_enc > 0:
            for side, s in (("left", l), ("right", r)):
                speed[side] = (s - prev_steps[side]) / PPR / dt_enc * 60
                prev_steps[side] = s
            prev_t_enc = t_enc

        # 2. Where is the edge?  pos 0 = black, 1 = white, 0.5 = on the edge
        c = color.color_raw[3]
        pos = min(1.0, max(0.0, (c - BLACK) / (WHITE - BLACK)))
        line_err = pos - 0.5                       # + = too white, - = too black
        d_line = (line_err - prev_line_err) / dt
        prev_line_err = line_err

        if pos < LOST_POS:
            white_since = None
        elif white_since is None:
            white_since = now
        lost_for = 0.0 if white_since is None else now - white_since

        ramp = min(1.0, now - start)               # ramp up over the first second
        if lost_for < LOST_TIME:
            # Follow: too white -> turn toward the line; too black -> turn away from it
            mode = "FOLLOW"
            steer = EDGE * (STEER_KP * line_err + STEER_KD * d_line)
            base = SPEED - (SPEED - MIN_SPEED) * 2 * abs(line_err)   # slow down in turns
            target = {"left":  max(0.0, base - steer) * ramp,
                      "right": max(0.0, base + steer) * ramp}
        else:
            # Lost: sweeps of LOST_TIME, 2x, 4x, ... alternating direction. The first one
            # (toward the line) already happened while following, so the next goes the other way.
            sweep = int(math.log2(lost_for / LOST_TIME + 1))
            turn = EDGE if sweep % 2 == 0 else -EDGE   # +1 = left, -1 = right
            mode = "SEARCH " + ("L" if turn > 0 else "R")
            target = {"left": 0.0 if turn > 0 else SEARCH, "right": SEARCH if turn > 0 else 0.0}

        # 3. PID on each wheel
        for side in SIDES:
            error = target[side] - speed[side]
            derivative = (error - prev_error[side]) / dt
            prev_error[side] = error
            out = KP * error + KI * (integral[side] + error * dt) + KD * derivative
            if -BRAKE < out < MAX_OUT:             # only integrate when not saturated (anti-windup)
                integral[side] += error * dt
            motor[side].value = max(-BRAKE, min(MAX_OUT, out))   # small reverse = brake

        print(f"C={c:5d}  pos={pos:4.2f}  {mode:8s}  target L={target['left']:5.1f} R={target['right']:5.1f}  "
              f"speed L={speed['left']:5.1f} R={speed['right']:5.1f} rpm  dt={dt*1000:.0f}ms")
except KeyboardInterrupt:
    pass
finally:
    for m in motor.values():
        m.stop()
    slp.off()
    encoders.close()
    print("Stopped.")
