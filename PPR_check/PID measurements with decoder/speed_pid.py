#! /usr/bin/env python3
# Simple PID speed control for both wheels of the rover.
# Edit the settings below, then run:  python3 speed_pid.py
# Stop with Ctrl+C (not Ctrl+Z). Test on the floor.

import time
from gpiozero import DigitalOutputDevice, Motor
from encoders import Encoders

# ---------------- Settings ----------------
SETPOINT = 60     # target wheel speed (rpm)
KP = 0.0055       # proportional gain
KI = 0.006        # integral gain
KD = 0.00001      # derivative gain
RUN_TIME = 5      # seconds
RAMP_TIME = 1.0   # seconds to ramp up slowly (avoids power dips / Pi reboots)
DT = 0.05         # loop period (s)
# ------------------------------------------

PPR = {"left": 700, "right": 700}   # measured encoder steps per wheel revolution

slp = DigitalOutputDevice("GPIO26")
motor = {"left": Motor("GPIO18", "GPIO12"), "right": Motor("GPIO13", "GPIO19")}
encoders = Encoders()   # the C++ decoder owns the encoder pins; no RotaryEncoder here


def steps(side):
    return encoders.left if side == "left" else encoders.right


integral = {"left": 0.0, "right": 0.0}
prev_error = {"left": 0.0, "right": 0.0}
prev_steps = {"left": steps("left"), "right": steps("right")}
prev_t_enc = encoders.t   # time the decoder line with these counts arrived

log = open("pid_log.csv", "w")
log.write("time_s,target_rpm,left_rpm,right_rpm,left_out,right_out\n")

slp.on()
start = last = time.time()
try:
    while time.time() - start < RUN_TIME:
        time.sleep(DT)
        now = time.time()
        dt = now - last          # real time since last loop (used by the PID)
        last = now
        t = now - start
        ramp = min(1.0, t / RAMP_TIME)          # goes from 0 to 1 during RAMP_TIME
        target = SETPOINT * ramp
        speed, out = {}, {}

        # Time covered by the counts: between the decoder lines, not between loops.
        # (The decoder updates every 20 ms and the loop runs every ~50 ms; using the
        # loop's dt made the speed alternate between ~20 % too high and too low.)
        t_enc = encoders.t
        dt_enc = t_enc - prev_t_enc
        prev_t_enc = t_enc

        for side in ("left", "right"):
            # 1. Measure speed: steps since last loop -> revolutions per minute
            s = steps(side)
            if dt_enc > 0:
                speed[side] = (s - prev_steps[side]) / PPR[side] / dt_enc * 60
            else:
                speed[side] = 0.0    # no new decoder line since the last loop
            prev_steps[side] = s

            # 2. PID: compute motor output from the error
            error = target - speed[side]
            integral[side] += error * dt
            derivative = (error - prev_error[side]) / dt
            prev_error[side] = error
            out[side] = KP * error + KI * integral[side] + KD * derivative

            # 3. Keep output in the allowed range -1..1 and send it to the motor
            out[side] = max(-1.0, min(1.0, out[side]))
            motor[side].value = out[side]

        print(f"t={t:4.1f}s  target={target:5.1f}  left={speed['left']:6.1f} rpm  "
              f"right={speed['right']:6.1f} rpm  out L={out['left']:.2f} R={out['right']:.2f}")
        log.write(f"{t:.3f},{target:.1f},{speed['left']:.1f},{speed['right']:.1f},"
                  f"{out['left']:.3f},{out['right']:.3f}\n")
finally:
    motor["left"].stop()
    motor["right"].stop()
    slp.off()
    encoders.close()     # stop the decoder so it releases the encoder pins
    log.close()
    print("Stopped. Data saved in pid_log.csv")