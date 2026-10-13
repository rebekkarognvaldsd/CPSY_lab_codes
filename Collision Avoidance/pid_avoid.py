#! /usr/bin/env python3
# PID speed control for both wheels, with collision avoidance:
# the rover stops when an object is within ~20 cm, waits until it has moved away,
# then ramps back up to speed and continues.
# Edit the settings below, then run:  python3 speed_pid_avoid.py
# Stop with Ctrl+C (not Ctrl+Z). Test on the floor.

import time
from gpiozero import DigitalOutputDevice, Motor, MCP3008
from encoders import Encoders

# ---------------- Settings ----------------
SETPOINT = 60     # target wheel speed (rpm)
KP = 0.0055       # proportional gain
KI = 0.006        # integral gain
KD = 0.00001      # derivative gain
RUN_TIME = 15      # seconds of DRIVING (time spent waiting for obstacles is not counted)
RAMP_TIME = 1.0   # seconds to ramp up slowly (avoids power dips / Pi reboots)
DT = 0.05         # loop period (s)

# Collision avoidance
STOP_DISTANCE_CM = 20   # only used in messages; the real threshold is STOP_VOLTAGE
STOP_VOLTAGE = 2.4      # stop at this voltage or higher (object at ~20 cm or closer)
CLEAR_VOLTAGE = 2.1     # resume only when the voltage is BELOW this (object clearly
                        # further away). Keep it well below STOP_VOLTAGE so the rover
                        # doesn't flicker between stop and go at the edge.
CLEAR_TIME = 1.0        # seconds the path must stay clear before resuming
SENSOR_CHANNEL = 0      # MCP3008 channel the sensor signal wire is on
SAMPLES = 5             # readings per loop; the median is used to reject spikes
CONFIRM = 2             # loops in a row above STOP_VOLTAGE before stopping
# ------------------------------------------

PPR = {"left": 700, "right": 700}   # measured encoder steps per wheel revolution

slp = DigitalOutputDevice("GPIO26")
motor = {"left": Motor("GPIO18", "GPIO12"), "right": Motor("GPIO13", "GPIO19")}
encoders = Encoders()   # the C++ decoder owns the encoder pins; no RotaryEncoder here
sensor = MCP3008(channel=SENSOR_CHANNEL)   # SPI on CE0, no pin clash with motors/encoders


def steps(side):
    return encoders.left if side == "left" else encoders.right


def read_voltage():
    """Median of a few quick readings, so one noisy sample can't stop (or miss) anything."""
    vals = sorted(sensor.voltage for _ in range(SAMPLES))
    return vals[len(vals) // 2]


def stop_motors():
    motor["left"].stop()
    motor["right"].stop()


def reset_pid():
    for side in ("left", "right"):
        integral[side] = 0.0
        prev_error[side] = 0.0


integral = {"left": 0.0, "right": 0.0}
prev_error = {"left": 0.0, "right": 0.0}
prev_steps = {"left": steps("left"), "right": steps("right")}
prev_t_enc = encoders.t   # time the decoder line with these counts arrived

log = open("pid_log.csv", "w")
log.write("time_s,state,target_rpm,left_rpm,right_rpm,left_out,right_out,sensor_v\n")

try:
    # Start waiting if something is already in front of the rover.
    v = read_voltage()
    state = "STOPPED" if v >= STOP_VOLTAGE else "DRIVING"
    if state == "STOPPED":
        print(f"Obstacle in front at start ({v:.2f} V). Waiting for it to move.")
    close_count = 0
    clear_since = None
    drive_time = 0.0

    slp.on()
    start = last = ramp_start = time.time()
    while drive_time < RUN_TIME:
        time.sleep(DT)
        now = time.time()
        dt = now - last          # real time since last loop (used by the PID)
        last = now
        t = now - start

        # 1. Measure speed every loop, also while stopped, so the step counts
        #    stay current and there is no speed spike when the rover resumes.
        #    Time covered by the counts: between the decoder lines, not between loops.
        t_enc = encoders.t
        dt_enc = t_enc - prev_t_enc
        prev_t_enc = t_enc
        speed = {}
        for side in ("left", "right"):
            s = steps(side)
            if dt_enc > 0:
                speed[side] = (s - prev_steps[side]) / PPR[side] / dt_enc * 60
            else:
                speed[side] = 0.0    # no new decoder line since the last loop
            prev_steps[side] = s

        # 2. Collision avoidance: decide whether to drive or wait
        v = read_voltage()
        if state == "DRIVING":
            close_count = close_count + 1 if v >= STOP_VOLTAGE else 0
            if close_count >= CONFIRM:
                stop_motors()
                reset_pid()          # don't let the integral wind up while stopped
                state = "STOPPED"
                close_count = 0
                clear_since = None
                print(f"t={t:4.1f}s  OBSTACLE within {STOP_DISTANCE_CM} cm ({v:.2f} V). "
                      f"Stopped, waiting for it to move.")
        else:  # STOPPED
            if v < CLEAR_VOLTAGE:
                if clear_since is None:
                    clear_since = now
                if now - clear_since >= CLEAR_TIME:
                    state = "DRIVING"
                    ramp_start = now     # ramp up gently again, like at the start
                    reset_pid()
                    print(f"t={t:4.1f}s  Path clear ({v:.2f} V). Resuming.")
            else:
                clear_since = None       # object still there (or came back): restart the wait

        # 3. PID only while driving
        out = {"left": 0.0, "right": 0.0}
        target = 0.0
        if state == "DRIVING":
            drive_time += dt
            ramp = min(1.0, (now - ramp_start) / RAMP_TIME)   # 0 -> 1 during RAMP_TIME
            target = SETPOINT * ramp
            for side in ("left", "right"):
                error = target - speed[side]
                integral[side] += error * dt
                derivative = (error - prev_error[side]) / dt
                prev_error[side] = error
                out[side] = KP * error + KI * integral[side] + KD * derivative
                out[side] = max(-1.0, min(1.0, out[side]))   # keep in -1..1
                motor[side].value = out[side]

        print(f"t={t:4.1f}s  {state:7s} target={target:5.1f}  left={speed['left']:6.1f} rpm  "
              f"right={speed['right']:6.1f} rpm  out L={out['left']:.2f} R={out['right']:.2f}  "
              f"sensor={v:.2f} V")
        log.write(f"{t:.3f},{state},{target:.1f},{speed['left']:.1f},{speed['right']:.1f},"
                  f"{out['left']:.3f},{out['right']:.3f},{v:.3f}\n")
finally:
    stop_motors()
    slp.off()
    encoders.close()     # stop the decoder so it releases the encoder pins
    sensor.close()
    log.close()
    print("Stopped. Data saved in pid_log.csv")