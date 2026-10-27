#! /usr/bin/env python3
# Line follower with collision avoidance.
# Line following (from line_following_rover.py): follows the EDGE of a black line on white
# with one colour sensor. Grey = on the edge (drive straight), whiter = drifted off (steer
# toward the line), darker = drifted onto it (steer away). If it sees only white for longer
# than LOST_TIME it sweeps the other way, then back further, until it finds the line.
# Collision avoidance (from speed_pid_avoidance.py): the rover stops when an object is inside
# the stop range, waits until it has moved out past the clear range, then ramps back up and
# carries on following the line.
#
# Run:        python3 line_following_avoid_rover.py
# Calibrate:  python3 line_following_avoid_rover.py cal   (motors off, prints C; set BLACK and WHITE)
# Stop with Ctrl+C (not Ctrl+Z, which leaves the pins busy).

import math                                   # log2, used to work out which search sweep we are in
import sys                                    # command-line arguments ("cal") and exiting
import time                                   # clock and sleeping between loops
import board                                  # Raspberry Pi pin names (SCL, SDA) for the I2C bus
import busio                                  # I2C bus that the colour sensor is connected to
import adafruit_tcs34725                      # driver for the TCS34725 colour sensor
from gpiozero import DigitalOutputDevice, Motor, MCP3008   # GPIO helpers: on/off pin, motor, ADC
from encoders import Encoders                 # our wrapper around the C++ decoder (wheel step counts)

# ---------------- Settings: line following ----------------
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
RAMP_TIME = 1.0       # s to ramp up to speed at the start and after an obstacle (avoids power dips)

# ---------------- Settings: collision avoidance ----------------
# The distance sensor voltage rises as an object gets closer (up to its peak at ~20 cm).
STOP_VOLTAGE = 2.05     # stop when the sensor reads this or higher (object close)
CLEAR_VOLTAGE = 1.511   # resume only once it reads below this (~40 cm or further)
CLEAR_TIME = 1.0        # seconds the path must stay clear before resuming
SENSOR_CHANNEL = 0      # MCP3008 channel the sensor signal wire is on
SAMPLES = 5             # readings per loop; the median is used to reject spikes
CONFIRM = 2             # loops in a row inside the stop range before stopping

# Measured calibration (cm, average volts), only used to print an approximate distance.
CALIBRATION = [
    (20, 2.403), (30, 1.925), (40, 1.511), (50, 1.237), (60, 1.051),
    (70, 0.922), (80, 0.834), (90, 0.788), (100, 0.745), (110, 0.692),
]
# ----------------------------------------------------------


def volts_to_cm_text(v):
    """Approximate distance for printing. Above the peak it can't tell 10 cm from 20 cm."""
    if v >= CALIBRATION[0][1]:                # higher than the 20 cm voltage
        return "<=20 cm"
    if v <= CALIBRATION[-1][1]:               # lower than the furthest calibrated voltage
        return f">{CALIBRATION[-1][0]} cm"
    for (d1, v1), (d2, v2) in zip(CALIBRATION, CALIBRATION[1:]):   # find the two points around v
        if v2 <= v <= v1:
            return f"~{d1 + (d2 - d1) * (v1 - v) / (v1 - v2):.0f} cm"   # interpolate the distance


assert CLEAR_VOLTAGE < STOP_VOLTAGE, "CLEAR_VOLTAGE must be lower than STOP_VOLTAGE"

PPR = 700                                     # encoder steps per wheel revolution
SIDES = ("left", "right")                     # the two wheels, used to loop over both

color = adafruit_tcs34725.TCS34725(busio.I2C(board.SCL, board.SDA))   # open the colour sensor on I2C
color.integration_time = 24   # ms; shorter than DT so every loop gets a new reading
color.gain = 4                                # sensor amplification (higher = brighter readings)

if len(sys.argv) > 1 and sys.argv[1] == "cal":   # started with "cal"
    print("Hold the sensor over black, then white, then the edge. Ctrl+C to stop.")
    try:
        while True:                           # keep printing until Ctrl+C
            print(f"C = {color.color_raw[3]:5d}")   # color_raw = (R, G, B, C); C = overall brightness
            time.sleep(0.2)                   # 5 readings per second is enough to read them
    except KeyboardInterrupt:                 # Ctrl+C pressed
        sys.exit()                            # quit before the motors are ever set up

slp = DigitalOutputDevice("GPIO26")           # motor driver sleep pin: on = motors enabled
motor = {"left": Motor("GPIO18", "GPIO12"), "right": Motor("GPIO13", "GPIO19")}   # forward/backward pins per motor
encoders = Encoders()                         # start the decoder; it counts wheel steps in the background
sensor = MCP3008(channel=SENSOR_CHANNEL)      # distance sensor through the ADC (SPI on CE0)
time.sleep(0.1)       # let the decoder send its first line


def read_voltage():
    """Median of a few quick readings, so one noisy sample can't stop (or miss) anything."""
    vals = sorted(sensor.voltage for _ in range(SAMPLES))   # SAMPLES readings, smallest first
    return vals[len(vals) // 2]               # the middle one


def stop_motors():
    for m in motor.values():                  # both motors
        m.stop()                              # stop the motor


def reset_pid():
    for side in SIDES:                        # both wheels
        integral[side] = 0.0                  # forget the summed error
        prev_error[side] = 0.0                # and the last error


integral = {"left": 0.0, "right": 0.0}        # PID integral (summed error) per wheel
prev_error = {"left": 0.0, "right": 0.0}      # last loop's error per wheel, for the PID derivative
speed = {"left": 0.0, "right": 0.0}           # last measured wheel speed (rpm)
l0, r0, prev_t_enc = encoders.sample          # starting step counts and the time they were read
prev_steps = {"left": l0, "right": r0}        # step counts at the last speed measurement
prev_line_err = 0.0                           # last loop's line error, for the steering derivative
white_since = None    # time it last saw anything but white (None = sees the line)

print(f"Stop at >= {STOP_VOLTAGE:.2f} V. "
      f"Resume below {CLEAR_VOLTAGE:.3f} V ({volts_to_cm_text(CLEAR_VOLTAGE)}) for {CLEAR_TIME:.1f} s.")

try:
    v = read_voltage()                        # check for an obstacle before moving
    state = "STOPPED" if v >= STOP_VOLTAGE else "DRIVING"   # start waiting if something is in front
    if state == "STOPPED":
        print(f"Obstacle in front at start ({volts_to_cm_text(v)}, {v:.2f} V). Waiting for it to move.")
    close_count = 0                           # loops in a row the obstacle has been inside the stop range
    clear_since = None                        # time the path became clear (None = not clear)

    slp.on()                                  # wake the motor driver
    last = ramp_start = time.monotonic()      # time of the last loop and of the start of the ramp
    while True:                               # main control loop, runs every DT seconds
        time.sleep(max(0.0, DT - (time.monotonic() - last)))   # wait out the rest of this loop period
        now = time.monotonic()                # time at the start of this loop
        dt, last = now - last, now            # actual time since the last loop (s)

        # 1. Wheel speeds (rpm), measured also while stopped so there is no spike on resuming.
        #    No new decoder line since last loop: keep the previous speed instead of reading 0.
        l, r, t_enc = encoders.sample         # latest left/right step counts and when they arrived
        dt_enc = t_enc - prev_t_enc           # time between this encoder reading and the previous one
        if dt_enc > 0:                        # only if the decoder sent a new reading
            for side, s in (("left", l), ("right", r)):   # do the same for both wheels
                speed[side] = (s - prev_steps[side]) / PPR / dt_enc * 60   # steps -> revolutions -> per minute
                prev_steps[side] = s          # remember the count for next time
            prev_t_enc = t_enc                # remember the time for next time

        # 2. Collision avoidance: decide whether to drive or wait
        v = read_voltage()                    # distance sensor voltage (higher = closer)
        if state == "DRIVING":
            close_count = close_count + 1 if v >= STOP_VOLTAGE else 0   # count loops inside the stop range
            if close_count >= CONFIRM:        # close for long enough: it's a real obstacle
                stop_motors()                 # stop right away
                reset_pid()                   # don't let the integral wind up while stopped
                state = "STOPPED"
                close_count = 0
                clear_since = None
                print(f"OBSTACLE at {volts_to_cm_text(v)} ({v:.2f} V). Stopped, waiting for it to move.")
        else:  # STOPPED
            if v < CLEAR_VOLTAGE:             # nothing within the clear range
                if clear_since is None:       # just became clear
                    clear_since = now         # start the clear timer
                if now - clear_since >= CLEAR_TIME:   # clear for long enough
                    state = "DRIVING"
                    ramp_start = now          # ramp up gently again, like at the start
                    reset_pid()               # start the wheel PID fresh
                    white_since = None        # the time spent waiting doesn't count as lost
                    prev_line_err = 0.0       # no steering kick from the old error
                    print(f"Path clear ({volts_to_cm_text(v)}, {v:.2f} V). Resuming.")
            else:
                clear_since = None            # object still there (or came back): restart the wait

        # 3. Where is the edge?  pos 0 = black, 1 = white, 0.5 = on the edge
        c = color.color_raw[3]                # brightness (C channel) under the sensor
        pos = min(1.0, max(0.0, (c - BLACK) / (WHITE - BLACK)))   # scale to 0 (black) .. 1 (white), clipped

        if state == "STOPPED":                # waiting for an obstacle: motors stay off
            print(f"C={c:5d}  pos={pos:4.2f}  STOPPED   sensor={v:.2f} V ({volts_to_cm_text(v)})")
            continue                          # skip steering and PID

        line_err = pos - 0.5                       # + = too white, - = too black
        d_line = (line_err - prev_line_err) / dt   # how fast the error is changing (for damping)
        prev_line_err = line_err              # remember the error for next time

        if pos < LOST_POS:                    # sees some black: not lost
            white_since = None                # reset the lost timer
        elif white_since is None:             # only white, and the timer isn't running yet
            white_since = now                 # start the lost timer
        lost_for = 0.0 if white_since is None else now - white_since   # seconds of only white so far

        ramp = min(1.0, (now - ramp_start) / RAMP_TIME)   # 0 -> 1 during RAMP_TIME
        if lost_for < LOST_TIME:              # not lost (or only briefly): follow the edge
            # Follow: too white -> turn toward the line; too black -> turn away from it
            mode = "FOLLOW"                   # shown in the printout
            steer = EDGE * (STEER_KP * line_err + STEER_KD * d_line)   # rpm difference between the wheels
            base = SPEED - (SPEED - MIN_SPEED) * 2 * abs(line_err)   # slow down in turns
            target = {"left":  max(0.0, base - steer) * ramp,   # positive steer = left wheel slower = turn left
                      "right": max(0.0, base + steer) * ramp}   # never a negative (backward) target
        else:                                 # white for too long: search for the line
            # Lost: sweeps of LOST_TIME, 2x, 4x, ... alternating direction. The first one
            # (toward the line) already happened while following, so the next goes the other way.
            sweep = int(math.log2(lost_for / LOST_TIME + 1))   # which sweep we're in: 0, 1, 2, ...
            turn = EDGE if sweep % 2 == 0 else -EDGE   # +1 = left, -1 = right
            mode = "SEARCH " + ("L" if turn > 0 else "R")   # shown in the printout
            target = {"left": 0.0 if turn > 0 else SEARCH, "right": SEARCH if turn > 0 else 0.0}   # one wheel drives, the other stops

        # 4. PID on each wheel
        for side in SIDES:                    # same controller for both wheels
            error = target[side] - speed[side]   # how far the wheel is from its target speed (rpm)
            derivative = (error - prev_error[side]) / dt   # how fast that error is changing
            prev_error[side] = error          # remember the error for next time
            out = KP * error + KI * (integral[side] + error * dt) + KD * derivative   # PID output (motor power)
            if -BRAKE < out < MAX_OUT:             # only integrate when not saturated (anti-windup)
                integral[side] += error * dt  # add this loop's error to the integral
            motor[side].value = max(-BRAKE, min(MAX_OUT, out))   # small reverse = brake

        print(f"C={c:5d}  pos={pos:4.2f}  {mode:8s}  target L={target['left']:5.1f} R={target['right']:5.1f}  "
              f"speed L={speed['left']:5.1f} R={speed['right']:5.1f} rpm  "
              f"sensor={v:.2f} V ({volts_to_cm_text(v)})")   # one status line per loop
except KeyboardInterrupt:                     # Ctrl+C pressed
    pass                                      # nothing to do here; cleanup is in finally
finally:                                      # always runs, even after an error
    stop_motors()                             # stop both motors
    slp.off()                                 # put the motor driver back to sleep
    encoders.close()                          # stop the decoder so it frees the encoder pins
    sensor.close()                            # release the ADC
    print("Stopped.")                         # confirm a clean shutdown
