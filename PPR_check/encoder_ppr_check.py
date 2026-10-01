#! /usr/bin/env python3
import time
from gpiozero import RotaryEncoder

# Parameters
PPR_RIGHT = 1.0   # replace with the measured value
PPR_LEFT = 1.0    # replace with the measured value
TSTOP = 20        # run time (s)
TSAMPLE = 0.02    # sampling period (s); the sleep frees the CPU for encoder edges
TDISP = 0.5       # display period (s)

# max_steps=0: the count never wraps around
enc_right = RotaryEncoder(17, 27, max_steps=0)  # Motor A
enc_left = RotaryEncoder(5, 6, max_steps=0)     # Motor B

print(f"Running for {TSTOP} s. Turn each wheel one revolution FORWARD by hand.")
tstart = time.perf_counter()
tlast_disp = 0.0
try:
    while (tcurr := time.perf_counter() - tstart) <= TSTOP:
        time.sleep(TSAMPLE)
        if tcurr - tlast_disp >= TDISP:
            tlast_disp = tcurr
            r, l = enc_right.steps, enc_left.steps
            print(f"t={tcurr:5.1f} s | right: {r:6d} steps {360 * r / PPR_RIGHT:8.1f} deg"
                  f" | left: {l:6d} steps {360 * l / PPR_LEFT:8.1f} deg")
finally:
    enc_right.close()
    enc_left.close()
    print("Done.")