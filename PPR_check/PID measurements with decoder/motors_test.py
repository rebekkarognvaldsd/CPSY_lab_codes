#! /usr/bin/env python3
# Drive both wheels forward for RUN_TIME seconds and print the encoder steps.
# Usage:  python3 drive_test.py decoder     (C++ decoder)
#         python3 drive_test.py gpiozero    (gpiozero RotaryEncoder)
import sys, time
from gpiozero import DigitalOutputDevice, Motor

MODE = sys.argv[1] if len(sys.argv) > 1 else "decoder"
SPEED = 0.3       # motor output 0..1
RUN_TIME = 5      # seconds
PPR = 699         # steps per wheel revolution

if MODE == "decoder":
    from encoders import Encoders
    enc = Encoders()
    get = lambda: (enc.left, enc.right)
else:
    from gpiozero import RotaryEncoder
    l, r = RotaryEncoder(5, 6, max_steps=0), RotaryEncoder(17, 27, max_steps=0)
    get = lambda: (l.steps, r.steps)

slp = DigitalOutputDevice("GPIO26")
left, right = Motor("GPIO18", "GPIO12"), Motor("GPIO13", "GPIO19")
try:
    slp.on()
    left.value = right.value = SPEED
    start = time.time()
    while time.time() - start < RUN_TIME:
        print(f"Steps Left: {get()[0]}, Right: {get()[1]}")
        time.sleep(0.25)
finally:
    left.stop(); right.stop(); slp.off()
    time.sleep(0.5)                       # let the wheels coast to a stop
    L, R = get()
    print(f"FINAL ({MODE}): left = {L} ({L/PPR:.2f} rev)  right = {R} ({R/PPR:.2f} rev)")
    if MODE == "decoder":
        enc.close()