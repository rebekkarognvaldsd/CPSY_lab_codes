import math
import time

import board
import busio
import adafruit_icm20x
import digitalio

class IMUSensor:

    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.icm20948 = adafruit_icm20x.ICM20948(self.i2c, address=0x68)

    def read_acceleration(self):
        return self.icm20948.acceleration

    def read_mag_field(self):
        return self.icm20948.magnetic

class LevelTracker:

    def __init__(self):
        self.LEVEL_TOL_DEG = 5.0

    def is_level(self, accel):
        ax, ay, az = accel

        tilt_rad = math.atan2(math.sqrt(ax * ax + ay * ay), abs(az))
        tilt_deg = math.degrees(tilt_rad)
        return tilt_deg < self.LEVEL_TOL_DEG

class CompassDirection:

    def __init__(self):
        self.DIR_TOL_DEG = 10.0

    def compute_direction_deg(self, mag, accel):
        mx, my, mz = mag
        ax, ay, az = accel

        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        roll = math.atan2(ay, az)

        mx_comp = mx * math.cos(pitch) + mz * math.sin(pitch)
        my_comp = (mx * math.sin(roll) * math.sin(pitch) + my * math.cos(roll) - mz * math.sin(roll) * math.cos(pitch))

        direction_rad = math.atan2(-my_comp, mx_comp)
        direction_deg = math.degrees(direction_rad)
        print(
        f"accel (m/s^2): x={ax:7.3f}  y={ay:7.3f}  z={az:7.3f}   |   mag (uT): x={mx:7.2f}  y={my:7.2f}  z={mz:7.2f}"
        )

        return direction_deg % 360

    def is_north(self, accel, mag):
        direction = self.compute_direction_deg(mag, accel)

        distance_to_north = min(direction, 360 - direction)
        return distance_to_north < self.DIR_TOL_DEG


class LEDIndicator:

    def __init__(self, pin):
        self._led = digitalio.DigitalInOut(pin)
        self._led.direction = digitalio.Direction.OUTPUT

    def on(self):
        self._led.value = True

    def off(self):
        self._led.value = False




class LabController:

    POLL_INTERVAL_S = 0.2

    def __init__(self):
        self.imu = IMUSensor()
        self.compass = CompassDirection()
        self.level_tracker = LevelTracker()
        self.north_led = LEDIndicator(board.D17)
        self.level_led = LEDIndicator(board.D27)

    def update_led(self):
        accel = self.imu.read_acceleration()
        mag = self.imu.read_mag_field()

        if self.compass.is_north(mag, accel):
            self.north_led.on()
        else:
            self.north_led.off()

        if self.level_tracker.is_level(accel):
            self.level_led.on()
        else:
            self.level_led.off()

    def run(self):
        while True:
            self.update_led()
            time.sleep(self.POLL_INTERVAL_S)


if __name__ == "__main__":
    controller = LabController()
    controller.run()