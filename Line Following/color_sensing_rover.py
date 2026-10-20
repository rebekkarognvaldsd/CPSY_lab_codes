import time

import board
import busio
import adafruit_tcs34725


class ColorSensor:
    """Wraps the TCS34725 color sensor."""

    def __init__(self, i2c):
        self.sensor = adafruit_tcs34725.TCS34725(i2c)
        self.sensor.integration_time = 50
        self.sensor.gain = 4

    def read_color(self):
        """Return a single (r, g, b) reading, each 0-255."""
        r, g, b = self.sensor.color_rgb_bytes
        return r, g, b


class ColorSensingApp:

    def __init__(self, sensor, interval=1.0):
        self.sensor = sensor
        self.interval = interval

    def run(self):
        try:
            while True:
                r, g, b = self.sensor.read_color()
                if r > g and r > b:
                    name = "red"
                elif g > r and g > b:
                    name = "green"
                elif b > r and b > g:
                    name = "blue"
                else:
                    name = "unknown"
                print(f"R={r} G={g} B={b} {name}")
                time.sleep(self.interval)
        except KeyboardInterrupt:
            print("Stopped by user")


def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = ColorSensor(i2c)
    app = ColorSensingApp(sensor, interval=1.0)
    app.run()


if __name__ == "__main__":
    main()