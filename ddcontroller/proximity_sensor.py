#!/usr/bin/env python

import VL53L1X
import time
from datetime import datetime
import threading
import queue

class ProximitySensor:
    """
    An interface class for the proximity sensor.

    ...

    Attributes:

    """

    def __init__(self, bus=5, address=0x29, roi='wide',interval=0.1):
        """
        Args:
            bus: int, optional
                i2c bus number (e.g. /dev/i2c-5). Default=5
            address: int, option
                i2c address of sensor. Default=0x29
            roi: str, optional
                region of interest for sensor
                wide: 30 degrees
                narrow: 15 degrees
            interval: int, optional
                Time in seconds between distance measurements
                Default: 0.1
        """

        self.tof = VL53L1X.VL53L1X(i2c_bus=bus, i2c_address=address)
        self.tof.open()

        # Left, right, top and bottom are relative to the SPAD matrix coordinates,
        # which will be mirrored in real scene coordinates.
        # (or even rotated, depending on the VM53L1X element alignment on the board and on the board position)
        #
        # ROI in SPAD matrix coords:
        #
        # 15  top-left
        # |  X____
        # |  |    |
        # |  |____X
        # |        bottom-right
        # 0__________15
        #


        if roi == 'wide':
            # Wide scan forward ~30deg angle
            print("Scan: wide")
            self.tof.set_user_roi(VL53L1X.VL53L1xUserRoi(0, 15, 15, 0))
        elif roi == "narrow":
            # Focused scan forward
            print("Scan: narrow")
            self.tof.set_user_roi(VL53L1X.VL53L1xUserRoi(6, 9, 9, 6))
        else:
            print("Scan: wide (default)")
            self.tof.set_user_roi(VL53L1X.VL53L1xUserRoi(0, 15, 15, 0))

        self.time_budget_microseconds = (int)(interval*.8*1e6)
        self.intermeasurement_period_ms = (int)(interval*.9*1e3)
        #self.tof.set_timing(self.time_budget_microseconds, self.intermeasurement_period_ms)
        self.tof.set_timing(66000, 70)
        self.tof.set_distance_mode(2)
        self.tof.start_ranging(0)
        self.sensor_thread = None
        self.running = False

        self._wait = (
            interval
        )  # corrected wait time between measurements (s)

        # Get first distance
        self.distance = self._get_distance()

    def start(self):
        """Starts the sensor loop

        Args:
            start_time (int): The starting timestamp in nanoseconds.
        """
        self.running = False

        self.sensor_thread = threading.Thread(
            target=self._sensor_loop
        )  # create sensor thread object      

        self.sensor_thread.start()

    def sleep(self, start_time):
        """Sleep for a specified amount of time.

        This method calculates the amount of time that has passed since the start time
        and subtracts that from the wait time specified in the `self._wait` attribute.
        If the calculated sleep time is negative, it is set to 0 instead. The method
        then calls the built-in `time.sleep()` method to pause the program for the
        calculated amount of time.

        Args:
            start_time (int): The starting timestamp in nanoseconds.

        Returns:
            float: The amount of time slept in seconds.
        """
        sleep_time = sorted(
            [self._wait - ((time.monotonic_ns() - start_time) / 1e9), 0]
        )[1]

        time.sleep(sleep_time)

        return sleep_time

    def _sensor_loop(self):
        
        self.running = True

        while (self.running):
            start_time = time.monotonic_ns()  # record loop start time

            # Reading sensor data takes between 5-6ms
            self.distance = self._get_distance()
            #print(f"Sensor time ms: {(time.monotonic_ns()-start_time)/1e6}")

            self.sleep(start_time)

            self.update_frequency = 1000/((time.monotonic_ns()-start_time)/1e6)
    
    def stop(self):

        """Stop the sensor.

        This method stops the robot by setting the motion to 0, stopping the control threads, and stopping the wheels.
        """
        if self.sensor_thread:
            print("Terminate sensor thread")
            self.running = False
            self.sensor_thread.join()
            self.sensor_thread = None
            print("Sensor thread terminated")

    def _get_distance(self):
        """
        Gets distance measurement from sensor

        Args: None

        Return: Distance in mm (float)
        """
        return self.tof.get_distance()

    def get_distance(self):
        """
        Gets distance measurement from sensor

        Args: None

        Return: Distance in mm (float)
        """
        return self.distance
    

    def __del__(self):

        # terminate the sensor thread if it is running
        print("ProximitySensor: delete")
        if (self.sensor_thread):
            self.stop()

        # Turn off the sensor
        self.tof.stop_ranging()
        self.tof.close()

if __name__ == "__main__":
    prox = ProximitySensor(interval=0.5)
    prox.start()
    try:
        while True:
            distance_mm = prox.get_distance()
            if distance_mm < 0:
                # Error -1185 may occur if you didn't stop ranging in a previous test
                print("Error: {}".format(distance_mm))
            else:
                print("Distance: {}cm".format(distance_mm/10))

            time.sleep(1)

    except KeyboardInterrupt:
        prox.stop()

