from re import sub
import subprocess
from datetime import datetime, timezone
from drivers.util import TimeoutException, time_limit
import os


class GPSNoSignal(Exception): pass

class GPSData():
    def __init__(self, current_time_utc, lon, lat, 
        headMot, numSV, gSpeed, sAcc, hAcc, headAcc):

        self.current_time_utc = current_time_utc;
        self.lon = lon
        self.lat = lat
        self.headMot = headMot
        self.numSV = numSV
        self.gSpeed = gSpeed
        self.sAcc = sAcc
        self.hAcc = hAcc
        self.headAcc = headAcc

class GPS:
    def __init__(self, gps, serial, timeout):
        self.timeout = timeout
        self.serial = serial
        self.gps = gps

    @classmethod
    def fromgpsconfig(cls, serial_port_name, timeout):
        with time_limit(timeout):
            from ublox_gps import UbloxGps
            import serial
            # Connect GPS module to GPS UART
            serial_port = serial.Serial(serial_port_name, baudrate=9600, timeout=timeout, stopbits=serial.STOPBITS_ONE,
                                            parity=serial.PARITY_NONE, bytesize=serial.EIGHTBITS)
            gps = UbloxGps(serial_port)

            return cls(gps, serial_port, timeout)


    def __del__(self):
        """Closes serial port when done"""
        try:
            self.serial_port.close()
        except Exception as e:
            pass #Expected for Unit tests


    def poll_sensor(self) -> GPSData:
        """Polls the GPS for sensor data, initializing the GPS if needed.
        More data in https://cdn.sparkfun.com/assets/0/9/4/3/5/u-blox8-M8_ReceiverDescrProtSpec__UBX-13003221__Public.pdf

        May raise Expected Exceptions: 
            TimeoutException, GPSNoSignal

        Also may raise critical errors (I don't know when these are called):
            ValueError, IOError

        Returns GPSData Object:
           lat     -> Latitude in Deg
           lon     -> Longitude in Deg
           current_time_utc -> Python datetime obj of current time in UTC
           headMot -> Heading of Motion in deg
           numSV   -> Number of Satellites used in Solution,
           gSpeed  -> Ground Speed in mm/s (2D)
           sAcc    -> Speed Accuracy in mm/s
           hAcc    -> Horizontal Accuracy in mm
           headAcc -> Accuracy of Heading in deg
        """

        with time_limit(self.timeout):
            gps_data = self.gps.geo_coords()

            current_date_time = datetime(gps_data.year, gps_data.month, gps_data.day,
                                        hour=gps_data.hour, minute=gps_data.min, second=gps_data.sec,
                                        tzinfo=timezone.utc)

            ret_data = GPSData (
                current_time_utc=current_date_time,
                lon=gps_data.lon,
                lat=gps_data.lat,
                headMot=gps_data.headMot,
                numSV=gps_data.numSV,
                gSpeed=gps_data.gSpeed,
                sAcc=gps_data.sAcc,
                hAcc=gps_data.hAcc,
                headAcc=gps_data.headAcc)

            if ret_data.numSV == 0:
                raise GPSNoSignal()
            else:
                return ret_data

    def update_system_clock(self):
        """Accurate to ~1s, because GPS Library only gives precision
        up to seconds.
        
        Requires root privledges."""

        try:
            gps_data = self.poll_sensor()
            time_data = gps_data.current_time_utc

            # convert time_data to a form the date -u command will accept: "20140401 17:32:04"
            gps_utc = "{:04d}{:02d}{:02d} {:02d}:{:02d}:{:02d}".format(time_data.year, time_data.month, time_data.day,
                                                                       time_data.hour, time_data.minute,
                                                                       time_data.second)


            # Checks if root, so we don't get a passwd prompt
            if os.geteuid() != 0:  # type: ignore
                raise subprocess.CalledProcessError(1, "sudo")

            subprocess.run(["sudo", "date", "-u", "--set={}".format(gps_utc)], timeout=2)


        # TODO: Better Logs here, this should be logged
        # Non critical, Expected Errors
        except TimeoutException as e:
            print("GPS Module Not Connected")
        except GPSNoSignal as e:
            print("No GPS Signal")
        except subprocess.CalledProcessError as e:
            print(e)
        except subprocess.TimeoutExpired as e:
            print(e)
        except ValueError as e:
            print(e)
        except IOError as e:
            print(e)