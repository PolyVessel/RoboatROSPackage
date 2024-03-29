try:
    #board is beaglebone
    import Adafruit_BBIO.GPIO as GPIO
except ImportError:
    #board is pi
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)

import time
import serial

class RadioResponseBad(Exception): pass

TRANSMIT_CH = 0

class Radio:

    def __init__(self, serial_port, m0_pin, m1_pin, aux_pin):
        # Connect to Radio Via UART
        self.serial_port = serial.Serial(serial_port, baudrate=9600, timeout=3, write_timeout=3)
        
        num = 0

        while True:
            num += 1
            print(f"Lora Radio GPIO init try #{num}")
           
            try:
                GPIO.setup(m0_pin, GPIO.OUT)
                self.m0_pin = m0_pin

                GPIO.setup(m1_pin, GPIO.OUT)
                self.m1_pin = m1_pin

                GPIO.setup(aux_pin, GPIO.IN)
                self.aux_pin = aux_pin

            except ValueError as e:
                print(e)
                time.sleep(1)
                continue

            break

        
        print(f"Sucessfully initialized all GPIO pins for radio!")
        
    def __del__(self):
        """Closes serial port when done"""
        try:
            self.serial_port.close()
            GPIO.cleanup()
        except Exception as e:
            pass #Expected for Unit tests

    def _normal_mode(self):
        """Will block until module is free and can swap the mode"""

        self._block_until_module_free()
        GPIO.output(self.m0_pin, GPIO.LOW)
        GPIO.output(self.m1_pin, GPIO.LOW)
    
    def _wake_up_mode(self):
        """Will block until module is free and can swap the mode
        
        Note: This mode will transmit a wake-up packet, so even if
        reciever is in power-saving mode, it can still hear it
        """

        self._block_until_module_free()
        GPIO.output(self.m0_pin, GPIO.HIGH)
        GPIO.output(self.m1_pin, GPIO.LOW)

    def _power_saving_mode(self):
        """Will block until module is free and can swap the mode
        
        Can't transmit and will only listen to transmissions from
        a transmitter in wake-up mode.
        """

        self._block_until_module_free()
        GPIO.output(self.m0_pin, GPIO.LOW)
        GPIO.output(self.m1_pin, GPIO.HIGH)

    def _sleep_mode(self):
        """Will block until module is free and can swap the mode"""

        self._block_until_module_free()
        GPIO.output(self.m0_pin, GPIO.LOW)
        GPIO.output(self.m1_pin, GPIO.HIGH)

    def ping_radio(self):
        """Requests version number of radio
        Will return a byte-string tuple, 
            ('\x00','\x00') - Indicates error
            Otherwise, first value is Version number and 
            second value is other module features.

        Can raise TimeoutException()
        """

        # Requires Sleep mode in order to ping radio
        # (For now don't deal with changing modes)
        # with time_limit(3):
        #     self._sleep_mode()


        self.serial_port.write(b'\xC3\xC3\xC3')
        radio_resp = self.serial_port.read(4)

        if len(radio_resp) != 4:
            raise RadioResponseBad(f"Did not return correct data length! Response: {radio_resp}")

        print(f"Radio Ping response {radio_resp} with length {len(radio_resp)}")

        # Asserts that radio is a 433MHz model and 
        # receieved correct amount of data
        
        if (not radio_resp[0] == 0xc3):
            raise RadioResponseBad("First byte is not 0xC3! Resp: " + str(radio_resp))
        
        if (not radio_resp[1] == 0x32):
            raise RadioResponseBad("Second byte is not 0x32! Resp: " + str(radio_resp))
        
        if (not len(radio_resp) == 4):
            raise RadioResponseBad("Radio Respone is not 4 bytes long! Resp: " + str(radio_resp))
        
        return (radio_resp[2],radio_resp[3])
    
    def receive(self):
        serial_resp = self.serial_port.read(self.serial_port.in_waiting)
        return serial_resp

    def _block_until_module_free(self):
        while not GPIO.input(self.aux_pin):
            pass # Block until Aux is 1
    
    def transmit(self, data: bytes):
        if len(data) > 512:
            raise ValueError("Data too long to transmit!")
        buf = bytearray(b'\xff' * 2)
        buf.extend(data)
        self.serial_port.write(buf)

    def bytes_waiting():
        return serial.inWaiting()