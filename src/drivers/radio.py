import Adafruit_BBIO.GPIO as GPIO
import serial

class RadioResponseBad(Exception): pass

class Radio:

    def __init__(cls, serial_port, m0_pin, m1_pin, aux_pin):
        # Connect to Radio Via UART
        cls.serial_port = serial.Serial(serial_port, baudrate=9600, timeout=3)
        
        GPIO.setup(m0_pin, GPIO.OUT)
        cls.m0_pin = m0_pin

        GPIO.setup(m1_pin, GPIO.OUT)
        cls.m1_pin = m1_pin

        GPIO.setup(aux_pin, GPIO.IN)
        cls.aux_pin = aux_pin
        
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
        """
        self.serial_port.write(b'\xC3\xC3\xC3')
        radio_resp = self.serial_port.read(4)

        # Asserts that radio is a 433MHz model and 
        # receieved correct amount of data
        
        if (not radio_resp[0] == 0xc3):
            raise RadioResponseBad("First byte is not 0xC3! Resp: " + str(radio_resp))
        
        if (not radio_resp[1] == 0x32):
            raise RadioResponseBad("Second byte is not 0x32! Resp: " + str(radio_resp))
        
        if (not len(radio_resp) == 4):
            raise RadioResponseBad("Radio Respone is not 4 bytes long! Resp: " + str(radio_resp))
        
        return (radio_resp[2],radio_resp[3]) 
        

    def (self):
       return self.serial_port.in_waiting

    def _block_until_module_free(self):
        while not GPIO.input(self.aux_pin):
            pass # Block until Aux is 1
    
    def transmit(data: bytes):
        pass
    
    
    def bytes_waiting()