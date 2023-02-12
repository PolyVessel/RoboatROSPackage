import typing
import time

class IMUException(Exception):
    pass

class Rotor:

    initialized = False

    def __init__(self, channel: int):
        self.channel = channel
        self.init()

    def init(self):
        from rcpy.servo import ESC
        import rcpy.servo as servo
        import rcpy

        # Disable the power rail on the servos channels
        # for safety
        servo.disable()

        rcpy.set_state(rcpy.RUNNING)
        self.esc = ESC(self.channel)

        # Wait for PRU Init
        time.sleep(1)

        # Start the Clock for the escs
        self.esc.start(0.02)

        # Wait for PRU Init
        time.sleep(1)

        # Arm ESC
        self.set(-0.1)
        
        # Blocking call, but in the future should probably use coroutines
        time.sleep(1)

        self.esc.set(0)


    def set(self, duty: float):
        self.esc.start(0.02)
        self.esc.set(duty)

    def off(self):
        self.esc.set(0)
    
    def __del__(self):
        self.off()
