import typing

class IMUException(Exception):
    pass

class Rotor:

    initialized = False

    def __init__(self, channel: int):
        self.channel = channel

    def init(self):
        import rcpy.servo as servo
        import rcpy.clock as clock
        import rcpy

        """Initializes the IMU. Is usually done automatically from poll_sensor()"""
        rcpy.set_state(rcpy.RUNNING)
        self.servo = servo.Servo(self.channel)
        self.clock = clock.Clock(self.servo, 0.02)
        
        self.initialized = True


    def set(self, duty: float):
        self.servo.enable()
        self.clock.start()

        self.servo.set(duty)

    def off(self):
        self.clock.stop()
        self.servo.disable()

