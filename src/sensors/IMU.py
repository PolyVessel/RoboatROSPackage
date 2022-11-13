class IMUException(Exception):
    pass

class IMU:

    initialized = False

    # Singleton Pattern (we only have 1 IMU)
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(IMU, cls).__new__(cls)
        return cls.instance

    def init(self):
        import rcpy
        import rcpy.mpu9250 as mpu9250
        
        self.mpu9250 = mpu9250

        """Initializes the IMU. Is usually done automatically from poll_sensor()"""
        rcpy.set_state(rcpy.RUNNING)
        mpu9250.initialize(enable_dmp = True, dmp_sample_rate = 4, enable_magnetometer=True)
        self.initialized = True


    def poll_sensor(self):
        
        """Polls the IMU for sensor data, initializing the IMU if needed.
        May Raise Critical Error:
            IMUException
        returns Dictionary:
            "heading" -> heading from magnetometer (radians)
            "accel"   -> 3-axis accelerations (m/s^2)
            "gyro"    -> 3-axis angular velocities (degree/s)
            "mag"     -> 3D magnetic field vector in (μT)
            "tb"      -> pitch/roll/yaw X/Y/Z angles (radians)
            "temp"    -> imu temperature in deg C
        """

        if not self.initialized:
            self.init()
        try:
            imu_data = self.mpu9250.read()
            temp = self.mpu9250.read_imu_temp()
        except self.mpu9250.error as e:
            # BETTER LOGGING
            raise IMUException()

        data = {
            "heading": imu_data["head"],
            "accel": imu_data["accel"],
            "gyro": imu_data["gyro"],
            "mag": imu_data["mag"],
            "tb": imu_data["tb"],
            "temp": temp,
        }

        return data