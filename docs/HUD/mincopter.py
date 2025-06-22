
## Stored MinCopter object

class MinCopter:

    def __init__(self):

        ## IMU Group
        self.accel_x = []
        self.accel_y = []
        self.accel_z = []
        self.gyro_x = []
        self.gyro_y = []
        self.gyro_z = []

        ## Baro Group
        self.temperature = []
        self.pressure = []
        self.altitude_calc = []

        ## Compass Group
        self.field_x = []
        self.field_y = []
        self.field_z = []

        ## GPS Group
        self.lat


    def parse(self, fpath):
        pass


    ## Parse Methods
    def parse_imu(self, vals):
        if len(vals)<6:
            return

        self.gyro_x.append(float(vals[0]))
        self.gyro_y.append(float(vals[1]))
        self.gyro_z.append(float(vals[2]))
        self.accel_x.append(float(vals[3]))
        self.accel_y.append(float(vals[4]))
        self.accel_z.append(float(vals[5]))

    def parse_baro(self, vals):
        if len(vals)<3:
            return

        self.temperature.append(float(vals[0]))
        self.pressure.append(float(vals[1]))
        self.altitude_calc.append(float(vals[2]))

    def parse_compass(self, vals):
        if len(vals)<3:
            return

        self.field_x.append(float(vals[0]))
        self.field_y.append(float(vals[1]))
        self.field_z.append(float(vals[2]))

    def parse_xx
