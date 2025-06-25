
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
        self.lat = []
        self.lng = []
        self.altcm = []
        self.velnorth = []
        self.veleast = []
        self.veldown = []

        ## AHRS Group
        self.ah_roll = []
        self.ah_pitch = []
        self.ah_yaw =[]

        self.acc_x = []
        self.acc_y = []
        self.acc_z = []

        self.ah_error_rp = []
        self.ah_error_y = []

        self.sim_roll = []
        self.sim_pitch = []
        self.sim_yaw = []

        ## INAV Group
        self.pos_x = []
        self.pos_y = []
        self.pos_z = []
        self.vel_x = []
        self.vel_y = []
        self.vel_z = []

        ## TODO Move simulated readings to their own packet type
        self.sim_px = []
        self.sim_py = []
        self.sim_pz = []
        self.sim_vx = []
        self.sim_vy = []
        self.sim_vz = []

        ## Motor Group
        self.mot1 = []
        self.mot2 = []
        self.mot3 = []
        self.mot4 = []

        ## MPC Group
        self.control_thrust = []
        self.control_rollt = []
        self.control_pitcht = []
        self.control_yawt = []
        self.p0 = []
        self.p1 = []
        self.p2 = []
        self.p3 = []


    ## Parse Methods
    def parse_imu(self, ir, vals):
        if len(vals)<6:
            return

        self.gyro_x.append((ir,float(vals[0])))
        self.gyro_y.append((ir,float(vals[1])))
        self.gyro_z.append((ir,float(vals[2])))
        self.accel_x.append((ir,float(vals[3])))
        self.accel_y.append((ir,float(vals[4])))
        self.accel_z.append((ir,float(vals[5])))

    def parse_baro(self, ir, vals):
        if len(vals)<3:
            return

        self.temperature.append((ir, float(vals[0])))
        self.pressure.append((ir, float(vals[1])))
        self.altitude_calc.append((ir, float(vals[2])))

    def parse_compass(self, ir, vals):
        if len(vals)<3:
            return

        self.field_x.append((ir, float(vals[0])))
        self.field_y.append((ir, float(vals[1])))
        self.field_z.append((ir, float(vals[2])))

    def parse_gps(self, ir, vals):
        if len(vals)<6:
            return

        self.lat.append((ir, float(vals[0])))
        self.lng.append((ir, float(vals[1])))
        self.altcm.append((ir, float(vals[2])))
        self.velnorth.append((ir, float(vals[3])))
        self.veleast.append((ir, float(vals[4])))
        self.veldown.append((ir, float(vals[5])))

    def parse_ahrs(self, ir, vals):

        if len(vals)<11:
            return

        self.ah_roll.append((ir, float(vals[0])))
        self.ah_pitch.append((ir, float(vals[1])))
        self.ah_yaw.append((ir, float(vals[2])))

        self.acc_x.append((ir, float(vals[3])))
        self.acc_y.append((ir, float(vals[4])))
        self.acc_z.append((ir, float(vals[5])))

        self.ah_error_rp.append((ir, float(vals[6])))
        self.ah_error_y.append((ir, float(vals[7])))

        self.sim_roll.append((ir, float(vals[8])))
        self.sim_pitch.append((ir, float(vals[9])))
        self.sim_yaw.append((ir, float(vals[10])))

    def parse_inav(self, ir, vals):
        if (len(vals)<12):
            return

        self.pos_x.append((ir, float(vals[0])))
        self.pos_y.append((ir, float(vals[1])))
        self.pos_z.append((ir, float(vals[2])))
        self.sim_px.append((ir, float(vals[3])))
        self.sim_py.append((ir, float(vals[4])))
        self.sim_pz.append((ir, float(vals[5])))

        self.vel_x.append((ir, float(vals[6])))
        self.vel_y.append((ir, float(vals[7])))
        self.vel_z.append((ir, float(vals[8])))
        self.sim_vx.append((ir, float(vals[9])))
        self.sim_vy.append((ir, float(vals[10])))
        self.sim_vz.append((ir, float(vals[11])))


    def parse_motor(self, ir, vals):
        if len(vals)<4:
            return

        self.mot1.append((ir, int(vals[0])))
        self.mot2.append((ir, int(vals[1])))
        self.mot3.append((ir, int(vals[2])))
        self.mot4.append((ir, int(vals[3])))

    def parse_mpc(self, ir, vals):
        if len(vals)<8:
            return

        self.control_thrust.append((ir, float(vals[0])))
        self.control_rollt.append((ir, float(vals[1])))
        self.control_pitcht.append((ir, float(vals[2])))
        self.control_yawt.append((ir, float(vals[3])))
        self.p0.append((ir, float(vals[4])))
        self.p1.append((ir, float(vals[5])))
        self.p2.append((ir, float(vals[6])))
        self.p3.append((ir, float(vals[7])))

    def parse(self, fpath):
        with open(fpath, 'r') as rfile:
            tgt = rfile.readlines()

        for l in tgt:
            splits = l.split(",")
            if splits[0]=='baro':
                self.parse_baro(int(splits[1]),splits[2:])
            if splits[0]=='imu':
                self.parse_imu(int(splits[1]),splits[2:])
            if splits[0]=='comp':
                self.parse_compass(int(splits[1]),splits[2:])
            if splits[0]=='gps':
                self.parse_gps(int(splits[1]),splits[2:])
            if splits[0]=='ah':
                self.parse_ahrs(int(splits[1]),splits[2:])
            if splits[0]=='inav':
                self.parse_inav(int(splits[1]),splits[2:])
            if splits[0]=='mpc':
                self.parse_mpc(int(splits[1]),splits[2:])


