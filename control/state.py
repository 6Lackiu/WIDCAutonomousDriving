from control import pid


class LaneState(object):
    """
    The lane state class using lateral control
    A target number of driving lane, which is the sum of two dashed lane line.

    Attributes:
        LEFT: left lane target, float type
        MID: middle lane target, float type
        RIGHT: right lane target, float type
    """

    def __init__(self, l=7.0, m=0.0, r=-7.0):
        """ Initializing three lane target number """
        self.LEFT = l
        self.MID = m
        self.RIGHT = r

    def set_leftlane(self, l):
        self.LEFT = l

    def set_midlane(self, m):
        self.MID = m

    def set_rightlane(self, r):
        self.RIGHT = r


class CarState(object):
    def __init__(self):
        self.speed = 0  # 车辆当前速度
        self.cao = 0  # 车辆当前姿态
        # self.midlane = -1  # 7 0 -7 latpid 参考 target

        self.positionnow = 0  # 两车道线A1求和

        self.delta_v = 0
        self.acc = 0
        self.dist = 10  # 与前车的实际距离

        self.yr = 0  # 车辆当前姿态
        self.speed_2 = 40

        # Add for task4
        self.cardecision = 'speedup'            # planning计算得到决策
        self.changing = False                   # 处于超车状态时为True
        self.lanestate = LaneState(6.7, 0, -6.7)    # 左中右车道线目标位置 6.6 6.7
        self.midlane = self.lanestate.MID       # 7.5 0 -8 latpid 参考 target
        self.lanefuture = 2.0                   # 车道线 x = 2 处的位置  原4.0
        self.finalflag = False                  # 超级加速阶段回到中间车道标志位
        self.finalcount = 0
        self.direction = 'mid'                  # 当前行驶方向
        self.time = 0                           # 超级加速阶段计时
        self.saftydistance = 25                 # 与前车的安全距离
        self.overtakeSum = 0                    # 超车计数


class ControlData(object):
    def __init__(self):
        """
        self.lat_kp = 1.10
        self.lat_ki = 0.08
        self.lat_kd = 6.2
        """
        # 横向
        # self.lat_kp = 1.35
        # self.lat_ki = 0.06
        # self.lat_kd = 10.2
        # self.latPid = pid.PID(self.lat_kp, self.lat_ki, self.lat_kd)
        self.lat_kp = 2.10
        self.lat_ki = 0.07
        self.lat_kd = 6.96
        self.latPid = pid.PID(self.lat_kp, self.lat_ki, self.lat_kd)

        # 角速度
        self.yr_kp = 1.0
        self.yr_ki = 0.10
        self.yr_kd = 0
        self.yrPid = pid.PID(self.yr_kp, self.yr_ki, self.yr_kd)

        # 速度
        self.targetSpeedInit = 40.0  # 想要到达的速度
        self.speed_kp = 1.20
        self.speed_ki = 0.02
        self.speed_kd = 0.5
        self.speedPid = pid.PID(self.speed_kp, 0, self.speed_kp)
        self.speedPidThread_1 = 10
        self.speedPidThread_2 = 2

        # Add for task4
        self.speeduplimit = 74             # 加速阶段控制速度   85  74
        self.superspeeduplimit = 75      # 超级加速阶段控制速度   80
        self.superspeeduplimittime = 25      # 超级加速阶段计时阈值   25
        self.followlimit = 70                # 跟车阶段控制速度     4
        self.saftylimit = 60
        self.overtakelimit = 71             # 超车阶段控制速度     75  71
        self.finalspeed = 101               # 最后阶段           100



    def initPID(self):
        self.speedPid.clear()  # lon 纵向
        self.latPid.clear()  # lat 横向
        self.yrPid.clear()  # lat
        self.speedPid.setSetpoint(self.targetSpeedInit)  # 保持40km/h
        self.latPid.setSetpoint(0)  # lat aim 0
        self.yrPid.setSetpoint(0)  # lat aim 0
