import ADCPlatform
import numpy as np

speedPidThread_1 = 10  # 控制阈值1
speedPidThread_2 = 2  # 控制阈值2


# 角速度控制器
def latitudeyrControlpos(yr, yrPid):
    yrPid.update(yr)
    yrPid.yrsteer_ = yrPid.output * -1


# 转向角控制器
def latitudeControlpos(positionnow, latPid):
    latPid.update(positionnow + 2.5)
    latPid.steer_ = latPid.output * -0.6        # ori: -1

    THRE_STEER = 60

    if abs(latPid.steer_) > THRE_STEER:
        latPid.steer_ = THRE_STEER if latPid.steer_ > 0 else -THRE_STEER


def lontitudeControlSpeed(speed, lonPid):
    lonPid.update(speed - 16.0)      # ori: -5 -16
    if (lonPid.output > speedPidThread_1):  # 加速阶段

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 1')
        # print("==============================================")
        lonPid.thorro_ = 1
        lonPid.brake_ = 0

    elif (lonPid.output > speedPidThread_2):  # 稳定控速阶段

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 2')
        # print("==============================================")

        lonPid.thorro_ = min((lonPid.output / speedPidThread_1) * 0.85, 1.0)
        lonPid.brake_ = min(((speedPidThread_1 - lonPid.output) / speedPidThread_1) * 0.1, 1.0)

    elif (lonPid.output > 0):  # 下侧 微调

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 3')
        # print("==============================================")
        lonPid.thorro_ = (lonPid.output / speedPidThread_2) * 0.3
        lonPid.brake_ = ((speedPidThread_2 - lonPid.output) / speedPidThread_2) * 0.2

    elif (lonPid.output < -1 * speedPidThread_1):  # 减速阶段

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 4')
        # print("==============================================")
        lonPid.thorro_ = (-1 * lonPid.output / 5) * 0.1
        lonPid.brake_ = 0.5

    else:
        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 5')
        # print("==============================================")
        lonPid.thorro_ = (-1 * lonPid.output / speedPidThread_2) * 0.2
        lonPid.brake_ = ((speedPidThread_2 - (-1 * lonPid.output)) / speedPidThread_2) * 0.4
    # print(lonPid.thorro_, '    ', lonPid.brake_)

# 这种情况的速度控制为25

def getTTC(current_speed, current_acceleration, dist, safe_dist):

    v = current_speed
    a = current_acceleration

    if a !=0:
        ttc = ( -v + np.sqrt(v **2 + 2 * a * (dist-safe_dist)) ) / a
    else:
        ttc = 999

    return ttc


def FindMidCar(myCar, radar_data_package):
    if radar_data_package is not None:
        print("All vehicle info：\n", radar_data_package.json)
        left_dis = mid_dis = right_dis = float('inf')
        # left_dis = mid_dis = right_dis = 9999
        for car in radar_data_package.json:
            car['Range'] /= 100     # cm to m
            if -90 < car['Angle'] < -8:
                if car['Range'] < left_dis:
                    left_dis = car['Range']
            elif -8 <= car['Angle'] <= 8:
                if car['Range'] < mid_dis:
                    mid_dis = car['Range']
            elif 8 < car['Angle'] < 90:
                if car['Range'] < right_dis:
                    right_dis = car['Range']

        return [left_dis, mid_dis, right_dis]


def run_task3_test(myCar, Controller, radar_data_package):
    Controller.speedPid.setSetpoint(32)
    _, mid_dis, _ = FindMidCar(myCar, radar_data_package)
    myCar.dist = mid_dis

    if myCar.dist > 70:
        Controller.speedPid.setSetpoint(40)
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif 45 < myCar.dist <= 70:
        Controller.speedPid.setSetpoint(30)
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif 36 < myCar.dist <= 45:
        Controller.speedPid.setSetpoint(23)
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif 30.5 < myCar.dist <= 36:
        Controller.speedPid.setSetpoint(20)
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif 27 < myCar.dist <= 30.5:
        Controller.speedPid.setSetpoint(18.8)
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif 23 < myCar.dist <= 27:
        Controller.speedPid.setSetpoint(18.8)
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif 20 < myCar.dist <= 23:
        Controller.speedPid.setSetpoint(15)
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif myCar.dist <= 20:
        Controller.speedPid.setSetpoint(15)
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    print("自车速度：'{}': ".format(myCar.speed), "\t相对距离:'{}'".format(myCar.dist, 2))
    print("相对速度：", myCar.delta_v)
    print("-------------------------------------------------------------------")
