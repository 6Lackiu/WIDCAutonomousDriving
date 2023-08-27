import time
import numpy as np
import ADCPlatform

speedPidThread_1 = 10  # 控制阈值1
speedPidThread_2 = 2  # 控制阈值2


# 转向角控制器
def latitudeControlpos(positionnow, latPid):
    latPid.update(positionnow + 6.5)
    latPid.steer_ = latPid.output * -1        # ori: -1

    THRE_STEER = 60

    if abs(latPid.steer_) > THRE_STEER:
        latPid.steer_ = THRE_STEER if latPid.steer_ > 0 else -THRE_STEER


def lontitudeControlSpeed(speed, lonPid):
    # 自车当前绝对车速，
    lonPid.update(speed - 5)
    if (lonPid.output > speedPidThread_1):  # 加速阶段 >10

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 1')
        # print("==============================================")
        lonPid.thorro_ = 0.8
        lonPid.brake_ = 0

    elif (lonPid.output > speedPidThread_2):  # 稳定控速阶段 >2

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 2')
        # print("==============================================")

        lonPid.thorro_ = min((lonPid.output / speedPidThread_1) * 0.85, 1.0)
        lonPid.brake_ = min(((speedPidThread_1 - lonPid.output) / speedPidThread_1) * 0.1, 1.0)

    elif (lonPid.output > 0):  # 下侧 微调  >0

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 3')
        # print("==============================================")
        lonPid.thorro_ = (lonPid.output / speedPidThread_2) * 0.3
        lonPid.brake_ = ((speedPidThread_2 - lonPid.output) / speedPidThread_2) * 0.2

    elif (lonPid.output < -1 * speedPidThread_1):  # 减速阶段 <-10

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 4')
        # print("==============================================")
        lonPid.thorro_ = (-1 * lonPid.output / 5) * 0.1
        lonPid.brake_ = 0.4


    else:  # <-2
        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 5')
        # print("==============================================")
        lonPid.thorro_ = (-1 * lonPid.output / speedPidThread_2) * 0.2
        lonPid.brake_ = ((speedPidThread_2 - (-1 * lonPid.output)) / speedPidThread_2) * 0.4
    # print(lonPid.thorro_, '    ', lonPid.brake_)


def getTTC(current_speed, current_acceleration, dist, safe_dist):
    v = current_speed
    a = current_acceleration

    if a != 0:
        ttc = (-v + np.sqrt(v ** 2 + 2 * a * (dist - safe_dist))) / a
    else:
        ttc = 999

    return ttc


def run_task2_test(myCar, Controller):
    # 期望速度
    Controller.speedPid.setSetpoint(18)  # 22
    # Controller.speedPid.setSetpoint(myCar.speed + myCar.delta_v + 0)

    # 纵向控制 thorro_ and brake_
    # lontitudeControlSpeed(myCar.speed, Controller.speedPid, 28)

    # 横向不需要控制

    Controller.latPid.steer_ = 0

    # print("spd:", myCar.speed)
    # print("position:", myCar.positionnow)

    # dt = 0.258
    # a = np.round(myCar.speed / dt, 3)

    if myCar.delta_v != 0:
        delata_dt = np.round(myCar.dist / myCar.delta_v, 3)
    else:
        delata_dt = 999

    delta_a = np.round(myCar.delta_v / delata_dt, 3)
    # print("delata_dt:", -delata_dt,"delta_a:",delta_a)
    ttc = getTTC(myCar.speed, delta_a, myCar.dist, safe_dist=0.5)

    if myCar.dist > 70:

        # Controller.speedPid.setSetpoint(Controller.speeduplimit)
        # lontitudeControlSpeed(myCar.speed, Controller.speedPid)

        # throttle, steering, brake, gear

        Controller.speedPid.setSetpoint(18)  # 20
        # Controller.speedPid.setSetpoint(myCar.speed + myCar.delta_v + 0)
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif 45 < myCar.dist <= 70:
        # Controller.speedPid.setSetpoint(32)
        Controller.speedPid.setSetpoint(myCar.speed + myCar.delta_v + 0.4)  # 0.6
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif 35 < myCar.dist <= 45:
        # Controller.speedPid.setSetpoint(32)
        Controller.speedPid.setSetpoint(myCar.speed + myCar.delta_v + 0.1)  # 0.3
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif 29.8 < myCar.dist <= 35:
        # Controller.speedPid.setSetpoint(31)
        Controller.speedPid.setSetpoint(myCar.speed + myCar.delta_v + 0)  # 0.2
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif 27 < myCar.dist <= 29.8:
        # Controller.speedPid.setSetpoint(30)
        Controller.speedPid.setSetpoint(myCar.speed + myCar.delta_v - 1)    # 0
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif 23 < myCar.dist <= 27:
        # Controller.speedPid.setSetpoint(29.5)
        Controller.speedPid.setSetpoint(myCar.speed + myCar.delta_v - 1.5)    # -1
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif 20 < myCar.dist <= 23:
        # Controller.speedPid.setSetpoint(29)
        Controller.speedPid.setSetpoint(myCar.speed + myCar.delta_v - 2)    # -2
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    elif myCar.dist <= 20:
        # Controller.speedPid.setSetpoint(10)
        Controller.speedPid.setSetpoint(myCar.speed + myCar.delta_v - 5)    # -5
        lontitudeControlSpeed(myCar.speed, Controller.speedPid)
        latitudeControlpos(myCar.positionnow, Controller.latPid)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, Controller.speedPid.brake_, 1)

    # print(time.time())
    print("相对距离:'{}'".format(myCar.dist, 2)," \t绝对速度：'{}': ".format(myCar.speed))
    print("相对速度：", myCar.delta_v)

    # print("绝对加速度：'{}'".format(a,3)))

    print("-----------------------------------------------------------------------------")
