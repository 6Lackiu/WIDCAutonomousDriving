import time

import ADCPlatform
import numpy as np

speedPidThread_1 = 10  # 控制阈值1
speedPidThread_2 = 2  # 控制阈值2
"""

"""


def lontitudeControlSpeed(speed, lonPid):
    lonPid.update(speed - 5.0)
    if lonPid.output > speedPidThread_1:  # 加速阶段

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 1')
        # print("==============================================")
        lonPid.thorro_ = 1
        lonPid.brake_ = 0

    elif lonPid.output > speedPidThread_2:  # 稳定控速阶段

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 2')
        # print("==============================================")

        lonPid.thorro_ = min((lonPid.output / speedPidThread_1) * 0.85, 1.0)
        lonPid.brake_ = min(((speedPidThread_1 - lonPid.output) / speedPidThread_1) * 0.1, 1.0)

    elif lonPid.output > 0:  # 下侧 微调

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 3')
        # print("==============================================")
        lonPid.thorro_ = (lonPid.output / speedPidThread_2) * 0.3
        lonPid.brake_ = ((speedPidThread_2 - lonPid.output) / speedPidThread_2) * 0.2

    elif lonPid.output < -1 * speedPidThread_1:  # 减速阶段

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


def getTTC(current_speed, current_acceleration, dist, safe_dist):
    v = current_speed
    a = current_acceleration

    if a != 0:
        ttc = (-v + np.sqrt(v ** 2 + 2 * a * (dist - safe_dist))) / a
    else:
        ttc = 999

    return ttc


def set_state(myCar, Controller, control_data_package, radar_data_package, line_data_package):
    # 当前车速
    spd = control_data_package.json['FS']

    # 当前偏航角
    yaw = control_data_package.json['CAO']

    # 当前角速度
    yr = control_data_package.json['YR']

    if radar_data_package is not None:
        # 筛选出距离最近的车
        mindis_car = None
        MIN_DIS = 9999.99
        mindis_car = min(radar_data_package.json, key=lambda car: car['Range'])

        if mindis_car is not None:
            # v: cm/s
            # 相对前车的速度
            delta_spd = mindis_car["Speed"] / 100  # m/s
            print("与前车相对速度：", delta_spd)

            # 相对前车的距离
            dist = mindis_car["Range"] / 100  # m
            print("与前车相对距离：", dist)

            myCar.dist = dist

            myCar.delta_v = delta_spd

            print("min dis car:", mindis_car)

    if line_data_package is not None:

        if len(line_data_package.json) == 4:
            myCar.positionnow = -6.5 + (line_data_package.json[2]['A1'] + line_data_package.json[1]['A1'])
        else:
            print("ERROR")
            myCar.positionnow = -7.0

    # 保存当前的状态

    myCar.speed = spd
    myCar.cao = yaw
    myCar.yr = yr

    return myCar


def run_task0_test(myCar, Controller, control_data_package, radar_data_package, line_data_package):
    # 根据题目重新定义myCar
    myCar = set_state(myCar, Controller, control_data_package, radar_data_package, line_data_package)

    # 如果想使用ttc模型的话，必须先测得到他的加速度

    Controller.speedPid.setSetpoint(60)

    # 纵向控制 thorro_ and brake_
    lontitudeControlSpeed(myCar.speed, Controller.speedPid)

    # 横向不需要控制

    Controller.latPid.steer_ = 0

    # print("spd:", myCar.speed)
    # print("position:", myCar.positionnow)

    if myCar.delta_v != 0:
        dt = np.round(myCar.dist / myCar.delta_v, 3)
        # dt = 0.3
    else:
        dt = 999

    delta_a = np.round(myCar.delta_v / dt, 3)

    # flag = True
    # while flag:
    #     t1 = time.time()

    # print("dt:", -dt,"delta_a:",delta_a)
    ttc = getTTC(myCar.speed, delta_a, myCar.dist, safe_dist=0.5)

    # dist = 10, ttc = 4
    # 30 --> 4, 20 --> 4 , 0 --> 2, avg = 20

    # 30 --> 4, 20 --> 3 , 0 --> 3, avg = 18
    # if myCar.dist < 10 and ttc < 4:

    DANGER = False

    if myCar.dist < 7.3:
        DANGER = True

    if myCar.dist < 18 and myCar.delta_v < -10:
        DANGER = True

    if ttc > 0.55 and not DANGER and ttc != 999:
        # 正常情况下使用PID来进行纵向控制
        print("正常情况PID：ttc:", ttc)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_, 0, 1)

    elif ttc <= 0.55 and not DANGER:
        print("软刹车：dist:", myCar.dist, "ttc:", ttc, "delta_a:", delta_a)
        brake_ = 0.14 * delta_a
        ADCPlatform.control(0, 0, brake_, 1)

    elif DANGER:
        # 如果太近了，硬刹车
        print("硬刹车：danger!")
        ADCPlatform.control(0, 0, 1, 1)

    # print("\t", "与前车距离：", myCar.dist, "\t", "自车速度：", myCar.speed, "\t", "相对速度：", myCar.delta_v, "\t", "相对加速度：", delta_a)
    print("----------------------------------------------------------------------------------------")
