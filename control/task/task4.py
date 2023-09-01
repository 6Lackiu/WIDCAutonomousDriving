import time
import numpy as np
import ADCPlatform

speedPidThread_1 = 10  # 控制阈值1
speedPidThread_2 = 2  # 控制阈值2
pre_disatance = [float('inf'), float('inf'), float('inf')]

def final_step(myCar, radar_data_package):
    if radar_data_package is not None:
        left_dis2 = mid_dis2 = right_dis2 = float('inf')
        for car in radar_data_package.json:
            car['Range'] /= 100  # cm to m
            if -90 < car['Angle'] < -1:
                if car['Range'] < left_dis2:
                    left_dis2 = car['Range']
            elif -1 <= car['Angle'] <= 1:
                if car['Range'] < mid_dis2:
                    mid_dis2 = car['Range']
            elif 1 < car['Angle'] < 90:
                if car['Range'] < right_dis2:
                    right_dis2 = car['Range']

        if myCar.midlane == myCar.lanestate.LEFT:
            left_dis2 = float('-inf')
        if myCar.midlane == myCar.lanestate.RIGHT:
            right_dis2 = float('-inf')

        return [left_dis2, mid_dis2, right_dis2]



def latitudeyrControlpos(yr, yrPid):
    yrPid.update(yr)
    yrPid.yrsteer_ = yrPid.output * -1


def latitudeControlpos(positionnow, latPid, MyCar, k=0.6):
    latPid.update(positionnow)
    latPid.steer_ = latPid.output * -0.8  # -0.8
    if MyCar.speed > 70:
        latPid.steer_ = latPid.output * -k        # -0.7?  0.6

    # print("lattel : ", latPid.steer_)
    # 缓慢变道尝试 可以但没必要 不利于提速
    if abs(latPid.steer_) > 164:
        latPid.steer_ = 164 if latPid.steer_ > 0 else -164


def lontitudeControlSpeed(speed, lonPid):
    lonPid.update(speed - 5.0)
    if lonPid.output > speedPidThread_1:  # 加速阶段
        lonPid.thorro_ = 1
        lonPid.brake_ = 0
    elif lonPid.output > speedPidThread_2:  # 稳定控速阶段
        lonPid.thorro_ = min((lonPid.output / speedPidThread_1) * 0.85, 1.0)
        lonPid.brake_ = min(((speedPidThread_1 - lonPid.output) / speedPidThread_1) * 0.1, 1.0)
    elif lonPid.output > 0:  # 下侧 微调
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 3')
        lonPid.thorro_ = (lonPid.output / speedPidThread_2) * 0.3
        # 0.5会有稍减速的效果40-38 防碰撞
        lonPid.brake_ = ((speedPidThread_2 - lonPid.output) / speedPidThread_2) * 0.3  # 0.5
    elif lonPid.output < -1 * speedPidThread_1:  # 大减速阶段
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 4')
        # lonPid.thorro_ = (-1 * lonPid.output / 5) * 0.3
        # lonPid.brake_ = 0.8
        lonPid.thorro_ = (-1 * lonPid.output / 5) * 0.1
        lonPid.brake_ = 0.4
    else:
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 5')
        lonPid.thorro_ = (-1 * lonPid.output / speedPidThread_2) * 0.15
        # 减速二阶段                 abs(2 - (2~10))/2 * 0.6
        lonPid.brake_ = min(abs((speedPidThread_2 - (-1 * lonPid.output)) / speedPidThread_2) * 0.4, 0.4)
        # lonPid.brake_ = 1.0
    # print(lonPid.thorro_, '    ', lonPid.brake_)


def speedupJob(Controller, MyCar):
    # if dis_data is not None:
    #     distance_left, distance_mid, distance_right = dis_data
    #     distance = dis_data
    # else:
    #     distance_left = distance_mid = distance_right = float('inf')
    #     distance = [distance_left, distance_mid, distance_right]
    # print("distance : \t", distance)

    if MyCar.time >= Controller.superspeeduplimittime \
            and MyCar.overtakeSum != 0 and not MyCar.finalflag:
        Controller.speeduplimit = Controller.superspeeduplimit
        print("开始加速")

        # if distance_mid < 2 * MyCar.saftydistance:
        #     Controller.speeduplimit = Controller.speeduplimit
        #     print("减速为超车做准备: \t", Controller.speeduplimit)

    # if MyCar.finalflag == True:
    #     print("进入最后阶段啦***********")
    #     MyCar.cardecision = 'finalspeedup'

    Controller.speedPid.setSetpoint(Controller.speeduplimit)

    # 纵向控制 thorro_ and brake_
    lontitudeControlSpeed(MyCar.speed, Controller.speedPid)
    # 横向控制 steer_
    latitudeControlpos(MyCar.positionnow, Controller.latPid, MyCar, k=0.60)  # 0.6

    throttle_ = Controller.speedPid.thorro_
    # TODO：overtake steer need recovery;系数变大()or限幅变大
    steer_ = 1*Controller.latPid.steer_  # 1.0
    steer_ = max(min(steer_, 45), -45)  # 50*  45
    brake_ = Controller.speedPid.brake_
    gears_ = 1
    if MyCar.finalflag:
        steer_ = max(min(steer_, 10), -10)

    print("throttle: ", round(throttle_, 3), "\tsteer: ", round(steer_, 3), "\tbrake: ", round(brake_, 3))
    ADCPlatform.control(throttle_,steer_, brake_, gears_)


def followJob(Controller, MyCar):
    """
    Followjob is to finish follow task.Send control
    command to ADCPlatform.
    """
    Controller.speedPid.setSetpoint(Controller.followlimit)
    # 纵向控制 thorro_ and brake_
    lontitudeControlSpeed(MyCar.speed, Controller.speedPid)
    # 横向控制 steer_
    latitudeControlpos(MyCar.positionnow, Controller.latPid, MyCar,k=0.6)

    throttle_ = Controller.speedPid.thorro_
    steer_ = Controller.latPid.steer_
    steer_ = max(min(steer_, 40), -40)
    brake_ = Controller.speedPid.brake_
    gears_ = 1

    print("throttle: ", round(throttle_, 3), "\tsteer: ", round(steer_, 3), "\tbrake: ", round(brake_, 3))
    ADCPlatform.control(throttle_,steer_, brake_, gears_)


def overtakeJob(Controller, MyCar, dis_data):
    """
    Overtakejob is to finish overtake task.Send control
    command to ADCPlatform.
    """
    if dis_data is not None:
        distance_left, distance_mid, distance_right = dis_data
        distance = dis_data
    else:
        distance_left = distance_mid = distance_right = float('inf')
        distance = [distance_left, distance_mid, distance_right]
    print("distance : \t", distance)

    Controller.speedPid.setSetpoint(Controller.overtakelimit)
    # 纵向控制 thorro_ and brake_
    lontitudeControlSpeed(MyCar.speed, Controller.speedPid)

    # overtake 车道中线调整
    if not MyCar.changing:
        # 最左侧不可左变道
        if MyCar.direction == 'left':
            MyCar.midlane = min(MyCar.lanestate.LEFT, MyCar.lanestate.LEFT + MyCar.midlane)
        # 最右侧不可右变道
        elif MyCar.direction == 'right':
            MyCar.midlane = max(MyCar.lanestate.RIGHT, MyCar.lanestate.RIGHT + MyCar.midlane)
        Controller.latPid.setSetpoint(MyCar.midlane)
        # 更新中线state 进入超车
        MyCar.changing = True

    # overtake 完成 切换 follow 状态跟车
    # print("minus : ", MyCar.midlane - MyCar.positionnow)
    # if (MyCar.changing and abs(MyCar.midlane - MyCar.positionnow) < 0.5):
    if (MyCar.changing and
            (distance_mid > 20  # 25  20
             or abs(MyCar.midlane - MyCar.positionnow) < 0.1)
    ):
        MyCar.cardecision = 'speedup'
        MyCar.direction = 'mid'
        MyCar.changing = False
        MyCar.overtakeSum += 1

    # 横向控制 steer_ 加入角度速度约束
    latitudeyrControlpos(MyCar.yr, Controller.yrPid)
    # print('yr is', MyCar.yr, 'steeryr is', Controller.yrPid.yrsteer_) # overtake >15 , normal < 3
    # print('latsteer is ', Controller.latPid.steer_)
    # print("total steer: \t", Controller.latPid.steer_ + 0.01 * Controller.yrPid.yrsteer_)

    latitudeControlpos(MyCar.positionnow, Controller.latPid, MyCar, k=0.6)

    throttle_ = Controller.speedPid.thorro_

    # steer_ = 0.65 * Controller.latPid.steer_    # ori: 0.6 * Controller.latPid.steer_ + 0.01 * Controller.yrPid.yrsteer_
    # print("Controller.latPid.steer_:  ", round(Controller.latPid.steer_, 3), "Controller.yrPid.yrsteer_:  ", round(Controller.yrPid.yrsteer_))
    steer_ = 0.62 * Controller.latPid.steer_
    steer_ = max(min(steer_, 75), -75)
    brake_ = Controller.speedPid.brake_
    gears_ = 1
    print("throttle: ", round(throttle_, 3), "\tsteer: ", round(steer_, 3), "\tbrake: ", round(brake_, 3))
    ADCPlatform.control(throttle_,steer_, brake_, gears_)


def run_task4_test(MyCar, Controller, dis_data, control_data_package, radar_data_package, landLine_package):
    x1 = MyCar.lanefuture ** 0
    x2 = MyCar.lanefuture ** 1
    x3 = MyCar.lanefuture ** 2
    x4 = MyCar.lanefuture ** 3

    # 平台bug 存在读不到数据的情况
    if landLine_package:
        if landLine_package.json:
            # 取中间两车道数据
            if len(landLine_package.json) >= 3 and landLine_package.json[1] and landLine_package.json[2]:
                # MyCar.positionnow = landLine_package.json[2]['A1'] + landLine_package.json[1]['A1']
                # 拿到车道线反映出的车身位置
                # temp0 = x1 * landLine_package.json[0]['A1'] + x2 * landLine_package.json[0]['A2'] + x3 * \
                #         landLine_package.json[0]['A3'] + x4 * landLine_package.json[0]['A4']
                temp1 = x1 * landLine_package.json[1]['A1'] + x2 * landLine_package.json[1]['A2'] + x3 * landLine_package.json[1]['A3'] + x4 * landLine_package.json[1]['A4']
                temp2 = x1 * landLine_package.json[2]['A1'] + x2 * landLine_package.json[2]['A2'] + x3 * landLine_package.json[2]['A3'] + x4 * landLine_package.json[2]['A4']
                # temp3 = x1 * landLine_package.json[3]['A1'] + x2 * landLine_package.json[3]['A2'] + x3 * \
                        # landLine_package.json[3]['A3'] + x4 * landLine_package.json[3]['A4']

                # print("Width: \t", temp2 - temp1)
                # print("temp0: ",temp0,"\t\t temp1: ",temp1, "\t\t temp2", temp2, "\t\t temp3: ", temp3)
                # print("lane0: ", landLine_package.json[0]['A1'], "\t\t lane1: ", landLine_package.json[1]['A1'], "\t\tlane2: ", landLine_package.json[2]['A1'], "\t\tlane3: ", landLine_package.json[3]['A1'], )

                MyCar.positionnow = temp1 + temp2
            else:
                pass
        else:
            pass
    else:
        pass

    if not control_data_package:
        print("任务结束")

    MyCar.speed = control_data_package.json['FS']   # 自车速度 km/h
    MyCar.cao = control_data_package.json['CAO']    # 自车偏航角 度
    MyCar.yr = control_data_package.json['YR']      # 自车转动角速率(yaw角速率) 度/s

    # 判断是否进入最终阶段
    global pre_disatance
    # distance_left, distance_mid, distance_right = dis_data
    # distance = [distance_left, distance_mid, distance_right]

    # _, dis_mid2, _ = final_step(MyCar, radar_data_package)

    if dis_data is not None:
        distance_left, distance_mid, distance_right = dis_data
        distance = dis_data
        dis_left2, dis_mid2, dis_right2 = final_step(MyCar, radar_data_package)  #

    else:
        distance_left = distance_mid = distance_right = float('inf')
        distance = [distance_left, distance_mid, distance_right]
        dis_left2 = dis_mid2 = dis_right2 = float('inf')

    if pre_disatance == distance and MyCar.cardecision == 'speedup':
    # if pre_disatance[1] == distance[1] and MyCar.cardecision == 'speedup':
    # if pre_disatance[1] == dis_mid2 and MyCar.cardecision == 'speedup' and dis_left2 > 0 and dis_right2 > 0:
        MyCar.finalcount += 1
    else:
        MyCar.finalcount = 0

    # # TODO 与前车距离大时快点
    # if distance_mid > 50:
    #     Controller.speeduplimit = Controller.superspeeduplimit

    # 5
    if MyCar.finalcount > 3 and MyCar.cardecision == 'speedup' and not MyCar.finalflag:
        Controller.speeduplimit = Controller.finalspeed
        MyCar.finalflag = True
        # distanceData.set_distance_left(float('inf'))
        # distanceData.set_distance_mid(float('inf'))
        # distanceData.set_distance_right(float('inf'))
        if MyCar.midlane == MyCar.lanestate.RIGHT:
            print("在右车道，回归中间车道")
            MyCar.cardecision = 'overtake'
            MyCar.direction = 'left'
        elif MyCar.midlane == MyCar.lanestate.LEFT:
            print("************在左车道，回归中间车道")
            MyCar.cardecision = 'overtake'
            MyCar.direction = 'right'

    # if not MyCar.finalflag and 60 < MyCar.dist < 150 and MyCar.start_flag:
    #     MyCar.start_flag = False
    #     Controller.speeduplimit = Controller.finalspeed

    # 有限3种状态任务
    if MyCar.cardecision == 'overtake':
        print("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t***   decision: overtake  ***")
        overtakeJob(Controller, MyCar, dis_data)
    elif MyCar.cardecision == 'speedup':
        print("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t      decision: speedup")
        speedupJob(Controller, MyCar)
    # elif (MyCar.cardecision == 'finalspeedup'):
    #     print("Final state **************")
    #     finalspeedup(Controller, MyCar)

    print("目前所在车道：", MyCar.midlane)
    # if MyCar.speed > 0:
    #     # speed_list = []
    #     # speed_list.append(0)
    #     # speed_list.append(MyCar.speed)
    #     speed_list = [0, MyCar.speed/100]
    #     dt = 0.258
    #     a = (speed_list[-1]-speed_list[-2])/dt
    #     print(a)
    # print(time.time())
    # print("偏航角",round(MyCar.cao,3))
    print("----------------------------------------------------------------------------------------------------------")

    # print("Dist: \t", distanceData.distance_left, distanceData.distance_mid, distanceData.distance_right)
    # print(MyCar.cardecision, MyCar.midlane, MyCar.direction)
    # print("temp1+temp2 : ", temp1 + temp2)
