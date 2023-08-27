import ADCPlatform
from control.state import CarState, ControlData

def all_other_cars(radar_data_package):
    if radar_data_package is None:
        pass
    else:
        # for car in radar_data_package.json:
        #     car['Range'] /= 100
        print(radar_data_package.json)


def distanceData(myCar, radar_data_package):
    if radar_data_package is not None:
        # print("All vehicle info：\n", radar_data_package.json)  #
        left_dis = mid_dis = right_dis = float('inf')
        # left_dis = mid_dis = right_dis = 9999
        for car in radar_data_package.json:
            car['Range'] /= 100     # cm to m
            if -90 < car['Angle'] < -3:  # 2
                if car['Range'] < left_dis:
                    left_dis = car['Range']
            elif -3 <= car['Angle'] <= 3:
                if car['Range'] < mid_dis:
                    mid_dis = car['Range']
            elif 3 < car['Angle'] < 90:
                if car['Range'] < right_dis:
                    right_dis = car['Range']

        if myCar.midlane == myCar.lanestate.LEFT:
            left_dis = float('-inf')
        if myCar.midlane == myCar.lanestate.RIGHT:
            right_dis = float('-inf')

        # 最后加速阶段
        if myCar.finalflag:
            left_dis = float('inf')
            mid_dis = float('inf')
            right_dis = float('inf')

        return [left_dis, mid_dis, right_dis]


def decision(dis_data, MyCar):
    if dis_data is not None:
        distance_left, distance_mid, distance_right = dis_data
        distance = dis_data
    else:
        distance_left = distance_mid = distance_right = float('inf')
        distance = [distance_left, distance_mid, distance_right]
    print("distance : \t", distance)

    # # 速度较小时安全距离也应该相应减小，防止与旁边后侧车道车辆相撞
    if MyCar.speed < 42:
        MyCar.saftydistance = 10
    if 52 < MyCar.speed < 62 and MyCar.cardecision == 'speedup':
        MyCar.saftydistance = 13
        # 高速时为15 相信速度控制
    elif 60 < MyCar.speed < 70:
        MyCar.saftydistance = 15        # 15
    elif MyCar.speed > 70:
        MyCar.saftydistance = 15.5        # 17 19  *18
    # print("Safe Distance is : \t", MyCar.saftydistance)
    # print("Ego car speed: \t", MyCar.speed)
    # print("相对距离：'{}'".format(MyCar.dist, 2))

    # find target lane
    if (MyCar.cardecision == 'speedup'
            and distance_mid < MyCar.saftydistance  # 13m
            and not MyCar.changing  # 保证超车只判断一次即可
        ):  # follow 已将车速降下来

        MyCar.cardecision = 'overtake'
        findpath(distance, MyCar)
        return


def findpath(distance, MyCar):
    # 当前所在车道
    whichlanenow = MyCar.midlane

    print("My car's lane: \t", whichlanenow)

    # 如果当前在左车道 overtake只能向右
    if whichlanenow == MyCar.lanestate.LEFT:
        MyCar.direction = 'right'
        print("*** 当前在最左边车道，只能向右变道 ***")
    elif whichlanenow == MyCar.lanestate.RIGHT:
        # if distance[0] < distance[1]:
        #     MyCar.direction = 'mid'
        #     # MyCar.changing = True
        #     MyCar.cardecision = 'follow'
        # else:
        MyCar.direction = 'left'
        print("*** 当前在最右边车道，只能向左变道 ***")
    # 中间车道要对比两侧距离后作出决策
    else:
        if distance[0] > distance[2]:
            MyCar.direction = 'left'
            print(">>> 左侧相对安全，向左变道 <<<")
        elif distance[0] == float('inf') and distance[2] == float('inf'):
            MyCar.direction = 'left'
            print(">>> 左右均可变道，优先向左变道 <<<")
        else:
            print(">>> 右侧相对安全，向右变道 <<<")
            MyCar.direction = 'right'
        # MyCar.direction = 'left'


# 启动算法  在此方法中调用实现算法控制代码
def run(task):
    running = True

    # 初始化智能体和控制器
    controller = ControlData()
    myCar = CarState()

    while running:
        # 毫米波真值传感器id
        radarId = 0
        # 摄像机传感器id
        cameraId = 0
        # 车道线传感器id
        landLineId = 0

        # print("当前车辆安装传感器为")
        sensors = ADCPlatform.get_sensors()
        for sensor in sensors:
            # print("传感器：\n", sensor.Name)
            if sensor.Name == "毫米波雷达":
                radarId = sensor.ID
            elif sensor.Name == "摄像机":
                cameraId = sensor.ID
            elif sensor.Name == "车道线传感器":
                landLineId = sensor.ID
            # print("名称：" + sensor.Name + ",ID:" + str(sensor.ID))

        # 获取车辆控制数据包
        control_data_package = ADCPlatform.get_control_data()
        radar_data_package = ADCPlatform.get_data(radarId)
        line_data_package = ADCPlatform.get_data(landLineId)


        if not control_data_package:
            print("任务结束")
            running = False
            break

        # 当前车速
        spd = control_data_package.json['FS']
        # print("当前车速：", spd)

        # 当前偏航角
        yaw = control_data_package.json['CAO']

        # 当前角速度
        yr = control_data_package.json['YR']

        if radar_data_package is not None:
            # v: cm/s
            # 相对前车的速度
            delta_spd = radar_data_package.json[0]["Speed"] / 100  # m/s

            # 相对前车的距离
            dist = radar_data_package.json[0]["Range"] / 100  # m

            # print("相对前车速度：'{}' ".format(delta_spd, 2))
            # print("相对前车距离：'{}' ".format(dist, 2))

            myCar.dist = dist   # 相对距离
            myCar.delta_v = delta_spd   # 相对车速

        if line_data_package is not None:
            # print("车道线信息：\n", line_data_package.json)

            if len(line_data_package.json) == 4:
                myCar.positionnow = -6.5 + (line_data_package.json[2]['A1'] + line_data_package.json[1]['A1'])
            else:
                print("ERROR: len(line_data_package.json) != 4")
                myCar.positionnow = -7.0

        # 保存当前的状态
        myCar.speed = spd   # 绝对车速
        myCar.cao = yaw
        myCar.yr = yr

        if task == 0:
            from control.task.task0 import run_task0_test
            run_task0_test(myCar, controller, control_data_package, radar_data_package, line_data_package)

        elif task == 1:
            from control.task.task1 import run_task1_test
            run_task1_test(myCar, controller, control_data_package, radar_data_package, line_data_package)

        elif task == 2:
            # controller.targetSpeedInit = 40.0
            from control.task.task2 import run_task2_test
            run_task2_test(myCar, controller)

        elif task == 3:
            from control.task.task3 import run_task3_test
            run_task3_test(myCar, controller, radar_data_package)

        elif task == 4:
            from control.task.task4 import run_task4_test
            # all_other_cars(radar_data_package)
            dis_data = distanceData(myCar, radar_data_package)
            decision(dis_data, myCar)
            run_task4_test(myCar, controller, dis_data, control_data_package, radar_data_package, line_data_package)
