import ADCPlatform
import control
"""
初赛最终版本
"""

if __name__ == '__main__':
    # 开启平台SDK
    # 设置服务器访问地址
    serverUrl = 'http://www.adchauffeur.cn/api/'
    # 设置登录用户名
    username = 'xxxx'
    # 设置登录密码
    password = 'xxxx'
    result = ADCPlatform.start(serverUrl, username, password)

    if result:
        print("算法接入成功！")
        print("启动任务")
        ADCPlatform.start_task()
        # 启动算法接入任务控制车辆

        # 选择任务的序号
        task_num = 4
        task_list = ['前车制动', '前车切入', '重叠跟车', '相邻车道多目标干扰', '动态交通流']
        task = task_list[task_num]

        print("========================")
        print("选择任务{}: {}".format(task_num, task_list[task_num]))
        print("========================")

        # 根据不同任务执行不同的算法
        if task_num == 0:
            control.run(task=0)
        elif task_num == 1:
            control.run(task=1)
        elif task_num == 2:
            control.run(task=2)
        elif task_num == 3:
            control.run(task=3)
        elif task_num == 4:
            control.run(task=4)
        else:
            print("没有'{}'任务".format(task))

        # 停止平台
        ADCPlatform.stop()

    else:
        # 停止平台
        ADCPlatform.stop()
