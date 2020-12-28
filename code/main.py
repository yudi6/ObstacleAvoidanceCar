import vrep
import time
import numpy as np
import cv2
from PID import PID
from matplotlib import pyplot as plt
# repeat until (simRemoteApi.start(19999,1300,false,true)~=-1)
class ObstacleAvoidanceCar():
    def __init__(self):
        # 客户端ID
        self.clientId = self._connection()
        self._get_handle()

        # self._motor(1)
        # PID控制
        self.PID_control = PID(0.2,0.01,0.08)
        self.PID_control.SetPoint = 0
        # self.PID_speed_control = PID(0.2,0.02)
        # self.PID_speed_control.SetPoint = 0.75
        self._run()
        # while True:
        #     self._get_image()
        #

    def _get_handle(self):
        # 左轮的句柄
        _, self.left_motor = vrep.simxGetObjectHandle(self.clientId, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_oneshot_wait)
        # 右轮的句柄
        _, self.right_motor = vrep.simxGetObjectHandle(self.clientId, "Pioneer_p3dx_rightMotor", vrep.simx_opmode_oneshot_wait)
        # 视觉传感器句柄
        _, self.vision_sensor = vrep.simxGetObjectHandle(self.clientId, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
        # 获取第一张图片 为空
        _, _, _ = vrep.simxGetVisionSensorImage(self.clientId, self.vision_sensor, 0, vrep.simx_opmode_streaming)
        # 间隔一段时间
        time.sleep(0.1)

    def _connection(self):
        # 关闭原有连接
        vrep.simxFinish(-1)
        while True:
            # 连接
            clientId = vrep.simxStart("127.0.0.1", 19997, True, True, 5000, 5)  # 建立和服务器的连接
            if clientId != -1:  # 连接成功
                print('connect successfully')
                break
        vrep.simxSynchronous(clientId,True); #Enable the synchronous mode (Blocking function call)
        #
        return clientId

    def _get_image(self):
        _, resolution, image = vrep.simxGetVisionSensorImage(self.clientId, self.vision_sensor, 0, vrep.simx_opmode_buffer)
        while len(image)==0:
            _, resolution, image = vrep.simxGetVisionSensorImage(self.clientId, self.vision_sensor, 0, vrep.simx_opmode_buffer)
        sensor_image = np.array(image, dtype=np.uint8)
        # print(resolution)
        # RGB图片 resolution为图片大小
        sensor_image.resize([resolution[1], resolution[0], 3])
        # 上下颠倒
        sensor_image = cv2.flip(sensor_image, 0)
        cv2.imshow('sensor_image', sensor_image)
        return sensor_image

    # 设置两个轮子的速度
    def _motor(self, speed):
        _ = vrep.simxSetJointTargetVelocity(self.clientId, self.left_motor, speed, vrep.simx_opmode_oneshot)
        _ = vrep.simxSetJointTargetVelocity(self.clientId, self.right_motor, speed, vrep.simx_opmode_oneshot)

    # 设置两个轮子的速度与速度差 实现转向
    def _steer(self, speed, turn):
        _ = vrep.simxSetJointTargetVelocity(self.clientId, self.left_motor, speed - turn, vrep.simx_opmode_oneshot)
        _ = vrep.simxSetJointTargetVelocity(self.clientId, self.right_motor, speed + turn, vrep.simx_opmode_oneshot)

    def _run(self):
        speed_now = 6
        error_before = 0
        no_dash=False
        during_sharp_turn=False
        vrep.simxStartSimulation(self.clientId,vrep.simx_opmode_oneshot)
        while True:
            vrep.simxSynchronousTrigger(self.clientId)
            img = self._get_image()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)#480*640
            binary_copy = binary.copy()
            binary_copy[binary_copy == 0] = 1
            binary_copy[binary_copy == 255] = 0
            binary_temp = binary_copy[-60:, 160:480].copy()#下方的色块
            #   选择屏幕中间的一部分来控制速度
            binary_speed = binary_copy[-(250+int(speed_now*60)):, 300:340].copy()
            #   黑色块的比重
            speed_p = np.sum(binary_speed)/(binary_speed.shape[0]*binary_speed.shape[1])
            #  比重很多 直接为10
            if speed_p > 0.5 and speed_p < 10:
                speed_now = 10
            #   比重还行    只减速不加速
            elif speed_p <= 0.5 and speed_p > 0.27:
                if speed_now > 5.4:
                    speed_now = 5.4
            #   比重特别低   正在转弯
            else:
                speed_now = 3.5
            print('speed:',speed_now)

            #   选择屏幕中间的一部分来判断是否出现虚线，也即dash
            is_dash=False#判断是否是出现虚线。按照之前的判断方式，中间且下方黑色快很少有可能被判断为急转弯
                         #新的方法是，采用一块较长较宽的区域，如果该区域内最大的黑色快事实上也很小，
                         # 且出现多个连通分量则认为出现了虚线，不将它作为急转弯或交叉路口处理
            binary_dash = binary_copy[100:-60, 200:440].copy()
            binary_dash_temp=binary[100:-60, 200:440].copy()
            cv2.imshow('binary_dash', binary_dash_temp)
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_dash.copy())
            #找出最大连通域面积大小
            max_area=0
            for index in range(0,num_labels):
                flag=False
                for index1 in range(binary_dash.shape[0]):
                    if flag:
                        break
                    for index2 in range(binary_dash.shape[1]):
                        if flag:
                            break
                        if labels[index1][index2]==index:
                            if binary_dash[index1][index2]==1:
                                max_area=max(max_area,stats[index][-1])
                            flag=True
                            
            print("max_area",max_area)
            if max_area<4000 and num_labels>=3:#如果最大连通区域面积很小且有多个黑色连通区域，标志当前出现了虚线
                is_dash=True
            if during_sharp_turn:
                binary_temp=binary_temp[:,80:-80]
            #这部分代码是为了解决虚线并线的问题
            #虚线并线会出现大幅抖动，针对这个问题，如果出现虚线，
            #我们去查看更远的地方是否可以找到多个连通分量
            #如果找到更多的连通分量，那么则用更大的区域计算方向
            contours, cnt = cv2.findContours(binary_temp.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if is_dash and len(contours)==1:
                binary_temp1=binary_copy[-120:, 160:480].copy()
                contours1, cnt1 = cv2.findContours(binary_temp1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours1)!=1:
                    binary_temp=binary_temp1
                    contours, cnt = cv2.findContours(binary_temp.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            #这部分代码用于解决虚线单线中下方突然什么都看不到的问题
            #这一问题可能在较短的虚线时带来严重的抖动，因为可能突然什么都看不见了，
            #原始的算法会无法更新direction
            #我们需要不停的上移，直到看到黑色块为止
            bias=480-60
            while is_dash and len(contours)==0:
                bias-=20
                binary_temp=binary_copy[bias:bias+60, 160:480].copy()
                contours, cnt = cv2.findContours(binary_temp.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # 寻找离中线最近的轮廓    保存中点
            nearest = 0
            for i in range(len(contours)):
                M = cv2.moments(contours[i])
                center_x = int(M["m10"] / (M["m00"]+0.00000001))
                if abs(center_x - (480-160)/2) < abs(nearest - (480-160)/2):
                    nearest = center_x
            # 确定方向
            direction = nearest - (480-160)/2
            black_partition = np.sum(binary_temp[-4:,:]) / (binary_temp.shape[1]*4)
            print("black_partition:",black_partition)
            print("speed_p:",speed_p)
            if no_dash:
                is_dash=False
            print("is_dash:",is_dash)
            if black_partition < 0.065 and speed_p <= 0.2 and is_dash==False:#既然是虚线，就不可能是急转弯
                print("急转弯")
                self._steer(1.3,np.sign(self.PID_control.output)*2)
                speed_now = 1.5
                no_dash=True
                during_sharp_turn=True
            elif (black_partition > 0.6 or (abs(error_before - direction) > 100 and len(contours)!= 1)) and is_dash==False:#既然是虚线，就不可能是交叉路口
            #交叉路口或转弯并线，这时沿着之前的转弯角度小幅度前进
                print("交叉路口")
                self._steer(1.5,self.PID_control.output/50)
                speed_now = 1.5
                no_dash=True
                during_sharp_turn=False
            else:
                error_before = direction
                # 方向PID的输入
                self.PID_control.update(direction)
                print("PID:",self.PID_control.output)
                self._steer(speed_now,self.PID_control.output/10)
                no_dash=False
                during_sharp_turn=False

            binary = binary[-60:, 160:480]
            if during_sharp_turn:
                binary = binary[:,80:-80]
            cv2.imshow('image', binary)
            if cv2.waitKey(1) == 27:
                break

car = ObstacleAvoidanceCar()