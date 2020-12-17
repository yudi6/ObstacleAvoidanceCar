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
        self.PID_control = PID(0.3,0.02)
        self.PID_control.SetPoint = 0
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
            clientId = vrep.simxStart("127.0.0.1", 19999, True, True, 5000, 5)  # 建立和服务器的连接
            if clientId != -1:  # 连接成功
                print('connect successfully')
                break
        return clientId

    def _get_image(self):
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
        linear_time=0
        while True:
            img = self._get_image()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # gray[gray==0]=255
            _, binary = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)
            #
            # print(binary)
            # binary=binary[]
            # k = np.ones((4, 4), np.uint8)
            # binary = cv2.dilate(binary, k, iterations=3)
            binary = binary[:, 160:480]
            # corners = cv2.goodFeaturesToTrack(binary, 20, 0.06, 50)
            # print(corners)
            # corners = np.int0(corners)
            # for i in corners:
            #     x, y = i.ravel()
            #     cv2.circle(binary, (x, y), 3, 255, -1)
            # plt.imshow(binary), plt.show()
            binary_temp = binary.copy()
            binary_temp[binary_temp == 0] = 1
            binary_temp[binary_temp == 255] = 0
            w = np.resize(np.array([50]*(binary_temp.shape[0]-55)+[200]*30+[500]*10+[2000]*15), (binary_temp.shape[1],binary_temp.shape[0])).transpose()
            pos_pos_area = np.full((binary_temp.shape[0],int(binary_temp.shape[1]/4)-40),3)
            pos_area = np.full((binary_temp.shape[0],int(binary_temp.shape[1]/4)-10),1)
            zero_area = np.zeros((binary_temp.shape[0],100))
            neg_area = np.full((binary_temp.shape[0],int(binary_temp.shape[1]/4)-10),-1)
            neg_neg_area = np.full((binary_temp.shape[0], int(binary_temp.shape[1] / 4) - 40), -3)
            final_area = np.concatenate((neg_neg_area,neg_area,zero_area,pos_area,pos_pos_area),axis=1)
            result = np.sum(w*binary_temp*final_area)/(binary_temp.shape[0]*binary_temp.shape[1])

            fraction = final_area * binary_temp
            fraction = np.sum(fraction, axis=0)
            np.diff(fraction)
            new_error = (np.sum(fraction) / (binary_temp.shape[0] * binary_temp.shape[1]))*20
            # if abs(result)>4:
            #     new_error=0
            print(result,new_error)
            result+=new_error
            self.PID_control.update(result)
            print(self.PID_control.output)
            speed_now = 0.5 / (abs(self.PID_control.output) + 5)
            if abs(self.PID_control.output) < 0.5:
                linear_time+=1
            else:
                linear_time=0
            if linear_time>=3 and abs(self.PID_control.output) < 0.5:
                speed_now = 1.5
            # speed_now = 1.5 if abs(self.PID_control.output) < 0.5 else 0.5/(abs(self.PID_control.output)+10)
            self._steer(speed_now,self.PID_control.output/5)
            cv2.imshow('image', binary)
            if cv2.waitKey(1) == 27:
                break
            # time.sleep(0.5)

car = ObstacleAvoidanceCar()