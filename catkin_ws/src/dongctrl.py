import pyqtgraph as pg
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QObject, Qt, QThread, QTimer

import time, random
import rospy
import ros_numpy
from threading import Thread

# from test_package.hello import Hello
# from moraictrl import moraiCtrl
import scv_control_controller

from moraictrl import moraiCtrl

from morai_msgs.msg import CtrlCmd
import numpy as np
#from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan , PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Float64 , Header
#from hunterctrl import hunterCtrl
import math
from sklearn.cluster import KMeans
from sklearn.neighbors import KDTree
from sklearn.cluster import DBSCAN
import pcl
from sklearn.linear_model import RANSACRegressor


class Car_Follower():
    def __init__(self):
        self.sub3D = rospy.Subscriber("/lidar3D", PointCloud2, self.callback3D)
        self.lidar_pub3D = rospy.Publisher("/refined_lidar3D", PointCloud2, queue_size=1)  # velodyne
        self.send = rospy.Publisher("/lp_ctrl", CtrlCmd, queue_size=1)
        self.pos = None
        self.flag = False
        self.steering = 0
        self.velocity = 0
        self.brake = 0
        # self.objsPos = np.empty((0,2),float)
        # self.objsSize = np.empty((0,2),float)
        self.objsPos = np.empty((0,6),float)
        self.objsSize = np.empty((0,2),float)
        self.direction = 0
        self.clusterLabel =list()

    def dbscan(self, points):  # dbscan eps = 1.5, min_size = 60
        #조건들 = esp(기준점부터의 원의 반지름 거리) , min_samples(esp로 설정된 원 내부에 존재 해야할 점의 수)
        dbscan = DBSCAN(eps=2, min_samples=2).fit(points)
        self.clusterLabel = dbscan.labels_

    def callback3D(self, msg):

        send = CtrlCmd()
        scan = PointCloud2()
        scan.header = msg.header
        scan.height = msg.height
        scan.width = msg.width
        scan.fields = msg.fields
        scan.is_bigendian = msg.is_bigendian
        scan.point_step = msg.point_step
        scan.row_step = msg.row_step
        scan.data = msg.data
        scan.is_dense = msg.is_dense

        pc = ros_numpy.numpify(scan)
        points = np.zeros((pc.shape[0], 3))
        # if all(pc):
        points[:, 0] = pc['x']
        points[:, 1] = pc['y']
        points[:, 2] = pc['z']

        roi = {"x": [0, 10], "y": [-1, 1], "z": [-0.44, 10]}  # z값 수정, X 값 수정으로 전,후방 범위 조절
        #z축 현재 위치 0.62 -> 바닥에서 차량의 바퀴까지 0.18, 그 밑에 부분은 차량이 충분히 넘을 수 있을거라 가정

        x_range = np.logical_and(points[:, 0] >= roi["x"][0], points[:, 0] <= roi["x"][1])
        y_range = np.logical_and(points[:, 1] >= roi["y"][0], points[:, 1] <= roi["y"][1])
        z_range = np.logical_and(points[:, 2] >= roi["z"][0], points[:, 2] <= roi["z"][1])

        pass_through_filter = np.where(np.logical_and(x_range, np.logical_and(y_range, z_range)) == True)[0]
        points = points[pass_through_filter, :]

        if np.any(points != 0):
        #dbscan속도 증진을 위해 0값 제거
            repoint = np.delete(points, np.where(points[:,0] == 0, points[:,1] == 0, points[:,2] == 0), axis=0)
            self.dbscan(repoint)
            self.velocity = 10
            for i in range(0, max(self.clusterLabel) + 1):
                #군집화로 객체가 분류된 배열의 가장 작은 값의 인덱스
                index = np.asarray(np.where(self.clusterLabel == i))
                # print(i, 'cluster 개수 : ', len(index[0]))
                x_min = np.min(repoint[index, 0])
                y_min = np.min(repoint[index, 1])
                x_max = np.max(repoint[index, 0])
                y_max = np.max(repoint[index, 1])
                x_size = np.max(repoint[index, 0]) - np.min(repoint[index, 0])  # x_max 3
                y_size = np.max(repoint[index, 1]) - np.min(repoint[index, 1])  # y_max 1.3
                x_mid = (x_min+x_max)/2
                y_mid = (y_min+y_max)/2
                # self.objsPos = np.append(self.objsPos, np.array([[x_min, y_min, x_max, y_max, x_mid, y_mid]]), axis=0)
                # self.objsSize = np.append(self.objsSize, np.array([[x_size, y_size]]), axis=0)
                # print(i,"번쨰","x_min: ", round(x_min,2),end=", ")
                # print("y_min: ", round(y_min,2), end=", ")
                # print("x_size: ", round(x_size,2), end=", ")
                # print("y_size: ", round(y_size,2),end=", ")
                # print("x_max: ", round(x_max, 2), end=", ")
                # print("y_max: ", round(y_max, 2), end=", ")
                # print("x_mid: ", round(x_mid, 2), end=", ")
                # print("y_mid: ", round(y_mid, 2))
                if points[:,1].size != 0:
                    if min(repoint[:,0]) < 1.5:
                        self.velocity = 0
                    elif round(abs(x_max),2) < round(abs(x_min),2):
                        print("장애물 있을때 오른쪽")
                        self.steering = self.steering + 0.7
                    elif round(abs(x_max),2) > round(abs(x_min),2):
                        print("장애물 있을때 왼쪽")
                        self.steering = self.steering - 0.7
                    else:
                        pass
                        # self.velocity = 0
        else:
            if round(self.steering) != 0 and round(self.steering) > 0:
                print("장애물 없을때 왼쪽으로 전진하고있으면 오른쪽으로 바퀴 각도  조정 ")
                self.steering = self.steering - 0.7
            elif round(self.steering) != 0 and round(self.steering) < 0:
                print("장애물 없을때 오른쪽")
                self.steering = self.steering + 0.7
            else:
                self.steering = 0
                self.velocity = 10


        '''
        x,y 좌표 분리해서 출력 
        xaxis = np.delete(points, [1, 2], axis=1)
        yaxis = np.delete(points, [0, 2], axis=1)
        re_xaxis = np.delete(xaxis,np.where(xaxis==0))
        re_yaxis = np.delete(yaxis, np.where(yaxis == 0))
        print("정제된 x값\n", re_xaxis)
        print("정제된 y값", re_yaxis)
        print("x 크기",re_xaxis.size)
        print("y 크기", re_yaxis.size)
        '''
        """
        #초기 장애물 회피 코드 23.02.25
        miny = min(re_xyaxis[:,1])
        maxy = max(re_xyaxis[:,1])
        minindex = np.where(re_xyaxis[:,1] == miny)
        maxindex = np.where(re_xyaxis[:,1] == maxy)
        min_xyaxis = re_xyaxis[minindex]
        max_xyaxis = re_xyaxis[maxindex]
        minlean = min_xyaxis[:, 1] / min_xyaxis[:, 0]
        maxlean = max_xyaxis[:, 1] / max_xyaxis[:, 0]

        print("좌표 :", re_xyaxis)
        print("최소 값 ", miny)
        print("최소 값의 인덱스 및 좌표들 ", re_xyaxis[minindex])
        print("최대 값 ", maxy)
        print("최대 값의 인덱스 및 좌표들 ", re_xyaxis[maxindex])

        xy = list()
        for i in range(re_xyaxis[:,1].size):
            xy.append(math.sqrt((re_xyaxis[i][1] ** 2) + (re_xyaxis[i][0] ** 2)))
        print("가장 가까운 점의 거리 ",min(xy))
        print(" 최소 값 기울기 ", minlean)
        print("최대 값 기울기 ", maxlean)
        
        
        if -1 < minlean < 1 or -1 < maxlean < 1:
            if min(xy) < 1.4:
                self.velocity = 0
                self.brake = 1
            else:
                self.brake = 0
                self.velocity = 20
                if self.flag == False:
                    if abs(minlean) > abs(maxlean):
                        self.steering = 90
                        self.flag = True
                        print("오른쪽 조건 만족 플레그 값", self.flag)
                    elif abs(minlean) < abs(maxlean):
                        self.steering = -90
                        print("왼쪽 조건 만족 플레그 값", self.flag)
        else:
            self.steering = 0
            self.flag = False
            self.velocity = 20

        """
        '''
        최소 최대 기울기 구하기 
        for i in range(re_xaxis.size and re_yaxis.size):
            xy.append(re_yaxis[i]/re_xaxis[i])
            xypoint.append(math.sqrt((re_xaxis[i]**2)+(re_yaxis[i]**2)))
        if min(xypoint) < 1.5:
            pass

        print("최소 기울기 ", min(xy))
        print("최대 기울기 ", max(xy))
        print("최소 거리 ", min(xypoint))
        print("최대 거리 ", max(xypoint))
        '''

        header = Header()
        header.frame_id = "velodyne"
        scan = point_cloud2.create_cloud_xyz32(header, points)
        scan.header.stamp = rospy.Time.now()

        send.velocity = self.velocity
        send.steering = self.steering

        # send.accel = accel
        self.lidar_pub3D.publish(scan)
        self.send.publish(send)


if __name__ == '__main__':
    mct = moraiCtrl()
    Car_Follower()
    # mct = hunterCtrl()
    mct.runCtrl()
