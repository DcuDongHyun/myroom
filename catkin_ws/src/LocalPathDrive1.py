import copy

import rospy
import ros_numpy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan , PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Float64 , Header
from morai_msgs.msg import CtrlCmd

class Car_Follower():
    def __init__(self):

        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.sub3D = rospy.Subscriber("/lidar3D", PointCloud2, self.callback3D)

        self.pub = rospy.Publisher("/follwer", AckermannDriveStamped, queue_size=1)
        self.lidar_pub = rospy.Publisher("/refined_lidar", LaserScan, queue_size=10)
        self.lidar_pub3D = rospy.Publisher("/refined_lidar3D", PointCloud2, queue_size=10) #velodyne
        self.speedpub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.po = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.po3D = rospy.Publisher("/speed", CtrlCmd, queue_size=1)
        self.count = 0


    def degTorad(self, deg):
        rad_diff = 0.5304
        rad = deg * (3.14 / 180)
        return rad + rad_diff

    def callback(self, msg):
        '''
        콜백의 해당 부분에서 LiDAR 데이터를 바탕으로 차량의 조향, 속도를 판단한 뒤, 아래의 steering_angle, speed에 값을 넣으면 됨.
        '''
        """angle_increment: 0.01745329238474369 이 값은 라디안 값 라디안 값을 우리가 쓰는 도 다위로 바꾸면 약 1"""
        calc_angle = 0.0
        max = 8000.0
        speed = 3000
        angle = 0.0

        scan = LaserScan()

        scan.header = msg.header

        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment

        scan.scan_time = msg.scan_time

        scan.range_min = msg.range_min
        scan.range_max = float("inf")
        tmp_range = list()
        size = len(msg.ranges)

        tmp_range = list(copy.deepcopy(msg.ranges))

        for i in range(0, size):
            # print(i)
            if (i >= 0 and i < 90) or (i >= 270 and i < 360):
                if msg.ranges[i] <= 1:
                    continue
                else:
                    # tmp_range[i] = 10
                    tmp_range[i] = float("inf")
            else:
                # tmp_range[i] = 10

                tmp_range[i] = float("inf")
                # tmp_range.append(msg.ranges[i])
        msgmin = min(tmp_range)
        msgminIndex = tmp_range.index(msgmin)
        calc_angle = msgminIndex
        if msgmin > 0.28 or 85< msgminIndex <90:
            speed = 3000
            if 0< msgminIndex < 80 or 320< msgminIndex <360:
                angle = 60
                self.count += 2
            elif 85< msgminIndex < 90:
                if self.count != 0:
                    self.count -= 1
                    angle = - 60
                else:
                    angle = 0
        else:
            speed = 0
            angle = 0

        print("카운터:", self.count)
        print("msg:", msgmin)
        print("앵글:", angle)
        # print("인덱스 2:",msgminIndex)

        # if msgmin <= 1:
        #     speed -= 4000.0
        #     if 0 < calc_angle < 180:
        #         angle = -calc_angle * 0.7
        #     elif 180 < calc_angle < 360:
        #         angle = calc_angle * 0.7
        #     else:
        #         angle = 0
        # elif 1< msgmin < 5:
        #     if speed <max:
        #         speed +=2000.0
        #     else:
        #         speed = max
        # else:
        #     speed = 0.0

        """라이다의 직선 상과 제일 근접한 포인트의 각도 구하기 """
        # print('test')
        # print(anglemin)
        min(tmp_range)
        scan.ranges = tmp_range
        scan.intensities = [0] * size

        # self.pub.publish(ackermann_data)
        self.lidar_pub.publish(scan)
        # self.speedpub.publish(speed)
        # self.po.publish(self.degTorad(angle))


    def callback3D(self,msg):
        ctrcmd = CtrlCmd()
        ctrcmd.accel = 1000

        scan = PointCloud2()
        scan.header = msg.header
        'header', 'height', 'width', 'fields', 'is_bigendian', 'point_step', 'row_step', 'data', 'is_dense'
        scan.height = msg.height
        scan.width = msg.width
        scan.fields = msg.fields
        scan.is_bigendian = msg.is_bigendian
        scan.point_step = msg.point_step
        scan.row_step = msg.row_step
        scan.data = msg.data
        scan.is_dense = msg.is_dense
        size = len(msg.data)
        tmp_range = list(copy.deepcopy(msg.data))

        pc = ros_numpy.numpify(scan)
        points = np.zeros((pc.shape[0], 3))

        points[:, 0] = pc['x']
        points[:, 1] = pc['y']
        points[:, 2] = pc['z']

        roi = {"x": [0,2], "y": [-2,2], "z": [0,3]}  # z값 수정, X 값 수정으로 전,후방 범위 조절

        x_range = np.logical_and(points[:, 0] >= roi["x"][0], points[:, 0] <= roi["x"][1])
        y_range = np.logical_and(points[:, 1] >= roi["y"][0], points[:, 1] <= roi["y"][1])
        z_range = np.logical_and(points[:, 2] >= roi["z"][0], points[:, 2] <= roi["z"][1])

        pass_through_filter = np.where(np.logical_and(x_range, np.logical_and(y_range, z_range)) == True)[0]
        points = points[pass_through_filter, :]
        if np.mean(points[:,0]) > 0.2:
            ctrcmd.accel = 0
            ctrcmd.brake = 10000
            ctrcmd.steering = 0
            ctrcmd.velocity =0
            ctrcmd.acceleration =0

        print("좌표:",points[:,0])
        print("평값:",np.mean(points[:,0]))
        # min(points)
        header = Header()
        header.frame_id = "velodyne"
        scan = point_cloud2.create_cloud_xyz32(header,points)


        scan.header.stamp = rospy.Time.now()
        #print(min(points[:,0]))
        self.lidar_pub3D.publish(scan)
        self.po3D.publish(ctrcmd)


if __name__ == '__main__':
    rospy.init_node("Car_Follwer")
    Car_Follower()
    rospy.spin()