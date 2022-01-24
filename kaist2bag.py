# # coding=utf-8
# # 这是一个示例 Python 脚本。
#
# # 按 Shift+F10 执行或将其替换为您的代码。
# # 按 双击 Shift 在所有地方搜索类、文件、工具窗口、操作和设置。
#
#
# def print_hi(name):
#     # 在下面的代码行中使用断点来调试脚本。
#     print("Hi, {0}".format(name))  # 按 Ctrl+F8 切换断点。
#
#
# # 按间距中的绿色按钮以运行脚本。
# if __name__ == '__main__':
#     print_hi('PyCharm')
#
# # 访问 https://www.jetbrains.com/help/pycharm/ 获取 PyCharm 帮助
import roslib
import numpy as np
import rospy
import codecs
import pandas as pd
import rospy
import csv
import cv2

from PIL import ImageFile
from PIL import Image as ImagePIL

import sensor_msgs.msg
from ros import rosbag
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from cv_bridge import CvBridge
from numpy import asarray


roslib.load_manifest('sensor_msgs')

# p = r'/media/devil/ZX1_512G/dataset/public/ubran28/sensor_data/data_stamp.csv'
p = r'/media/devil/ZX1_512G/dataset/public/ubran28/sensor_data/data_stamp(复件).csv'
with open(p) as f:
    data_stamp = np.loadtxt(f,str,delimiter = ",")
data_stamp[0,0]
#图片的路径
image_path_left = '/media/devil/ZX1_512G/dataset/public/ubran28/image/stereo_left/'
image_path_right = '/media/devil/ZX1_512G/dataset/public/ubran28/image/stereo_right/'

#imu的数据
imu_path = '/media/devil/ZX1_512G/dataset/public/ubran28/sensor_data/xsens_imu.csv'


#encoder的数据
encoder_path = '/media/devil/ZX1_512G/dataset/public/ubran28/sensor_data/encoder.csv'


# #gps的数据
gps_path = '/media/devil/ZX1_512G/dataset/public/ubran28/sensor_data/gps.csv'

imu_data = np.loadtxt(imu_path,str,delimiter = ",")

imu_dict = {}
num = imu_data.shape[0]
for i in range(num):
    imu_dict[imu_data[i, 0]] = imu_data[i, 1:17]

encoder_data = np.loadtxt(encoder_path,str,delimiter = ",")

encoder_dict = {}
num = encoder_data.shape[0]
for j in range(num):
    encoder_dict[encoder_data[j, 0]] = encoder_data[j, 1:]

gps_data = np.loadtxt(gps_path,str,delimiter = ",")
gps_dict = {}
num = gps_data.shape[0]
for j in range(num):
    gps_dict[gps_data[j, 0]] = gps_data[j, 1:]

encoderin = -1
temp_timestamp_begin = 0
temp_timestamp_end = 0
wheel_begin_left = 0
wheel_begin_right = 0
wheel_end_left = 0
wheel_end_right = 0

# encoder Intrinsic
kl = 0.00047820240382508
kr = 0.00047768621928995
b = 1.52439

try:
    bag =rosbag.Bag("urban28_5.bag", 'w')
    num = data_stamp.shape[0]
    for i in range(num):
        if data_stamp[i,1] == 'stereo':

            imgs_left = image_path_left
            imgs_right = image_path_right

            temp_timestamp = int(data_stamp[i,0])

            fp_left = open( image_path_left+data_stamp[i,0]+".png", "r" )

            fp_right = open(image_path_right + data_stamp[i, 0] + ".png", "r")


            p = ImageFile.Parser()

            '''read image size'''
            imgpil = ImagePIL.open(image_path_left+data_stamp[i,0]+".png")
            img_l = cv2.imread(image_path_left+data_stamp[i,0]+".png")
            imgpil = ImagePIL.open(image_path_right+data_stamp[i,0]+".png")
            img_r = cv2.imread(image_path_right+data_stamp[i,0]+".png")
            Img_left = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
            Img_right = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

            width, height = imgpil.size

            while 1:
                s = fp_left.read(1024)
                if not s:
                    break
                p.feed(s)

            im = p.close()

            temp_timestamp=np.asarray([temp_timestamp], dtype=np.float64)/ 1e9
            temp_timestamp = temp_timestamp[0]
           # print temp_timestamp
            Stamp = rospy.rostime.Time.from_sec(temp_timestamp)

            '''set image information '''
            Img_left = Image()
            Img_left.header.stamp = Stamp
            Img_left.height = height
            Img_left.width = width
            Img_left.header.frame_id = "camera"

            Img_right = Image()
            Img_right.header.stamp = Stamp
            Img_right.height = height
            Img_right.width = width
            Img_right.header.frame_id = "camera"

            '''for mono8'''
            Img_left.encoding = "mono8"
            Img_left_data = [pix for pixdata in [im.getdata()] for pix in pixdata]
            Img_left.step = Img_left.width
            #
            Img_right.encoding = "mono8"
            Img_right_data = [pix for pixdata in [im.getdata()] for pix in pixdata]
            Img_right.step = Img_right.width


            Img_left.data = Img_left_data
            Img_right.data = Img_right_data

            bag.write('/image/left_raw', Img_left, Stamp)
            bag.write('/image/right_raw', Img_right, Stamp)

        elif data_stamp[i,1] == 'imu':
            imu = Imu()
            imu.header.frame_id = "imu"

            temp_timestamp = int(data_stamp[i, 0])
            a = temp_timestamp
            temp_timestamp = np.asarray([temp_timestamp], dtype=np.float64) / 1e9
            temp_timestamp = temp_timestamp[0]

            imu.header.stamp = rospy.rostime.Time.from_sec(temp_timestamp)
            temp = str(data_stamp[i,0])

            imu.orientation.x = float(imu_dict[temp][0])
            imu.orientation.y = float(imu_dict[temp][1])
            imu.orientation.z = float(imu_dict[temp][2])
            imu.orientation.w = float(imu_dict[temp][3])

            imu.linear_acceleration.x = float(imu_dict[temp][10])
            imu.linear_acceleration.y = float(imu_dict[temp][11])
            imu.linear_acceleration.z = float(imu_dict[temp][12])

            imu.angular_velocity.x = float(imu_dict[temp][7])
            imu.angular_velocity.y = float(imu_dict[temp][8])
            imu.angular_velocity.z = float(imu_dict[temp][9])

            bag.write('/imu/data', imu, t=imu.header.stamp)
        elif data_stamp[i, 1] == 'encoder':

            encoder = Odometry()
            encoder_wheel = TwistStamped()

            temp_timestamp_begin = temp_timestamp_end
            wheel_begin_left = wheel_end_left
            wheel_begin_right = wheel_end_right

            temp_timestamp = int(data_stamp[i, 0])
            temp_timestamp_end = np.asarray([temp_timestamp], dtype=np.float64) / 1e9
            temp_timestamp_end = temp_timestamp_end[0]

            temp_timestamp_end = rospy.rostime.Time.from_sec(temp_timestamp_end)

            temp = str(data_stamp[i, 0])

            wheel_end_left = float(encoder_dict[temp][0])
            wheel_end_right = float(encoder_dict[temp][1])

            encoderin = encoderin + 1

            if encoderin != 0:
                left_dist = (wheel_end_left - wheel_begin_left) * kl
                right_dist = (wheel_end_right - wheel_begin_right) * kr
                delta_yaw = (right_dist - left_dist) / b
                delta_dist = (right_dist + left_dist) * 0.5

                t1 = temp_timestamp_begin.to_sec()
                t2 = temp_timestamp_end.to_sec()
                delta_dist = (right_dist + left_dist) * 0.5 / (t2 - t1)
                delta_yaw = delta_yaw = (right_dist - left_dist) / (b*(t2 - t1))
                timestamp = (t1 + t2) * 0.5
                encoder.twist.twist.linear.x = delta_dist
                encoder.twist.twist.linear.y = 0
                encoder.twist.twist.linear.z = 0
                encoder.twist.twist.angular.x = 0
                encoder.twist.twist.angular.y = 0
                encoder.twist.twist.angular.z = delta_yaw
                encoder.header.stamp = rospy.rostime.Time.from_sec(timestamp)

                encoder_wheel.twist.linear.x = delta_dist
                encoder_wheel.twist.linear.y = 0
                encoder_wheel.twist.linear.z = 0
                encoder_wheel.twist.angular.x = 0
                encoder_wheel.twist.angular.y = 0
                encoder_wheel.twist.angular.z = delta_yaw
                encoder_wheel.header.stamp = rospy.rostime.Time.from_sec(timestamp)
                bag.write('/Odometry/encoder', encoder, t=encoder.header.stamp)
                bag.write('/TwistStamped/encoder', encoder_wheel, t=encoder_wheel.header.stamp)

            encoder_cout = Odometry()
            encoder_cout.header.frame_id = "encoder_cout"

            temp_timestamp = int(data_stamp[i, 0])
            a = temp_timestamp
            temp_timestamp = np.asarray([temp_timestamp], dtype=np.float64) / 1e9
            temp_timestamp = temp_timestamp[0]


            encoder_cout.header.stamp = rospy.rostime.Time.from_sec(temp_timestamp)
            temp = str(data_stamp[i, 0])

            encoder_cout.twist.twist.linear.x = float(encoder_dict[temp][0])
            encoder_cout.twist.twist.linear.y = float(encoder_dict[temp][1])
            bag.write('/Odometry/encoder/count', encoder_cout, t = encoder_cout.header.stamp)

        elif data_stamp[i, 1] == 'gps':
            gps = NavSatFix()
            gps.header.frame_id = "gps"

            temp_timestamp = int(data_stamp[i, 0])
            a = temp_timestamp
            temp_timestamp = np.asarray([temp_timestamp], dtype=np.float64) / 1e9
            temp_timestamp = temp_timestamp[0]

            gps.header.stamp = rospy.rostime.Time.from_sec(temp_timestamp)
            temp = str(data_stamp[i, 0])

            gps.latitude = float(gps_dict[temp][0])
            gps.longitude = float(gps_dict[temp][1])
            gps.altitude = float(gps_dict[temp][2])
            for j in range(0, 9):
                gps.position_covariance[j] = float(gps_dict[temp][3+j])

            bag.write('/gps/data_raw', imu, t=gps.header.stamp)

finally:
    bag.close()





