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

from PIL import ImageFile
from PIL import Image as ImagePIL
from ros import rosbag
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


roslib.load_manifest('sensor_msgs')

p = r'/media/devil/ZX1_512G/dataset/public/ubran28/sensor_data/data_stamp(复件).csv'

with open(p) as f:
    data_stamp = np.loadtxt(f,str,delimiter = ",")
data_stamp[0,0]
#图片的路径
image_path_left = '/media/devil/ZX1_512G/dataset/public/ubran28/image/stereo_left/'
image_path_right = '/media/devil/ZX1_512G/dataset/public/ubran28/image/stereo_right/'

#imu的数据
imu_path = '/media/devil/ZX1_512G/dataset/public/ubran28/sensor_data/xsens_imu(复件).csv'

#e的数据
encoder_path = '/media/devil/ZX1_512G/dataset/public/ubran28/sensor_data/encoder(复件).csv'

# imu_dict = {}
# with codecs.open(imu_path, 'r') as imu:
#     imu_message = csv.reader(imu)
#     for imu_key in imu_message:
#         # print('字典的key值：%s' % imu_key)
#         imu_reader = csv.DictReader(imu, fieldnames=imu_key)
#         for row in imu_reader:
#             imu_dict[row['timestamp']] = [row['quaternion x'], row['quaternion y'], row['quaternion z'], row['quaternion w'], row['Euler x'], row['Euler y'], row['Euler z'], row['Gyro x'], row['Gyro y'], row['Gyro z'], row['Acceleration x'], row['Acceleration y'], row['Acceleration z'], row['MagnetField x'], row['MagnetField y'], row['MagnetField z']]
#             # a= int(row['timestamp']
#             # imu_dict.[a]=imu_dict.pop(row['timestamp'])
imu_data = np.loadtxt(imu_path,str,delimiter = ",")

imu_dict = {}
num = imu_data.shape[0]
for i in range(num):
    imu_dict[imu_data[i, 0]] = imu_data[i, 1:17]


# imu_date_length = range(imu_data[0])
# encoder_dict = {}
# with codecs.open(encoder_path, 'r') as encoder:
#     encoder_message = csv.reader(encoder)
#     for encoder_key in encoder_message:
#         # print('字典的key值：%s' % encoder_key)
#         encoder_reader = csv.DictReader(encoder, fieldnames=encoder_key)
#         for row in encoder_reader:
#             encoder_dict[row['timestamp']] = [row['left count'], row['right count']]
            # a= int(row['timestamp']
            # imu_dict.[a]=imu_dict.pop(row['timestamp'])
encoder_data = np.loadtxt(encoder_path,str,delimiter = ",")

encoder_dict = {}
num = encoder_data.shape[0]
for j in range(num):
    encoder_dict[encoder_data[j, 0]] = encoder_data[j, 1:]
# encoder_date_length = range(encoder_data[0])
# print(imu_dict)

# a = 1544590798706652875
# b=str(a)
# print('wo'+b)
#
# print(imu_dict[b])
#
# c="1544590798706652875"
#
# print(imu_dict[c])



#轮速记的数据
# encoder_path ='/media/devil/ZX1_512G/dataset/public/ubran28/sensor_data/encoder.csv'
# with codecs.open(encoder_path, 'r') as encoder:
#     encoder_message = csv.reader(encoder)
#     for encoder_key in encoder_message:
#         # print('字典的key值：%s' % encoder_key)
#         encoder_reader = csv.DictReader(encoder, fieldnames=encoder_key)
#         # print('DictReader()方法返回值：%s' % encoder_reader)
#         for row in encoder_reader:
#             encoder_dict = dict(row)


# print(data_stamp.shape[0])
# fp = open( image_path+"/1544590798702415701"+".png", "r" )
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
    bag =rosbag.Bag("urban28_2.bag", 'w')
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
            width, height = imgpil.size
            # print "size:",width,height

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


            # '''for rgb8'''
            # Img.encoding = "rgb8"
            # Img_data = [pix for pixdata in im.getdata() for pix in pixdata]
            # Img.step = Img.width * 3

            '''for mono8'''
            Img_left.encoding = "mono8"
            Img_left_data = [pix for pixdata in [im.getdata()] for pix in pixdata]
            Img_left.step = Img_left.width

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

            # kite

            # a = float(imu_data[temp,1 ])
            # b = float(imu_data[temp][1])
            # c = float(imu_data[temp][2])
            # d = float(imu_data[temp][3])
            # f = float(imu_data[temp][10])
            # g = float(imu_data[temp][11])
            # m = float(imu_data[temp][12])
            # h = float(imu_data[temp][7])
            # i = float(imu_data[temp][8])
            # j = float(imu_data[temp][9])

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
        elif data_stamp[i, 1] == 'encoder' :

            encoder = Odometry()
            encoder_wheel = TwistStamped()

            temp_timestamp_begin = temp_timestamp_end
            wheel_begin_left = wheel_end_left
            wheel_begin_right = wheel_end_right


            temp_timestamp = int(data_stamp[i, 0])
            temp_timestamp_end = np.asarray([temp_timestamp], dtype=np.float64) / 1e9
            temp_timestamp_end = temp_timestamp_end[0]


            temp_timestamp_end= rospy.rostime.Time.from_sec(temp_timestamp_end)

            temp = str(data_stamp[i, 0])

            wheel_end_left = float(encoder_dict[temp][0])
            wheel_end_right = float(encoder_dict[temp][1])

            encoderin = encoderin + 1

            if encoderin != 0:
                left_dist = (wheel_end_left-wheel_begin_left)*kl
                right_dist = (wheel_end_right - wheel_begin_right) * kl
                delta_yaw = (right_dist-left_dist)/b
                delta_dist = (right_dist+left_dist)*0.5
                timestamp  = (temp_timestamp_begin.to_sec()+temp_timestamp_end.to_sec())*0.5


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
                # t = rospy.rostime.Time.from_sec(encoder.header.stamp)
            #
            # if (i+1)<num:
            #     temp_timestamp = int(data_stamp[i, 0])
            #     temp_timestamp_begin = np.asarray([temp_timestamp], dtype=np.float64) / 1e9
            #
            #     temp = str(data_stamp[i, 0])
            #     en_left = float(encoder_dict[temp][0])
            #     en_right = float(encoder_dict[temp][1])
            #
            #     temp_timestamp = int(data_stamp[i+1, 0])
            #     temp_timestamp_end = np.asarray([temp_timestamp], dtype=np.float64) / 1e9
            #
            #     temp_timestamp = (temp_timestamp_begin+temp_timestamp_end)*0.5
            #
            #     # temp_timestamp = temp_timestamp[0]
            #     encoder.header.stamp = rospy.rostime.Time.from_sec(temp_timestamp)
            #
            #     m_left_begin = float(encoder_dict[temp][0])
            #     m_right_begin =
            #
            #
            #
            # temp = str(data_stamp[i, 0])
            #
            #
            # temp = str(data_stamp[i, 0])
            # # kite
            # m = float(encoder_dict[temp][0])
            # n = float(encoder_dict[temp][1])
            #
            # encoder.twist.twist.linear.x=m
            # encoder.twist.twist.linear.x = n
                bag.write('/Odometry/encoder', encoder, t = encoder.header.stamp)
                bag.write('/TwistStamped/encoder', encoder_wheel, t = encoder_wheel.header.stamp)

finally:
    bag.close()







#print(data)


#data_stamp = np.genfromtxt('/media/devil/PortableSSD/small_dataset/ubran28/sensor_data/data_stamp.csv', delimiter=',')
#data_stamp = pd.read_csv("/media/devil/PortableSSD/small_dataset/ubran28/sensor_data/data_stamp.csv",header=None,names=["timestamp","sensor"])

#print(data_stamp)
# print(data_stamp)






