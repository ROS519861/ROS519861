#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
print(cv2.__version__)  # 输出示例：4.5.5
# 全局变量
bridge = CvBridge()
current_image = None

def image_callback(msg):
    global current_image
    # 将ROS图像消息转换为OpenCV格式
    current_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def process_image(image):
    # 转换为灰度图
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 二值化处理
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    # 提取轮廓
    _, contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
    if contours:
        # 找到最大轮廓（黑线）
        largest_contour = max(contours, key=cv2.contourArea)
        # 计算黑线中心
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return cx, cy
    return None, None

def pid_control(error):
    Kp = 0.01  # 比例系数
    Ki = 0.0   # 积分系数
    Kd = 0.0   # 微分系数
    return Kp * error

def line_following():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        global current_image
        if current_image is not None:
            cx, cy = process_image(current_image)
            if cx is not None:
                # 计算偏差（黑线中心与图像中心的偏差）
                error = cx - current_image.shape[1] / 2
                # 创建Twist消息
                twist = Twist()
                twist.linear.x = 0.2  # 前进速度
                twist.angular.z = -pid_control(error)  # 转向控制
                # 发布速度指令
                pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('line_follower')
        # 订阅摄像头图像话题
        rospy.Subscriber('/cam', Image, image_callback)
        # 运行巡线逻辑
        line_following()
    except rospy.ROSInterruptException:
        pass
