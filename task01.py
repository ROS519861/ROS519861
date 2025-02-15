#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class RobustLineFollower:
    def __init__(self):
        rospy.init_node('robust_line_follower')
        
        # 初始化参数
        self._init_params()
        
        # 图像处理相关
        self.bridge = CvBridge()
        self.current_frame = None
        
        # 控制相关
        self.error_history = []
        self.prev_error = 0
        self.integral = 0
        
        # ROS接口
        self.image_sub = rospy.Subscriber("/cam", Image, self._image_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
       # rospy.loginfo("Line Follower Initialized [OpenCV 3.4.16]")

    def _init_params(self):
        """初始化可调参数"""
        # 图像处理
        self.roi_height_ratio = 0.4      # 底部40%作为关注区域
        self.gaussian_kernel = (7, 7)    # 高斯模糊核
        
        # PID参数
        self.Kp = 0.45     # 比例系数/0.35
        self.Ki = 0.0025    # 积分系数/0.0015
        self.Kd = 0.25     # 微分系数/0.16/0.20
        
        # 运动控制
        self.base_speed = 0.65     # 基础线速度(m/s)
        self.max_angular = 1.2     # 最大角速度(rad/s)/1.4
        self.search_speed = 0.25   # 丢失路线时的搜索速度

    def _image_callback(self, msg):
        """图像回调函数"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn("Image conversion error: %s" % e)

    def _process_image(self):
        """图像处理流水线"""
        # 转换为灰度图
        gray = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2GRAY)  
        # 高斯模糊降噪
        blurred = cv2.GaussianBlur(gray, self.gaussian_kernel, 0)      
        # 自适应阈值处理
        _, binary = cv2.threshold(blurred, 0, 255, 
                                cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)       
        # 提取ROI区域
        h, w = binary.shape
        roi = binary[int(h*(1-self.roi_height_ratio)):h, 0:w]
        # OpenCV 
        _, contours, _ = cv2.findContours(roi.copy(), 
                                        cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # 查找最大轮廓
            max_contour = max(contours, key=cv2.contourArea)
            
            # 计算轮廓矩
            M = cv2.moments(max_contour)
            if M["m00"] > 10:  # 忽略小噪点
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                # 转换为全局坐标
                global_cy = cy + int(h*(1-self.roi_height_ratio))
                return cx, global_cy
        return None, None

    def _pid_controller(self, error):
        """PID控制器实现"""
        self.integral += error
        derivative = error - self.prev_error
        output = (self.Kp * error + 
                self.Ki * self.integral + 
                self.Kd * derivative)
        self.prev_error = error
        return np.clip(output, -self.max_angular, self.max_angular)

    def _safe_publish(self, twist):
        """安全发布控制指令"""
        try:
            self.cmd_pub.publish(twist)
        except rospy.ROSException as e:
            rospy.logerr("Command publish failed: %s" % e)

    def run(self):
        """主控制循环"""
        rate = rospy.Rate(10)  # 10Hz
        twist = Twist()
        
        while not rospy.is_shutdown():
            if self.current_frame is not None:
                line_x, line_y = self._process_image()
                
                if line_x is not None:
                    # 计算横向偏差
                    frame_center = self.current_frame.shape[1] // 2
                    error = frame_center - line_x
                    # PID计算
                    angular = self._pid_controller(error)
                    # 动态速度调节
                    speed_factor = 1 - min(abs(angular)/self.max_angular, 0.7)
                    twist.linear.x = self.base_speed * speed_factor
                    twist.angular.z = angular
                else:
                    # 丢失路线处理
                    twist.linear.x = self.search_speed * 0.3
                    twist.angular.z = self.search_speed
                   # rospy.loginfo_throttle(1, "Searching for line...")
                
                self._safe_publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        follower = RobustLineFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
