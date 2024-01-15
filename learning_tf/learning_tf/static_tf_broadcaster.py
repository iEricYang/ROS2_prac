#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                                                                 # ROS2 Python介面庫
from rclpy.node import Node                                                  # ROS2 Node
from geometry_msgs.msg import TransformStamped                               # 座標變換訊息
import tf_transformations                                                    # TF座標變換庫
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster  # TF靜態坐標系廣播器類

class StaticTFBroadcaster(Node):
    
    def __init__(self, name):
        super().__init__(name)                                                  # ROS2節點父類別初始化
        self.tf_broadcaster = StaticTransformBroadcaster(self)                  # 建立一個TF廣播器對象

        static_transformStamped = TransformStamped()                            # 建立一個座標變換的訊息對象
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()  # 設定座標變換訊息的時間戳

        static_transformStamped.header.frame_id = 'world'                       # 設定一個座標變換的源座標系
        static_transformStamped.child_frame_id  = 'house'                       # 設定一個座標變換的目標座標系

        static_transformStamped.transform.translation.x = 5.0                  # 設定座標變換中的X、Y、Z向的平移
        static_transformStamped.transform.translation.y = 2.0                    
        static_transformStamped.transform.translation.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)          # 將歐拉角轉換為四元數（roll, pitch, yaw）
        static_transformStamped.transform.rotation.x = quat[0]                  # 設定座標變換中的X、Y、Z向的旋轉（四元數）
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(static_transformStamped)              # 廣播靜態座標變換，廣播後兩個座標系的位置關係不變

def main(args=None):
    rclpy.init(args=args)                                # ROS2 Python介面初始化
    node = StaticTFBroadcaster("static_tf_broadcaster")  # 建立ROS2節點物件並進行初始化
    rclpy.spin(node)                                     # 循環等待ROS2退出
    node.destroy_node()                                  # 銷毀節點對象
    rclpy.shutdown()
