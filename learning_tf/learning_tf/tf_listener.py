#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                                              # ROS2 Python介面庫
from rclpy.node import Node                               # ROS2 Node
import tf_transformations                                 # TF座標變換庫
from tf2_ros import TransformException                    # TF轉換異常類
from tf2_ros.buffer import Buffer                         # 儲存座標變換資訊的緩衝類
from tf2_ros.transform_listener import TransformListener  # 監聽座標變換的監聽器類

class TFListener(Node):

    def __init__(self, name):
        super().__init__(name)                                      # ROS2節點父類別初始化

        self.declare_parameter('source_frame', 'world')             # 建立一個來源座標系名的參數
        self.source_frame = self.get_parameter(                     # 優先使用外部設定的參數值，否則用預設值
            'source_frame').get_parameter_value().string_value

        self.declare_parameter('target_frame', 'house')             # 建立一個目標座標系名的參數
        self.target_frame = self.get_parameter(                     # 優先使用外部設定的參數值，否則用預設值
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()                                   # 建立保存座標變換資訊的緩衝區
        self.tf_listener = TransformListener(self.tf_buffer, self)  # 建立座標變換的監聽器

        self.timer = self.create_timer(1.0, self.on_timer)          # 建立一個固定週期的定時器，處理座標訊息

    def on_timer(self):
        try:
            now = rclpy.time.Time()                                 # 取得ROS系統的當前時間
            trans = self.tf_buffer.lookup_transform(                # 監聽當下時刻源座標係到目標座標系的座標變換
                self.target_frame,
                self.source_frame,
                now)
        except TransformException as ex:                            # 如果座標變換取得失敗，進入異常報告
            self.get_logger().info(
                f'Could not transform {self.target_frame} to {self.source_frame}: {ex}')
            return

        pos  = trans.transform.translation                          # 獲取位置信息
        quat = trans.transform.rotation                             # 取得姿態資訊（四元數）
        euler = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.get_logger().info('Get %s --> %s transform: [%f, %f, %f] [%f, %f, %f]' 
          % (self.source_frame, self.target_frame, pos.x, pos.y, pos.z, euler[0], euler[1], euler[2]))

def main(args=None):
    rclpy.init(args=args)                       # ROS2 Python介面初始化
    node = TFListener("tf_listener")            # 建立ROS2節點物件並進行初始化
    rclpy.spin(node)                            # 循環等待ROS2退出
    node.destroy_node()                         # 銷毀節點對象
    rclpy.shutdown()                            # 關閉ROS2 Python介面
