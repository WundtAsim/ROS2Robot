import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
import numpy as np

class Test_publisher(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        self.command_publisher_ = self.create_publisher(Image,"test_publisher", 10) # the topic name is ""
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.bridge = CvBridge()
    
    def timer_callback(self):
        """
        定时器回调函数
        """
        im = cv2.imread('/home/yangqi/Pictures/rabbit.jpg',1)
        msg = self.bridge.cv2_to_imgmsg(im)  # Convert the image to a message

        self.command_publisher_.publish(msg) 
        self.get_logger().info(f'发布了image')    #打印一下发布的数据

def main():
    rclpy.init() # 初始化rclpy
    node = Test_publisher("test_publisher")  # 新建一个节点which name is ""
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy