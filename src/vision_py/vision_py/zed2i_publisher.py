import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import numpy as np

class Zed2i_publisher(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("%s created by vigilYang, published left_image_raw & left_camera_info." % name)
        self.image_publisher_ = self.create_publisher(Image,"my_zed2i/left_image_raw", 10) # the topic name is ""
        self.info_publisher_ = self.create_publisher(CameraInfo,"my_zed2i/left_camera_info", 10) # the topic name is ""
        self.image_timer = self.create_timer(1/30, self.image_timer_callback)
        self.info_timer = self.create_timer(1, self.info_timer_callback)
        self.bridge = CvBridge()

        # camera settings
        self.width = 1280
        self.height = 720
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width * 2)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
    
    def image_timer_callback(self):
        # call back of images
        if self.cap.isOpened() == 0:
            exit(-1)

        retval, frame = self.cap.read()
        images = np.split(frame, 2, axis=1)
        left_image = images[0]
        # (720, 1280, 3)
        # cv2.imshow("left_raw", left_image)
        # cv2.waitKey(1)

        msg = self.bridge.cv2_to_imgmsg(left_image)  # Convert the image to a message

        self.image_publisher_.publish(msg) 
        # self.get_logger().info(f'发布了image')    #打印一下发布的数据

    def info_timer_callback(self):
        # callback of info
        camera_info_msg = CameraInfo()
        camera_info_msg.width = self.width
        camera_info_msg.height = self.height
        # camera_info_msg.distortion_model = "plumb_bob"
        camera_info_msg.distortion_model = "rational_polynomial"
        camera_info_msg.k = [538.57, 0., 635.63,
                            0., 538.5, 350.7075,
                            0., 0., 1.]
        # camera_info_msg.d = [-0.06635, 0.041401, 0.000292541, -0.000472588, -0.0151654] # k1,k2,p1,p2,k3 
        camera_info_msg.d = [13.258600234985352,
                            -10.219599723815918,
                            0.0003258869983255863,
                            -0.0002168240025639534,
                            1.5960400104522705,
                            13.655200004577637,
                            -10.19260025024414,
                            1.4755799770355225]
        camera_info_msg.r = [1., 0., 0., 
                            0., 1., 0.,
                            0., 0., 1.]
        camera_info_msg.p = [538.57, 0., 635.63, 0.,
                            0., 538.5, 350.7075, 0.,
                            0., 0., 1., 0.]
        camera_info_msg.binning_x = 0
        camera_info_msg.binning_y = 0
        camera_info_msg.header.frame_id = "left_image_raw"
        camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.info_publisher_.publish(camera_info_msg)
        # print("published camera info")


def main():
    rclpy.init() # 初始化rclpy
    node = Zed2i_publisher("Zed2i_publisher")  # 新建一个节点which name is ""
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown() # 关闭rclpy