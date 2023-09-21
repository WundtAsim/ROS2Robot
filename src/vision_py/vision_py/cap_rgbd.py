import argparse

from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image

import cv2
import numpy as np
import  UR_TCP_RTDE as UR

from vision_py.subscribed_img_viewer import ImageSubscriber



class Cap_rgbd(Node):
    def __init__(self, name):
        # initiate the node and give it a name
        super().__init__(name)
        # declare parameters
        self.declare_parameter('image_topic', '/zed2i/zed_node/depth/depth_registered')
        image_topic = self.get_parameter("image_topic").value
        # create the img_subscriber, receive the image
        self.img_subscription = self.create_subscription(
            msg_type = Image,
            topic = image_topic,
            callback = self.img_callback,
            qos_profile = 10
        ) 

        self.count = 0
        self.R_grp2base = []
        self.t_grp2base = []


    def img_callback(self, msg):
        # convet the ROS image to Opencv bgr image
        current_frame = self.br.imgmsg_to_cv2(msg)
        # change the float32 to uint8, with 4 channels(low-bit-->high-bit)
        depth_image = current_frame.view(np.uint8).reshape(current_frame.shape + (4,))
        
        # get the gripper2base tranlstation and rotation
        grp2base_r, grp2base_t = self.gripper2base_tcp()
        
        cv2.imshow('RGBD', depth_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            # 采集深度图与位姿
            cv2.imwrite('./data/depth/depth_{}.png'.format(self.count),
                        depth_image)
            self.R_grp2base.append(grp2base_r)
            self.t_grp2base.append(grp2base_t)
            self.count+=1
            print("collected {} frames.".format(self.count))
        elif key == ord('q'):
            s_grp2base_R = np.stack(self.R_grp2base)
            np.save('./data/depth/grp2base_R', s_grp2base_R)
            s_grp2base_t = np.stack(self.t_grp2base)
            np.save('./data/depth/grp2base_t', s_grp2base_t)
            print("flag:8/27 15:08")
            quit()


    def gripper2base_tcp(self):
        # for tcp connected
        self.TCP_socket = UR.connect('192.168.1.24',30003)
        data = self.TCP_socket.recv(1116)
        position = UR.get_position(data)
        print('position:=',position)
        pos = position[:3]
        rotation = position[3:] # rotation vector
        UR.disconnect(self.TCP_socket)
        print("TCP disconnected...")
        return rotation, pos

    


def main():
    rclpy.init()

    calculator = Cap_rgbd(
        'cap_rgbd'
        )
    try:
        rclpy.spin(calculator)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    # destory the node explicitly
    calculator.destroy_node()
    # shutdown the ROS client library for python
    rclpy.shutdown()