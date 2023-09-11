import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np

from vision_py.subscribed_img_viewer import ImageSubscriber



class Calc_tgt2cam(ImageSubscriber):
    def __init__(self, name, 
                 image_topic,
                 info_topic):
        # initiate the node and give it a name
        super().__init__(name, image_topic)

        # create the img_subscriber, receive the image
        self.img_subscription = self.create_subscription(
            msg_type = Image,
            topic = image_topic,
            callback = self.img_callback,
            qos_profile = 10
        ) 

        # create the info subscriber, receive the camera info
        self.info_subscription = self.create_subscription(
            msg_type = CameraInfo,
            topic = info_topic,
            callback = self.info_callback,
            qos_profile = 10
        )

    def info_callback(self, msg):
        self.cam_info = msg.k.reshape(3,3)
        self.cam_dist = np.array(msg.d)

    def img_callback(self, msg):
        # convet the ROS image to Opencv bgr image
        current_frame = self.br.imgmsg_to_cv2(msg)
        # get the tgt2cam matrix
        points_2d, tgt2cam_r, tgt2cam_t = self.calc_tgt2cam(current_frame)
            
        # show the corners
        success = points_2d is not None
        if success:
            text = 'successful!'
        else:
            text = 'No chess board Conners found!'
        cv2.putText(current_frame, text, org=(10,50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0,0,200), thickness=2)
        cv2.drawChessboardCorners(current_frame, self.pattern_shape, points_2d, success)
        cv2.imshow('findCorners', current_frame)
        cv2.waitKey(500)
    
    def calc_tgt2cam(self, img:np.ndarray):
        # conver to gray image
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # get points in world axis, mm
        self.pattern_shape = (4,6) # 4*6
        self.pattern_size = 0.055 ## 55mm
        self.corners3d_tgt = np.zeros((self.pattern_shape[0] * self.pattern_shape[1], 3), np.float32)
        self.corners3d_tgt[:, :2] = np.mgrid[0:self.pattern_shape[0], 0:self.pattern_shape[1]].T.reshape(-1, 2)
        self.corners3d_tgt *= self.pattern_size # 55mm
        success, points_2d = cv2.findChessboardCorners(gray_img, self.pattern_shape, None)
        if not success:
            self.get_logger().info("NO Chessboard corners were found.")
            return None, None, None
        
        if self.cam_info is None:
            self.get_logger().info("NO Camera info was found.")
            return None, None, None
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        exact_corners = cv2.cornerSubPix(gray_img, points_2d, (11, 11), (-1, -1), criteria)
        
        # self.get_logger().info("\n[cam_info]:\n{}\n[cam_dist]:\n{}".format(self.cam_info, self.cam_dist))
        # 获取外参
        _, tgt2cam_r, tgt2cam_t, inliers = cv2.solvePnPRansac(self.corners3d_tgt, exact_corners, self.cam_info, self.cam_dist)
        # success, tgt2cam_r, tgt2cam_t= cv2.solvePnP(self.corners3d_tgt, exact_corners, self.cam_info, self.cam_dist)
        self.get_logger().info("\n[tgt2cam_r]:\n{}\n[tgt2cam_t]:\n{}".format(tgt2cam_r, tgt2cam_t))
        return exact_corners, tgt2cam_r, tgt2cam_t


def main():
    parser = argparse.ArgumentParser()
    # 添加命令行参数
    parser.add_argument("--img", type=str, default='/zed2i/zed_node/left/image_rect_color', help="发布图像topic")
    parser.add_argument("--info", type=str, default='/zed2i/zed_node/left/camera_info', help="发布相机参数topic")

    # 解析命令行参数
    args = parser.parse_args()

    rclpy.init()

    calculator = Calc_tgt2cam(
        'calculator',
        args.img,
        args.info
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