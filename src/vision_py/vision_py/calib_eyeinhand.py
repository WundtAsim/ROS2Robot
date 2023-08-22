import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import cv2
import numpy as np
from vision_py.tgt2cam import Calc_tgt2cam

class Calibrator_eyeinhand(Calc_tgt2cam):
    def __init__(self, name, 
                 image_topic,
                 info_topic,
                 base_link,
                 gripper_link):
        # initiate the node and give it a name
        super().__init__(name, image_topic, info_topic)
        
        self.base_link = base_link
        self.gripper_link = gripper_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # images that have taken
        self.count = 0
        self.R_tgt2cam = []
        self.t_tgt2cam = []
        self.R_grp2base = []
        self.R_grp2base_Quat = []
        self.t_grp2base = []

    def img_callback(self, msg):
        # convet the ROS image to Opencv bgr image
        current_frame = self.br.imgmsg_to_cv2(msg)
        
        # get the tgt2cam matrix
        points_2d, tgt2cam_r, tgt2cam_t = self.calc_tgt2cam(current_frame)
        
        # get the gripper2base tranlstation and rotation
        grp2base_r, grp2base_t = self.gripper2base()
        
        # show the corners & tips
        success = points_2d is not None
        if success:
            text = 'Press [space] to capture images, [{}] have taken.'.format(self.count)
        else:
            text = 'No chess board Conners found!'
        cv2.putText(current_frame, text, org=(10,30), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0,0,255), thickness=2)
        if self.count>=3:
            cv2.putText(current_frame, 'Press [Enter] to calibrate & quit.', 
                        org=(10,65), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0,0,255), thickness=2)
        
        # show the image
        cv2.drawChessboardCorners(current_frame, self.pattern_shape, points_2d, success)
        cv2.imshow('findCorners', current_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            # 旋转向量转为旋转矩阵
            # tgt2cam_r = cv2.Rodrigues(src=tgt2cam_r, jacobian=None)[0]
            self.R_tgt2cam.append(tgt2cam_r)
            self.t_tgt2cam.append(tgt2cam_t)

            # 四元数转为旋转矩阵
            self.R_grp2base_Quat.append(grp2base_r)
            self.R_grp2base.append(self.quaternion_to_rotation_matrix(grp2base_r))
            self.t_grp2base.append(grp2base_t)

            self.count+=1
        elif key == 13 and self.count>3:# Enter
            s_tgt2cam_R = np.stack(self.R_tgt2cam)
            np.save('./data/calib/tgt2cam_R', s_tgt2cam_R)
            s_tgt2cam_t = np.stack(self.t_tgt2cam)
            np.save('./data/calib/tgt2cam_t', s_tgt2cam_t)

            s_grp2base_R = np.stack(self.R_grp2base)
            np.save('./data/calib/grp2base_R', s_grp2base_R)
            np.save('./data/calib/grp2base_R_Quat', np.stack(self.R_grp2base_Quat))
            s_grp2base_t = np.stack(self.t_grp2base)
            np.save('./data/calib/grp2base_t', s_grp2base_t)

            R_cam2grp, t_cam2grp = cv2.calibrateHandEye(self.R_grp2base, self.t_grp2base, 
                                                        self.R_tgt2cam, self.t_tgt2cam)
            print("cam2grp_t:",t_cam2grp)
            np.save('./data/cam2grp_R', R_cam2grp)
            np.save('./data/cam2grp_t', t_cam2grp)
            quit()
        elif key == ord('q'):
            quit()
        

    def gripper2base(self):
        if self.tf_buffer.can_transform(self.base_link, self.gripper_link, rclpy.time.Time()):
            gripper2base = self.tf_buffer.lookup_transform(
                self.base_link,
                self.gripper_link,
                rclpy.time.Time()
            )

            gripper2base_t = gripper2base.transform.translation
            gripper2base_r = gripper2base.transform.rotation

            gripper2base_t_array = np.array([gripper2base_t.x,
                                             gripper2base_t.y,
                                             gripper2base_t.z])
            gripper2base_r_array = np.array([gripper2base_r.x,
                                             gripper2base_r.y,
                                             gripper2base_r.z,
                                             gripper2base_r.w])
            self.get_logger().info('[grp2base_r]:{}\n[grp2base_t]:{}'.format(gripper2base_r_array, gripper2base_t_array))
            return gripper2base_r_array, gripper2base_t_array
        else:
            return None, None
        
    def quaternion_to_rotation_matrix(self, q):  # x, y ,z ,w
        x = q[0]
        y = q[1]
        z = q[2]
        w = q[3]
        rot_matrix = np.array(
            [[1.0 - 2*(y**2 + z**2),    2*(x*y - w*z),          2*(x*z + w*y)],
            [2*(x*y + w*z),             1.0 - 2*(x**2 + z**2),  2*(y*z - w*x)],
            [2*(x*z - w*y),             2*(y*z - w*x),          1.0 - 2*(x**2 + y**2)]],
            dtype=q.dtype)
        return rot_matrix

    


def main():
    parser = argparse.ArgumentParser()
    # 添加命令行参数
    parser.add_argument("--img", type=str, default='/zed2i/zed_node/left_raw/image_raw_color', help="发布图像topic")
    parser.add_argument("--info", type=str, default='/zed2i/zed_node/left/camera_info', help="发布相机参数topic")
    parser.add_argument("--base", type=str, default='base_link_inertia', help="base 坐标系")
    parser.add_argument("--grp", type=str, default='wrist_3_link', help="gripper 坐标系")


    # 解析命令行参数
    args = parser.parse_args()

    rclpy.init()

    calibrator = Calibrator_eyeinhand(
        'calibrator',
        args.img,
        args.info,
        args.base,
        args.grp
        )
    try:
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    # destory the node explicitly
    calibrator.destroy_node()
    # shutdown the ROS client library for python
    rclpy.shutdown()