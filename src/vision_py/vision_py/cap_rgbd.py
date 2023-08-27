import argparse
import rclpy
from sensor_msgs.msg import Image
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import cv2
import numpy as np

from vision_py.subscribed_img_viewer import ImageSubscriber



class Cap_rgbd(ImageSubscriber):
    def __init__(self, name, 
                 image_topic,
                 base_link,
                 gripper_link):
        # initiate the node and give it a name
        super().__init__(name, image_topic)

        self.base_link = base_link
        self.gripper_link = gripper_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        grp2base_r, grp2base_t = self.gripper2base()
        
        cv2.imshow('RGBD', depth_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            # 采集深度图与位姿
            cv2.imwrite('./data/depth/depth_{}.png'.format(self.count),
                        depth_image)
            self.R_grp2base.append(self.quaternion_to_rotation_matrix(grp2base_r))
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
        '''
        same as transforms3d.quaternions.quat2mat: use parameter (w,x,y,z)
        '''
        x = q[0]
        y = q[1]
        z = q[2]
        w = q[3]
        rot_matrix = np.array(
            [[1.0 - 2*(y**2 + z**2),    2*(x*y - w*z),          2*(x*z + w*y)],
            [2*(x*y + w*z),             1.0 - 2*(x**2 + z**2),  2*(y*z - w*x)],
            [2*(x*z - w*y),             2*(y*z + w*x),          1.0 - 2*(x**2 + y**2)]],
            dtype=q.dtype)
        return rot_matrix

    


def main():
    parser = argparse.ArgumentParser()
    # 添加命令行参数
    parser.add_argument("--img", type=str, default='/zed2i/zed_node/depth/depth_registered', help="发布图像topic")
    parser.add_argument("--base", type=str, default='base', help="base 坐标系")
    parser.add_argument("--grp", type=str, default='tool0', help="gripper 坐标系")

    # 解析命令行参数
    args = parser.parse_args()

    rclpy.init()

    calculator = Cap_rgbd(
        'cap_rgbd',
        args.img,
        args.base,
        args.grp
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