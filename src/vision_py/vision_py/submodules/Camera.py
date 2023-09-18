import cv2
import numpy as np
import  UR_TCP_RTDE as UR


class Camera:
    def __init__(self):
        
        self.img_width = 1280
        self.img_height = 720
        self.CAM_NUM = 0
        self.R_tgt2cam = []
        self.t_tgt2cam = []
        self.R_grp2base = []
        self.t_grp2base = []
        self.count = 0
        
    def open(self):
        self.cap = cv2.VideoCapture(self.CAM_NUM)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_width * 2)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_height)
        return self.cap.isOpened()
    def close(self):
        self.cap.release()
    def read(self):
        retval, frame = self.cap.read()
        # left camera
        self.image = np.split(frame, 2, axis=1)[0] 
        # image color turn to RGB
        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)  
        self.image = cv2.resize(self.image, (self.img_width, self.img_height)) 
        # for show corners
        self.image_corners = self.image.copy()  

    def tgt2cam(self):
        # conver to gray image
        gray_img = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        # get points in world axis, mm
        self.pattern_shape = (4,6) # 4*6
        self.pattern_size = 0.055 ## 55mm
        corners3d_tgt = np.zeros((self.pattern_shape[0] * self.pattern_shape[1], 3), np.float32)
        corners3d_tgt[:, :2] = np.mgrid[0:self.pattern_shape[0], 0:self.pattern_shape[1]].T.reshape(-1, 2)
        corners3d_tgt *= self.pattern_size 
        success, points_2d = cv2.findChessboardCorners(gray_img, self.pattern_shape, None)
        if not success:
            print("no chess board!!")
            self.tgt2cam_r = None
            self.tgt2cam_t = None
            self.image_corners =None
            return None
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        exact_corners = cv2.cornerSubPix(gray_img, points_2d, (11, 11), (-1, -1), criteria)
        
        self.cam_info = np.array([[538.57, 0., 635.63],
                        [0., 538.5, 350.7075],
                        [0., 0., 1.]])
        self.cam_dist = np.array([13.258600234985352,
                        -10.219599723815918,
                        0.0003258869983255863,
                        -0.0002168240025639534,
                        1.5960400104522705,
                        13.655200004577637,
                        -10.19260025024414,
                        1.4755799770355225])
        # 获取外参
        _, tgt2cam_r, tgt2cam_t, inliers = cv2.solvePnPRansac(corners3d_tgt, exact_corners, self.cam_info, self.cam_dist)
        print("\n[tgt2cam_r]:\n{}\n[tgt2cam_t]:\n{}".format(tgt2cam_r, tgt2cam_t))
        self.tgt2cam_r = tgt2cam_r
        self.tgt2cam_t = tgt2cam_t
        self.image_corners = cv2.drawChessboardCorners(self.image_corners, self.pattern_shape, exact_corners, exact_corners is not None)

    # save one item for calibration
    def save_(self):
        cv2.imwrite('./data/calib_bk/images/left_{}.png'.format(self.count), 
                    cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR))
        self.R_tgt2cam.append(self.tgt2cam_r)
        self.t_tgt2cam.append(self.tgt2cam_t)

        # get the gripper2base tranlstation and rotation
        grp2base_r, grp2base_t = self.gripper2base_tcp()

        # 四元数转为旋转矩阵
        # self.R_grp2base_Quat.append(grp2base_r) # use rotation vector
        self.R_grp2base.append(grp2base_r)
        self.t_grp2base.append(grp2base_t)

        self.count+=1
    
    # get robot pose from tcp
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
    
    # calculate the matrix cam2grp
    def calc_cam2grp(self):
        s_tgt2cam_R = np.stack(self.R_tgt2cam)
        np.save('./data/calib_bk/tgt2cam_R', s_tgt2cam_R)
        s_tgt2cam_t = np.stack(self.t_tgt2cam)
        np.save('./data/calib_bk/tgt2cam_t', s_tgt2cam_t)

        s_grp2base_R = np.stack(self.R_grp2base)
        np.save('./data/calib_bk/grp2base_R', s_grp2base_R)
        s_grp2base_t = np.stack(self.t_grp2base)
        np.save('./data/calib_bk/grp2base_t', s_grp2base_t)

        self.R_cam2grp, self.t_cam2grp = cv2.calibrateHandEye(self.R_grp2base, self.t_grp2base, 
                                                    self.R_tgt2cam, self.t_tgt2cam)
        print("cam2grp_t:",self.t_cam2grp)
        np.save('./data/calib_bk/cam2grp_R', self.R_cam2grp)
        np.save('./data/calib_bk/cam2grp_t', self.t_cam2grp)
        print("flag:09/17 15:32")