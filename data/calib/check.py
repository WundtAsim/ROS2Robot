import numpy as np
import cv2
from transforms3d.quaternions import quat2mat

PATH='./data/calib/'
def distance(c1, c2):
    c = np.sum((c1-c2)**2)
    return np.sqrt(c)

def target2camera(file):
    '''
    :param file: 单张标定图片
    :return: （旋转矩阵，平移向量）
    '''
    w = 4
    h = 6
    mtx = np.array([[532.72, 0, 639.53],
                    [0, 532.72, 360.6],
                    [0, 0, 1]], np.float32)
    dist = np.array([0,0,0,0,0], np.float32)
    
    image = cv2.imread(file)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # get points in world axis, mm
    obj_point = np.zeros((w * h, 3), np.float32)
    obj_point[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2)
    obj_point = obj_point * 0.055  # 55mm
    # get points in target board
    ok, corners = cv2.findChessboardCorners(gray, (4,6), None)
    if ok:
        exact_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        cv2.drawChessboardCorners(gray, (4, 6), exact_corners, ok)
        cv2.imshow('findCorners', gray)
        cv2.waitKey(500)
        # 获取外参
        # retval, rvec, tvec = cv2.solvePnP(obj_point, exact_corners, self.mtx, self.dist)
        _, rvec, tvec, inliers = cv2.solvePnPRansac(obj_point, exact_corners, mtx, dist)
        return rvec, tvec

tgt2cam_t = np.load(PATH+'tgt2cam_t.npy')
grp2base_t = np.load(PATH+'grp2base_t.npy')
# cam2grp_t = np.load('cam2grp_t.npy')

tgt2cam_r = np.load(PATH+'tgt2cam_R.npy')
grp2base_r = np.load(PATH+'grp2base_R.npy')
# grp2base_r_quat = np.load('grp2base_R_Quat.npy')
# cam2grp_r = np.load('cam2grp_R.npy')


grp2base_t = grp2base_t.reshape(-1,3,1)
tgt2cam_t = tgt2cam_t.reshape(-1,3,1)
print('tgt2cam_t',tgt2cam_t.shape, tgt2cam_t)
print('grp2base_t',grp2base_t.shape, grp2base_t)

print('tgt2cam_r',tgt2cam_r.shape, tgt2cam_r)
print('grp2base_r',grp2base_r.shape, grp2base_r)


R_cam2grp, t_cam2grp = cv2.calibrateHandEye(grp2base_r, grp2base_t, 
                                            tgt2cam_r, tgt2cam_t)
print('cam2grp_t',t_cam2grp)

print(target2camera(PATH+'/images/left_0.png'))




                            
