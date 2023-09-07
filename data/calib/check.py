import numpy as np
import cv2
from transforms3d.quaternions import quat2mat

PATH='./data/calib/'
def distance(c1, c2):
    c = np.sum((c1-c2)**2)
    return np.sqrt(c)

def quaternion_to_rotation_matrix(q):  # w, x, y ,z
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

tgt2cam_t = np.load(PATH+'tgt2cam_t.npy')
grp2base_t = np.load(PATH+'grp2base_t.npy')
# cam2grp_t = np.load('cam2grp_t.npy')

tgt2cam_r = np.load(PATH+'tgt2cam_R.npy')
grp2base_r = np.load(PATH+'grp2base_R.npy')
# grp2base_r_quat = np.load('grp2base_R_Quat.npy')
# cam2grp_r = np.load('cam2grp_R.npy')


grp2base_t = grp2base_t.reshape(-1,3,1)
tgt2cam_t = tgt2cam_t.reshape(-1,3,1)
print('tgt2cam_t',tgt2cam_t.shape)
print('grp2base_t',grp2base_t.shape)

print('tgt2cam_r',tgt2cam_r.shape)
print('grp2base_r',grp2base_r.shape)
# print(cv2.Rodrigues(np.array([1.7870476 , 2.29510956 ,0.17030112]))[0])
# print(grp2base_t[-1])

R_cam2grp, t_cam2grp = cv2.calibrateHandEye(grp2base_r, grp2base_t, 
                                            tgt2cam_r, tgt2cam_t)
print('cam2grp_t',t_cam2grp)





                            
