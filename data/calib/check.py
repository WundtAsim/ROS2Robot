import grp
from hmac import new
import numpy as np
import cv2

def distance(c1, c2):
    c = np.sum((c1-c2)**2)
    return np.sqrt(c)

tgt2cam_t = np.load('tgt2cam_t.npy')
grp2base_t = np.load('grp2base_t.npy')
# cam2grp_t = np.load('cam2grp_t.npy')

tgt2cam_r = np.load('tgt2cam_R.npy')
grp2base_r = np.load('grp2base_R.npy')
grp2base_r_quat = np.load('grp2base_R_Quat.npy')
# cam2grp_r = np.load('cam2grp_R.npy')


grp2base_t = grp2base_t.reshape(-1,3,1)
tgt2cam_t = tgt2cam_t.reshape(-1,3,1)
print('tgt2cam_t',tgt2cam_t.shape)
print('grp2base_t',grp2base_t.shape)

print('tgt2cam_r',tgt2cam_r.shape)
print('grp2base_r',grp2base_r.shape)

R_cam2grp, t_cam2grp = cv2.calibrateHandEye(grp2base_r, grp2base_t, 
                                            tgt2cam_r, tgt2cam_t)
print('cam2grp_t',t_cam2grp)

for i in range(25):
    a1 = (grp2base_r[i] * np.array([0,0,0]).reshape(3,-1))[0].reshape(3,-1) + grp2base_t[i]*100
    a2 = (grp2base_r[i] * np.array([100,0,0]).reshape(3,-1))[0].reshape(3,-1) + grp2base_t[i]*100
    print(distance(a1,a2))


                            
