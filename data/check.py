import grp
from hmac import new
import numpy as np
import cv2

def distance(c1, c2):
    c = np.sum((c1-c2)**2)
    return np.sqrt(c)

tgt2cam_t = np.load('tgt2cam_t.npy')
grp2base_t = np.load('grp2base_t.npy')
cam2grp_t = np.load('cam2grp_t.npy')

tgt2cam_r = np.load('tgt2cam_R.npy')
grp2base_r = np.load('grp2base_R.npy')
cam2grp_r = np.load('cam2grp_R.npy')


grp2base_t = grp2base_t.reshape(-1,3,1)
tgt2cam_t = tgt2cam_t.reshape(-1,3,1)
print('tgt2cam_t',tgt2cam_t.shape)
print('grp2base_t',grp2base_t.shape)

print('tgt2cam_r',tgt2cam_r.shape)
print('grp2base_r',grp2base_r.shape)

R_cam2grp, t_cam2grp = cv2.calibrateHandEye(grp2base_r, grp2base_t, 
                                            tgt2cam_r, tgt2cam_t)
print('cam2grp_t',t_cam2grp)
cam2grp = np.vstack((np.hstack((R_cam2grp, t_cam2grp)), np.array([0,0,0,1])))

# test the rotation matrix
r = cv2.Rodrigues(tgt2cam_r[-1])[0]
cmw = np.vstack((np.hstack((r, tgt2cam_t[-1])), np.array([0,0,0,1])))
print('cmw:', cmw)
print('the (0.11, 0) in cam:', cmw@np.array([0.11, 0, 0, 1]))
print('distance:', distance(cmw@np.array([0.11, 0, 0, 1]), 
                            cmw@np.array([0, 0, 0, 1])))


grp2bas = np.vstack((np.hstack((grp2base_r[-1], grp2base_t[-1].reshape(-1,1))), np.array([0,0,0,1])))
print('grp2base:',grp2bas)
print('the (0.11, 0) in base:', grp2bas@np.array([0.11, 0, 0, 1]))
print('distance:', distance(grp2bas@np.array([0.11, 0, 0, 1]), 
                            grp2bas@np.array([0, 0, 0, 1])))

print("cam2base:",grp2bas@cam2grp@cmw@(np.array([0.11,0,0,1]).reshape(4,1)))
print('distance:', distance(grp2bas@cam2grp@cmw@np.array([0.11,0,0,1]).reshape(4,1), 
                            grp2bas@cam2grp@cmw@np.array([0,0,0,1]).reshape(4,1)))
                            
