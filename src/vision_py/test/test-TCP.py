from turtle import pos
import  UR_TCP_RTDE as UR
import  time
import cv2
import numpy as np

TCP_socket = UR.connect('192.168.1.24',30003)
while True:

    # print('TCP',TCP_socket)
    data = TCP_socket.recv(1024)
    if len(data) == 1024:
        
        # print(data)
        position = UR.get_position(data)
        print(type(position), position.shape)
        rotation = position[3:]
        print(position[:3])
        print(rotation)

TCP = UR.disconnect(TCP_socket)
# print('TCP',TCP_socket)


