from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import cv2 as cv2
import numpy as np
from zmq import NULL
from .submodules.Camera import Camera
 
class Ui_MainWindow(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)  # 父类的构造函数
 
        self.timer_camera = QtCore.QTimer()  # 定义定时器，用于控制显示视频的帧率
        self.camera = Camera()
        
        # if calib or no
        self.calib = False
 
        self.set_ui()  # 初始化程序界面
        self.slot_init()  # 初始化槽函数
 
    '''程序界面布局'''
 
    def set_ui(self):
        # layout
        self.__layout_main = QtWidgets.QHBoxLayout()  # 总布局
        self.__layout_fun_button = QtWidgets.QHBoxLayout()  # 按键布局
        self.__layout_data_show = QtWidgets.QVBoxLayout()  # 数据(视频)显示布局

        # button 
        self.button_open = QtWidgets.QPushButton('open camera')  # 建立用于打开摄像头的按键
        self.button_close = QtWidgets.QPushButton('close')  # 建立用于退出程序的按键
        # self.button_open.setMinimumHeight(50)  # 设置按键大小
        # self.button_close.setMinimumHeight(50)
        self.button_calib = QtWidgets.QPushButton('start calib')
        self.button_save = QtWidgets.QPushButton('save')
        
        
        '''vedio label'''
        self.label_show_camera = QtWidgets.QLabel()  # 定义显示视频的Label
        self.label_show_camera.setFixedSize(self.camera.img_width+2, self.camera.img_height+2)  # 给显示视频的Label设置大小为641x481
        
        '''把按键加入到按键布局中'''
        self.__layout_fun_button.addWidget(self.button_open)  # 把打开摄像头的按键放到按键布局中
        self.__layout_fun_button.addWidget(self.button_calib)  
        self.__layout_fun_button.addWidget(self.button_save)
        self.__layout_fun_button.addWidget(self.button_close)

        # add widget to show layout
        self.__layout_data_show.addWidget(self.label_show_camera)
        self.__layout_data_show.addLayout(self.__layout_fun_button)
        
        '''把控件加入到总布局中'''
        self.__layout_main.addLayout(self.__layout_data_show)  # 把用于显示视频的Label加入到总布局中
        
        '''总布局布置好后就可以把总布局作为参数传入下面函数'''
        self.setLayout(self.__layout_main)  # 到这步才会显示所有控件
 
    '''初始化所有槽函数'''
 
    def slot_init(self):
        # openv camera button callback
        self.button_open.clicked.connect(self.button_open_clicked)  
        # timer callback
        self.timer_camera.timeout.connect(self.show_camera)  
        # openv camera button callback
        self.button_calib.clicked.connect(self.button_calib_clicked)  
        # openv camera button callback
        self.button_save.clicked.connect(self.button_save_clicked)  
        # close camera button callback
        self.button_close.clicked.connect(self.close)  # 若该按键被点击，则调用close()，注意这个close是父类QtWidgets.QWidget自带的，会关闭程序
 
    
    # open camera button callback
    def button_open_clicked(self):
        if self.timer_camera.isActive() == False:  # 若定时器未启动
            succ = self.camera.open()  
            if succ == False:  # flag表示open()成不成功
                msg = QtWidgets.QMessageBox.warning(self, 'warning', "请检查相机于电脑是否连接正确", buttons=QtWidgets.QMessageBox.Ok)
            else:
                self.timer_camera.start(30)  # 定时器开始计时30ms，结果是每过30ms从摄像头中取一帧显示
                self.button_open.setText('close camera')
                print("camera opened.")
        else:
            self.timer_camera.stop()  # 关闭定时器
            self.camera.close()  # 释放视频流
            self.label_show_camera.clear()  # 清空视频显示区域
            self.button_open.setText('open camera')
            print("camera closed.")
    

    def show_camera(self):
        self.camera.read()  # 从视频流中读取
        if self.calib:
            self.camera.tgt2cam()
            show = self.camera.image_corners \
                if self.camera.image_corners is not NULL else self.camera.image
        else:
            show = self.camera.image
                
        showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0],
                                 QtGui.QImage.Format_RGB888)  # 把读取到的视频数据变成QImage形式
        self.label_show_camera.setPixmap(QtGui.QPixmap.fromImage(showImage))  # 往显示视频的Label里 显示QImage
    
    # calib button callback
    def button_calib_clicked(self):
        if not self.timer_camera.isActive():
            msg = QtWidgets.QMessageBox.warning(self, 'warning', "please open camera first!", buttons=QtWidgets.QMessageBox.Ok)
        else:
            self.calib = not self.calib
            if self.calib:
                self.button_calib.setText('stop calib')
            else:
                self.button_calib.setText('start calib')
    
    # save button callback
    def button_save_clicked(self):
        if self.timer_camera.isActive() == False:  # 若定时器未启动
            flag = self.cap.open(self.CAM_NUM)  # 参数是0，摄像头，参数是视频文件路径则打开视频
            if flag == False:  # flag表示open()成不成功
                msg = QtWidgets.QMessageBox.warning(self, 'warning', "请检查相机于电脑是否连接正确", buttons=QtWidgets.QMessageBox.Ok)
            else:
                self.timer_camera.start(30)  # 定时器开始计时30ms，结果是每过30ms从摄像头中取一帧显示
                self.button_open.setText('close camera')
        else:
            self.timer_camera.stop()  # 关闭定时器
            self.cap.release()  # 释放视频流
            self.label_show_camera.clear()  # 清空视频显示区域
            self.button_open.setText('open camera')
 

def main():
    app = QtWidgets.QApplication(sys.argv)  # 固定的，表示程序应用
    ui = Ui_MainWindow()  # 实例化Ui_MainWindow
    ui.show()  # 调用ui的show()以显示。同样show()是源于父类QtWidgets.QWidget的
    sys.exit(app.exec_())  # 不加这句，程序界面会一闪而过
