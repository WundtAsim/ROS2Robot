
import datetime
from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import cv2 as cv2
import numpy as np
from .submodules.Camera import Camera 
from .submodules.Ui_MainWindow import Ui_MainWindow
from .submodules.Setting_cam import Ui_cam_setting
from .submodules.Setting_robot import Ui_robot_setting


class MyThread(QtCore.QThread):
    #设置线程变量
    trigger = QtCore.pyqtSignal(str)
 
    def __init__(self, parent=None):
        super(MyThread, self).__init__(parent)
 
    def run_(self, message):
        '''
        向信号trigger发送消息
        '''
        self.trigger.emit(message)
 
 
class Main(QtWidgets.QMainWindow,Ui_MainWindow):
    def __init__(self, parent=None):
        super(Main, self).__init__(parent)
        self.camera = Camera()
        # define timer to control video show freq
        self.timer_camera = QtCore.QTimer() 
        # if calib or no
        self.calib = False

        self.setupUi(self)
        # open camera callback
        self.check_cam.stateChanged.connect(self.button_open_clicked)
        # timer callback
        self.timer_camera.timeout.connect(self.show_camera)
        # calib callback
        self.check_calib.stateChanged.connect(self.button_calib_clicked)  
        # open camera button callback
        self.btn_save.clicked.connect(self.button_save_clicked)  
        # calc button callback
        self.btn_calc.clicked.connect(self.button_calc_clicked)

        # action: setting_cam callback
        self.actionCamera_setting.triggered.connect(self.setting_cam)
        # action: setting robot callback
        self.actionrobot_setting.triggered.connect(self.setting_robot)
        # close camera button callback
        # 若该按键被点击，则调用close()，注意这个close是父类QtWidgets.QWidget自带的，会关闭程序
        self.actionquit.triggered.connect(self.close)  

        #自定义线程类
        self.threads = MyThread(self) 
        #当信号接收到消息时，更新数据#当信号接收到消息时，更新数据
        self.threads.trigger.connect(self.update_text)  
        self.thread_no = 0 #序号
    
    # open camera button callback
    def button_open_clicked(self):
        if self.timer_camera.isActive() == False:  # 若定时器未启动
            succ = self.camera.open()  
            if succ == False:  # flag表示open()成不成功
                msg = QtWidgets.QMessageBox.warning(self, 'warning', "请检查相机于电脑是否连接正确", buttons=QtWidgets.QMessageBox.Ok)
            else:
                self.timer_camera.start(30)  # 定时器开始计时30ms，结果是每过30ms从摄像头中取一帧显示
                print("camera opened.")
                message = datetime.datetime.now().strftime("%H:%M:%S")+\
                        "\tCamera opened."
                self.threads.run_(message)  # start the thread
        else:
            self.timer_camera.stop()  # 关闭定时器
            self.camera.close()  # 释放视频流
            self.label_cam.clear()  # 清空视频显示区域
            print("camera closed.")
            message = datetime.datetime.now().strftime("%H:%M:%S")+\
                "\tCamera closed."
            self.threads.run_(message)  # start the thread
        
    # show camera callback
    def show_camera(self):
        self.camera.read()  # 从视频流中读取
        if self.calib:
            self.camera.tgt2cam()
            show = self.camera.image_corners \
                if self.camera.image_corners is not None else self.camera.image
        else:
            show = self.camera.image
                
        showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0],
                                 QtGui.QImage.Format_RGB888)  # 把读取到的视频数据变成QImage形式
        self.label_cam.setPixmap(QtGui.QPixmap.fromImage(showImage))  # 往显示视频的Label里 显示QImage
    
    # calib button callback
    def button_calib_clicked(self):
        if not self.timer_camera.isActive():
            msg = QtWidgets.QMessageBox.warning(self, 'warning', 
                            "\tplease open camera first!", buttons=QtWidgets.QMessageBox.Ok)
        else:
            self.calib = not self.calib
            if self.calib:
                message = datetime.datetime.now().strftime("%H:%M:%S")+\
                        "\tStart calibrating..."
                self.threads.run_(message)  # start the thread
            else:
                message = datetime.datetime.now().strftime("%H:%M:%S")+\
                        "\tStop calibrating."
                self.threads.run_(message)  # start the thread
                
    
    # save button callback
    def button_save_clicked(self):
        if not self.calib:
            msg = QtWidgets.QMessageBox.warning(self, 'warning', 
                            "\tplease start calibrating first!", buttons=QtWidgets.QMessageBox.Ok)
        else:
            self.camera.save_()
            message = datetime.datetime.now().strftime("%H:%M:%S")+\
                            "\tsaved:{0} pictures".format(self.camera.count)
            self.threads.run_(message)  # start the thread

    # question box
    def button_calc_clicked(self):
        if self.camera.count<3:
            msg = QtWidgets.QMessageBox.warning(self, 'warning', 
                    "Must have 3+ pictures! Quiting...", buttons=QtWidgets.QMessageBox.Ok)
            return
        reply = QtWidgets.QMessageBox.question(self, 'Warning', 'Save result? ', 
                                               QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.No)
        if reply == QtWidgets.QMessageBox.Yes:
            self.camera.calc_cam2grp()
            message = datetime.datetime.now().strftime("%H:%M:%S")+\
                "\tcalibration finished.\nt_cam2grp:\n{}".format(self.camera.t_cam2grp)
            self.threads.run_(message)  # start the thread
        else:
            pass
 
    def update_text(self, message):
        '''
        添加信息到日志栏中(即控件QTextBrowser中)
        '''
        self.textBrowser.append(message)

    def setting_cam(self):
        # setting camera widget callback
        self.form2 = QtWidgets.QWidget()
        self.ui2 = Ui_cam_setting()
        self.ui2.setupUi(self.form2)
        self.form2.show()

    def setting_robot(self):
        # setting robot widget callback
        self.form3 = QtWidgets.QWidget()
        self.ui3 = Ui_cam_setting()
        self.ui3.setupUi(self.form3)
        self.form3.show()



def main():
    # fixed line: application available
    app = QtWidgets.QApplication(sys.argv) 
    # 实例化Ui_MainWindow 
    myWindow = Main()  
    # 调用的show()以显示。同样show()是源于父类QtWidgets.QWidget的
    myWindow.show()  
    # 不加这句，程序界面会一闪而过
    sys.exit(app.exec_())  

if __name__ == '__main__':
    main()
