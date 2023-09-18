# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Setting_cam.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_cam_setting(object):
    def setupUi(self, cam_setting):
        cam_setting.setObjectName("cam_setting")
        cam_setting.resize(859, 426)
        self.formLayoutWidget = QtWidgets.QWidget(cam_setting)
        self.formLayoutWidget.setGeometry(QtCore.QRect(90, 20, 311, 291))
        self.formLayoutWidget.setObjectName("formLayoutWidget")
        self.formLayout = QtWidgets.QFormLayout(self.formLayoutWidget)
        self.formLayout.setFormAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.formLayout.setVerticalSpacing(12)
        self.formLayout.setObjectName("formLayout")
        self.brightnessLabel = QtWidgets.QLabel(self.formLayoutWidget)
        self.brightnessLabel.setObjectName("brightnessLabel")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.brightnessLabel)
        self.contrastLabel = QtWidgets.QLabel(self.formLayoutWidget)
        self.contrastLabel.setObjectName("contrastLabel")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.contrastLabel)
        self.hueLabel = QtWidgets.QLabel(self.formLayoutWidget)
        self.hueLabel.setObjectName("hueLabel")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.hueLabel)
        self.saturationLabel = QtWidgets.QLabel(self.formLayoutWidget)
        self.saturationLabel.setObjectName("saturationLabel")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.saturationLabel)
        self.sharpnessLabel = QtWidgets.QLabel(self.formLayoutWidget)
        self.sharpnessLabel.setObjectName("sharpnessLabel")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.LabelRole, self.sharpnessLabel)
        self.gammaLabel = QtWidgets.QLabel(self.formLayoutWidget)
        self.gammaLabel.setObjectName("gammaLabel")
        self.formLayout.setWidget(5, QtWidgets.QFormLayout.LabelRole, self.gammaLabel)
        self.whiteBalanceLabel = QtWidgets.QLabel(self.formLayoutWidget)
        self.whiteBalanceLabel.setObjectName("whiteBalanceLabel")
        self.formLayout.setWidget(6, QtWidgets.QFormLayout.LabelRole, self.whiteBalanceLabel)
        self.gainLabel = QtWidgets.QLabel(self.formLayoutWidget)
        self.gainLabel.setObjectName("gainLabel")
        self.formLayout.setWidget(7, QtWidgets.QFormLayout.LabelRole, self.gainLabel)
        self.exporesureLabel = QtWidgets.QLabel(self.formLayoutWidget)
        self.exporesureLabel.setObjectName("exporesureLabel")
        self.formLayout.setWidget(8, QtWidgets.QFormLayout.LabelRole, self.exporesureLabel)
        self.horizontalSlider = QtWidgets.QSlider(self.formLayoutWidget)
        self.horizontalSlider.setMaximum(10)
        self.horizontalSlider.setProperty("value", 4)
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider)
        self.horizontalSlider_2 = QtWidgets.QSlider(self.formLayoutWidget)
        self.horizontalSlider_2.setMaximum(10)
        self.horizontalSlider_2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_2.setObjectName("horizontalSlider_2")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_2)
        self.horizontalSlider_3 = QtWidgets.QSlider(self.formLayoutWidget)
        self.horizontalSlider_3.setMaximum(10)
        self.horizontalSlider_3.setProperty("value", 4)
        self.horizontalSlider_3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_3.setObjectName("horizontalSlider_3")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_3)
        self.horizontalSlider_4 = QtWidgets.QSlider(self.formLayoutWidget)
        self.horizontalSlider_4.setMaximum(10)
        self.horizontalSlider_4.setProperty("value", 4)
        self.horizontalSlider_4.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_4.setObjectName("horizontalSlider_4")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_4)
        self.horizontalSlider_5 = QtWidgets.QSlider(self.formLayoutWidget)
        self.horizontalSlider_5.setMaximum(10)
        self.horizontalSlider_5.setProperty("value", 4)
        self.horizontalSlider_5.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_5.setTickInterval(4)
        self.horizontalSlider_5.setObjectName("horizontalSlider_5")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_5)
        self.horizontalSlider_6 = QtWidgets.QSlider(self.formLayoutWidget)
        self.horizontalSlider_6.setMaximum(10)
        self.horizontalSlider_6.setProperty("value", 8)
        self.horizontalSlider_6.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_6.setObjectName("horizontalSlider_6")
        self.formLayout.setWidget(5, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_6)
        self.horizontalSlider_7 = QtWidgets.QSlider(self.formLayoutWidget)
        self.horizontalSlider_7.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_7.setObjectName("horizontalSlider_7")
        self.formLayout.setWidget(6, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_7)
        self.horizontalSlider_8 = QtWidgets.QSlider(self.formLayoutWidget)
        self.horizontalSlider_8.setProperty("value", 1)
        self.horizontalSlider_8.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_8.setObjectName("horizontalSlider_8")
        self.formLayout.setWidget(7, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_8)
        self.horizontalSlider_9 = QtWidgets.QSlider(self.formLayoutWidget)
        self.horizontalSlider_9.setProperty("value", 53)
        self.horizontalSlider_9.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_9.setObjectName("horizontalSlider_9")
        self.formLayout.setWidget(8, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_9)
        self.pushButton = QtWidgets.QPushButton(cam_setting)
        self.pushButton.setGeometry(QtCore.QRect(200, 330, 89, 25))
        self.pushButton.setObjectName("pushButton")
        self.widget = QtWidgets.QWidget(cam_setting)
        self.widget.setGeometry(QtCore.QRect(470, 20, 345, 392))
        self.widget.setObjectName("widget")
        self.formLayout_2 = QtWidgets.QFormLayout(self.widget)
        self.formLayout_2.setContentsMargins(0, 0, 0, 0)
        self.formLayout_2.setObjectName("formLayout_2")
        self.label = QtWidgets.QLabel(self.widget)
        self.label.setObjectName("label")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label)
        self.textBrowser = QtWidgets.QTextBrowser(self.widget)
        self.textBrowser.setObjectName("textBrowser")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.textBrowser)
        self.label_2 = QtWidgets.QLabel(self.widget)
        self.label_2.setObjectName("label_2")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_2)
        self.textBrowser_2 = QtWidgets.QTextBrowser(self.widget)
        self.textBrowser_2.setObjectName("textBrowser_2")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.textBrowser_2)

        self.retranslateUi(cam_setting)
        QtCore.QMetaObject.connectSlotsByName(cam_setting)

    def retranslateUi(self, cam_setting):
        _translate = QtCore.QCoreApplication.translate
        cam_setting.setWindowTitle(_translate("cam_setting", "Camera Settings"))
        self.brightnessLabel.setText(_translate("cam_setting", "Brightness"))
        self.contrastLabel.setText(_translate("cam_setting", "Contrast"))
        self.hueLabel.setText(_translate("cam_setting", "Hue"))
        self.saturationLabel.setText(_translate("cam_setting", "Saturation"))
        self.sharpnessLabel.setText(_translate("cam_setting", "Sharpness"))
        self.gammaLabel.setText(_translate("cam_setting", "Gamma"))
        self.whiteBalanceLabel.setText(_translate("cam_setting", "WhiteBalance"))
        self.gainLabel.setText(_translate("cam_setting", "Gain"))
        self.exporesureLabel.setText(_translate("cam_setting", "Exporesure"))
        self.pushButton.setText(_translate("cam_setting", "Save Values"))
        self.label.setText(_translate("cam_setting", "intrinsics:"))
        self.textBrowser.setHtml(_translate("cam_setting", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; line-height:19px; background-color:#ffffff;\"><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">538.57</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">,     </span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">0</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">.,     </span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">635.63</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">0</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">.,     </span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">538.5</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">,     </span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">350.7075</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">0</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">.,     </span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">0</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">.,     </span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">1</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">.</span></p></body></html>"))
        self.label_2.setText(_translate("cam_setting", "Distoration:"))
        self.textBrowser_2.setHtml(_translate("cam_setting", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; line-height:19px; background-color:#ffffff;\"><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000; background-color:#ffffff;\">(rational polynomial)</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; line-height:19px; background-color:#ffffff;\"><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">13.258600234985352</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">,</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">-</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">10.219599723815918</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">,</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">0.0003258869983255863</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">,</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">-</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">0.0002168240025639534</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">,</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">1.5960400104522705</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">,</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">13.655200004577637</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">,</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">-</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">10.19260025024414</span><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#000000;\">,</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Droid Sans Mono\',\'monospace\',\'monospace\'; font-size:14px; color:#098658;\">1.4755799770355225</span></p></body></html>"))