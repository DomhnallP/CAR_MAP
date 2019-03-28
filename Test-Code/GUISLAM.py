
import sys

from PyQt5.QtWidgets import (QWidget, QToolTip, QLineEdit,
    QPushButton, QApplication, QGridLayout,QHBoxLayout, QVBoxLayout, QFileDialog, QLabel)
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from SlamPython import *

import os
import os.path
import orbslam2
import time
import cv2
import threading
import subprocess


class PlateMapper(QWidget):
    
    def __init__(self):
        super().__init__()        
        self.initUI()
        
        
    def initUI(self):
        
        QToolTip.setFont(QFont('SansSerif', 10))
        
        self.SeqLabel = QLineEdit("/usr/local/home/u180107/FYP/dataset/sequences/00")  
        self.SeqLabel.setFixedHeight(30)
        self.SeqLabel.setReadOnly(True)  
        self.SeqBtn = QPushButton('Select Sequence File', self)
        self.SeqBtn.clicked.connect(self.buttonClicked)

        self.VocabLabel = QLineEdit("/usr/local/home/u180107/FYP/Resources/ORBvoc.txt") 
        self.VocabLabel.setFixedHeight(30)
        self.VocabLabel.setReadOnly(True)    
        self.VocabBtn = QPushButton('Select Vocab File', self)
        self.VocabBtn.clicked.connect(self.buttonClicked)

        self.CalibLabel = QLineEdit("/usr/local/home/u180107/FYP/Resources/KITTI00-02.yaml")  
        self.CalibLabel.setFixedHeight(30) 
        self.CalibLabel.setReadOnly(True)
        self.CalibBtn = QPushButton('Select Calibration File', self)
        self.CalibBtn.clicked.connect(self.buttonClicked)

        self.RunBtn = QPushButton("Run Slam", self)
        self.RunBtn.clicked.connect(self.slamBTNEventHandler)
        
        Hbox = QGridLayout()
        Hbox.addWidget(self.SeqBtn, 0, 0)
        Hbox.addWidget(self.VocabBtn, 1, 0)
        Hbox.addWidget(self.CalibBtn, 2, 0)
        Hbox.addWidget(self.SeqLabel, 0, 1)
        Hbox.addWidget(self.VocabLabel, 1, 1)
        Hbox.addWidget(self.CalibLabel, 2, 1)
        Hbox.addWidget(self.RunBtn, 3, 0)
        Hbox.setAlignment(Qt.AlignTop)
        self.setLayout(Hbox)
        self.setGeometry(800, 400, 0, 0)
        self.resize(800,400)
        self.setWindowTitle('PlateMapper')    
        self.show()

    def slamBTNEventHandler(self):
        vocab = self.VocabLabel.text()
        calib = self.CalibLabel.text()
        sequence = self.SeqLabel.text()

        self.close()
        #this version works but requires paths to the python files so not great, also doesn't output to console properly
        #subprocess.call(["python3.5", "/usr/local/home/u180107/FYP/Python/SlamPython.py", vocab , calib , sequence], stdout=subprocess.PIP)
        
        #This version works but is super unadvisable (os.system basically opens a new shell and that opens up to lots of security problems)
        os.system("rosrun ORB_SLAM2 Mono " + vocab + " " + calib)
        
        # Multithreading attempt at fixing below issue, still doesn't work  
        # print("Starting Daemon")
        # self.close()
        # t = threading.Thread(target=runSlam(vocab, calib, sequence), name="Running Slam")
        # t.daemon = True
        # t.start()
        
        #this method crashes
        #runSlam(vocab, calib, sequence)


    def buttonClicked(self):
  
        label = self.sender()
        self.getfile(label.text())


    def getfile(self, label):
        if(label=="Select Sequence File"):
            path = self.openDirectoryNameDialog()
            self.SeqLabel.setText(path)
        if(label=="Select Vocab File"):
            path = self.openFileNameDialog()
            self.VocabLabel.setText(path)
        if(label=="Select Calibration File"):
            path = self.openFileNameDialog()
            self.CalibLabel.setText(path)

    def openFileNameDialog(self):  
        fileDialog = QFileDialog()
        fileName, _ = fileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","All Files (*);;")
        if fileName:
            return(fileName)

    def openDirectoryNameDialog(self):  
        fileDialog = QFileDialog()
        directoryName = fileDialog.getExistingDirectory(self, "Select Output Folder")
        if directoryName:
            return(directoryName)

    
if __name__ == '__main__':
    
    app = QApplication(sys.argv)
    ex = PlateMapper()
    #slamtest = ORBSLAM("/usr/local/home/u180107/FYP/Resources/ORBvoc.txt", "/usr/local/home/u180107/FYP/Resources/KITTI00-02.yaml", "/usr/local/home/u180107/FYP/dataset/sequences/00")
    sys.exit(app.exec_())
