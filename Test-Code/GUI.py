import sys
from PyQt5.QtWidgets import (QWidget, QToolTip, QLineEdit,
    QPushButton, QApplication, QGridLayout,QHBoxLayout, QVBoxLayout, QFileDialog, QLabel)
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import os.path
import orbslam2
import time
import cv2


class PlateMapper(QWidget):
    
    def __init__(self):
        super().__init__()        
        self.initUI()
        
        
    def initUI(self):
        
        QToolTip.setFont(QFont('SansSerif', 10))
        
        self.SeqLabel = QLineEdit()  
        self.SeqLabel.setFixedHeight(30)
        self.SeqLabel.setReadOnly(True)  
        self.SeqBtn = QPushButton('Select Sequence File', self)
        self.SeqBtn.clicked.connect(self.buttonClicked)
        self.RunBtn = QPushButton("Run Slam", self)

        self.VocabLabel = QLineEdit() 
        self.VocabLabel.setFixedHeight(30)
        self.VocabLabel.setReadOnly(True)    
        self.VocabBtn = QPushButton('Select Vocab File', self)
        self.VocabBtn.clicked.connect(self.buttonClicked)

        self.CalibLabel = QLineEdit()  
        self.CalibLabel.setFixedHeight(30) 
        self.CalibLabel.setReadOnly(True)
        self.CalibBtn = QPushButton('Select Calibration File', self)
        self.CalibBtn.clicked.connect(self.buttonClicked)
        
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
        self.setGeometry(1000, 800, 0, 0)
        self.resize(1000,800)
        self.setWindowTitle('PlateMapper')    
        self.show()

    def buttonClicked(self):
  
        label = self.sender()
        self.getfile(label.text())


    def getfile(self, label):
        path = self.openFileNameDialog()
        if(label=="Select Sequence File"):
            self.SeqLabel.setText(path)
        if(label=="Select Vocab File"):
            self.VocabLabel.setText(path)
        if(label=="Select Calibration File"):
            self.CalibLabel.setText(path)

    def openFileNameDialog(self):    
        fileName, _ = QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","All Files (*);;")
        if fileName:
            return(fileName)

    
if __name__ == '__main__':
    
    app = QApplication(sys.argv)
    ex = PlateMapper()
    sys.exit(app.exec_())
