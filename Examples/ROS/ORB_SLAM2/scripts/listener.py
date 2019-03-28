#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from multiprocessing import Process
from pyquaternion import Quaternion
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import sys
import time

pose = Pose(position = (0,0,0), orientation = (0,0,0,0))

print(pose)
class Map(object):
    def __init__(self):
        self.path = dict()
        self.app = QtGui.QApplication(sys.argv)
        self.w = gl.GLViewWidget()
        self.w.opts['distance'] = 40
        self.w.setWindowTitle('CARMAP')
        self.w.setGeometry(0, 110, 800, 800)
        self.w.setBackgroundColor(8,14,20, 1)
        self.w.show()

        # create the background grids

        # gx.rotate(90, 0, 1, 0)
        # gx.translate(-10, 0, 10)
        # self.w.addItem(gx)
        # gy = gl.GLGridItem()
        # gy.rotate(90, 1, 0, 0)
        # gy.translate(0, -10, 10)
        # self.w.addItem(gy)
        gz = gl.GLGridItem()
        gz.setSize(x=100, y=100)
        gz.translate(0, 0, 0)
        self.w.addItem(gz)

        graph_data = open('traj.txt', 'r').read()
        lines = graph_data.split('\n')
        self.xs = []
        self.ys = []
        self.zs = []
        for line in lines:
            if(line):
                x,y,z = line.split(',')
                self.xs.append(round(float(x), 2))
                self.ys.append(round(float(y), 2))
                self.zs.append(round(float(z), 2))

        vertices = np.array([
            [
                x, y, z
            ] for x, y, z in zip(self.xs, self.ys, self.zs)],dtype=np.float32)

        self.path = gl.GLLinePlotItem(pos=vertices, antialias=True, width = 2)
        self.w.addItem(self.path)

        point_data = open('keyframes.txt', 'r').read()
        points = graph_data.split('\n')
        self.xs = []
        self.ys = []
        self.zs = []
        count = 0
        for point in points:
            if(point):
                x,y,z = point.split(',')
                self.xs.append(round(float(x), 2))
                self.ys.append(round(float(y), 2))
                self.zs.append(round(float(z), 2))
            count+=1

        pointsArray = np.array([
            [
                x, y, z
            ] for x, y, z in zip(self.xs, self.ys, self.zs)],dtype=np.float32)

        object_data = open('keyframes.txt', 'r').read()
        objects = object_data.split('\n')
        self.xs = []
        self.ys = []
        self.zs = []
        count = 0
        for obj in objects:
            if(obj):
                x,y,z = obj.split(',')
                self.xs.append(round(float(x), 2))
                self.ys.append(round(float(y), 2))
                self.zs.append(round(float(z), 2))
            count+=1

        objectsArray = np.array([
            [
                x, y, z
            ] for x, y, z in zip(self.xs, self.ys, self.zs)],dtype=np.float32)

        self.path = gl.GLLinePlotItem(pos=vertices, antialias=True, width = 2)
        self.w.addItem(self.path)

        self.pointsOfInterest = gl.GLScatterPlotItem(pos=pointsArray, color=(255,0,0,1), size=2)
        self.w.addItem(self.pointsOfInterest)

        self.objectsArray = gl.GLScatterPlotItem(pos=objectsArray, color=(0,255,0,1), size=10)
        self.w.addItem(self.objectsArray)

    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()

    def set_plotdata(self, points, width):
        self.path.setData(pos=points, width=width, antialias=True)

    def set_plotpoints(self, points, size):
        self.pointsOfInterest.setData(pos=points, color=(255,0,0,1), size=size)

    def set_plotobjects(self, points, size):
        self.objectsArray.setData(pos=points, color=(0,255,0,1), size=size)

    def update(self):
        graph_data = open('traj.txt', 'r').read()
        lines = graph_data.split('\n')
        self.xs = []
        self.ys = []
        self.zs = []
        for line in lines:
            if(line):
                try:
                    x,y,z = line.split(',')
                    self.xs.append(round(float(x), 2))
                    self.ys.append(round(float(y), 2))
                    self.zs.append(round(float(z), 2))
                except:
                    print(line + " is not a valid coordinate format")

        vertices = np.array([
            [
                x, y, z
            ] for x, y, z in zip(self.xs, self.ys, self.zs)],dtype=np.float32)

        point_data = open('keyframes.txt', 'r').read()
        points = point_data.split('\n')
        self.xs = [0]
        self.ys = [0]
        self.zs = [0]
        count = 1
        for point in points:
            if(point and count%5==0):
                x,y,z = point.split(',')
                self.xs.append(round(float(x), 2))
                self.ys.append(round(float(y), 2))
                self.zs.append(round(float(z), 2))
                count=1
            else:
                count += 1

        pointsArray = np.array([
            [
                x, y, z
            ] for x, y, z in zip(self.xs, self.ys, self.zs)],dtype=np.float32)

        objects_data = open('objects.txt', 'r').read()
        objects = objects_data.split('\n')
        self.xs = [0]
        self.ys = [0]
        self.zs = [0]
        count = 1
        for obj in objects:
            if(obj and count%1==0):
                x,y,z = obj.split(',')
                self.xs.append(round(float(x), 2))
                self.ys.append(round(float(y), 2))
                self.zs.append(round(float(z), 2))
                count=1
            else:
                count += 1

        objectsArray = np.array([
            [
                x, y, z
            ] for x, y, z in zip(self.xs, self.ys, self.zs)],dtype=np.float32)

        self.set_plotdata(points=vertices, width=2)
        self.set_plotpoints(points=pointsArray,size=2)
        self.set_plotobjects(points=objectsArray,size=10)
        

    def animation(self):
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(20)
        self.start()


def callback(data):
    x = data.poses[0].position.x
    y = data.poses[0].position.z
    z = data.poses[0].position.y
    q1 = data.poses[0].orientation.x
    q2 = data.poses[0].orientation.y
    q3 = data.poses[0].orientation.z
    q4 = data.poses[0].orientation.w

    global pose 
    pose = Pose(position = (x, y, z), orientation = (q1, q2, q3, q4))
    with open('traj.txt','a+') as f:
        f.write('\n ' + str(x) + "," + str(y) + "," + str(z))
            
    count = 1;
    for point in data.poses:
        if count%10==0:
            with open('keyframes.txt','a+') as f:
                f.write('\n ' + str(point.position.x) + "," + str(point.position.z) + "," + str(point.position.y))
            count=1
        else:
            count+=1

def updateMap(data):
    with open('traj.txt','w+') as f:
            f.write("0,0,0")
    for pose in data.poses:
        x = pose.position.x
        y = pose.position.z
        z = pose.position.y
        # print(str(x) + " " + str(y))
        with open('traj.txt','a+') as f:
            f.write('\n ' + str(x) + "," + str(y) + "," + str(z))

def updateObjectPose(data):

    myquaternion = Quaternion(pose.orientation[3]*-1, pose.orientation[0], pose.orientation[2], pose.orientation[1])
    rot_matrix = myquaternion.rotation_matrix
    vec1 = myquaternion.rotate(np.array([data.x/100, data.z/250, data.y/250]))
    vec2 =  np.array([pose.position[0], pose.position[1], pose.position[2]])
    finalPos = np.add(vec1, vec2)

    x = finalPos[0]
    y = finalPos[1]
    z = finalPos[2]

    with open('objects.txt','a+') as f:
        f.write('\n ' + str(x) + "," + str(y) + "," + str(z))


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('pts_and_pose', PoseArray, callback)
    rospy.Subscriber('all_kf_and_pts', PoseArray, updateMap)
    rospy.Subscriber('object_pose', Point, updateObjectPose)
    rospy.spin()

def mapWindow():
    map = Map()
    map.animation()

if __name__ == '__main__':
    print(sys.version)
    with open('traj.txt','w') as f:
        f.write(str("0,0,0"))
    with open('keyframes.txt','w') as f:
        f.write(str("0,0,0"))
    p1 = Process(target = mapWindow)
    p1.start()
    p2 = Process(target = listener)
    p2.start()
    
