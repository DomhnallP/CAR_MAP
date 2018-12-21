#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from multiprocessing import Process
import sys


fig = plt.figure()
ax = fig.gca()
with open('traj.txt','w+') as f:
       f.write("0, 0")

def callback(data):
    x = data.poses[0].position.x
    y = data.poses[0].position.z
    with open('traj.txt','a+') as f:
        f.write('\n ' + str(x) + "," + str(y))
    # with open('traj.txt','w+') as f:
    #         f.write("0, 0")

    # for pose in data.poses:
    #     x = pose.position.x
    #     y = pose.position.z
    #     with open('traj.txt','a+') as f:
    #         f.write('\n ' + str(x) + "," + str(y))

def animate(i):
    graph_data = open('traj.txt', 'r').read()
    lines = graph_data.split('\n')
    xs = []
    ys = []
    for line in lines:
        x,y = line.split(',')

        xs.append(round(float(x), 3))
        ys.append(round(float(y), 3))
    ax.clear()
    ax.plot(xs,ys,'ro')

def runAnimate():
    ani = animation.FuncAnimation(fig, animate, interval=20)
    plt.show()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('pts_and_pose', PoseArray, callback)
    rospy.spin()

if __name__ == '__main__':
    print(sys.version)
    # listener()
    p1 = Process(target = listener)
    p1.start()
    p2 = Process(target = runAnimate)
    p2.start()

