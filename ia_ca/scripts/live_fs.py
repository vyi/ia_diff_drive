#!/usr/bin/env python

from __future__ import division
import sys
import rospy
import numpy as np
import math
from math import sin, cos
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.cm as cmx
import matplotlib.colors as colors
from matplotlib.patches import Polygon

A = []
B = []

MARGIN = 2

class interactivePlot():
    def  __init__(self):
        self.fig, self.ax = plt.subplots()
        plt.subplots_adjust(left=0.15, bottom=0.04, top=0.95, right=0.85)
        #self.ax.axis('equal')
        self.A = []
        self.B = []
        self.ptick = 0

    def plot_sec(self, a,b,h,k,r_inner,r_outer,clr="c"):
        if (b<a):
            b = b+2*math.pi
        
        t = np.linspace(a,b)
        x1 = [(r_outer*math.cos(i))+h for i in t]
        y1 = [(r_outer*math.sin(i))+k for i in t]
        x2 = [(r_inner*math.cos(i))+h for i in t]
        y2 = [(r_inner*math.sin(i))+k for i in t]    
        x = x1 + x2[::-1]+ [x1[0]]
        y = y1 + y2[::-1] + [y1[0]]

        x = np.array(x).T
        y = np.array(y).T
        xy = [x, y]
        xy = np.array(xy).T
        
        fov= Polygon(xy, closed=True, color=clr, alpha=0.5)
        return fov

    def callb1(self, data):
        correctData = False
        self.ptick += 1

        try:
            exec('temp='+data.data)
            correctData=True
        except:
            correctData=False

        ## The plotting is pretty lame. We do not want to do it too fast
        if correctData and self.ptick>3:

            self.ptick  = 0
            # Plot 3 patches
            # Bot Circle
            # Bot sector
            # Obstruected Patch
            self.ax.clear()
            #sects =  [ temp[0], temp[1] ]
            a = self.plot_sec(*temp[1], clr='y')
            self.ax.add_artist(a)
            a = self.plot_sec(*temp[2], clr='y')
            self.ax.add_artist(a)
            
            for t in temp[3:]:
                a = self.plot_sec(*t, clr='r')
                self.ax.add_artist(a)


            

            ## Set axis properties
            h= temp[1][2]
            k= temp[1][3]
            self.ax.arrow( h, k, 0.35*cos(temp[0][0]), 0.35*sin(temp[0][0]), head_width=0.04, head_length=0.2, fc='k', ec='r', alpha=0.3)
            self.ax.arrow( h, k, 0.35*cos(temp[0][0]+temp[0][2]), 0.35*sin(temp[0][0]+temp[0][2]), head_width=0.04, head_length=0.2, fc='k', ec='k', alpha=0.3)
            self.ax.set_xlim([h-MARGIN,h+MARGIN])
            self.ax.set_ylim([k-MARGIN,k+MARGIN])
            #self.ax.axis('equal')
            self.fig.canvas.draw()

    def show(self):
        plt.show()



def process():
    

    ip = interactivePlot()
    rospy.init_node('live_fs', anonymous=True)
    rospy.Subscriber('/bot_X/free_sec', String, ip.callb1)
    
    ip.show()  

    # rate = rospy.Rate(2) # 10hz


    # while not rospy.is_shutdown():
    #     print "Going On"
    
        

if __name__ == '__main__':
    process()
    # try:
    #     process()
    # except:
    #     print("Unexpected error:", sys.exc_info()[0])
    # finally:
    #     ## close any files opened for writing
    #     print "Node closing"
