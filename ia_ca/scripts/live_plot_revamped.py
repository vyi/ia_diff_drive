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
from matplotlib.transforms import Affine2D

A = []
B = []

MARGIN = 8

class interactivePlot():
    def  __init__(self):
        self.fig, (self.ax, self.ax2) = plt.subplots(1,2, figsize=(15,8))
        plt.subplots_adjust(left=0.05, bottom=0.10, top=0.90, right=0.98)
        #self.ax.axis('equal')
        self.A = []
        self.B = []
        self.ptick = 0
        self.is_initialized = False

    def plot_sec(self, a,b,h,k,r_inner,r_outer,clr="c", alpha=0.5):
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
        
        fov= Polygon(xy, closed=True, color=clr, alpha=alpha)
        return fov

    # def callb1(self, data):
    #     correctData = False
    #     self.ptick += 1

    #     try:
    #         exec('temp='+data.data)
    #         correctData=True
    #     except:
    #         correctData=False

    #     ## The plotting is pretty lame. We do not want to do it too fast
    #     if correctData and self.ptick:

    #         self.ptick  = 0
    #         # Plot 3 patches
    #         # Bot Circle
    #         # Bot sector
    #         # Obstruected Patch
    #         self.ax.clear()
    #         self.ax2.clear()
    #         #sects =  [ temp[0], temp[1] ]
    #         a = self.plot_sec(*temp[1], clr='y')
    #         b = self.plot_sec(*temp[1], clr='y')
    #         self.ax.add_artist(a)
    #         self.ax2.add_artist(b)

    #         a = self.plot_sec(*temp[2], clr='y')
    #         b = self.plot_sec(*temp[2], clr='y')
    #         self.ax.add_artist(a)
    #         self.ax2.add_artist(b)

    #         for t in temp[3:7]:
    #             a = self.plot_sec(*t, clr='r')
    #             b = self.plot_sec(*t, clr='r')
    #             self.ax.add_artist(a)
    #             self.ax2.add_artist(b)
            
    #         for t in temp[7:]:
    #             a = self.plot_sec(*t, clr='r')
    #             self.ax.add_artist(a)
            

    #         ## Set axis properties
    #         h= temp[1][2]
    #         k= temp[1][3]
    #         self.ax.arrow( h, k, 0.35*cos(temp[0][0]), 0.35*sin(temp[0][0]), head_width=0.04, head_length=0.2, fc='k', ec='r', alpha=0.3)
    #         self.ax.arrow( h, k, 0.35*cos(temp[0][0]+temp[0][2]), 0.35*sin(temp[0][0]+temp[0][2]), head_width=0.04, head_length=0.2, fc='k', ec='k', alpha=0.3)

    #         self.ax2.arrow( h, k, 0.35*cos(temp[0][0]), 0.35*sin(temp[0][0]), head_width=0.04, head_length=0.2, fc='k', ec='r', alpha=0.3)
    #         self.ax2.arrow( h, k, 0.35*cos(temp[0][0]+temp[0][2]), 0.35*sin(temp[0][0]+temp[0][2]), head_width=0.04, head_length=0.2, fc='k', ec='k', alpha=0.3)

    #         self.ax.set_xlim([h-MARGIN,h+MARGIN])
    #         self.ax.set_ylim([k-MARGIN,k+MARGIN])

    #         self.ax2.set_xlim([h-1,h+1])
    #         self.ax2.set_ylim([k-1,k+1])
    #         #self.ax.axis('equal')
    #         #self.ax2.axis('equal')
    #         self.fig.canvas.draw()
    def callb1(self, data):
        correctData = False
        self.ptick += 1

        try:
            exec('temp='+data.data)
            correctData=True
        except:
            correctData=False

        ## Plot everything only once
        if correctData:
            if (not self.is_initialized):
                ## plot the sectors and keep a reference

                self.is_initialized = True # Next time only update will occur. No record keeping
                Wm, Vm, Ts = 0.3, 0.15, 2

                fwd_s = temp[1]
                fwd_s[2] = 0
                fwd_s[3] = 0
                fwd_s[0] = -Wm*Ts
                fwd_s[1] = Wm*Ts


                self.bot_fwd_sector = self.plot_sec(*fwd_s, clr='y')
                self.bot_fwd_sector_alias = self.plot_sec(*fwd_s, clr='y')
                
                ## 
                print("Creating reference")
                self.transData1 = self.ax.transData #self.bot_fwd_sector.get_transform()
                self.transData2 = self.ax2.transData #self.bot_fwd_sector_alias.get_transform()

                self.ax.add_artist(self.bot_fwd_sector)
                self.ax2.add_artist(self.bot_fwd_sector_alias)

                h = temp[1][2]
                k = temp[1][3]
                theta = temp[0][0]

                t1 = Affine2D().rotate(theta).translate(h,k) + self.transData1
                self.bot_fwd_sector.set_transform(t1)

                t2 = Affine2D().rotate(theta).translate(h,k) + self.transData1
                self.bot_fwd_sector.set_transform(t2)
                


                ## Create a patch factory for occluded sectors
                self.patch_factory = []
                for i in range(10):
                    p = self.plot_sec(*temp[1], clr='r', alpha=0)
                    self.ax2.add_artist(p)
                    self.patch_factory.append(p)

                self.walls = [] #record keeping

                for t in temp[-6:]:
                    this_wall = self.plot_sec(*t, clr='r')
                    this_wall_alias = self.plot_sec(*t, clr='r')
                    self.ax.add_artist(this_wall)
                    #self.ax2.add_artist(this_wall_alias)
                    self.walls.append(this_wall)


            else:
                #bot_fwd_sector = self.plot_sec(*temp[1], clr='y')
                #bot_fwd_sector_alias = self.plot_sec(*temp[1], clr='y')

                h = temp[1][2]
                k = temp[1][3]
                theta = temp[0][0]

                t1 = Affine2D().rotate(theta).translate(h,k) + self.transData1
                t2 = Affine2D().rotate(theta).translate(h,k) + self.transData2

                self.bot_fwd_sector.set_transform(t1)
                self.bot_fwd_sector_alias.set_transform(t2)

                ## clear patches before using
                for p in self.patch_factory:
                    p.set_alpha(0)

                ## Use as many patches from the patch_factory to represent occluded sectors
                for i in range(3,3+len(temp[3:-4])):
                    if i >9:
                        ## ignore
                        continue
                    #pass
                    #alias_patch = self.plot_sec(*temp[i], clr='r', alpha=0.5)
                    #self.patch_factory[i].update_from(alias_patch)
                    #self.patch_factory[i].set_transform(alias_patch.get_transform)

                self.ax.set_xlim([h-MARGIN,h+MARGIN])
                self.ax.set_ylim([k-MARGIN,k+MARGIN])

                self.ax2.set_xlim([h-1,h+1])
                self.ax2.set_ylim([k-1,k+1])
                #self.ax.axis('equal')
                #self.ax2.axis('equal')
                self.fig.canvas.draw()
                
                ### what information is to be seen here
                ## print(datetime.datetime.now())
    def getXbot(self, xb):
        '''
            Reshuffling the indices of xb to be compatile for plotting
        '''
        indices = [2,3,4,5,0,1]
        temp = [0 for i in xb]

        jdx = 0
        for idx in indices:
            temp[jdx] = xb[idx]
            jdx = jdx + 1

        return temp
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
