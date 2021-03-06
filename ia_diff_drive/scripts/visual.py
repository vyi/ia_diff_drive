#!/usr/bin/env python

from __future__ import division
import sys
import rospy
import numpy as np
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.cm as cmx
import matplotlib.colors as colors
from matplotlib.patches import Polygon

LIM = 11.0

def plot_sec(start_angle,end_angle, c_x, c_y, r_inner, r_outer, clr, alp):
    a = start_angle
    b = end_angle
    h = c_x
    k = c_y
    
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
    
    fov= Polygon(xy, closed=True, color=clr, alpha=alp)
    return fov


def get_cmap(N):
    '''Returns a function that maps each index in 0, 1, ... N-1 to a distinct 
    RGB color.'''
    color_norm  = colors.Normalize(vmin=0, vmax=N-1)
    scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv') 
    def map_index_to_rgb_color(index):
        return scalar_map.to_rgba(index)
    return map_index_to_rgb_color

class PoseCollector(object):
    def __init__(self, name):
        self.name = name
        self.allPoses = {}
        self.pose = []
        
    def callback(self, data):
        #self.allPoses = data.data
        #rospy.loginfo(data)
        if data.data:
            exec('self.allPoses='+data.data)
        
        #rospy.loginfo(rospy.get_caller_id() + 'Data =  %s', data.data)
        #print "***********"
        #print type(self.allPoses)
        #print self.allPoses
        #print "***********"

    def getAllPose(self):
        return self.allPoses

class DynamicUpdate():
    def __init__(self, NumBots=10):
        self.xdata=[]
        self.ydata=[]
        self.cmap = get_cmap(NumBots)
        self.botList = {}
        self.pos = [[0,0,0] for i in range(NumBots)]
        #global points
        self.fig, self.ax = plt.subplots(1, 1)
        #self.ax.plot(points[:,0],points[:,1])
        self.lines, = self.ax.plot(self.xdata,self.ydata,color='g', linewidth=2.0)
        self.ax.set_autoscaley_on(True)
        
        
        
        
        #self.ax.set_xticks(np.arange(-7,7))
        #self.ax.set_yticks(np.arange(-7,7))

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_xlim([-LIM/2, LIM/2])
        self.ax.set_ylim([-LIM/2, LIM/2])
        self.ax.grid()

    def PlotData(self, bot_pose, bot_id):
        # self.xdata.append(x)
        # self.ydata.append(y)
        # self.lines.set_data(self.xdata,self.ydata)
        if not bot_id in self.botList:
            print("Adding bot :{} to observe list".format(bot_id))
            self.botList[bot_id] = len(self.botList)
            self.pos.append([bot_pose[0],bot_pose[1]])

        v_max = 0.15
        v_min = 0.00
        w_max = 0.3
        size_bot = 0.23
      
        ts = 3.0
        ## clear all old data and plot new window
        j = self.botList[bot_id]
        self.pos[j] = bot_pose
        self.ax.clear()

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_xlim([-LIM/2, LIM/2])
        self.ax.set_ylim([-LIM/2, LIM/2])
        #self.ax.set_xlim([-3, 3])
        #self.ax.set_ylim([-4, 2])
        self.ax.grid()

        for i in range(0, len(self.botList) ):
            clr = self.cmap(i)
            
            #self.ax.plot(self.pos[i][0], self.pos[i][1], marker='.', color='k')
            

            x2 = self.pos[i][0]
            y2 = self.pos[i][1]
            theta2 = self.pos[i][2]


                
            start_angle2 = np.arctan2(np.sin(theta2- w_max*ts), np.cos(theta2- w_max*ts))
            end_angle2 = np.arctan2(np.sin(theta2+ w_max*ts), np.cos(theta2+ w_max*ts)) 
            if end_angle2 < start_angle2:
                end_angle2_temp = end_angle2 + 2*math.pi
            else:
                end_angle2_temp = end_angle2

            

            #PLOTTING the larger sector
            x_o = ((2*size_bot)*(np.cos(start_angle2) + np.cos(end_angle2))/(np.sin(start_angle2 - end_angle2)))+ x2;
            y_o = ((2*size_bot)*(np.sin(start_angle2) + np.sin(end_angle2))/(np.sin(start_angle2 - end_angle2)))+ y2;
            min_r = ((2*size_bot)/(np.sin((end_angle2_temp - start_angle2)/2))) - (2*size_bot);
            max_r = ((2*size_bot)/(np.sin((end_angle2_temp - start_angle2)/2))) + (2*size_bot) + (v_max)*ts;
            alp = 0.4
            ccc = plot_sec( self.pos[i][2] - w_max*ts, self.pos[i][2]+w_max*ts, x_o, y_o, min_r, max_r, clr, alp)
            self.ax.add_artist(ccc)

            ## PLOTTING the bot circle
            t = np.arange(0.0,2*np.pi,2*np.pi/100.0)
            x = [np.cos(k)*size_bot +self.pos[i][0] for k in t]
            y = [np.sin(k)*size_bot +self.pos[i][1] for k in t]
            x = np.array(x).T
            y = np.array(y).T
            xy = [x, y]
            xy = np.array(xy).T
            alp=1.0
            fov= Polygon(xy, closed=True, color='w', alpha=alp)
            self.ax.add_artist(fov)
            alp = 0.8

            #PLOTTING the smaller sector
            #ccc = plot_sec( self.pos[i][2] - w_max*ts, self.pos[i][2]+w_max*ts, self.pos[i][0]+size_bot*np.cos(self.pos[i][2]), self.pos[i][1]+size_bot*np.sin(self.pos[i][2]), v_min*ts, v_max*ts, clr, alp)
            ccc = plot_sec( self.pos[i][2] - w_max*ts, self.pos[i][2]+w_max*ts, self.pos[i][0], self.pos[i][1], v_min*ts, v_max*ts, clr, alp)
            self.ax.add_artist(ccc)

            # disThreshold = v_max * ts
            # RR = 4*(disThreshold + size_bot)
            # t = np.arange(0.0,2*np.pi,2*np.pi/100.0)
            # x = [np.cos(k)*RR +self.pos[i][0] for k in t]
            # y = [np.sin(k)*RR +self.pos[i][1] for k in t]
            # x = np.array(x).T
            # y = np.array(y).T
            # xy = [x, y]
            # xy = np.array(xy).T
            # alp=0.6
            # fov= Polygon(xy, closed=True, color='g', alpha=alp)
            # self.ax.add_artist(fov)

            ## 
            # IF collision is detected then plot the free sectors 
            ##
            # status = collision_detection(0, v_max*ts, start_angle1, end_angle1, x1+size_bot*np.cos(theta1), y1+size_bot*np.sin(theta1), min_r, max_r, start_angle2, end_angle2, x_o, y_o)  

            # if status:
            #     ## get free sector
            #     print "collision detected with :{}".format(obs_bot)
            #     fs_current = collision_avoidance(0, v_max*ts, start_angle1, end_angle1, x1+size_bot*np.cos(theta1), y1+size_bot*np.sin(theta1), min_r, max_r, start_angle2, end_angle2, x_o, y_o)  
            #     occupied_sects.append(fs_current)

        

        self.ax.relim()
        self.ax.autoscale_view()
        #We need to draw *and* flush
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def process(bot_name):
    plot = DynamicUpdate()
    PO = PoseCollector(bot_name)

    rospy.init_node('{}_con'.format(bot_name), anonymous=True)
    rospy.Subscriber('/global_pose', String, PO.callback)
    #pub = rospy.Publisher('/{}/cmd_vel'.format(bot_name), Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    lopcount = 0
    while not rospy.is_shutdown():
        aBots =  PO.getAllPose()
        for bot,pose in aBots.iteritems():
            plot.PlotData(pose, bot)
        rate.sleep()








def get_s_intersection(a,b):
    ans = []
    ### case 1    

    #are they really intersecting?
    if not (angle_between(a[2],a[3],b[2]) or angle_between(a[2],a[3],b[3])):
        return [a]
    
    if (b[0] > a[1]):
        return [a]
        
    
    if (b[2]-a[2] >0):
        temp_sector = [a[0],a[1],a[2],min(b[2],a[3])]
        ans.append(temp_sector)

    if a[3]-b[3] >0:
        temp_sector = [a[0],a[1],max(b[3],a[2]),a[3]]
        ans.append(temp_sector)
    
    
    temp_sector = [ a[0], min(b[0],a[1]), b[2], b[3]]
    ans.append(temp_sector)

    return ans

def angle_between(start_angle, end_angle, check_angle):
    if end_angle < start_angle:
        end_angle = end_angle+math.pi*2
        
    if( check_angle >= start_angle) and (check_angle <= end_angle):
        return True
    
    return False


def collision_detection(I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y):
    status_inter_line1 = line_line(I1_x,I1_y,I1_min_theta,I1_min_r,I1_max_r,I2_x,I2_y,I2_min_theta,I2_min_r,I2_max_r)
    status_inter_line2 = line_line(I1_x,I1_y,I1_min_theta,I1_min_r,I1_max_r,I2_x,I2_y,I2_max_theta,I2_min_r,I2_max_r)
    status_inter_line3 = line_line(I1_x,I1_y,I1_max_theta,I1_min_r,I1_max_r,I2_x,I2_y,I2_min_theta,I2_min_r,I2_max_r)
    status_inter_line4 = line_line(I1_x,I1_y,I1_max_theta,I1_min_r,I1_max_r,I2_x,I2_y,I2_max_theta,I2_min_r,I2_max_r)
    
    status_inter_arcu_arcu = arc_arc(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_max_r,I2_x,I2_y,I2_min_theta,I2_max_theta,I2_max_r)
    status_inter_arcu_arcl = arc_arc(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_max_r,I2_x,I2_y,I2_min_theta,I2_max_theta,I2_min_r)
    status_inter_arcl_arcu = arc_arc(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_min_r,I2_x,I2_y,I2_min_theta,I2_max_theta,I2_max_r)
    status_inter_arcl_arcl = arc_arc(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_min_r,I2_x,I2_y,I2_min_theta,I2_max_theta,I2_min_r)

    status_inter_arc_line1 = arc_line(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_max_r,I2_x,I2_y,I2_min_theta,I2_min_r,I2_max_r)
    status_inter_arc_line2 = arc_line(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_max_r,I2_x,I2_y,I2_max_theta,I2_min_r,I2_max_r)
    status_inter_arc_line3 = arc_line(I2_x,I2_y,I2_min_theta,I2_max_theta,I2_max_r,I1_x,I1_y,I1_min_theta,I1_min_r,I1_max_r)
    status_inter_arc_line4 = arc_line(I2_x,I2_y,I2_min_theta,I2_max_theta,I2_max_r,I1_x,I1_y,I1_max_theta,I1_min_r,I1_max_r)

    status_inter_arc_line5 = arc_line(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_min_r,I2_x,I2_y,I2_min_theta,I2_min_r,I2_max_r)
    status_inter_arc_line6 = arc_line(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_min_r,I2_x,I2_y,I2_max_theta,I2_min_r,I2_max_r)
    status_inter_arc_line7 = arc_line(I2_x,I2_y,I2_min_theta,I2_max_theta,I2_min_r,I1_x,I1_y,I1_min_theta,I1_min_r,I1_max_r)
    status_inter_arc_line8 = arc_line(I2_x,I2_y,I2_min_theta,I2_max_theta,I2_min_r,I1_x,I1_y,I1_max_theta,I1_min_r,I1_max_r)
        
    if((status_inter_line1 == 1) or (status_inter_line2 == 1) or (status_inter_line3 == 1) or (status_inter_line4 == 1) or (status_inter_arcu_arcu == 1) or (status_inter_arcu_arcl == 1) or (status_inter_arcl_arcu == 1) or (status_inter_arcl_arcl == 1) or (status_inter_arc_line1 == 1) or (status_inter_arc_line2 == 1) or (status_inter_arc_line3 == 1) or (status_inter_arc_line4 == 1) or (status_inter_arc_line5 == 1) or (status_inter_arc_line6 == 1) or (status_inter_arc_line7 == 1) or (status_inter_arc_line8 == 1)):
        #collision_status = 1
        return 1
    else:
        bot_inside_obs = inclusion_interval(I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y)
        obs_inside_bot = inclusion_interval(I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y,I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y)

        if(bot_inside_obs == 1 or obs_inside_bot == 1):
            #collision_status = 1
            return 1
        else:
            #collision_status = 0
            return 0
        
def inclusion_interval(I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y):
    
    I1x1 = I1_x + I1_min_r * np.cos(I1_min_theta)
    I1y1 = I1_y + I1_min_r * np.sin(I1_min_theta)
    I1x1d = I1_x + I1_min_r * np.cos(I1_max_theta)
    I1y1d = I1_y + I1_min_r * np.sin(I1_max_theta)
    
    I1x2 = I1_x + I1_max_r * np.cos(I1_min_theta)
    I1y2 = I1_y + I1_max_r * np.sin(I1_min_theta)
    I1x2d = I1_x + I1_max_r * np.cos(I1_max_theta)
    I1y2d = I1_y + I1_max_r * np.sin(I1_max_theta)

    d_I2xy_I1x1y1 = abs(np.sqrt((I2_x-I1x1)*(I2_x-I1x1) + (I2_y-I1y1)*(I2_y-I1y1)))
    a_I2xy_I1x1y1 = np.arctan2((I1y1-I2_y),(I1x1-I2_x))
    d_I2xy_I1x1dy1d = abs(np.sqrt((I2_x-I1x1d)*(I2_x-I1x1d) + (I2_y-I1y1d)*(I2_y-I1y1d)))
    a_I2xy_I1x1dy1d = np.arctan2((I1y1d-I2_y),(I1x1d-I2_x))
    d_I2xy_I1x2y2 = abs(np.sqrt((I2_x-I1x2)*(I2_x-I1x2) + (I2_y-I1y2)*(I2_y-I1y2)))
    a_I2xy_I1x2y2 = np.arctan2((I1y2-I2_y),(I1x2-I2_x))
    d_I2xy_I1x2dy2d = abs(np.sqrt((I2_x-I1x2d)*(I2_x-I1x2d) + (I2_y-I1y2d)*(I2_y-I1y2d)))
    a_I2xy_I1x2dy2d = np.arctan2((I1y2d-I2_y),(I1x2d-I2_x))
    
    if(d_I2xy_I1x1y1 >= I2_min_r and d_I2xy_I1x1dy1d >= I2_min_r and d_I2xy_I1x2y2 >= I2_min_r and d_I2xy_I1x2dy2d >= I2_min_r and d_I2xy_I1x1y1 <= I2_max_r and d_I2xy_I1x1dy1d <= I2_max_r and d_I2xy_I1x2y2 <= I2_max_r and d_I2xy_I1x2dy2d <= I2_max_r): 
        if(check_contain(I2_min_theta,a_I2xy_I1x1y1,I2_max_theta) == 1 and check_contain(I2_min_theta,a_I2xy_I1x1dy1d,I2_max_theta) == 1 and check_contain(I2_min_theta,a_I2xy_I1x2y2,I2_max_theta) == 1 and check_contain(I2_min_theta,a_I2xy_I1x2dy2d,I2_max_theta) == 1):
            #status_inclusion = 1;
            return 1
        else:
            #status_inclusion = 0;
            return 0
    else:
        #status_inclusion = 0;
        return 0

def arc_arc(x1,y1,theta1,theta1d,r1,x2,y2,theta2,theta2d,r2):
    arc1_circ2 = 0
    arc2_circ1 = 0
    arc1_arc2 = 0
    flag_theta1d_neg = 0
    flag_theta2d_neg = 0
    if theta1d<theta1:
        theta1d = theta1d + 2*np.pi
        flag_theta1d_neg = 1
    if theta2d<theta2:
        theta1d = theta1d + 2*np.pi
        flag_theta2d_neg = 1

    x1r = x1 + r1 * np.cos(theta1)
    y1r = y1 + r1 * np.sin(theta1)
    x1rd = x1 + r1 * np.cos(theta1d)
    y1rd = y1 + r1 * np.sin(theta1d)
    x1rdd = x1 + r1 * np.cos(theta1 + (theta1d-theta1)/2)
    y1rdd = y1 + r1 * np.sin(theta1 + (theta1d-theta1)/2)
    x2p = x2 + r2 * np.cos(theta2)
    y2p = y2 + r2 * np.sin(theta2)
    x2pd = x2 + r2 * np.cos(theta2d)
    y2pd = y2 + r2 * np.sin(theta2d)
    x2pdd = x2 + r2 * np.cos(theta2 + (theta2d-theta2)/2)
    y2pdd = y2 + r2 * np.sin(theta2 + (theta2d-theta2)/2)
    
    dist_x1y1_x2y2 = np.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))
    dist_x1y1_x2py2p = np.sqrt((x1-x2p)*(x1-x2p) + (y1-y2p)*(y1-y2p))
    dist_x1y1_x2pdy2pd = np.sqrt((x1-x2pd)*(x1-x2pd) + (y1-y2pd)*(y1-y2pd))
    dist_x1y1_x2pddy2pdd = np.sqrt((x1-x2pdd)*(x1-x2pdd) + (y1-y2pdd)*(y1-y2pdd))
    dist_x2y2_x1ry1r = np.sqrt((x1r-x2)*(x1r-x2) + (y1r-y2)*(y1r-y2))
    dist_x2y2_x1rdy1rd = np.sqrt((x1rd-x2)*(x1rd-x2) + (y1rd-y2)*(y1rd-y2))
    dist_x2y2_x1rddy1rdd = np.sqrt((x1rdd-x2)*(x1rdd-x2) + (y1rdd-y2)*(y1rdd-y2))

    if dist_x1y1_x2y2 > (r1+r2):
            arc1_circ2 = 0
            arc2_circ1 = 0
            status_arc_arc = 0
    else:
            if (dist_x1y1_x2py2p < r1 and dist_x1y1_x2pdy2pd > r1) or (dist_x1y1_x2py2p > r1 and dist_x1y1_x2pdy2pd < r1):
                arc2_circ1 = 1 
            else:
                slope_x1y1_x2y2 = np.arctan2((y1-y2),(x1-x2))
                if (flag_theta2d_neg == 1 and slope_x1y1_x2y2 < 0):
                    slope_x1y1_x2y2 = slope_x1y1_x2y2 + 2*np.pi
                if (dist_x1y1_x2py2p > r1 and dist_x1y1_x2pdy2pd > r1):
                    if (check_contain(theta2,slope_x1y1_x2y2,theta2d)):
                        arc2_circ1 = 2
                    else:
                        arc2_circ1 = 0
                elif (dist_x1y1_x2py2p < r1 and dist_x1y1_x2pdy2pd < r1):
                    if (check_contain(theta2+np.pi,slope_x1y1_x2y2,theta2d+np.pi)):
                        arc2_circ1 = 2
                    else:
                        arc2_circ1 = 0
                        
                

            if ((dist_x2y2_x1ry1r < r2 and dist_x2y2_x1rdy1rd > r2) or (dist_x2y2_x1ry1r > r2 and dist_x2y2_x1rdy1rd < r2)):
                arc1_circ2 = 1 
            else:
                slope_x2y2_x1y1 = np.arctan2((y2-y1),(x2-x1))
                if (flag_theta1d_neg == 1 and slope_x2y2_x1y1 < 0):
                    slope_x2y2_x1y1 = slope_x2y2_x1y1 + 2*np.pi
                if (dist_x2y2_x1ry1r > r2 and dist_x2y2_x1rdy1rd > r2):
                    if (check_contain(theta1,slope_x2y2_x1y1,theta1d)):
                        arc1_circ2 = 2
                    else:
                        arc1_circ2 = 0
                elif ( dist_x2y2_x1ry1r < r2 and dist_x2y2_x1rdy1rd < r2):    
                    if ( check_contain( theta1+np.pi, slope_x2y2_x1y1, theta1d+np.pi)):
                        arc1_circ2 = 2
                    else:
                        arc1_circ2 = 0
                        
                
                    
    
            if (arc2_circ1 == 2 and arc1_circ2 == 2):
                arc1_arc2 = 2
            elif ((arc2_circ1 == 2 and arc1_circ2 == 1) or (arc2_circ1 == 1 and arc1_circ2 == 2)):
                arc1_arc2 = 1
            elif (arc2_circ1 == 1 and arc1_circ2 == 1):
                angleI1 = np.arctan2(np.sin((theta1+theta1d)/2),np.cos((theta1+theta1d)/2))
                angleI2 = np.arctan2(np.sin((theta2+theta2d)/2),np.cos((theta2+theta2d)/2))
       
                angle_bw_centers = np.arctan2((y2 - y1),(x2 - x1))
                angleI1 = np.arctan2(np.sin(angleI1 - angle_bw_centers),np.cos(angleI1 - angle_bw_centers))
                angleI2 = np.arctan2(np.sin(angleI2 - angle_bw_centers),np.cos(angleI2 - angle_bw_centers))
       
                if ((angleI1 > 0 and angleI1 < 180 and angleI2 > 0 and angleI2 < 180) or (angleI1 > -180 and angleI1 < 0 and angleI2 > -180 and angleI2 < 0)): 
                    arc1_arc2 = 1
                else:
                    arc1_arc2 = 0
                
       
    if (arc1_arc2 == 1 or arc1_arc2 == 2):
            #status_arc_arc = 1
        return 1
    else:
        #status_arc_arc = 0    
        return 0

def arc_line(x1,y1,theta1,theta1d,r1,x2,y2,theta2,r2l,r2u):
    if (theta1d<theta1):
        theta1d = theta1d + 2*np.pi
    
    x2p = x2 + r2u * np.cos(theta2)
    y2p = y2 + r2u * np.sin(theta2)
    x2pd = x2 + r2l * np.cos(theta2)
    y2pd = y2 + r2l * np.sin(theta2)

    x1r = x1 + r1 * np.cos(theta1)
    y1r = y1 + r1 * np.sin(theta1)
    x1rd = x1 + r1 * np.cos(theta1d)
    y1rd = y1 + r1 * np.sin(theta1d)
    
    line_arc =  0
    line_circ = 0

    l_perp = 0
    x1_lperp_q = 0
    y1_lperp_q = 0
    x1_lperp_qd = 0
    y1_lperp_qd = 0
    y1q=0
    x1q=0
    line_arc=0
    theta_x1y1_x1qy1q =0
    # (x2pd,y2pd) and (x2p,y2p) are the end points of the line
    # (x1,y1) is the center of the arc and (x1r,y1r),(x1rd,y1rd) are the
    # extreme points of the arc.

    d_x1y1_x2pdy2pd = np.sqrt((x2pd-x1)*(x2pd-x1) + (y2pd-y1)*(y2pd-y1))
    d_x1y1_x2py2p = np.sqrt((x2p - x1)*(x2p - x1) + (y2p-y1)*(y2p-y1))

    if (d_x1y1_x2pdy2pd < r1 and d_x1y1_x2py2p > r1):
        line_circ = 1
        beta1 = np.arctan2((y1r-y2pd),(x1r-x2pd))
        beta2 = np.arctan2((y1rd-y2pd),(x1rd-x2pd))
        if (beta1>beta2):
            beta2 = beta2 + 2*np.pi
            if (theta2 < 0):
                alpha = theta2 + 2*np.pi
            else:
                alpha = theta2
        else:
            alpha = theta2
        
        if (alpha>beta1 and alpha<beta2):
            line_arc = 1
        else:
            line_arc = 0
        
    elif (d_x1y1_x2pdy2pd > r1 and d_x1y1_x2py2p < r1): # always you need to check beta1<theta2<beta2 condition the point of line segment which is inside the circle
        line_circ = 1
        alpha1 = theta2 + np.pi
        alpha1 = np.arctan2(np.sin(alpha1),np.cos(alpha1))

        beta1 = np.arctan2((y1r-y2p),(x1r-x2p))
        beta2 = np.arctan2((y1rd-y2p),(x1rd-x2p))
        if (beta1>beta2):
            beta2 = beta2+2*np.pi
            if (alpha1<0):
                alpha11 = alpha1 +2*np.pi
            else:
                alpha11 = alpha1
            
        else:
            alpha11 = alpha1
        
        if (alpha11>beta1 and alpha11<beta2):
            line_arc = 1
        else:
            line_arc = 0
        
    else:
        if ((d_x1y1_x2pdy2pd > r1 and d_x1y1_x2py2p > r1)):
            l_perp = abs((x1*np.tan(theta2) - y1 - np.tan(theta2)*x2 + y2)/(np.sqrt(1+np.tan(theta2)*np.tan(theta2))))
            x1_lperp_q = x1 + l_perp * np.cos(theta2+np.pi/2)
            y1_lperp_q = y1 + l_perp * np.sin(theta2+np.pi/2)
            x1_lperp_qd = x1 + l_perp * np.cos(theta2-np.pi/2)
            y1_lperp_qd = y1 + l_perp * np.sin(theta2-np.pi/2)

        if (-0.0001 < ((y1_lperp_q-y2)-np.tan(theta2)*(x1_lperp_q-x2)) and ((y1_lperp_q-y2)-np.tan(theta2)*(x1_lperp_q-x2)) < 0.0001):
            x1q = x1 + r1 * np.cos(theta2+np.pi/2)
            y1q = y1 + r1 * np.sin(theta2+np.pi/2)
            theta_x1y1_x1qy1q = np.arctan2(np.sin(theta2+np.pi/2),np.cos(theta2+np.pi/2))
        elif(-0.0001 < ((y1_lperp_qd-y2)-np.tan(theta2)*(x1_lperp_qd-x2)) and ((y1_lperp_qd-y2)-np.tan(theta2)*(x1_lperp_qd-x2)) < 0.0001):
            x1q = x1 + r1 * np.cos(theta2-np.pi/2)
            y1q = y1 + r1 * np.sin(theta2-np.pi/2)
            theta_x1y1_x1qy1q = np.arctan2(np.sin(theta2-np.pi/2),np.cos(theta2-np.pi/2))
        
        d_x2y2_lperp = np.sqrt(d_x1y1_x2pdy2pd * d_x1y1_x2pdy2pd - l_perp * l_perp)
        d_x2py2p_lperp = np.sqrt(d_x1y1_x2py2p * d_x1y1_x2py2p - l_perp * l_perp)
        
        if(d_x2y2_lperp < (r2u-r2l) and d_x2py2p_lperp < (r2u-r2l) and l_perp < r1):
            line_circ = 2
            if (((x1r*np.tan(theta2)-y1r-np.tan(theta2)*x2+y2)<0 and (x1rd*np.tan(theta2)-y1rd-np.tan(theta2)*x2+y2)>0) or ((x1r*np.tan(theta2)-y1r-np.tan(theta2)*x2+y2)>0 and (x1rd*np.tan(theta2)-y1rd-np.tan(theta2)*x2+y2)<0)):
                line_arc = 1
            else: 
                if (((x1r*np.tan(theta2)-y1r-np.tan(theta2)*x2+y2)<0 and (x1q*np.tan(theta2)-y1q-np.tan(theta2)*x2+y2)>0) or ((x1r*np.tan(theta2)-y1r-np.tan(theta2)*x2+y2)>0 and (x1q*np.tan(theta2)-y1q-np.tan(theta2)*x2+y2)<0)):
    #                  if(theta1<180 && theta1d>180)
    #                      if(theta_x1y1_x1qy1q < 0)
    #                          theta_x1y1_x1qy1q = theta_x1y1_x1qy1q +360;
    #                      end
    #                  end
    #                if(theta_x1y1_x1qy1q > theta1 && theta_x1y1_x1qy1q < theta1d)
                     if (check_contain(theta1,theta_x1y1_x1qy1q,theta1d) == 1):   
                         line_arc = 2
                     else:
                         line_arc = 0
                     

                else: 
                    line_arc = 0
                
            
        else:
            line_circ = 0
            line_arc = 0
        
        
    

    if(line_arc == 1 or line_arc == 2):
        #status_arc_line = 1;
        return 1
    else:
        #status_arc_line = 0;
        return 0

def line_line(x1,y1,theta1,r1l,r1u,x2,y2,theta2,r2l,r2u):
    x1ru = x1 + r1u*np.cos(theta1)
    y1ru = y1 + r1u*np.sin(theta1)
    x1rl = x1 + r1l*np.cos(theta1)
    y1rl = y1 + r1l*np.sin(theta1)

    x2ru = x2 + r2u*np.cos(theta2)
    y2ru = y2 + r2u*np.sin(theta2)
    x2rl = x2 + r2l*np.cos(theta2)
    y2rl = y2 + r2l*np.sin(theta2)

    slope_x1y1_x1ry1r = np.arctan2((y1ru-y1),(x1ru-x1))
    slope_x2y2_x2ry2r = np.arctan2((y2ru-y2),(x2ru-x2))

    if((slope_x1y1_x1ry1r == slope_x2y2_x2ry2r) or (slope_x1y1_x1ry1r + np.pi == slope_x2y2_x2ry2r) or (slope_x1y1_x1ry1r == slope_x2y2_x2ry2r + np.pi)):
        #status_line_line = 0
        return 0
    else:
        if((((y1ru-y1)*(x2rl-x1) - (x1ru-x1)*(y2rl-y1))>0 and ((y1ru-y1)*(x2ru-x1) - (x1ru-x1)*(y2ru-y1))<0) or (((y1ru-y1)*(x2rl-x1) - (x1ru-x1)*(y2rl-y1))<0 and ((y1ru-y1)*(x2ru-x1) - (x1ru-x1)*(y2ru-y1))>0)):
            if((((y2ru-y2)*(x1rl-x2) - (x2ru-x2)*(y1rl-y2))>0 and ((y2ru-y2)*(x1ru-x2) - (x2ru-x2)*(y1ru-y2))<0) or (((y2ru-y2)*(x1rl-x2) - (x2ru-x2)*(y1rl-y2))<0 and ((y2ru-y2)*(x1ru-x2) - (x2ru-x2)*(y1ru-y2))>0)):
                #status_line_line = 1
                return 1
            else:
                #status_line_line = 0
                return 0
        else:
            #status_line_line = 0
            return 0

def check_contain(lower_angle,check_angle,upper_angle):
    l_angle = np.arctan2(np.sin(lower_angle),np.cos(lower_angle))
    u_angle = np.arctan2(np.sin(upper_angle),np.cos(upper_angle))
    c_angle = np.arctan2(np.sin(check_angle),np.cos(check_angle))

    if (u_angle < l_angle):
        u_angle = u_angle + 2*np.pi
        if (c_angle < 0):
            c_angle = c_angle + 2*np.pi
 
    if (l_angle < c_angle and u_angle > c_angle):
        #status_contain = 1
        return 1
    else:
        #status_contain = 0;
        return 0



def collision_avoidance(I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y):
    I_obs_min_r = 0
    I_obs_max_r = 0
    I_obs_min_theta = 0
    I_obs_max_theta = 0
    k=0
    h=0
    N=10
    M=10
    I_robot_free = []
    
    if(I1_min_theta > I1_max_theta):
        I1_max_theta = I1_max_theta + 2*math.pi
    

    if(I2_min_theta > I2_max_theta):
        I2_max_theta = I2_max_theta + 2*math.pi
    
    [I_min_left, I_max_left, I_min_right, I_max_right] = bisect(I1_min_r,I1_max_r)

    collision_status_left = collision_detection(I_min_left,I_max_left,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y)
    collision_status_right = collision_detection(I_min_right,I_max_right,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y)

    while (k < N): 
        if ((collision_detection(I_min_right,I_max_right,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 0) and (collision_detection(I_min_left,I_max_left,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1)):
            [ I_min_leftleft, I_max_leftleft, I_min_leftright, I_max_leftright] = bisect(I_min_left,I_max_left)
            I_min_left1 = I_min_leftleft
            I_max_left1 = I_max_leftleft
            I_min_right1 = I_min_leftright
            I_max_right1 = I_max_leftright
        elif ((collision_detection(I_min_left,I_max_left,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 0) and (collision_detection(I_min_right,I_max_right,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1)):
            [I_min_rightleft,I_max_rightleft,I_min_rightright,I_max_rightright] = bisect(I_min_right,I_max_right)    
            I_min_left1 = I_min_rightleft
            I_max_left1 = I_max_rightleft
            I_min_right1 = I_min_rightright
            I_max_right1 = I_max_rightright
        else:

            [I_min_leftleft, I_max_leftleft, I_min_leftright, I_max_leftright] = bisect(I_min_left,I_max_left)
            [I_min_rightleft, I_max_rightleft, I_min_rightright, I_max_rightright] = bisect(I_min_right,I_max_right)

            if(collision_detection(I_min_leftleft,I_max_leftleft,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1):
                I_min_left1 = I_min_leftleft
                I_max_left1 = I_max_leftleft
        #          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_left1,I_max_left1,'m');
            else:
                I_min_left1 = I_min_leftright
                I_max_left1 = I_max_leftright
        #          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_left1,I_max_left1,'y');
            
            if(collision_detection(I_min_rightright,I_max_rightright,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1):
                I_min_right1 = I_min_rightright
                I_max_right1 = I_max_rightright
        #          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_right1,I_max_right1,'b');
            else:
                I_min_right1 = I_min_rightleft
                I_max_right1 = I_max_rightleft
        #          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_right1,I_max_right1,'c');
            
        I_min_left = I_min_left1
        I_max_left = I_max_left1
        I_min_right = I_min_right1
        I_max_right = I_max_right1
        k = k+1
    
    I_obs_min_r = I_min_left
    I_obs_max_r = I_max_right

    # bisection w.r.t theta
    
    [I_min_left, I_max_left, I_min_right, I_max_right] = bisect(I1_min_theta,I1_max_theta)
    collision_status_left = collision_detection(I1_min_r,I1_max_r,I_min_left,I_max_left,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y)
    collision_status_right = collision_detection(I1_min_r,I1_max_r,I_min_right,I_max_right,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y)

    
    while(h < M): # bisction w.r.t theta
        if((collision_detection(I1_min_r,I1_max_r,I_min_right,I_max_right,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 0) and (collision_detection(I1_min_r,I1_max_r,I_min_left,I_max_left,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1)):
            [I_min_leftleft, I_max_leftleft, I_min_leftright, I_max_leftright] = bisect(I_min_left,I_max_left)
            I_min_left1 = I_min_leftleft
            I_max_left1 = I_max_leftleft
            I_min_right1 = I_min_leftright
            I_max_right1 = I_max_leftright
        elif((collision_detection(I1_min_r,I1_max_r,I_min_left,I_max_left,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 0) and (collision_detection(I1_min_r,I1_max_r,I_min_right,I_max_right,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1)):
            [I_min_rightleft,I_max_rightleft,I_min_rightright,I_max_rightright] = bisect(I_min_right,I_max_right)    
            I_min_left1 = I_min_rightleft
            I_max_left1 = I_max_rightleft
            I_min_right1 = I_min_rightright
            I_max_right1 = I_max_rightright
        else:
            [I_min_leftleft,  I_max_leftleft, I_min_leftright, I_max_leftright] = bisect(I_min_left,I_max_left)
            [I_min_rightleft, I_max_rightleft, I_min_rightright, I_max_rightright] = bisect(I_min_right,I_max_right)

            if(collision_detection(I1_min_r,I1_max_r,I_min_leftleft,I_max_leftleft,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1):
                I_min_left1 = I_min_leftleft
                I_max_left1 = I_max_leftleft
    #             plot_sec(I_min_left1,I_max_left1,I1_x,I1_y,I1_min_r,I1_max_r,'m');    
            else:
                I_min_left1 = I_min_leftright
                I_max_left1 = I_max_leftright
    #         plot_sec(I_min_left1,I_max_left1,I1_x,I1_y,I1_min_r,I1_max_r,'y');
        
            if(collision_detection(I1_min_r,I1_max_r,I_min_rightright,I_max_rightright,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1):
                I_min_right1 = I_min_rightright
                I_max_right1 = I_max_rightright
    #         plot_sec(I_min_right1,I_max_right1,I1_x,I1_y,I1_min_r,I1_max_r,'b');
            else:
                I_min_right1 = I_min_rightleft
                I_max_right1 = I_max_rightleft
    #         plot_sec(I_min_right1,I_max_right1,I1_x,I1_y,I1_min_r,I1_max_r,'c');
        
        I_min_left = I_min_left1
        I_max_left = I_max_left1
        I_min_right = I_min_right1
        I_max_right = I_max_right1
        h = h+1
    
    I_obs_min_theta = I_min_left
    I_obs_max_theta = I_max_right
    
    I_robot_obs = [I_obs_min_r, I_obs_max_r, I_obs_min_theta, I_obs_max_theta]
    #I_robot_free = free_sectors(I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y,I_robot_obs)
    #return I_robot_free
    return I_robot_obs

def bisect(I_min,I_max):
    I_min_left = I_min
    I_max_left = I_min + (I_max-I_min)/2
    I_min_right = I_min + (I_max-I_min)/2
    I_max_right = I_max
    return [I_min_left, I_max_left, I_min_right, I_max_right]



if __name__ == '__main__':
    print("Process started ")
    plt.ion()
    process('bot_1')




    # A = np.array([[0.1, 0.3, 0.6], [0.4, 0.4, 0.2], [0.1, 0.5,0.4]])

    # plot.PlotData(2,2, 'bot_1')
    # plot.PlotData(2,-4, 'bot_2') 
    # plot.PlotData(2.1,2, 'bot_1')
    # plot.PlotData(1.9,-4, 'bot_2') 
    # plot.PlotData(2.3,2, 'bot_1')
    # plot.PlotData(1.8,-4, 'bot_2') 


    ans = raw_input("Go ?")
    while ans:
        ans = raw_input("Go ?")

