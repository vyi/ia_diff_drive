#!/usr/bin/env python
from __future__ import division

import sys
import rospy
import numpy as np
from math import pi, sin, cos, atan2
import json

import tf
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from Bot import Bot
from ia_helper import collision_avoidance, collision_detection, get_s_intersection, dist


### Globals
pose_b1 = [0,0,0]     # What does this container do?
pose_b2 = [0,0,0]	  # What does this container do?

VEL = 0.1             # Belief of other robot's linear velocity
OMEGA = 0.25          # Belief of other robot's angular velocity

SIZEBOT = 0.6         # radius corresponding to circular footprint of robot

fn = '/home/vivek/ros_files/sc635_workspace/src/ia_ca/log.txt' 			  # absolute path of log file
traj = []

TIMES = 3.0           # How far we anticipate the other robot to travel. 
SAFEDIST = 4*(VEL*TIMES + SIZEBOT)

botname=""
listener = ""
buf = ""

## a helper function
def checkC(A,B):
    ''' Checks if sector defined by A and B are intersecting 
        Input
        -----
        A : sector 1
        B : sector 2
        
        Returns 
        -------
        Status: True if there is a intersection of the two sectors
        Occupied sector
    ''' 
    #print A,',',B, '*****************'
    coll_FLAG = False
    non_free_sector = [0,0,0,0]
    if collision_detection( *(A+B)):
        coll_FLAG = True
        non_free_sector = collision_avoidance( *(A+B) )  

    return coll_FLAG, non_free_sector

## heuristic for choosing control action
def choose_sect(SList, sA, eA):
    '''
    Input:
        SList: List of available free sectors
        sA   : start angle of the full sector
        eA   : end angle of the full sector
    Output:
        The v,w to drive bot towards safer course
        ma   : Middle angle of the sector

    '''

    ## Defining constants for reference
    Vm = VEL
    Wm = OMEGA
    Ts = TIMES

    ## Choose a free sector
    areas = []
    velos = []

    for sector in SList:
        # For each sector calculate the corresponding area and the max velocity
        vel = sector[1]

        # Calculate sector angle
        end     = sector[3]
        start   = sector[2]
        # If end sector is smaller i.e. start < pi, end > pi
        if end < start:
            end = end + pi*2
        theta = end-start
        
        ## Area 
        area = (abs(sector[1] - sector[0])**2)*theta*0.5
        
        ## Save velocity and area
        areas.append(area)
        velos.append(vel)
        

    # How much is the total area? (We use this value for detecting no intersection)
    total_area = sum(areas)
    fullArea = ((Vm*Ts)**2)*(eA-sA)*0.5

    # How much area is now free?
    print("Free Area:{}, Total area:{}".format(total_area, fullArea))

    ## If the free area is small. We stop the bot.
    if total_area < 0.001:
        return 0.0, 0.0, 0                                    ## E X I T       case 1

    # When whole sector is free?
    if (fullArea - total_area) < 0.001:    
        return 0.0, 0.0, 1                                    ## E X I T       case 2

    #else:
        #print "{} out of {} area is free".format(total_area, fullArea)
        
    # velocity : We choose the minimum 
    chosen_v = min(velos)

    print("Choosen {} among {}".format(chosen_v, velos))

    # Omega(w): Max velocity sector -> Max Area -> w
    velo_m = max(velos)
    v_idx = [vv for vv, x in enumerate(velos) if x == velo_m]

    areax = []
    for e in v_idx:
        areax.append(areas[e])
    arx = max(areax)
    
    # i contains the sector which offers most room for w
    i = areas.index(arx)
    
    # Debug message. scrap after reading
    print "{} **** Chosen sector".format(SList[i])

    ## Calculate Omega(w) 
      # w : W_min
      # w : W_max
      # w : W_min < w < W_max

    ## What are the start and end angles for the chosen sector
    csSA = SList[i][2]
    csEA = SList[i][3]

    if csEA < csSA:
        csEA = csEA + 2*pi
    if eA < sA:
        eA = eA + 2*pi
    
    ## Initialize the variables
    w = 0
    v = chosen_v/Ts
    ## test booundary cases
    if abs(csSA - sA)<0.01:
        ## The start angle is closer to start
        w = -Wm
        print "The sector is closer to boundary S {},{} chosen w:{}".format(csSA, sA, w)

    elif abs(csEA - eA)<0.01:
        ## The end angle is closer to end
        w = Wm
        print "The sector is closer to boundary E {},{} chosen w:{}".format(csEA, eA, w)

    else:
        
        print("The sector is between S:{}, {}, {}, E:{}".format(sA, csSA, csEA, eA))
        mp = (eA-sA)/2
        mpc = (csEA - csSA)/2

        si = 0.0

        if (mp-mpc)>0:
            si = -1.0
        else:
            si = +1.0

        w = areas[i]/fullArea ## Weight

        w = si*Wm*w/Ts

    print "v:{}, w:{}".format(v,w)
    return v, w, 2
    

    #######
    # Strategy 2
       # Max V

    # Strategy 3
       # Max w

    # Strategy 4
        # Random

def callb1(data):
    '''
    Callback receives the odometry data
    '''
    global pose_b1, botname, listener, buf
    
    # # get the pose of the bot
    # x_pos = data.pose.pose.position.x
    # y_pos = data.pose.pose.position.y
    
    # # for calculation of yaw we need to convert quaternion to euler angles
    # # here we collect the quaterion data
    # x = data.pose.pose.orientation.x
    # y = data.pose.pose.orientation.y
    # z = data.pose.pose.orientation.z
    # w = data.pose.pose.orientation.w

    # # using inbuilt method euler_from_quaternion to get euler from quaternion
    # euler = tf.transformations.euler_from_quaternion((x,y,z,w))

    try:
        transformObject = buf.lookup_transform('map', '{}/base_footprint'.format(botname), rospy.Time(0), rospy.Duration(3.0))
        # finally we create a list with x, y, and theta
        trans = transformObject.transform.translation
        rot = transformObject.transform.rotation
        # print(trans)

        euler = tf.transformations.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
        pose_b1 = [trans.x, trans.y, euler[2]]
    except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException) as e:
        print("E exception wpc {}".format(botname))
        print(e)
        
    
    # print("Pose : {}".format(pose_b1))
    #traj.append((pose_b1, rospy.get_time()))


def controller(kw):
    '''
    Controls the bot from collisions.

    Input
    -----
    bot id: The controller will attach iTself to the given bot

    Output
    ------
    None
    '''
    global botname, listener, buf
    botname = kw[0]

    

    print "$$$ botname is {}".format(botname)
    global pose_b1
    ## Do the ros rituals
    rospy.init_node('{}_con'.format(botname), anonymous=True)
    #rospy.Subscriber('/{}/odom'.format(kw[0]), Odometry, callb1)

    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)

    # wait a second for frames to accumulate. buf.lookup_transform seems to fail immediately if it hasn't yet                                                                                                                                                                              
    # gotten any transfroms for the 'odom' frame, instead of waiting for the timeout                                                                                                                                                                                                       
    rospy.sleep(1.0)
    
    #pub = rospy.Publisher('/{}/cmd_vel'.format(kw[0]), Twist, queue_size=10)
    pub = rospy.Publisher('/{}/mobile_base/commands/velocity'.format(kw[0]), Twist, queue_size=10)
    pub2 = rospy.Publisher('/bot_X/free_sec', String, queue_size=10)

    rospy.Subscriber('/{}/odom'.format(botname), Odometry, callb1)
    
    neighbors = []
    for name in kw[1:]:
        b8 = Bot(name)
        neighbors.append(b8)

    print("XXXXXXXX Neighbors : {}".format(neighbors))
    ## Settings start here
    Vm = VEL
    Wm = OMEGA
    
    ## Sampling time
    Ts = TIMES

    ## The radii of the robot
    SizeBot = SIZEBOT
    
    ## Generate random signals to drive the bot
    v = 0.10#
    w = 0.00 #

    motion = Twist()
    
    WL = 15
    RATE = 2
    ## Set up the execution frequency
    rate = rospy.Rate(RATE) # 10hz
    lopcount = 0

    ## I'm reinventing the wheel because the previous wheel doesn't work
    while not rospy.is_shutdown():
        lopcount += 1           ## Repositioning may be required for this statement
        Cf=[]
        Cb=[]
        
        
        ## disThreshold defines the distance the bot would travel in Ts seconds with Vm velocity
        disThreshold = Vm * Ts

################################################################################ CA with walls


        ## define the parameters for bot1
        x1 = pose_b1[0]
        y1 = pose_b1[1]
        theta1 = pose_b1[2]

        ## The function atan(sin x, cos x) transforms angles remain within -pi to +pi
        start_angle1 = atan2( sin(theta1- Wm*Ts), cos(theta1- Wm*Ts)) 
        end_angle1 = atan2( sin(theta1+ Wm*Ts), cos(theta1+ Wm*Ts)) 

        start_angle2 = atan2( sin(theta1+pi- Wm*Ts), cos(theta1+pi- Wm*Ts)) 
        end_angle2 = atan2( sin(theta1+pi+ Wm*Ts), cos(theta1+pi+ Wm*Ts)) 

        ## Create Sector for robot fwd motion
        Af = [  0, Vm*Ts, start_angle1, end_angle1, x1, y1]

        ## Create Sector for robot backward motion
        Ab = [ 0, Vm*Ts, start_angle2, end_angle2, x1, y1]

        ## Create Sector for wall 1
        #B1 = [pi/4.0, pi/2.0, (WL/2.0-SizeBot), -(WL/2.0-SizeBot), 0, WL]
        B1 = [0, WL, pi/4.0, pi/2.0, (WL/2.0-SizeBot), -(WL/2.0-SizeBot)]

        ## Create Sector for wall 1
        #B2 = [3*pi/4.0, pi, (WL/2.0-SizeBot), (WL/2.0-SizeBot),0, WL]
        B2 = [0, WL, 3*pi/4.0, pi, (WL/2.0-SizeBot), (WL/2.0-SizeBot)]

        ## Create Sector for wall 1
        #B3 = [-3*pi/4.0, -pi/2.0, -(WL/2.0-SizeBot), (WL/2.0-SizeBot), 0, WL]
        B3 = [0, WL, -3*pi/4.0, -pi/2.0, -(WL/2.0-SizeBot), (WL/2.0-SizeBot)]

        ## Create Sector for wall 1
        #B4 = [-pi/4.0, 0.0, -(WL/2.0-SizeBot), -(WL/2.0-SizeBot), 0, WL]
        B4 = [ 0, WL, -pi/4.0, 0.0, -(WL/2.0-SizeBot), -(WL/2.0-SizeBot)]

        #####
        #    Here we find the collisions with other bots who are nearby
        #
        B_BOTS = []
        #for n in neighbors:
            # print n.name, ' ', n.getPose()
            ## Which bots are inside the Threshold

        #b_inside = [n.getPose() if dist(n.getPose(),pose_b1)<SAFEDIST else None for n in neighbors]
        b_inside = [n.getPose() for n in neighbors]
        #b_inside = [x for x in b_inside if x is not None]
        print "^^^^^^^SAFEDIST : {} ^^^^^^^^^ B_inside is {}".format(SAFEDIST, b_inside)
        for bi in b_inside:
            xi = bi[0]
            yi = bi[1]
            thetai = bi[2]
            
            start_anglei = atan2( sin(thetai- Wm*Ts), cos(thetai- Wm*Ts)) 
            end_anglei = atan2( sin(thetai+ Wm*Ts), cos(thetai+ Wm*Ts))
            
            B_BOTS.append([0, Vm*Ts, start_anglei, end_anglei , xi,yi])
        

        #####
        
        #motion.linear.x = v   
        if lopcount%(RATE*2) == 0:
            w = np.random.uniform(-Wm, Wm)
        #motion.angular.z = w
        #v = 0.15
        ####               ####               ####               ####

        walls = [B1,B2,B3,B4] + B_BOTS

        s1, c1 = 0,0
        s2, c2 = 0,0

        ## Food

        #for i in range(4):
        for i in range(len(walls)):
            ## Checking for collision with i'th WALL
            s1, c1 = checkC(Af, walls[i])       #In forward Direction
            s2, c2 = checkC(Ab, walls[i])       #In backward Direction

            if s1:                              #If Collision is there. Keep the sector which is now occluded.
                Cf.append(c1)
                #print c1
                #print "FWD Collision with wall {} _ {}".format(i, c1)                
                
            if s2:
                Cb.append(c2)
                #print c2
                #print "BKW Collision with wall {} _ {}".format(i, c2)                
                
            #ffree_sectors = [[ 0, Vm*Ts, start_angle1, end_angle1], [ 0, Vm*Ts, start_angle2, end_angle2]]
            free_sectors = [[ 0, Vm*Ts, start_angle1, end_angle1]]   
            ## C contains all those occupied sectors
            ## Lets find the remaining sectors
            if len(Cf):
                for obstacle in Cf:
                    res=[]
                    for sector_x in free_sectors:
                        free_sectors_x = get_s_intersection(sector_x, obstacle)
                        res = res + free_sectors_x
                    free_sectors = res

        print "Number of y free sectors {}".format(len(free_sectors))
        print free_sectors
        print ""
        

        v_cmd = 0.0
        w_cmd = 0.0
        
        v_temp = 0
        w_temp = 0
        if len(free_sectors):
            v_temp, w_temp, GoFlag = choose_sect(free_sectors, start_angle1, end_angle1)
            print "con :{}, v_temp :{}, w_temp :{}".format(GoFlag, v_temp, w_temp)

            if GoFlag == 0:
                # Sector is totally occluded
                print "Look for backward Motion"
            elif GoFlag == 1:
                v_cmd = v
                w_cmd = w
            elif GoFlag == 2:
                v_cmd = v_temp
                w_cmd = -w_temp
                
            
            

        print "v_cmd :{}, w_cmd:{}".format(v_cmd, w_cmd)
        
        

        motion.linear.x = 0#v_cmd
        motion.angular.z = 0#w_cmd
################################################################################## CA ENDS
        
        ## We are not controlling the motion at the moment
        #if lopcount%18 == 0:
        ## pub.publish(motion)

        


        ## Make a dummy publisher to publish in realtime
        d = [ [theta1, v_cmd, w_temp, 0], [ Af[2], Af[3], x1, y1, Af[0], Af[1] ], [ Ab[2], Ab[3], x1, y1, Ab[0], Ab[1] ]]
        for c1 in Cf+Cb:
            d.append([ c1[2], c1[3], x1, y1, c1[0], c1[1] ])

        d.append([-pi/4.0, 0.0, -(WL/2.0-SizeBot), -(WL/2.0-SizeBot), 0, WL])
        d.append([-3*pi/4.0, -pi/2.0, -(WL/2.0-SizeBot), (WL/2.0-SizeBot), 0, WL])
        d.append([3*pi/4.0, pi, (WL/2.0-SizeBot), (WL/2.0-SizeBot),0, WL])
        d.append([pi/4.0, pi/2.0, (WL/2.0-SizeBot), -(WL/2.0-SizeBot), 0, WL])

        X_BOTS = []
        for xb in B_BOTS:
            x = getXbot(xb)
            X_BOTS.append(x)
        d = d+X_BOTS

        msg = json.dumps(d)
        pub2.publish(msg)
        rate.sleep()

def getXbot(xb):
    indices = [2,3,4,5,0,1]
    temp = [0 for i in xb]

    jdx = 0
    for idx in indices:
        temp[jdx] = xb[idx]
        jdx = jdx + 1

    return temp

if __name__ == '__main__':
    try:
        #fn = '/home/vivek/ros_files/catkin_ws/src/test/gazebo/log/{}.csv'.format(sys.argv[1])
        #print "d:",fn
        #print "TEST TYPE:{}, data :{}".format(type(sys.argv[1:]),sys.argv[1:] )
        print("args :{}".format(sys.argv))
        bos = []
        for b in sys.argv[1:]:
            if 'bot_' in b:
                print(b)
                bos.append(b)
        controller(bos)

    except rospy.ROSInterruptException:
        with open(fn, 'w') as f:
            for pt in traj:
                f.write(str(pt[1]))
                f.write(',')
                f.write(str(pt[0]))
                f.write('\r\n')
        print('log written to disk {}. Done.'.format(fn))
        print "Bye"
