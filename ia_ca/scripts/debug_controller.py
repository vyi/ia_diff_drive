#!/usr/bin/env python
from __future__ import division

import sys
import rospy
import numpy as np
from math import pi, sin, cos, atan2, copysign
import json
import datetime

import tf
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates


from Bot import Bot
from ia_helper import collision_avoidance, collision_detection, get_s_intersection, dist, is_higher

DEBUG = True
### Globals
pose_b1 = [0,0,0]     # What does this container do?
pose_b2 = [0,0,0]	  # What does this container do?

VEL = 0.10             # Belief of other robot's linear velocity
OMEGA = 0.25          # Belief of other robot's angular velocity

SIZEBOT = 0.6         # radius corresponding to circular footprint of robot

fn = '/home/vivek/ros_files/sc635_workspace/src/ia_ca/log.txt' 			  # absolute path of log file
traj = []

TIMES = 2.0           # How for into future we are anticipating the obstacle's movement. #'s' of instances 
SAFEDIST = 4*(VEL*TIMES + SIZEBOT)



botname=""
listener = ""
buf = ""
decisionFLAG = False


## a helper function
def checkC(A,B):
    ''' Checks if sector defined by A and B are intersecting 
        Input
        -----
        A : sector 1
        B : sector 2
        
        Returns 
        -------
        Status: True if there is an intersection of the two sectors
        Occupied sector
    ''' 
    #print A,',',B, '*****************'
    coll_FLAG = False
    non_free_sector = [0,0,0,0]
    if collision_detection( *(A+B)):
        coll_FLAG = True
        non_free_sector = collision_avoidance( *(A+B) )  

    return coll_FLAG, non_free_sector



### Compute the mid angle for a CC sector
def get_cc_mid_angle(start, end):

    if end*start < 0: 
        if end < 0:
            end = 2*pi + end

    mid_angle = 0.5*(end + start)
    return mid_angle

### Compute total angle fo a CC sector
def get_cc_angular_diff(start, end):
    diff_angle = 0.0

    if end*start > 0: ## both have same sign
            diff_angle = end - start
    else:
        if end < 0:
            diff_angle = 2*pi - start + end
        else:
            diff_angle = end - start
    return diff_angle


## Strategy for control 
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
    # DEBUG LINE
    # print("Inside choose_sector() \t sA:{:.3f}, eA:{:.3f}".format(sA*180/pi, eA*180/pi))
    # for s in SList:
    #     print("\t\t\t sA:{:.3f}, eA:{:.3f} \t{} \t{}".format(s[2]*180/pi, s[3]*180/pi, s[0], s[1]))
    global decisionFLAG

    ## Defining constants for reference
    Vm = VEL
    Wm = OMEGA
    Ts = TIMES

    ## Choose a free sector
    areaS = []
    veloS = []
    angleS = []

    ###### DEBUG remap angles to 0, 2*pi
    eA = eA 
    sA = sA 
    for sector in SList:
        # '''For each sector compute : Free Area, Maximum velocity, Sector mid heading/angle
        # These quantities will be used in selecting a control command to navigate robot 
        # out of obstacle course.'''

        # Calculate sector angle
        start   = sector[2] 
        end     = sector[3] 

        diff_angle = get_cc_angular_diff(sector[2], sector[3])

        ## Area available in sector 
        area = (abs(sector[1] - sector[0])**2)*diff_angle*0.5
        
        ## Save 
        areaS.append(area)

        ## Save the max horizon velocity in each sector
        vel = sector[1]
        veloS.append(vel)
        
        ## Save the max horizon velocity in each sector
        mid_angle = get_cc_mid_angle(sector[2], sector[3])
        angleS.append(mid_angle)
        

    # How much is the total area? (We use this value for detecting no intersection)
    total_area = sum(areaS)
    fullArea = ((Vm*Ts)**2)*(get_cc_angular_diff(sA, eA))*0.5

    free_area_in_percentage = int((100*total_area)/fullArea) 

    ## Print the free area available
    if (total_area > fullArea):
        print("Free area :{}  >  Total area {} ? ERRR......".format(total_area, fullArea))
    else:
        print("Free Area:{}, Total area:{}   :: {} ".format(total_area, fullArea, free_area_in_percentage))

    ## If the free area is small. We stop the forward motion and look for dirctions with safe escape
    if free_area_in_percentage < 5:
        ## Other ways to escape collision ?
        if DEBUG:
	    print("\n\nRobot is Stuck !!!!\n\n")
        return 0.0, 0.0, False                                    ## E X I T       case 1
    
    # Is most of the sector free? If almost whole sector is free, we go straight.. i.e. keep current v,w
    if free_area_in_percentage > 98:    
        return Vm, 0.0, True                                      ## E X I T       case 2
    
    ##### Thisfollowing part covers for the 5-95 % case

    ##### Choose a sector
    robot_heading = get_cc_mid_angle(sA, eA)
    
    if DEBUG:
        print("Current robot heading : {} ({})".format(robot_heading, robot_heading*180/pi))
    
    ####################################################################
    # # # # #     Strategy  : Choose direction- max vel sector   # # # #
    ####################################################################
    ma = max(veloS)           # Maximum fwd velocitty across all sectors
    ma_idx = veloS.index(ma)  # id of the sector that has max fwd velocity

    ma_angle = angleS[ma_idx] # choose the mid-angle of that sector

    ## print("Robot heading is {}({}) and sector {} heading is {} ({})".format(robot_heading, robot_heading*180/pi, SList[ma_idx], angleS, ma_angle*180/pi))
    w = copysign(OMEGA, 1 if is_higher(ma_angle, robot_heading) else -1)
    if w>0:
        print("Go CCW\n")
    elif w<0:
        print("Go CW\n")

    
    ####################################################################
    # # # # #     Strategy  :  Choose velocity with lasser area sector #
    ####################################################################
    

    mi = min(areaS)
    mi_idx = areaS.index(mi)       
    v = (SList[mi_idx][1])/(Ts)    
    
    if len(SList) == 1:
	## DUBUG PRINT
	if DEBUG:
	    print("\nCollision detected, only one sector free")
        v = v/2  ## Moving faster will lead us into collision, so slow down
        w = OMEGA ## Try to steer away from the collision
    if DEBUG:
        print("\nChosen course : \n{} from {} \n{} from {}".format(v,mi_idx, w, ma_idx))

    # decisionFLAG = True
    return v,w,True
    ####################################################################
    # # # # #     Strategy  : Choose backward motion if all else fails #
    ####################################################################

    

    # Strategy 5 : Choose a free sector at random
    # print("Using Random Strategy")

    # i = np.random.randint(len(SList))
    # v = SList[i][1]
    # w = copysign(OMEGA, 1 if is_higher(angleS[i], robot_heading) else -1)

    # return v,w, True
    # Strategy 5 : Choose a linear combination of above 3 strategies

    ##### Generate control signal
    # Strategy 1 : Use ON-OFF control (using maximum velocities to escape the obstacle)


    # Strategy 2 : PID control ()
       

    

def callb1(msg):
    '''Get robot pose from /gazebo/model_states'''
    global botname, pose_b1

    names = msg.name
    idx  = names.index(botname)
    
    pose_b1[0] = msg.pose[idx].position.x
    pose_b1[1] = msg.pose[idx].position.y
    x = msg.pose[idx].orientation.x
    y = msg.pose[idx].orientation.y
    z = msg.pose[idx].orientation.z
    w = msg.pose[idx].orientation.w
    euler = tf.transformations.euler_from_quaternion((x,y,z,w))
    pose_b1[2] = euler[2]


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
    global botname, listener, buf, pose_b1, decisionFLAG
    
    #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### DEBUG _ vy
    
    # botname = 'bot_1'
    botname = kw[0]
    print "$$$ botname is {}".format(botname)
    
    ## Do the ros rituals
    rospy.init_node('{}_con'.format(botname), anonymous=True)
    
    pub = rospy.Publisher('/{}/mobile_base/commands/velocity'.format(botname), Twist, queue_size=1)
    pub2 = rospy.Publisher('/bot_X/free_sec', String, queue_size=1)

    rospy.Subscriber('/gazebo/model_states', ModelStates, callb1)
    
    neighbors = []
    #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### DEBUG _ vy
    #for name in kw[1:]:
    for name in ['bot_2', 'bot_3']:
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
    v = 0.10 #
    w = 0.00 #

    motion = Twist()
    
    WL = 15
    RATE = 2

    ## Set up the execution frequency
    rate = rospy.Rate(RATE) # 10hz
    


    ## One time declaration of walls (stationary obstacles)

    B1 = [0, WL, pi/4.0, pi/2.0, (WL/2.0-SizeBot), -(WL/2.0-SizeBot)]
    
    B2 = [0, WL, 3*pi/4.0, pi, (WL/2.0-SizeBot), (WL/2.0-SizeBot)]
    
    B3 = [0, WL, -3*pi/4.0, -pi/2.0, -(WL/2.0-SizeBot), (WL/2.0-SizeBot)]
    
    B4 = [ 0, WL, -pi/4.0, 0.0, -(WL/2.0-SizeBot), -(WL/2.0-SizeBot)]
    

    ## disThreshold defines the distance the bot would travel in Ts seconds with Vm velocity
    ## using Vm is the maximum possible velocity : pessimistic approach
    disThreshold = Vm * Ts

    ### 
    print("One time setup done. Entering while loop:")
    while not rospy.is_shutdown():
        print("")
        ################################################################################ CA with obstacles (static + dynamic)
        ############################################
        ### Define the full sector for the bot
        ############################################

        x1 = pose_b1[0]
        y1 = pose_b1[1]
        theta1 = pose_b1[2]


        ###### Forward sector
        ## The function atan(sin x, cos x) transformed angles remain within -pi to +pi
        start_angle1 = atan2( sin(theta1- Wm*Ts), cos(theta1- Wm*Ts)) 
        end_angle1 = atan2( sin(theta1+ Wm*Ts), cos(theta1+ Wm*Ts)) 

        ###### Backward sector
        start_angle2 = atan2( sin(theta1+pi- Wm*Ts), cos(theta1+pi- Wm*Ts)) 
        end_angle2 = atan2( sin(theta1+pi+ Wm*Ts), cos(theta1+pi+ Wm*Ts)) 

        ## Formulate forward sector
        Af = [  0, Vm*Ts, start_angle1, end_angle1, x1, y1] 

        ## Formulate backward sector
        Ab = [ 0, Vm*Ts, start_angle2, end_angle2, x1, y1] 


        #####
        #    Which bots are in range (nearby)
        
        b_nearby = [n.getPose2() if dist(n.getPose2(),pose_b1)<SAFEDIST else None for n in neighbors]
        
        ## print "{} Robots inside the range {} \n {}".format(len(b_nearby), SAFEDIST, b_nearby)

        B_BOTS = []
        for bi in b_nearby:
            if bi is None:
                continue
            xi = bi[0]
            yi = bi[1]
            thetai = bi[2]

            
            ## Formulate the other robot's interval
            start_anglei = atan2( sin(thetai- Wm*Ts), cos(thetai- Wm*Ts)) 
            end_anglei = atan2( sin(thetai+ Wm*Ts), cos(thetai+ Wm*Ts))

            # xo = xi #((SizeBot)*(np.cos(start_anglei) + np.cos(end_anglei))/(np.sin(start_anglei - end_anglei))) + xi
            # yo = yi #((SizeBot)*(np.sin(start_anglei) + np.sin(end_anglei))/(np.sin(start_anglei - end_anglei))) + yi

            # min_r = ((SizeBot)/(np.sin((end_anglei - start_anglei)/2))) - (SizeBot)
            # max_r = ((SizeBot)/(np.sin((end_anglei - start_anglei)/2))) + (SizeBot) + (Vm)*Ts
            
            # B_BOTS.append([min_r, max_r, start_anglei, end_anglei , xo, yo])
            
            xo = ((SizeBot)*(np.cos(start_anglei) + np.cos(end_anglei))/(np.sin(get_cc_angular_diff(start_anglei, end_anglei)))) + xi
            yo = ((SizeBot)*(np.sin(start_anglei) + np.sin(end_anglei))/(np.sin(get_cc_angular_diff(start_anglei, end_anglei)))) + yi

            min_r = ((SizeBot)/(np.sin((get_cc_angular_diff( start_anglei, end_anglei))/2))) - (SizeBot)
            max_r = ((SizeBot)/(np.sin((get_cc_angular_diff( start_anglei, end_anglei))/2))) + (SizeBot) + (Vm)*Ts

            mid_r = 0.5*(max_r)
            dx = mid_r*cos(thetai)
            dy = mid_r*sin(thetai)
            B_BOTS.append([min_r, max_r, start_anglei, end_anglei , xi-dx, yi-dy])


        ## All obstacles constitutes of the 4 walls and the robots in range currently
        all_obstacles = [B1,B2,B3,B4] + B_BOTS

        s1, c1 = 0,0
        s2, c2 = 0,0

        Cf=[]     ## To store forward sector collisions ??
        Cb=[]     ## backward collisions 

        for this_obs in all_obstacles:
            
            flag1, os1 = checkC(Af, this_obs)       # Check collision with forward sector
            flag2, os2 = checkC(Ab, this_obs)       # Check collision with backward sector

            

            if flag1:                               # If Collision is there => flagX is True.  The sector osX which is now occluded.
                Cf.append(os1)
                
            if flag2:
                Cb.append(os2)
                              
            free_sectors = [[ 0, Vm*Ts, start_angle1, end_angle1]]   #### W T F !!! Why does the free_sectors has the whole sector to begin with ?
            
            ## Cx contains occupied sectors
            ## Lets find the remaining sectors
            if len(Cf):
                for obstacle in Cf:
                    res=[]
                    for sector_x in free_sectors:

                        free_sectors_x = get_s_intersection(sector_x, obstacle)
                        print("$$$$$$$$$^^^^^^^^^^^^ fx {}".format(free_sectors_x))
                        res = res + free_sectors_x
                    free_sectors = res

        v_cmd = 0.0
        w_cmd = 0.0
        
        v_temp = 0
        w_temp = 0

        ## 1 or more forward sector is free
        if len(free_sectors):
            v_temp, w_temp, GoFlag = choose_sect(free_sectors, start_angle1, end_angle1)
            print "con :{}, v_temp :{}, w_temp :{}".format(GoFlag, v_temp, w_temp)
            

            # if GoFlag == 0:
            #     # Sector is totally occluded
            #     print "Look for backward Motion"
            # elif GoFlag == 1:
            #     v_cmd = v
            #     w_cmd = w
            # elif GoFlag == 2:
            #     v_cmd = v_temp
            #     w_cmd = -w_temp
                
            
        
        v_cmd = v_temp
        w_cmd = w_temp
        motion.linear.x = v_cmd
        motion.angular.z =w_cmd
################################################################################## CA ENDS
        
        ## We are not controlling the motion at the moment
        
        #print("v_cmd :{}, w_cmd:{} : {}".format(v_cmd, w_cmd, datetime.datetime.now()))
        #if decisionFLAG:
        #    pub.publish(motion)
        #    decisionFLAG = False
        pub.publish(motion)
        


        ## Visualization data preparation
        ## 'd' is a list containing robot pose
        d = [ [theta1, v_cmd, w_temp, 0], [ Af[2], Af[3], x1, y1, Af[0], Af[1] ], [ Ab[2], Ab[3], x1, y1, Ab[0], Ab[1] ]]


        for c1 in Cf+Cb:
            d.append([ c1[2], c1[3], x1, y1, c1[0], c1[1] ])

        

        X_BOTS = []
        for xb in B_BOTS:
            x = getXbot(xb)
            X_BOTS.append(x)
        d = d+X_BOTS


        d.append([-pi/4.0, 0.0, -(WL/2.0-SizeBot), -(WL/2.0-SizeBot), 0, WL])
        d.append([-3*pi/4.0, -pi/2.0, -(WL/2.0-SizeBot), (WL/2.0-SizeBot), 0, WL])
        d.append([3*pi/4.0, pi, (WL/2.0-SizeBot), (WL/2.0-SizeBot),0, WL])
        d.append([pi/4.0, pi/2.0, (WL/2.0-SizeBot), -(WL/2.0-SizeBot), 0, WL])

        msg = json.dumps(d)
        pub2.publish(msg)
        rate.sleep()

def getXbot(xb):
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
        # with open(fn, 'w') as f:
        #     for pt in traj:
        #         f.write(str(pt[1]))
        #         f.write(',')
        #         f.write(str(pt[0]))
        #         f.write('\r\n')
        # print('log written to disk {}. Done.'.format(fn))
        print "Bye"
