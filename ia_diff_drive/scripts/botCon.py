#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

myvar = 0.9

class PoseCollector(object):
    '''
    Collects the pose information coming from the message_filter callback.
    The pose of the bots comes in a dictionary type.
    The method callback is to be supplied to the subscriber of the `global_pose` node.

    '''
    def __init__(self, name):
        self.name = name
        self.allPoses = {}
        self.pose = []
        
    def callback(self, data):
        if data.data:
			exec('self.allPoses={}'.format(data.data))
        #print('self.allPoses={}'.format(data.data))
        #print("allPoses : {}".format(self.allPoses))
        

    def getAllPose(self):
        return self.allPoses


class ObstacleAvoid(object):
    def __init__(self, pa):
        self.pa = pa
    
    def detect_obstacle(self, ):
        pass

def dist(p0, p1):
    x1 = p0[0] 
    x2 = p1[0]
    y1 = p0[1] 
    y2 = p1[1]
    return np.sqrt((x1-x2)**2 + (y1-y2)**2)
    

def findNeighbors(allB, thisB, tH):
    '''
    allB : all bots data
    '''
    res=[]
    dis=[]
    if len(allB)>0:
        mypose = allB[thisB]
        
        for key, value in allB.iteritems():
            if not key==thisB:
                ddd = dist(mypose, value)
                if ddd < tH:
                    res.append(key)
                    dis.append(ddd)
    return res, dis
            
    

def controller(bot_name):
    PO = PoseCollector(bot_name)

    rospy.init_node('{}_con'.format(bot_name), anonymous=True)
    rospy.Subscriber('/global_pose', String, PO.callback)
    #pub = rospy.Publisher('/{}/mobile_base/commands/velocity'.format(bot_name), Twist, queue_size=10)
    pub = rospy.Publisher('/{}/cmd_vel'.format(bot_name), Twist, queue_size=10)
    ####  Settings start here
    v_max = 0.15
    v_min = 0.00

    
    w_max = 0.3
    
    size_bot = 0.23
    
    ts = 3.0

    v =0.08 #np.random.uniform(v_min, v_max)
    w = 0.0

    motion = Twist()
    ####  Settings end
    rate = rospy.Rate(20) # 10hz
    lopcount = 0
    coll_FLAG = False
    while not rospy.is_shutdown():
        #print 't=',rospy.get_time()
        #print "cur_pose : {}".format(PO.getAllPose())
        lopcount += 1 
        
        motion.linear.x = v
        
        #w = np.random.uniform(-w_max/2.0, w_max/2.0)
        
        motion.angular.z = w
        ####
        # Lets find the neighbor
        
        # Reasoning of threshold calculation
        disThreshold = v_max * ts

        ## All bot poses are inside aBots
        aBots =  PO.getAllPose()
        
        if len(aBots)>0:
            x1, y1, theta1 = aBots[bot_name]
        else:
            x1 =0
            y1 =0
            theta1=0
        # Nearby robots within the threshold circle 
        bots_nearby, bot_distances = findNeighbors(aBots, bot_name, 4*(disThreshold + size_bot))
        

        start_angle1 = np.arctan2(np.sin(theta1- w_max*ts), np.cos(theta1- w_max*ts)) 
        end_angle1 = np.arctan2(np.sin(theta1+ w_max*ts), np.cos(theta1+ w_max*ts)) 
        obstacle_flag = False
        occupied_sects=[]
        if len(bots_nearby):
            #print "\nALERT : {} is very close to {} d={}\n".format(bot_name, bots_nearby, bot_distances)
             
            

            for obs_bot in bots_nearby:

                x2,y2,theta2 = aBots[obs_bot]
                #### HERE we have to take Omega_max*0.5 
                start_angle2 = np.arctan2(np.sin(theta2- w_max*ts), np.cos(theta2- w_max*ts))
                end_angle2 = np.arctan2(np.sin(theta2+ w_max*ts), np.cos(theta2+ w_max*ts)) 
                if end_angle2 < start_angle2:
                    end_angle2_temp = end_angle2 + 2*math.pi
                else:
                    end_angle2_temp = end_angle2
                    
                x_o = ((2*size_bot)*(np.cos(start_angle2) + np.cos(end_angle2))/(np.sin(start_angle2 - end_angle2)))+ x2;
                y_o = ((2*size_bot)*(np.sin(start_angle2) + np.sin(end_angle2))/(np.sin(start_angle2 - end_angle2)))+ y2;
                min_r = ((2*size_bot)/(np.sin((end_angle2_temp - start_angle2)/2))) - (2*size_bot);
                max_r = ((2*size_bot)/(np.sin((end_angle2_temp - start_angle2)/2))) + (2*size_bot) + (v_max)*ts;


                
                #status = collision_detection(0, v_max*ts, start_angle1, end_angle1, x1+size_bot*np.cos(theta1), y1+size_bot*np.sin(theta1), min_r, max_r, start_angle2, end_angle2, x_o, y_o)  
                status = collision_detection(0, v_max*ts, start_angle1, end_angle1, x1, y1, min_r, max_r, start_angle2, end_angle2, x_o, y_o)  

                if status:
                    ## get free sector
                    coll_FLAG = True
                    print "collision detected with :{}".format(obs_bot)
                    fs_current = collision_avoidance(0, v_max*ts, start_angle1, end_angle1, x1, y1, min_r, max_r, start_angle2, end_angle2, x_o, y_o)  
                    occupied_sects.append(fs_current)

        ## Wall free sector calculation
        if collision_detection(0, v_max*ts, start_angle1, end_angle1, x1, y1, 0, 15, math.pi/4.0, math.pi/2.0, (7-size_bot), -(7-size_bot)):
            coll_FLAG = True
            fs_current = collision_avoidance(0, v_max*ts, start_angle1, end_angle1, x1, y1, 0, 15, math.pi/4.0, math.pi/2.0, (7-size_bot), -(7-size_bot))  
            occupied_sects.append(fs_current)
            print "Collision detected with wall 1"

        if collision_detection(0, v_max*ts, start_angle1, end_angle1, x1, y1, 0, 15, 3*math.pi/4.0, math.pi, (7-size_bot), (7-size_bot)):
            coll_FLAG = True
            fs_current = collision_avoidance(0, v_max*ts, start_angle1, end_angle1, x1, y1, 0, 15, 3*math.pi/4.0, math.pi, (7-size_bot), (7-size_bot))
            occupied_sects.append(fs_current)
            print "Collision detected with wall 2"

        if collision_detection(0, v_max*ts, start_angle1, end_angle1, x1, y1, 0, 15, -3*math.pi/4.0, -math.pi/2.0, -(7-size_bot), (7-size_bot)):
            coll_FLAG = True
            fs_current = collision_avoidance(0, v_max*ts, start_angle1, end_angle1, x1, y1, 0, 15, -3*math.pi/4.0, -math.pi/2.0, -(7-size_bot), (7-size_bot))
            occupied_sects.append(fs_current)
            print "Collision detected with wall 3"

        if collision_detection(0, v_max*ts, start_angle1, end_angle1, x1, y1, 0, 15, -math.pi/4.0,  0, -(7-size_bot), -(7-size_bot)):
            coll_FLAG = True
            fs_current = collision_avoidance(0, v_max*ts, start_angle1, end_angle1, x1, y1, 0, 15, -math.pi/4.0,  0, -(7-size_bot), -(7-size_bot))
            occupied_sects.append(fs_current)
            print "Collision detected with wall 4"

                    #print("Collision with {} ".format(obs_bot))
                    ## Free sector
        
        


        #full_sector = [ 0, v_max*ts, start_angle1, end_angle1]
        if coll_FLAG:
            coll_FLAG = False
            final_free_sectors = [[ 0, v_max*ts, start_angle1, end_angle1]]

            if len(occupied_sects)>0:
                obstacle_flag = True

                ## Find the intersection of all free sectors
                for obstacle in occupied_sects:
                    res=[]
                    for sector_x in final_free_sectors:
                        free_sectors_x = get_s_intersection(sector_x, obstacle)
                        res = res + free_sectors_x
                    final_free_sectors = res

            ## Now we have the final free sector inside the final_free_sectors variable.
            


            print "Number of free sectors for {} :{} \r\n".format(bot_name, len(final_free_sectors))
            print final_free_sectors
            print "\n\r ----"
            ## Issue Immediate control action
            
            ## Calculate the Free sector Area
            areas = []

            for fs in final_free_sectors:
                end = fs[3]
                start  = fs[2]

                if end < start:
                    end = end + math.pi*2
                
                theta = end-start 

                area = ((fs[1] - fs[0])**2)*theta*0.5
                areas.append(area)

            # Free sector with max area 
            ar = max(areas)
            i = areas.index(ar)

            # Choose the velocity as per the length of Free sector with the maximum area
            vv = final_free_sectors[i][1]/ts
            #vv = final_free_sectors[i][1]
            
            
            # Choose the direction that offers more space
            d1 = abs(theta1 - final_free_sectors[i][2])
            d2 = abs(theta1 - final_free_sectors[i][3])

            if d1>d2:
                ww = (final_free_sectors[i][2] - theta1 )/ts
            else:
                ww = (final_free_sectors[i][3] - theta1 )/ts
            
            ## This will constraint omega between -pi to +pi
            ww = np.arctan2(np.sin(ww),np.cos(ww))
            
            ## This is to saturate the omega
            if abs(ww) > w_max:
                ww = np.sign(ww)*w_max
            if abs(vv) > v_max:
                vv = np.sign(vv)*v_max
            print "{}: Choosing velocity as v:{} w:{}".format(bot_name, vv, ww)

            ## If we had encountered an collision then we need to publish correction command.
            if (obstacle_flag):

                motion.linear.x = vv
                motion.angular.z = ww
                pub.publish(motion)
                lopcount = 1
                print "w:",ww
                print "v:",vv

        ####
        #if lopcount % 25 == 0:
        pub.publish(motion)
        rate.sleep()

# def get_
#     x_Gbot1_obs1 = ((size_bot1+size_bot2)*(cosd(Ibot1_obs1_min_theta) + cosd(Ibot1_obs1_max_theta))/(sind(Ibot1_obs1_min_theta - Ibot1_obs1_max_theta)))+ Ibot1_obs1_x;
#     y_Gbot1_obs1 = ((size_bot1+size_bot2)*(sind(Ibot1_obs1_min_theta) + sind(Ibot1_obs1_max_theta))/(sind(Ibot1_obs1_min_theta - Ibot1_obs1_max_theta)))+ Ibot1_obs1_y;
#     Ibot1_obs1_min_r = ((size_bot1+size_bot2)/(sind((Ibot1_obs1_max_theta1 - Ibot1_obs1_min_theta)/2))) - (size_bot1+size_bot2);
#     Ibot1_obs1_max_r = ((size_bot1+size_bot2)/(sind((Ibot1_obs1_max_theta1 - Ibot1_obs1_min_theta)/2))) + (size_bot1+size_bot2) + (v_bot2_max)*n1_tsamp*Tsamp;

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

# def free_sectors(I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y,I_robot_obs):
#     stop_flag = 0
#     k=0
#     N=10
    
#     I_obs_min_theta = I_robot_obs[2]
#     I_obs_max_theta = I_robot_obs[3]
#     I_obs_min_r = I_robot_obs[0]
#     I_obs_max_r = I_robot_obs[1]

#     if(I_obs_min_theta == I1_min_theta and I_obs_max_theta < I1_max_theta and I_obs_min_r == I1_min_r and I_obs_max_r < I1_max_r):
#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I_obs_max_theta
#         I1_free_min_r = I_obs_max_r
#         I1_free_max_r = I1_max_r

#         I2_free_min_theta = I_obs_max_theta
#         I2_free_max_theta = I1_max_theta
#         I2_free_min_r = I1_min_r
#         I2_free_max_r = I1_max_r

#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta]
#         #I_robot_free(2,:) = [I2_free_min_r I2_free_max_r I2_free_min_theta I2_free_max_theta]
#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[I2_free_min_r, I2_free_max_r, I2_free_min_theta, I2_free_max_theta],[0, 0, 0, 0],[0, 0, 0, 0]]
    
#     elif(I_obs_min_theta == I1_min_theta and I_obs_max_theta < I1_max_theta and I_obs_min_r > I1_min_r and I_obs_max_r < I1_max_r):
#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I_obs_max_theta
#         I1_free_min_r = I1_min_r
#         I1_free_max_r = I_obs_min_r

#         I2_free_min_theta = I1_min_theta
#         I2_free_max_theta = I_obs_max_theta
#         I2_free_min_r = I_obs_max_r
#         I2_free_max_r = I1_max_r

#         I3_free_min_theta = I_obs_max_theta
#         I3_free_max_theta = I1_max_theta
#         I3_free_min_r = I1_min_r
#         I3_free_max_r = I1_max_r

#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#         #I_robot_free(2,:) = [I2_free_min_r I2_free_max_r I2_free_min_theta I2_free_max_theta];
#         #I_robot_free(3,:) = [I3_free_min_r I3_free_max_r I3_free_min_theta I3_free_max_theta];
#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[I2_free_min_r, I2_free_max_r, I2_free_min_theta, I2_free_max_theta],[I3_free_min_r, I3_free_max_r, I3_free_min_theta, I3_free_max_theta],[0, 0, 0, 0]] 
#     elif(I_obs_min_theta == I1_min_theta and I_obs_max_theta < I1_max_theta and I_obs_min_r > I1_min_r and I_obs_max_r == I1_max_r):
#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I_obs_max_theta
#         I1_free_min_r = I1_min_r
#         I1_free_max_r = I_obs_min_r

#         I2_free_min_theta = I_obs_max_theta
#         I2_free_max_theta = I1_max_theta
#         I2_free_min_r = I1_min_r
#         I2_free_max_r = I1_max_r

#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#         #I_robot_free(2,:) = [I2_free_min_r I2_free_max_r I2_free_min_theta I2_free_max_theta];
#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[I2_free_min_r, I2_free_max_r, I2_free_min_theta, I2_free_max_theta],[0, 0, 0, 0],[0, 0, 0, 0]]

#     elif(I_obs_min_theta > I1_min_theta and I_obs_max_theta < I1_max_theta and I_obs_min_r > I1_min_r and I_obs_max_r == I1_max_r):
#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I_obs_min_theta
#         I1_free_min_r = I1_min_r
#         I1_free_max_r = I1_max_r

#         I2_free_min_theta = I_obs_min_theta
#         I2_free_max_theta = I_obs_max_theta
#         I2_free_min_r = I1_min_r
#         I2_free_max_r = I_obs_min_r

#         I3_free_min_theta = I_obs_max_theta
#         I3_free_max_theta = I1_max_theta
#         I3_free_min_r = I1_min_r
#         I3_free_max_r = I1_max_r

#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta], [I2_free_min_r, I2_free_max_r, I2_free_min_theta, I2_free_max_theta],[I3_free_min_r, I3_free_max_r, I3_free_min_theta, I3_free_max_theta],[0, 0, 0, 0]]     
#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#         #I_robot_free(2,:) = [I2_free_min_r I2_free_max_r I2_free_min_theta I2_free_max_theta];
#         #I_robot_free(3,:) = [I3_free_min_r I3_free_max_r I3_free_min_theta I3_free_max_theta];

#     #%     plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');
#     #%     plot_sec(I2_free_min_theta,I2_free_max_theta,I1_x,I1_y,I2_free_min_r,I2_free_max_r,'g');
#     #%     plot_sec(I3_free_min_theta,I3_free_max_theta,I1_x,I1_y,I3_free_min_r,I3_free_max_r,'g');

#     elif(I_obs_min_theta > I1_min_theta and I_obs_max_theta == I1_max_theta and I_obs_min_r > I1_min_r and I_obs_max_r == I1_max_r):
#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I_obs_min_theta
#         I1_free_min_r = I1_min_r
#         I1_free_max_r = I1_max_r

#         I2_free_min_theta = I_obs_min_theta
#         I2_free_max_theta = I1_max_theta
#         I2_free_min_r = I1_min_r
#         I2_free_max_r = I_obs_min_r

#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[I2_free_min_r, I2_free_max_r, I2_free_min_theta, I2_free_max_theta],[0, 0, 0, 0],[0, 0, 0, 0]]
#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#         #I_robot_free(2,:) = [I2_free_min_r I2_free_max_r I2_free_min_theta I2_free_max_theta];

#     #%     plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');
#     #%        plot_sec(I2_free_min_theta,I2_free_max_theta,I1_x,I1_y,I2_free_min_r,I2_free_max_r,'g');


#     elif(I_obs_min_theta > I1_min_theta and I_obs_max_theta == I1_max_theta and I_obs_min_r > I1_min_r and I_obs_max_r < I1_max_r):

#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I_obs_min_theta
#         I1_free_min_r = I1_min_r
#         I1_free_max_r = I1_max_r

#         I2_free_min_theta = I_obs_min_theta
#         I2_free_max_theta = I1_max_theta
#         I2_free_min_r = I_obs_max_r
#         I2_free_max_r = I1_max_r

#         I3_free_min_theta = I_obs_min_theta
#         I3_free_max_theta = I1_max_theta
#         I3_free_min_r = I1_min_r
#         I3_free_max_r = I_obs_min_r

#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[I2_free_min_r, I2_free_max_r, I2_free_min_theta, I2_free_max_theta],[I3_free_min_r, I3_free_max_r, I3_free_min_theta, I3_free_max_theta],[0, 0, 0, 0]]    
#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#         #I_robot_free(2,:) = [I2_free_min_r I2_free_max_r I2_free_min_theta I2_free_max_theta];
#         #I_robot_free(3,:) = [I3_free_min_r I3_free_max_r I3_free_min_theta I3_free_max_theta];

#     #%     plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');
#     #%        plot_sec(I2_free_min_theta,I2_free_max_theta,I1_x,I1_y,I2_free_min_r,I2_free_max_r,'g');
#     #%        plot_sec(I3_free_min_theta,I3_free_max_theta,I1_x,I1_y,I3_free_min_r,I3_free_max_r,'g');

#     elif(I_obs_min_theta > I1_min_theta and I_obs_max_theta == I1_max_theta and I_obs_min_r == I1_min_r and I_obs_max_r < I1_max_r):
#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I_obs_min_theta
#         I1_free_min_r = I1_min_r
#         I1_free_max_r = I1_max_r

#         I2_free_min_theta = I_obs_min_theta
#         I2_free_max_theta = I1_max_theta
#         I2_free_min_r = I_obs_max_r
#         I2_free_max_r = I1_max_r

#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[I2_free_min_r, I2_free_max_r, I2_free_min_theta, I2_free_max_theta],[0, 0, 0, 0],[0, 0, 0, 0]]    
#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#         #I_robot_free(2,:) = [I2_free_min_r I2_free_max_r I2_free_min_theta I2_free_max_theta];

#     #%     plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');
#     #%        plot_sec(I2_free_min_theta,I2_free_max_theta,I1_x,I1_y,I2_free_min_r,I2_free_max_r,'g');


#     elif(I_obs_min_theta > I1_min_theta and I_obs_max_theta < I1_max_theta and I_obs_min_r == I1_min_r and I_obs_max_r < I1_max_r):    

#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I_obs_min_theta
#         I1_free_min_r = I1_min_r
#         I1_free_max_r = I1_max_r

#         I2_free_min_theta = I_obs_min_theta
#         I2_free_max_theta = I_obs_max_theta
#         I2_free_min_r = I_obs_max_r
#         I2_free_max_r = I1_max_r

#         I3_free_min_theta = I_obs_max_theta
#         I3_free_max_theta = I1_max_theta
#         I3_free_min_r = I1_min_r
#         I3_free_max_r = I1_max_r
#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[I2_free_min_r, I2_free_max_r, I2_free_min_theta, I2_free_max_theta],[I3_free_min_r, I3_free_max_r, I3_free_min_theta, I3_free_max_theta],[0, 0, 0, 0,]]
#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#         #I_robot_free(2,:) = [I2_free_min_r I2_free_max_r I2_free_min_theta I2_free_max_theta];
#         #I_robot_free(3,:) = [I3_free_min_r I3_free_max_r I3_free_min_theta I3_free_max_theta];

#     #%     plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');
#     #%     plot_sec(I2_free_min_theta,I2_free_max_theta,I1_x,I1_y,I2_free_min_r,I2_free_max_r,'g');
#     #%     plot_sec(I3_free_min_theta,I3_free_max_theta,I1_x,I1_y,I3_free_min_r,I3_free_max_r,'g');

#     elif(I_obs_min_theta == I1_min_theta and I_obs_max_theta < I1_max_theta and I_obs_min_r == I1_min_r and I_obs_max_r == I1_max_r):    

#         I1_free_min_theta = I_obs_max_theta
#         I1_free_max_theta = I1_max_theta
#         I1_free_min_r = I1_min_r
#         I1_free_max_r = I1_max_r

#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[0, 0 ,0, 0],[0, 0, 0, 0],[0, 0, 0, 0]]    
#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#     #%     plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');


#     elif(I_obs_min_theta > I1_min_theta and I_obs_max_theta == I1_max_theta and I_obs_min_r == I1_min_r and I_obs_max_r == I1_max_r):    

#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I_obs_min_theta
#         I1_free_min_r = I1_min_r
#         I1_free_max_r = I1_max_r

#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0]]
#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#     #%     plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');

#     elif(I_obs_min_theta == I1_min_theta and I_obs_max_theta == I1_max_theta and I_obs_min_r == I1_min_r and I_obs_max_r < I1_max_r):
#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I1_max_theta
#         I1_free_min_r = I_obs_max_r
#         I1_free_max_r = I1_max_r
        
#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0]]

#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#     #%     plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');
#     elif(I_obs_min_theta == I1_min_theta and I_obs_max_theta == I1_max_theta and I_obs_min_r > I1_min_r and I_obs_max_r == I1_max_r):
#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I1_max_theta
#         I1_free_min_r = I1_min_r
#         I1_free_max_r = I_obs_min_r
#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0]]
#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#     #%     plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');
#     elif(I_obs_min_theta > I1_min_theta and I_obs_max_theta < I1_max_theta and I_obs_min_r == I1_min_r and I_obs_max_r == I1_max_r):    

#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I_obs_min_theta
#         I1_free_min_r = I1_min_r
#         I1_free_max_r = I1_max_r

#         I2_free_min_theta = I_obs_max_theta
#         I2_free_max_theta = I1_max_theta
#         I2_free_min_r = I1_min_r
#         I2_free_max_r = I1_max_r
#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[I2_free_min_r, I2_free_max_r, I2_free_min_theta, I2_free_max_theta],[0, 0, 0, 0],[0, 0, 0, 0,]]
#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#         #I_robot_free(2,:) = [I2_free_min_r I2_free_max_r I2_free_min_theta I2_free_max_theta];
#     #%     plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');
#     #%        plot_sec(I2_free_min_theta,I2_free_max_theta,I1_x,I1_y,I2_free_min_r,I2_free_max_r,'g');

#     elif(I_obs_min_theta == I1_min_theta and I_obs_max_theta == I1_max_theta and I_obs_min_r > I1_min_r and I_obs_max_r < I1_max_r):    

#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I1_max_theta
#         I1_free_min_r = I1_min_r
#         I1_free_max_r = I_obs_min_r

#         I2_free_min_theta = I1_min_theta
#         I2_free_max_theta = I1_max_theta
#         I2_free_min_r = I_obs_max_r
#         I2_free_max_r = I1_max_r
#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[I2_free_min_r, I2_free_max_r, I2_free_min_theta, I2_free_max_theta],[0, 0, 0, 0],[0 ,0,0, 0]]
#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#         #I_robot_free(2,:) = [I2_free_min_r I2_free_max_r I2_free_min_theta I2_free_max_theta];
#     #%     plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');
#     #%        plot_sec(I2_free_min_theta,I2_free_max_theta,I1_x,I1_y,I2_free_min_r,I2_free_max_r,'g');

#     elif(I_obs_min_theta > I1_min_theta and I_obs_max_theta < I1_max_theta and I_obs_min_r > I1_min_r and I_obs_max_r < I1_max_r):

#         I1_free_min_theta = I1_min_theta
#         I1_free_max_theta = I_obs_min_theta
#         I1_free_min_r = I1_min_r
#         I1_free_max_r = I1_max_r

#         I2_free_min_theta = I_obs_min_theta
#         I2_free_max_theta = I_obs_max_theta
#         I2_free_min_r = I1_min_r
#         I2_free_max_r = I_obs_min_r

#         I3_free_min_theta = I_obs_min_theta
#         I3_free_max_theta = I_obs_max_theta
#         I3_free_min_r = I_obs_max_r
#         I3_free_max_r = I1_max_r

#         I4_free_min_theta = I_obs_max_theta
#         I4_free_max_theta = I1_max_theta
#         I4_free_min_r = I1_min_r
#         I4_free_max_r = I1_max_r
#         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[I2_free_min_r, I2_free_max_r, I2_free_min_theta, I2_free_max_theta],[I3_free_min_r, I3_free_max_r, I3_free_min_theta, I3_free_max_theta],[I4_free_min_r, I4_free_max_r, I4_free_min_theta, I4_free_max_theta]]
#         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#         #I_robot_free(2,:) = [I2_free_min_r I2_free_max_r I2_free_min_theta I2_free_max_theta];
#         #I_robot_free(3,:) = [I3_free_min_r I3_free_max_r I3_free_min_theta I3_free_max_theta];
#         #I_robot_free(4,:) = [I4_free_min_r I4_free_max_r I4_free_min_theta I4_free_max_theta];
#     #%         plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'b');
#     #%     plot_sec(I2_free_min_theta,I2_free_max_theta,I1_x,I1_y,I2_free_min_r,I2_free_max_r,'b');
#     #%     plot_sec(I3_free_min_theta,I3_free_max_theta,I1_x,I1_y,I3_free_min_r,I3_free_max_r,'b');
#     #%     plot_sec(I4_free_min_theta,I4_free_max_theta,I1_x,I1_y,I4_free_min_r,I4_free_max_r,'b');
#     elif(I_obs_min_theta == I1_min_theta and I_obs_max_theta == I1_max_theta and I_obs_min_r == I1_min_r and I_obs_max_r == I1_max_r):

#         bot_inside_obs = inclusion_interval(I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y)
#         if(bot_inside_obs == 1):
#             I1_free_min_theta = 0
#             I1_free_max_theta = 0
#             I1_free_min_r = 0
#             I1_free_max_r = 0

#             I2_free_min_theta = 0
#             I2_free_max_theta = 0
#             I2_free_min_r = 0
#             I2_free_max_r = 0

#             I3_free_min_theta = 0
#             I3_free_max_theta = 0
#             I3_free_min_r = 0
#             I3_free_max_r = 0
#         else:
#             [I_min_left_theta, I_max_left_theta, I_min_right_theta, I_max_right_theta] = bisect(I1_min_theta,I1_max_theta)
#             while (~stop_flag):  
#                 status_inclu_I1left = inclusion_interval(I1_min_r,I1_max_r,I_min_left_theta,I_max_left_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y)
#                 status_inclu_I1right = inclusion_interval(I1_min_r,I1_max_r,I_min_right_theta,I_max_right_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y)


#                 if (status_inclu_I1left == 0):
#                     [I_min_left_I1left_r, I_max_left_I1left_r, I_min_right_I1left_r, I_max_right_I1left_r] = bisect(I1_min_r,I1_max_r)

#                     while (k < N): #% bisection w.r.t r
#                     #%keyboard
#                         if ((collision_detection(I_min_right_I1left_r,I_max_right_I1left_r,I_min_left_theta,I_max_left_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 0) and (collision_detection(I_min_left_I1left_r,I_max_left_I1left_r,I_min_left_theta,I_max_left_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1)):
#                             [I_min_leftleft, I_max_leftleft, I_min_leftright, I_max_leftright] = bisect(I_min_left_I1left_r,I_max_left_I1left_r)
#                             I_min_left1 = I_min_leftleft
#                             I_max_left1 = I_max_leftleft
#                             I_min_right1 = I_min_leftright
#                             I_max_right1 = I_max_leftright
#                         elif ((collision_detection(I_min_left_I1left_r,I_max_left_I1left_r,I_min_left_theta,I_max_left_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 0) and (collision_detection(I_min_right_I1left_r,I_max_right_I1left_r,I_min_left_theta,I_max_left_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1)):
#                             [I_min_rightleft, I_max_rightleft, I_min_rightright, I_max_rightright] = bisect(I_min_right_I1left_r,I_max_right_I1left_r)    
#                             I_min_left1 = I_min_rightleft
#                             I_max_left1 = I_max_rightleft
#                             I_min_right1 = I_min_rightright
#                             I_max_right1 = I_max_rightright
#                         else:

#                             [I_min_leftleft, I_max_leftleft, I_min_leftright, I_max_leftright] = bisect(I_min_left_I1left_r,I_max_left_I1left_r)
#                             [I_min_rightleft, I_max_rightleft, I_min_rightright, I_max_rightright] = bisect(I_min_right_I1left_r,I_max_right_I1left_r)

#                             if(collision_detection(I_min_leftleft,I_max_leftleft,I_min_left_theta,I_max_left_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1):
#                                 I_min_left1 = I_min_leftleft
#                                 I_max_left1 = I_max_leftleft
#                     #%          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_left1,I_max_left1,'m');
#                             else:
#                                 I_min_left1 = I_min_leftright
#                                 I_max_left1 = I_max_leftright
#                     #%          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_left1,I_max_left1,'y');
                        

#                             if (collision_detection(I_min_rightright,I_max_rightright,I_min_left_theta,I_max_left_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1):
#                                 I_min_right1 = I_min_rightright
#                                 I_max_right1 = I_max_rightright
#                     #%          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_right1,I_max_right1,'b');
#                             else:
#                                 I_min_right1 = I_min_rightleft
#                                 I_max_right1 = I_max_rightleft
#                     #%          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_right1,I_max_right1,'c');
                        


#                         #end
#                         I_min_left_I1left_r = I_min_left1
#                         I_max_left_I1left_r = I_max_left1
#                         I_min_right_I1left_r = I_min_right1
#                         I_max_right_I1left_r = I_max_right1
#                         k = k+1;
#                     #end
#                     I_obs_min_I1left_r = I_min_left_I1left_r
#                     I_obs_max_I1left_r = I_max_right_I1left_r
#                 else:
#                     I_obs_min_I1left_r = I1_min_r
#                     I_obs_max_I1left_r = I1_max_r
#                 #end
#                 k=0
                
#                 if (status_inclu_I1right == 0):
#                     [I_min_left_I1right_r, I_max_left_I1right_r, I_min_right_I1right_r, I_max_right_I1right_r] = bisect(I1_min_r,I1_max_r)

#                     while(k < N): #% bisection w.r.t r
#                     #%keyboard
#                         if ((collision_detection(I_min_right_I1right_r,I_max_right_I1right_r,I_min_right_theta,I_max_right_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 0) and (collision_detection(I_min_left_I1right_r,I_max_left_I1right_r,I_min_right_theta,I_max_right_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1)):
#                             [I_min_leftleft, I_max_leftleft, I_min_leftright, I_max_leftright] = bisect(I_min_left_I1right_r,I_max_left_I1right_r)
#                             I_min_left1 = I_min_leftleft
#                             I_max_left1 = I_max_leftleft
#                             I_min_right1 = I_min_leftright
#                             I_max_right1 = I_max_leftright
#                         elif ((collision_detection(I_min_left_I1right_r,I_max_left_I1right_r,I_min_right_theta,I_max_right_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 0) and (collision_detection(I_min_right_I1right_r,I_max_right_I1right_r,I_min_right_theta,I_max_right_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1)):
#                             [I_min_rightleft, I_max_rightleft, I_min_rightright, I_max_rightright] = bisect(I_min_right_I1right_r,I_max_right_I1right_r)    
#                             I_min_left1 = I_min_rightleft
#                             I_max_left1 = I_max_rightleft
#                             I_min_right1 = I_min_rightright
#                             I_max_right1 = I_max_rightright
#                         else:

#                             [I_min_leftleft, I_max_leftleft, I_min_leftright, I_max_leftright] = bisect(I_min_left_I1right_r,I_max_left_I1right_r)
#                             [I_min_rightleft,I_max_rightleft,I_min_rightright,I_max_rightright] = bisect(I_min_right_I1right_r,I_max_right_I1right_r)

#                             if (collision_detection(I_min_leftleft,I_max_leftleft,I_min_right_theta,I_max_right_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1):
#                                 I_min_left1 = I_min_leftleft
#                                 I_max_left1 = I_max_leftleft
#                     #%          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_left1,I_max_left1,'m');
#                             else:
#                                 I_min_left1 = I_min_leftright
#                                 I_max_left1 = I_max_leftright
#                     #%          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_left1,I_max_left1,'y');
#                             #end

#                             if(collision_detection(I_min_rightright,I_max_rightright,I_min_right_theta,I_max_right_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1):
#                                 I_min_right1 = I_min_rightright
#                                 I_max_right1 = I_max_rightright
#                     #%          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_right1,I_max_right1,'b');
#                             else:
#                                 I_min_right1 = I_min_rightleft
#                                 I_max_right1 = I_max_rightleft
#                     #%          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_right1,I_max_right1,'c');
#                             #end


#                         #end
#                         I_min_left_I1right_r = I_min_left1
#                         I_max_left_I1right_r = I_max_left1
#                         I_min_right_I1right_r = I_min_right1
#                         I_max_right_I1right_r = I_max_right1
#                         k = k+1
#                     #end
#                     I_obs_min_I1right_r = I_min_left_I1right_r
#                     I_obs_max_I1right_r = I_max_right_I1right_r
#                 else:     
#                     I_obs_min_I1right_r = I1_min_r
#                     I_obs_max_I1right_r = I1_max_r
#                 #end
    
#                 if ((I_obs_min_I1left_r == I1_min_r and I_obs_max_I1left_r == I1_max_r)):
#                     stop_flag1 = 0
#                 else:
#                     stop_flag1 = 1
#                     if(I_obs_min_I1left_r > I1_min_r and I_obs_max_I1left_r == I1_max_r):
#                         I1_free_min_theta = I_min_left_theta
#                         I1_free_max_theta = I_max_left_theta
#                         I1_free_min_r = I1_min_r
#                         I1_free_max_r = I_obs_min_I1left_r
#     #%                 plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');
#                         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#                         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[0, 0, 0, 0],[0, 0, 0, 0],[0 ,0, 0, 0]]    
                    
#                     elif (I_obs_min_I1left_r > I1_min_r and I_obs_max_I1left_r < I1_max_r):    
#                         I1_free_min_theta = I_min_left_theta
#                         I1_free_max_theta = I_max_left_theta
#                         I1_free_min_r = I1_min_r
#                         I1_free_max_r = I_obs_min_I1left_r

#                         I2_free_min_theta = I_min_left_theta
#                         I2_free_max_theta = I_max_left_theta
#                         I2_free_min_r = I_obs_max_I1left_r
#                         I2_free_max_r = I1_max_r

#     #%                 plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');
#     #%                 plot_sec(I2_free_min_theta,I2_free_max_theta,I1_x,I1_y,I2_free_min_r,I2_free_max_r,'g');

#                         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#                         #I_robot_free(2,:) = [I2_free_min_r I2_free_max_r I2_free_min_theta I2_free_max_theta];
#                         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[I2_free_min_r, I2_free_max_r, I2_free_min_theta, I2_free_max_theta],[0, 0, 0, 0],[0, 0, 0, 0]]
                    
#                     elif(I_obs_min_I1left_r == I1_min_r and I_obs_max_I1left_r < I1_max_r):   
#                         I1_free_min_theta = I_min_left_theta
#                         I1_free_max_theta = I_max_left_theta
#                         I1_free_min_r = I_obs_max_I1left_r
#                         I1_free_max_r = I1_max_r

#     #%                 plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');

#                         #I_robot_free(1,:) = [I1_free_min_r I1_free_max_r I1_free_min_theta I1_free_max_theta];
#                         return [[I1_free_min_r, I1_free_max_r, I1_free_min_theta, I1_free_max_theta],[0, 0, 0, 0],[0, 0, 0, 0],[0 ,0, 0, 0]]
#                     #end
#                 #end

#                 if (I_obs_min_I1right_r == I1_min_r and I_obs_max_I1right_r == I1_max_r):
#                     stop_flag2 = 0
#                 else:
#                     stop_flag2 = 1

#                     if (I_obs_min_I1right_r > I1_min_r and I_obs_max_I1right_r == I1_max_r):
#                         I3_free_min_theta = I_min_right_theta
#                         I3_free_max_theta = I_max_right_theta
#                         I3_free_min_r = I1_min_r
#                         I3_free_max_r = I_obs_min_I1right_r

#     #%                 plot_sec(I3_free_min_theta,I3_free_max_theta,I1_x,I1_y,I3_free_min_r,I3_free_max_r,'g');

#                         #I_robot_free(3,:) = [I3_free_min_r I3_free_max_r I3_free_min_theta I3_free_max_theta];
#                         return [[0, 0, 0, 0],[0, 0, 0, 0,],[I3_free_min_r, I3_free_max_r, I3_free_min_theta, I3_free_max_theta],[0, 0, 0, 0]]
#                     elif (I_obs_min_I1right_r > I1_min_r and I_obs_max_I1right_r < I1_max_r):    
#                         I3_free_min_theta = I_min_right_theta
#                         I3_free_max_theta = I_max_right_theta
#                         I3_free_min_r = I1_min_r
#                         I3_free_max_r = I_obs_min_I1right_r

#                         I4_free_min_theta = I_min_right_theta
#                         I4_free_max_theta = I_max_right_theta
#                         I4_free_min_r = I_obs_max_I1right_r
#                         I4_free_max_r = I1_max_r

#     #%                 plot_sec(I1_free_min_theta,I1_free_max_theta,I1_x,I1_y,I1_free_min_r,I1_free_max_r,'g');
#     #%                 plot_sec(I2_free_min_theta,I2_free_max_theta,I1_x,I1_y,I2_free_min_r,I2_free_max_r,'g');

#                         #I_robot_free(3,:) = [I3_free_min_r I3_free_max_r I3_free_min_theta I3_free_max_theta];
#                         #I_robot_free(4,:) = [I4_free_min_r I4_free_max_r I4_free_min_theta I4_free_max_theta];
#                         return [[0, 0, 0, 0],[0, 0, 0, 0],[I3_free_min_r, I3_free_max_r, I3_free_min_theta, I3_free_max_theta],[I4_free_min_r, I4_free_max_r, I4_free_min_theta, I4_free_max_theta]]
                    
#                     elif (I_obs_min_I1right_r == I1_min_r and I_obs_max_I1right_r < I1_max_r):   
#                         I3_free_min_theta = I_min_right_theta
#                         I3_free_max_theta = I_max_right_theta
#                         I3_free_min_r = I_obs_max_I1right_r
#                         I3_free_max_r = I1_max_r

#     #%                 plot_sec(I3_free_min_theta,I3_free_max_theta,I1_x,I1_y,I3_free_min_r,I3_free_max_r,'g');

#                         #I_robot_free(3,:) = [I3_free_min_r I3_free_max_r I3_free_min_theta I3_free_max_theta];
#                         return [[0, 0, 0, 0],[0, 0, 0, 0],[I3_free_min_r, I3_free_max_r, I3_free_min_theta, I3_free_max_theta],[0 ,0,0, 0]]    
#                     #end

#                 #end

#                 if(stop_flag1 == 1 or stop_flag2 == 1):
#                     stop_flag = 1
#                 else:
#                     stop_flag = 0
#                 #end
                
#                 if(stop_flag==0):
#                     [I_min_left_theta1, I_max_left_theta1, I_min_right_theta1, I_max_right_theta1] = bisect(I_min_left_theta,I_max_left_theta)
#                     [I_min_left_theta2, I_max_left_theta2, I_min_right_theta2, I_max_right_theta2] = bisect(I_min_right_theta,I_max_right_theta)
#                     I_min_left_theta = I_min_left_theta1
#                     I_max_left_theta = I_max_left_theta1
#                     I_min_right_theta = I_min_right_theta2
#                     I_max_right_theta = I_max_right_theta2
#                     k=0
#                 #end
#             #end
#         #end
#     #end
    










if __name__ == '__main__':
    try:
        #print "$$$^^^^received the commands:{}".format(sys.argv[1])
        controller(sys.argv[1])
        
    except rospy.ROSInterruptException:
        pass
