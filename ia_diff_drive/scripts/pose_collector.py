#!/usr/bin/env python


import rospy
import rosgraph.masterapi
import tf
import json
import message_filters
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class PoseCollector(object):
    '''
	Implements a class to contain/hold the pose data from all other bots.
	
	'''

    def __init__(self):
        # initialize dict (to hold the data)
        self.all_pose = {}
        self.pose = None

    def callback(self, *args):
    	''' This callback will be passed to the publishing node or a message_filters object:w

    	'''
    	self.pose = args
        #print("args : {}".format(args))

        
        
    def update(self, b_id, pose):
        ''' store the data 
        
        '''
        
        # Adding to the dictionary 
        self.pose[b_id] = pose

        ## Uncomment the below lines to print a list of the data being saved

        #for k,v in self.pose.iteritems():
        #    print k,v
    
    
    def get_all_poses(self ):
        '''method to read the stored pose from all the bots

        '''
        args = self.pose
        if not args:
            return None
        for data in args:

            # Read the data header
            id_str = data.header.frame_id
            
            # publishing node ?
            bot_id = id_str.split('/')[0]
            #print("registering bot id {}".format(bot_id))
            
            # get the pose of the bot
            x_pos = data.pose.pose.position.x
            y_pos = data.pose.pose.position.y
            
            # for calculation of yaw we need to convert quaternion to euler angles
            # here we collect the quaterion data
            x = data.pose.pose.orientation.x
            y = data.pose.pose.orientation.y
            z = data.pose.pose.orientation.z
            w = data.pose.pose.orientation.w

            # using inbuilt method euler_from_quaternion to get euler from quaternion
            euler = tf.transformations.euler_from_quaternion((x,y,z,w))

            # finally we create a list with x, y, and theta
            pose = [x_pos, y_pos, euler[2]]
            
            #  save the bot pose 
            self.all_pose[bot_id] = pose

        return self.all_pose       
      
         


def make_subscriptions():
    '''
    Subscribe to all visible odom topics
    Collect data from all the topics into a dictionary
    Publish the collected data
    '''
    

    m = rosgraph.masterapi.Master('/rostopic')
    all_topics_str = m.getPublishedTopics('/')

    # create a data storing object
    pc_object = PoseCollector()
    allHandles=[]
    # get list of all the available topics 
    for [i,j] in all_topics_str:
        # is the topic an odometry topic ? If yes, subscribe to it using a message filter
        if 'Odometry' in j:
            handler = message_filters.Subscriber(i, Odometry)
            # Some visual for what is happening
            print "Subscribing to {}".format(i)
            allHandles.append(handler)

    # create a ApproximateTimeSynchronizer object to generate assimilated data vector
    ts = message_filters.ApproximateTimeSynchronizer(allHandles, 2, 0.1)

    # register callback function
    ts.registerCallback(pc_object.callback)
    
    # Publish data 
    pub = rospy.Publisher('global_pose', String, queue_size=20)
    rospy.init_node('pose_collector', anonymous=True)
    rate = rospy.Rate(10) # 1 Hz



    while not rospy.is_shutdown():
        
        d = pc_object.get_all_poses()
        # #print 'runnig:{}'.format(0)
        # pose = ''
        # for k,v in d.iteritems():
        #     pose = pose + "{}:{}".format(k,v)
        msg = ''
        if d:   
            msg = json.dumps(d)
        else:
            d = "No data fetched yet"
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        make_subscriptions()
    except rospy.ROSInterruptException:
        print "Process Finished"


# Subscribe to the all poses topic
#    rospy.Subscriber('/global_pose', String, callback_Fcn)
