import rospy
import tf
import tf2_ros
from nav_msgs.msg import Odometry
from math import pi as PI


class Bot(object):
    def __init__(self, name):
        '''
        Initialize and create a subscriber
        '''
        self.name = name
        self.pose = [0.0, 0.0, 0.0]
        self.quat = [0.0, 0.0, 0.0, 0.0]

        #rospy.init_node("p_{}".format(self.name), anonymous=True)

        #rospy.Subscriber('/{}/odom'.format(name), Odometry, self.callbk)

        self.buf      = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf)
        self.tf_pose = []
        rospy.sleep(1.0)


    # def callbk(self, data):
    #     '''
    #     Callback receives the odometry data
    #     '''
        
    #     # get the pose of the bot
    #     x_pos = data.pose.pose.position.x
    #     y_pos = data.pose.pose.position.y
        
    #     # for calculation of yaw we need to convert quaternion to euler angles
    #     # here we collect the quaterion data
    #     x = data.pose.pose.orientation.x
    #     y = data.pose.pose.orientation.y
    #     z = data.pose.pose.orientation.z
    #     w = data.pose.pose.orientation.w

    #     # using inbuilt method euler_from_quaternion to get euler from quaternion
    #     euler = tf.transformations.euler_from_quaternion((x,y,z,w))

    #     # finally we create a list with x, y, and theta
    #     self.quat[0] = x
    #     self.quat[1] = y
    #     self.quat[2] = z
    #     self.quat[3] = w
    #     pose = [x_pos, y_pos, euler[2]]

        
    #     try:
    #         #transformObject = self.buf.lookup_transform('map', '{}/base_footprint'.format(self.name), rospy.Time(0))
    #         transformObject = self.buf.lookup_transform('map', '{}/base_footprint'.format(self.name), rospy.Time.now(), rospy.Duration(3.0))

    #         trans = transformObject.transform.translation
    #         rot = transformObject.transform.rotation
    #         # print(trans)

    #         euler = tf.transformations.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
            
    #         self.tf_pose = [trans.x, trans.y, euler[2]]
            
    #         angle = euler[2]+pose[2]
    #         if (angle > 2*PI):
    #             angle = angle%2*PI
    #         elif (angle<0):
    #             angle = angle + 2*PI
    #         #self.pose = [trans.x+pose[0], trans.y+pose[1], angle]
    #         self.pose = [trans.x, trans.y, euler[2]]

    #         # print("Happy : {}".format(self.pose))

    #     except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException) as e:
    #         print("F exception wpc {}".format(self.name))
    #         print(e)
    
    #     # finally we create a list with x, y, and theta
        

    def getPose(self):

        try:
            #transformObject = self.buf.lookup_transform('map', '{}/base_footprint'.format(self.name), rospy.Time(0))
            transformObject = self.buf.lookup_transform('map', '{}/base_footprint'.format(self.name), rospy.Time.now(), rospy.Duration(3.0))

            trans = transformObject.transform.translation
            rot = transformObject.transform.rotation
            # print(trans)

            euler = tf.transformations.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
            
            #self.tf_pose = [trans.x, trans.y, euler[2]]
            
            # angle = euler[2]+pose[2]
            # if (angle > 2*PI):
            #     angle = angle%2*PI
            # elif (angle<0):
            #     angle = angle + 2*PI
            # self.pose = [trans.x+pose[0], trans.y+pose[1], angle]
            self.pose = [trans.x, trans.y, euler[2]]

            # print("Happy : {}".format(self.pose))

        except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException) as e:
            print("F exception wpc {}".format(self.name))
            print(e)

        return self.pose

    def getQuat(self):
        return self.quat


