#!/usr/bin/env python3
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Path 
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import numpy as np 
import pandas as pd
from datetime import datetime

#calculate distance between point (x1,y1) and (x2,y2)
def distance(x1,y1,x2,y2):
    return ((x1-x2)**2+(y1-y2)**2)**0.5

#calculate smallest distance between pozyx position and past poses in the SLAM path
def get_smallest_distance(pozyx_pose, slam_path):
    # collect the distances between the pozyx and all of the SLAM pose
    distances = []
    for pose in slam_path.poses:
        distances.append(distance(pose.pose.position.x, pose.pose.position.y, pozyx_pose[0], pozyx_pose[1]))
    # return the minimum distance
    return min(distances)

# caluclatet root mean squared error
def rmse(errors):
    # collect the list of squared error
    se = []
    for e in errors:
        se.append(e*e) 
    # calculate the mean
    mse = np.mean(se)
    # get the root
    return mse**0.5 

# subscribing pozyx coordinates
def pozyx_coordinates(msg):
    global trans_pozyx, delta_heading
    # milimeters to meters conversion:
    trans_pozyx = (msg.x*0.001, msg.y*0.001)
    # transformation rotation:
    trans_pozyx = transformation_rotation(trans_pozyx[0], trans_pozyx[1], delta_heading)

# transform position based on theta
def transformation_rotation(x,y,theta):
    # about the transformation rotation: https://www.khanacademy.org/math/cc-eighth-grade-math/geometric-transformations/rotations-8th/a/rotations-review
    return x*np.cos(theta)-y*np.sin(theta), x*np.sin(theta)+y*np.cos(theta)

# degree to radian
def deg2rad(deg):
    return deg*np.pi/180.0

# radian to degree
def rad2deg(rad):
    return rad*180.0/np.pi

if __name__ == '__main__':
    global trans_pozyx, delta_heading
    trans_pozyx = None # intially None before updated via pozyx_coordinates function
    rospy.init_node('gmapping_benchmark')

    # initialize tf
    listener = tf.TransformListener()

    # get the parameters
    world_frame = rospy.get_param('~world_frame')
    slam_pose_frame = rospy.get_param('~slam_pose_frame')
    pozyx_topic = rospy.get_param('~pozyx_topic')
    delta_heading = deg2rad(rospy.get_param('~delta_heading')) # this is the delta difference between SLAM and pozyx which should be tuned
    #pozyx_pose_frame = rospy.get_param('~pozyx_pose_frame')

    # init the publisher
    current_rmse_pub = rospy.Publisher('current_rmse', Float32,queue_size=1)
    current_distance_error_pub = rospy.Publisher('current_distance_error', Float32,queue_size=1)
    list_distance_error_pub = rospy.Publisher('list_distance_error', Float32MultiArray,queue_size=1)
    slam_path_pub = rospy.Publisher('slam_path', Path,queue_size=1)
    pozyx_path_pub = rospy.Publisher('pozyx_path', Path,queue_size=1)
    pozyx_marker_pub = rospy.Publisher('pozyx_marker', Marker,queue_size=1)
    slam_marker_pub = rospy.Publisher('slam_marker', Marker,queue_size=1)
    # init the subscribers
    rospy.Subscriber(pozyx_topic, Point, pozyx_coordinates)
    
    # init the variables to be published
    current_rmse = Float32()
    current_distance_error = Float32()
    list_distance_error = Float32MultiArray()
    list_distance_error.data = []
    slam_path = Path() 
    slam_path.header.frame_id = world_frame
    slam_path.poses = []
    pozyx_path = Path() 
    pozyx_path.header.frame_id = world_frame
    pozyx_path.poses = []

    pozyx_marker = Marker()
    pozyx_marker.header.frame_id = world_frame
    pozyx_marker.header.stamp = rospy.Time()
    pozyx_marker.ns = "pozyx_marker"
    pozyx_marker.id = 0
    pozyx_marker.type = 2
    pozyx_marker.action = 0
    pozyx_marker.pose.position.z = 0.5
    pozyx_marker.pose.orientation.x = 0.0
    pozyx_marker.pose.orientation.y = 0.0
    pozyx_marker.pose.orientation.z = 0.0
    pozyx_marker.pose.orientation.w = 1.0
    pozyx_marker.scale.x = 0.2
    pozyx_marker.scale.y = 0.2
    pozyx_marker.scale.z = 0.1
    pozyx_marker.color.a = 1.0
    pozyx_marker.color.r = 1.0
    pozyx_marker.color.g = 0.0
    pozyx_marker.color.b = 0.0
    
    slam_marker = Marker()
    slam_marker.header.frame_id = world_frame
    slam_marker.header.stamp = rospy.Time()
    slam_marker.ns = "slam_marker"
    slam_marker.id = 0
    slam_marker.type = 2
    slam_marker.action = 0
    slam_marker.pose.position.z = 0.5
    slam_marker.pose.orientation.x = 0.0
    slam_marker.pose.orientation.y = 0.0
    slam_marker.pose.orientation.z = 0.0
    slam_marker.pose.orientation.w = 1.0
    slam_marker.scale.x = 0.2
    slam_marker.scale.y = 0.2
    slam_marker.scale.z = 0.1
    slam_marker.color.a = 1.0
    slam_marker.color.r = 0.0
    slam_marker.color.g = 1.0
    slam_marker.color.b = 0.0

    rate = rospy.Rate(10.0)

    # waiting for pozyx location
    rospy.loginfo("Waiting for pozyx topics")
    while trans_pozyx == None:
        rate.sleep()
    rospy.loginfo("Got pozyx topics")

    data_to_write = {'SLAM (x)': [],
                'SLAM (y)': [],
                'POZYX (x)': [],
                'POZYX (y)': []}
    now = datetime.now()
    excel_filename = "/home/abc/"+now.strftime("%Y-%m-%d %H-%M-%S")+".xlsx"

    first_iteration = True
    while not rospy.is_shutdown():
        # get slam pose (base_footprint relative to the world frame (pozyx))
        try:
            (trans_slam,rot_slam) = listener.lookupTransform(world_frame, slam_pose_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Cannot transform tf")
            continue

        if first_iteration:
            first_iteration = False 
            # get the delta (location shift) between SLAM and pozyx pose
            delta_x = trans_pozyx[0]-trans_slam[0]
            delta_y = trans_pozyx[1]-trans_slam[1]

        # update the SLAM path and pozyx path
        p1 = PoseStamped()
        p1.pose.position.x = trans_slam[0]
        p1.pose.position.y = trans_slam[1]
        p1.pose.position.z = 0.5
        p1.pose.orientation.x = 0.0
        p1.pose.orientation.y = 0.0
        p1.pose.orientation.z = 0.0
        p1.pose.orientation.w = 1.0
        slam_path.poses.append(p1)
        p2 = PoseStamped()
        p2.pose.position.x = trans_pozyx[0]-delta_x
        p2.pose.position.y = trans_pozyx[1]-delta_y
        p2.pose.position.z = 0.5
        p2.pose.orientation.x = 0.0
        p2.pose.orientation.y = 0.0
        p2.pose.orientation.z = 0.0
        p2.pose.orientation.w = 1.0
        pozyx_path.poses.append(p2)

        # get the errors:
        # use this if the SLAM and pozyx clock is the same:
        #current_distance_error.data = distance(trans_slam[0],trans_slam[1],(trans_pozyx[0]-delta_x),(trans_pozyx[1]-delta_y))
        # use this if pozyx topics late
        current_distance_error.data = get_smallest_distance(((trans_pozyx[0]-delta_x),(trans_pozyx[1]-delta_y)), slam_path)
        list_distance_error.data.append(current_distance_error.data)
        current_rmse = rmse(list_distance_error.data)
        print("RMSE",current_rmse)
        print("current_distance_error",current_distance_error)

        # publish some topics:
        current_distance_error_pub.publish(current_distance_error)
        current_rmse_pub.publish(current_rmse)
        list_distance_error_pub.publish(list_distance_error)
        slam_path_pub.publish(slam_path)
        pozyx_path_pub.publish(pozyx_path)

        pozyx_marker.pose.position.x = trans_pozyx[0]-delta_x
        pozyx_marker.pose.position.y = trans_pozyx[1]-delta_y
        pozyx_marker_pub.publish(pozyx_marker)

        slam_marker.pose.position.x = trans_slam[0]
        slam_marker.pose.position.y = trans_slam[1]
        slam_marker_pub.publish(slam_marker)

        data_to_write['SLAM (x)'].append(trans_slam[0])
        data_to_write['SLAM (y)'].append(trans_slam[1])
        data_to_write['POZYX (x)'].append(trans_pozyx[0]-delta_x)
        data_to_write['POZYX (y)'].append(trans_pozyx[1]-delta_y)
        df = pd.DataFrame(data_to_write)

        writer = pd.ExcelWriter(excel_filename, engine='xlsxwriter')

        df.to_excel(writer, sheet_name='Sheet1', index=False)

        writer.save()

        rate.sleep()
