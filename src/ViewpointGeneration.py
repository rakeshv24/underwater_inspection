#!/usr/bin/env python3

import rospy
import GenerateClouds
import SliceCloud
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import tf
from geometry_msgs.msg import Pose, Point, PoseArray
from inspection_planner_msgs.msg import Viewpoint, ViewpointList
import math

class ViewpointGenerator:
    def __init__(self):
        self.rob_pose = None
        self.debug_markers = Marker()
        self.marker_pub = rospy.Publisher("/rob537/viewpoints_generated_debug_markers", MarkerArray, queue_size=10)
        self.viewpoint_pub = rospy.Publisher("/rob537/viewpoints_generated", PoseArray, queue_size=10)
        self.viewpoint_sel_pub = rospy.Publisher("/rob537/viewpoints_spherical_cap", ViewpointList, queue_size=10)
        self.image_sub = rospy.Subscriber("/raven/odom", Odometry, self.odom_callback, queue_size=1)
    
    def odom_callback(self, odom):
        euler_angles = tf.transformations.euler_from_quaternion((odom.pose.pose.orientation.x, 
                                                                 odom.pose.pose.orientation.y, 
                                                                 odom.pose.pose.orientation.z,
                                                                 odom.pose.pose.orientation.w))
        self.rob_pose = [odom.pose.pose.position.x,
                         odom.pose.pose.position.y,
                         odom.pose.pose.position.z,
                         euler_angles[0],
                         euler_angles[1],
                         euler_angles[2]
                         ]
        
        viewpoints = GenerateClouds.generateUniformCloud(self.rob_pose[0], self.rob_pose[1], self.rob_pose[2], 2, numPoints=100)
        # viewpoints = GenerateClouds.generateRandomCloud(self.rob_pose[0], self.rob_pose[1], self.rob_pose[2], 5, numPoints=1000)
        sectionPoints = SliceCloud.sliceSphericalCap(viewpoints, self.rob_pose[0], self.rob_pose[1], self.rob_pose[2], 2, euler_angles[1], euler_angles[2], 1)
        

        self.marker_idx = 0
        self.debug_markers = MarkerArray()
        self.vps = PoseArray()
        # self.selected_vps = PoseArray()
        self.selected_vps = ViewpointList()
        self.vps.header.frame_id = 'world'    
        self.selected_vps.header.frame_id = 'world'
        
        for v in viewpoints:
            vx = v[0]
            vy = v[1]
            vz = v[2]
            roll = 0.0
            pitch = -v[3]
            yaw = v[4]
            viewpoint_orientation = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            
            p = Pose()
            p.position.x = vx
            p.position.y = vy
            p.position.z = vz
            p.orientation.x = viewpoint_orientation[0]
            p.orientation.y = viewpoint_orientation[1]
            p.orientation.z = viewpoint_orientation[2]
            p.orientation.w = viewpoint_orientation[3]
            
            # Conversion from euler angles to quaternions
            
            # cy = math.cos(yaw * 0.5)
            # sy = math.sin(yaw * 0.5)
            # cp = math.cos(pitch * 0.5)
            # sp = math.sin(pitch * 0.5)
            # cr = math.cos(roll * 0.5)
            # sr = math.sin(roll * 0.5)
            # p.orientation.x = sr * cp * cy - cr * sp * sy
            # p.orientation.y = cr * sp * cy + sr * cp * sy
            # p.orientation.z = cr * cp * sy - sr * sp * cy
            # p.orientation.w = cr * cp * cy + sr * sp * sy
            
            self.vps.poses.append(p)
        
        for s in sectionPoints:
            sx = s[0]
            sy = s[1]
            sz = s[2]
            roll = 0.0
            pitch = -s[3]
            yaw = s[4]
            viewpoint_orientation = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            
            # p = Pose()
            # p.position.x = sx
            # p.position.y = sy
            # p.position.z = sz
            # p.orientation.x = viewpoint_orientation[0]
            # p.orientation.y = viewpoint_orientation[1]
            # p.orientation.z = viewpoint_orientation[2]
            # p.orientation.w = viewpoint_orientation[3]
            
            q = Viewpoint()
            q.x = sx
            q.y = sy
            q.z = sz
            q.yaw = yaw
            self.selected_vps.viewpoints.append(q)
            
        self.viewpoint_pub.publish(self.vps)
        self.viewpoint_sel_pub.publish(self.selected_vps)
        self.marker_publisher(self.debug_markers)
    
    def marker_publisher(self, viewpoint_markers):
        self.marker_pub.publish(viewpoint_markers)
    

if __name__ == '__main__':
    rospy.init_node('viewpoint_generator', anonymous=True)
    vg = ViewpointGenerator()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down the node")