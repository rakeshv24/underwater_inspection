#!/usr/bin/env python3

import rospy
import GenerateClouds
import SliceCloud
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import tf
from geometry_msgs.msg import Pose, Point, PoseArray
import math

class ViewpointGenerator:
    def __init__(self):
        self.rob_pose = None
        self.debug_markers = Marker()
        self.marker_pub = rospy.Publisher("/rob537/viewpoints_generated_debug_markers", MarkerArray, queue_size=10)
        self.viewpoint_pub = rospy.Publisher("/rob537/viewpoints_generated", PoseArray, queue_size=10)
        self.viewpoint_sel_pub = rospy.Publisher("/rob537/viewpoints_selected", PoseArray, queue_size=10)
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
        
        viewpoints = GenerateClouds.generateUniformCloud(self.rob_pose[0], self.rob_pose[1], self.rob_pose[2], 5, numPoints=1000)
        # viewpoints = GenerateClouds.generateRandomCloud(self.rob_pose[0], self.rob_pose[1], self.rob_pose[2], 5, numPoints=1000)
        sectionPoints = SliceCloud.sliceSphericalCap(viewpoints, self.rob_pose[0], self.rob_pose[1], self.rob_pose[2], 5, euler_angles[1], euler_angles[2], 3)
        

        self.marker_idx = 0
        self.debug_markers = MarkerArray()
        self.vps = PoseArray()
        self.vps1 = PoseArray()
        self.vps.header.frame_id = 'world'    
        self.vps1.header.frame_id = 'world'    
        for v in viewpoints:
            vx = v[0]
            vy = v[1]
            vz = v[2]
            viewpoint_orientation = tf.transformations.quaternion_from_euler(0, v[3], v[4])
            
            # marker = Marker()
            # marker.header.frame_id = 'world'
            # marker.header.stamp = rospy.Time.now()
            # marker.id = self.marker_idx
            # marker.type = Marker.ARROW
            # marker.action = Marker.ADD
            # marker.pose.orientation.x = viewpoint_orientation[0]
            # marker.pose.orientation.y = viewpoint_orientation[1]
            # marker.pose.orientation.z = viewpoint_orientation[2]
            # marker.pose.orientation.w = viewpoint_orientation[3]
            # marker.color.r = 0.0
            # marker.color.g = 1.0
            # marker.color.b = 1.0
            # marker.color.a = 1.0
            # marker.scale.x = 0.1
            # marker.scale.y = 0.1
            # marker.scale.z = 0.1
            # marker.points = [Point(vx, vy, vz), Point(vx + 0.5, vy, vz)]
            # self.debug_markers.markers.append(marker)
            # self.marker_idx += 1 
            # self.debug_markers.points.append(v)
            
            p = Pose()
            p.position.x = vx
            p.position.y = vy
            p.position.z = vz
            p.orientation.x = viewpoint_orientation[0]
            p.orientation.y = viewpoint_orientation[1]
            p.orientation.z = viewpoint_orientation[2]
            p.orientation.w = viewpoint_orientation[3]
            self.vps.poses.append(p)
        
        for s in sectionPoints:
            sx = s[0]
            sy = s[1]
            sz = s[2]
            viewpoint_orientation = tf.transformations.quaternion_from_euler(0, s[3], s[4])
            
            p = Pose()
            p.position.x = sx
            p.position.y = sy
            p.position.z = sz
            p.orientation.x = viewpoint_orientation[0]
            p.orientation.y = viewpoint_orientation[1]
            p.orientation.z = viewpoint_orientation[2]
            p.orientation.w = viewpoint_orientation[3]
            self.vps1.poses.append(p)
            
        # print(viewpoints)
        # print(self.rob_pose)
        # print(sectionPoints.shape)
        # print(self.debug_markers)
        # print("")
        self.viewpoint_pub.publish(self.vps)
        self.viewpoint_sel_pub.publish(self.vps1)
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