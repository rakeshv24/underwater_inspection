#!/usr/bin/env python3
from operator import index
import rospy
import numpy as np
import ParetoFronts
import tf
from geometry_msgs.msg import PoseArray, Pose, Point
from underwater_inspection.msg import ViewpointInfo, MultiViewpointInfo
from underwater_inspection.srv import DepleteBattery
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


def deplete_battery_service(distance):
    rospy.wait_for_service('underwater_inspection/deplete_battery')
    try:
        handle = rospy.ServiceProxy('underwater_inspection/deplete_battery', DepleteBattery)
        resp = handle(distance)
        return resp
    except rospy.ServiceException as e:
        rospy.logerr("Deplete battery service call failed %s", e)
        

class ViewpointSelector:
    def __init__(self):
        self.viewpoint_info_sub = rospy.Subscriber("/rob537/viewpoints_info", MultiViewpointInfo,
                                                   self.viewpoint_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber("/raven/odom", Odometry, self.odom_callback, queue_size=1)
        # self.selected_viewpoint_pub = rospy.Publisher("/rob537/viewpoint_selected", ViewpointInfo, queue_size=10)
        self.selected_viewpoint_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=1)
        self.viz_marker_pub = rospy.Publisher("/waypoints_marker", Marker, queue_size=1)
        self.path_length = 0.0
        self.traversed_path = []
        
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
        
    def viewpoint_callback(self, msg):
        points = []
        for viewpoint in msg.vp_info:
            rov_properties = [viewpoint.battery, self.path_length + viewpoint.cost, viewpoint.reward]
            points.append(rov_properties)
        
        objectiveValues = np.array(points)
        fitnessValues = ParetoFronts.calcSPFitnessValues(objectiveValues)
        fitnessValues = fitnessValues.tolist()     
        idx = fitnessValues.index(min(fitnessValues))
        vp_selected = msg.vp_info[idx]
        self.path_length += vp_selected.cost
        # print(fitnessValues)
        waypoint = PoseArray()
        waypoint.header.frame_id = "world"
        waypoint.header.stamp = rospy.Time.now()
        
        selected_point = Pose()
        selected_point.position.x = vp_selected.x
        selected_point.position.y = vp_selected.y
        selected_point.position.z = vp_selected.z
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, vp_selected.yaw)
        selected_point.orientation.x = q[0]
        selected_point.orientation.y = q[1]
        selected_point.orientation.z = q[2]
        selected_point.orientation.w = q[3]
        
        viz_marker = Point()
        viz_marker.x = vp_selected.x
        viz_marker.y = vp_selected.y
        viz_marker.z = vp_selected.z
        
        if len(self.traversed_path) == 0.0:
            current_location = Point()
            current_location.x = self.rob_pose[0]
            current_location.y = self.rob_pose[1]
            current_location.z = self.rob_pose[2]
            self.traversed_path.append(current_location)
            
        waypoint.poses.append(selected_point)
        self.traversed_path.append(viz_marker)
        self.visualize_path()
        self.selected_viewpoint_pub.publish(waypoint)
        deplete_battery_service(vp_selected.cost)
        print(vp_selected, self.path_length)
        
        # self.selected_viewpoint_pub.publish(vp_selected)

    def visualize_path(self):
        points = Marker()
        line_strip = Marker()
        points.header.frame_id = line_strip.header.frame_id = "world"
        points.header.stamp = line_strip.header.stamp = rospy.Time.now()
        points.ns = line_strip.ns = "points_and_lines"
        points.action = line_strip.action = Marker.ADD
        points.pose.orientation.w = line_strip.pose.orientation.w = 1.0
        
        points.id = 0
        line_strip.id = 1

        points.type = Marker.POINTS
        line_strip.type = Marker.LINE_STRIP

        # POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.05
        points.scale.y = 0.05
        
        # LINE_STRIP markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.05

        # Points are green
        points.color.g = 1.0
        points.color.a = 1.0

        # Line strip is red
        line_strip.color.r = 1.0
        line_strip.color.a = 1.0
        
        for i in range(len(self.traversed_path)):
            points.points.append(self.traversed_path[i])
            line_strip.points.append(self.traversed_path[i])
            
        self.viz_marker_pub.publish(points)
        self.viz_marker_pub.publish(line_strip)


if __name__ == '__main__':
    rospy.init_node('viewpoint_selection_node', anonymous=True)
    vs = ViewpointSelector()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down the node")
