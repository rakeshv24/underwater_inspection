#!/usr/bin/env python3
from operator import index
import rospy
import numpy as np
import ParetoFronts
import tf
from geometry_msgs.msg import PoseArray, Pose
from underwater_inspection.msg import ViewpointInfo, MultiViewpointInfo
from underwater_inspection.srv import DepleteBattery


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
        self.viewpoint_info_sub = rospy.Subscriber("/rob537/viewpoints_info", MultiViewpointInfo, self.viewpoint_callback, queue_size=1)
        # self.selected_viewpoint_pub = rospy.Publisher("/rob537/viewpoint_selected", ViewpointInfo, queue_size=10)
        self.selected_viewpoint_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=1)
        
    def viewpoint_callback(self, msg):
        points = []
        for viewpoint in msg.vp_info:
            rov_properties = [viewpoint.battery, viewpoint.cost, viewpoint.reward]
            points.append(rov_properties)
        
        objectiveValues = np.array(points)
        fitnessValues = ParetoFronts.calcSPFitnessValues(objectiveValues)
        fitnessValues = fitnessValues.tolist()     
        idx = fitnessValues.index(min(fitnessValues))
        vp_selected = msg.vp_info[idx]
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
        
        waypoint.poses.append(selected_point)

        self.selected_viewpoint_pub.publish(waypoint)
        deplete_battery_service(vp_selected.cost)
        # self.selected_viewpoint_pub.publish(vp_selected)


if __name__ == '__main__':
    rospy.init_node('viewpoint_selection_node', anonymous=True)
    vs = ViewpointSelector()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down the node")