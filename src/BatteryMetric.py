#!/usr/bin/env python3
# NOTE! the input distribution should be a continuous distribution
# scipy.stats.<distribution>
import rospy
from inspection_planner_msgs.msg import Viewpoint, ViewpointList
from underwater_inspection.msg import ViewpointInfo, MultiViewpointInfo
from underwater_inspection.srv import DepleteBattery, BatteryUsage


class Battery():
	# constructor
	def __init__(self, battery, distribution):
		# the battery here is defined as the max distance the AUV can travel
		# if we start at 50 and move a distance of 2, then the battery level would be 48
		self._battery = battery

		# to simulate noise, we use a scipy random distribution specified on contruction
		self._distribution = distribution

		# self.viewpoint_sel_sub = rospy.Subscriber("/rob537/viewpoints_map_info", ViewpointList, self.viewpoint_callback,
        #                                           queue_size=1)
		# self.viewpoint_info_pub = rospy.Publisher("/rob537/viewpoints_info", MultiViewpointInfo, queue_size=10)

	def viewpoint_callback(self, msg):
		viewpoint_info = MultiViewpointInfo()
		viewpoint_info.vp_info = []
		for viewpoint in msg.viewpoints:
			vp = ViewpointInfo()
			vp.x = viewpoint.x
			vp.y = viewpoint.y
			vp.z = viewpoint.z
			vp.yaw = viewpoint.yaw
			vp.cost = viewpoint.cost
			vp.reward = viewpoint.reward
			vp.battery = self._battery - viewpoint.cost
			
			viewpoint_info.vp_info.append(vp)
		self.viewpoint_info_pub.publish(viewpoint_info)
        
	# # getters/setters
	@property
	def battery(self):
		return self._battery

	# member methods
	def depleteBattery(self, req):
		# depletes the battery by the distance
		# checking if we have a random distribution
		if self._distribution is None:
			self._battery -= req.distance
		else:
			self._battery -= req.distance * self._distribution.rvs()
		return True, "Battery depleted!"

	def BatteryUsage(self, req):
		if req.flag:
			viewpoint_info = MultiViewpointInfo()
			viewpoint_info.vp_info = []			
			for viewpoint in req.vps_map.viewpoints:
				vp = ViewpointInfo()
				vp.x = viewpoint.x
				vp.y = viewpoint.y
				vp.z = viewpoint.z
				vp.yaw = viewpoint.yaw
				vp.cost = viewpoint.cost
				vp.reward = viewpoint.reward
				vp.battery = self._battery - viewpoint.cost	
				viewpoint_info.vp_info.append(vp)
			return viewpoint_info, True, "Sent battery info!"


if __name__ == '__main__':
    rospy.init_node('battery_node', anonymous=True)
    b = Battery(100.0, None)
    db = rospy.Service('underwater_inspection/deplete_battery', DepleteBattery, b.depleteBattery)
    bu = rospy.Service('underwater_inspection/battery_usage', BatteryUsage, b.BatteryUsage)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down the node")