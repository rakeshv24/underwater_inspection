#!/usr/bin/env python3
import rospy
import GenerateClouds
import SliceCloud
import random
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Pose, Point, PoseArray
from inspection_planner_msgs.msg import Viewpoint, ViewpointList
from underwater_inspection.msg import ViewpointInfo, MultiViewpointInfo
from underwater_inspection.srv import DepleteBattery, BatteryUsage, CarveMap
from visualization_msgs.msg import Marker
import numpy as np
import copy
import ParetoFronts
import time


def battery_usage_service(flag, viewpoints):
    rospy.wait_for_service('underwater_inspection/battery_usage')
    try:
        handle = rospy.ServiceProxy('underwater_inspection/battery_usage', BatteryUsage)
        resp = handle(flag, viewpoints)
        if resp.success == False:
            print("[PEAS][battery_usage_service] Not successful")
            return battery_usage_service(flag, viewpoints)
        else:
            return resp.vps_battery
    except rospy.ServiceException as e:
        rospy.logerr("Battery usage service call failed: %s", e)

        
def deplete_battery_service(distance):
    rospy.wait_for_service('underwater_inspection/deplete_battery')
    try:
        handle = rospy.ServiceProxy('underwater_inspection/deplete_battery', DepleteBattery)
        resp = handle(distance)
        return resp
    except rospy.ServiceException as e:
        rospy.logerr("Deplete battery service call failed: %s", e)


def carve_map_service(flag, viewpoints):
    rospy.wait_for_service('underwater_inspection/carve_map_service')
    try:
        handle = rospy.ServiceProxy('underwater_inspection/carve_map_service', CarveMap)
        resp = handle(flag, viewpoints)
        if resp.success == False:
            print("[PEAS][carve_map_service] Not successful")
            time.sleep(1)
            return carve_map_service(flag, viewpoints)
        else:
            # print(resp.vps_map)
            print("[PEAS][carve_map_service] Successful")
            return resp.vps_map
    except rospy.ServiceException as e:
        rospy.logerr("Map carving service call failed: %s", e)
        
        
class PEAS:
    def __init__(self):
        self.rob_pose = None
        self.generations = 1
        self.initial_population = []
        self.mutated_population = None
        self.crossed_population = None
        self.m_population = []
        self.c_population = []
        self.combined_population = []
        self.selected_population = []
        self.selected_viewpoints = []
        self.pop_size = 5
        self.cap_fitness = []
        # self.avg_cap_fitness = []
        self.mutation_rate = 0.5
        self.sphere_radii = [1.0, 2.0, 3.0]
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        self.path_length = 0.0
        self.traversed_path = []
        self.distance_tolerance = 0.05
        self.yaw_tolerance = 0.005
        self.move_flag = False
        self.angle_range = np.radians(45)
        
        # bop_panel 
        # xmin: -14.8
        # xC: -10
        # xmax: -5.2
        # ymin: 11.05
        # yc: 15
        # ymax: 18.95 
        # zmin: 82.5
        # zC: 88.7  
        # zmax: 94.9
        # Volume = l*b*h = 9.6*7.9*12.4 = 940.416 m^3
        self.bop_panel_origin = np.array([-10, 15, 88.7])
        self.bop_panel_dimensions = np.array([4.8, 3.95, 6.2]) # l/2, b/2, h/2
        self.bop_diagonal = np.linalg.norm(self.bop_panel_dimensions)
        # self.safe_tolerance = self.bop_diagonal + 0.5
        
        # turbine lander
        # xC 20
        # yC 35  
        # zC 94.9
        self.turbine_lander_origin = np.array([12.7, 42.39, 94.9])
        self.safe_tolerance = 2.0
        
        self.battery_threshold = 30.0
        self.battery_remaining = 100.0
        self.mission_start_time = 0.0
        self.mission_end_time = 0.0
        self.mission_duration = 0.0
        self.init_flag = True
        
        self.sphere_viewpoints_pub = rospy.Publisher("/rob537/sphere_viewpoints", ViewpointList, queue_size=10)
        self.generated_viewpoints_pub = rospy.Publisher("/rob537/generated_viewpoints", ViewpointList, queue_size=10)
        self.selected_viewpoints_pub = rospy.Publisher("/rob537/selected_viewpoints", ViewpointList, queue_size=10)
        self.selected_view_pub = rospy.Publisher("/rob537/selected_view", ViewpointList, queue_size=10)
        self.odom_sub = rospy.Subscriber("/raven/odom", Odometry, self.odom_callback, queue_size=1)
        self.best_viewpoint_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=1)
        self.viz_marker_pub = rospy.Publisher("/path_traversed", Marker, queue_size=1)

    def wrap_angle(self, angle):
        return ((angle + np.pi) % (2 * np.pi)) - np.pi

    def odom_callback(self, odom):
        euler_angles = tf.transformations.euler_from_quaternion((odom.pose.pose.orientation.x,
                                                                 odom.pose.pose.orientation.y,
                                                                 odom.pose.pose.orientation.z,
                                                                 odom.pose.pose.orientation.w))
        self.azimuth_angle = euler_angles[2]

        self.rob_pose = [odom.pose.pose.position.x,
                         odom.pose.pose.position.y,
                         odom.pose.pose.position.z,
                         euler_angles[0],
                         euler_angles[1],
                         euler_angles[2]
                         ]
        
        if self.init_flag:
            self.mission_start_time = time.time()
            print("Timer started!")
            self.init_flag = False
        
        if self.battery_remaining > self.battery_threshold:
            if self.move_flag:
                self.wait_until_move()
            else:
                self.generate_init_pop()
                i = 0
                while i < self.generations:
                    self.m_population = []
                    self.c_population = []
                    p_size = 0
                    j = 0
                    self.mutated_population = None
                    self.crossed_population = None
                    self.combined_population = copy.deepcopy(self.initial_population)
                    
                    while p_size <= self.pop_size:
                        if random.random() < self.mutation_rate:
                            self.m_population.append(self.mutate(j))
                            self.mutated_population = np.array(self.m_population)
                            self.combined_population = np.vstack((self.combined_population, self.mutated_population))
                            p_size += 1
                        else:
                            self.crossover()
                            self.crossed_population = np.array(self.c_population)
                            self.combined_population = np.vstack((self.combined_population, self.crossed_population))
                            p_size += 2
                        j += 1
                    
                    self.combined_population = np.unique(self.combined_population, axis=0)

                    self.selected_population = self.viewpoint_selection()
                    if self.selected_population is None:
                        print("No viewpoints were selected")
                        self.angle_range += np.radians(15)
                        self.generate_init_pop()
                    else:
                        self.angle_range = np.radians(45)
                        i += 1
        else:
            rospy.loginfo("[PEAS][odom_callback] Mission accomplished!!")
            self.mission_end_time = time.time()
            self.mission_duration = (self.mission_end_time - self.mission_start_time) / 60.0
            print(self.mission_duration)

    def generate_init_pop(self):
        self.initial_population = []

        for i in self.sphere_radii:
            # Front
            self.initial_population.append([self.altitude_angle, self.azimuth_angle, i, 0.5 * i])
            # Left
            self.initial_population.append([self.altitude_angle, self.wrap_angle(
                self.azimuth_angle - self.angle_range), i, 0.5 * i])
            # Right
            self.initial_population.append([self.altitude_angle, self.wrap_angle(
                self.azimuth_angle + self.angle_range), i, 0.5 * i])
            # Up
            self.initial_population.append(
                [self.wrap_angle(self.altitude_angle - self.angle_range), self.azimuth_angle, i, 0.5 * i])
            # Down
            self.initial_population.append(
                [self.wrap_angle(self.altitude_angle + self.angle_range), self.azimuth_angle, i, 0.5 * i])

        self.initial_population = np.array(self.initial_population)

    def mutate(self, idx):
        r = random.random()
        mutated = copy.deepcopy(self.initial_population[idx, :])
        if r < 0.5:
            mutated[0] = random.uniform(-self.angle_range, self.angle_range) 
        else:
            a1 = self.wrap_angle(mutated[1] - self.angle_range)
            a2 = self.wrap_angle(mutated[1] + self.angle_range)
            min_angle = min(a1, a2)
            max_angle = max(a1, a2)
            mutated[1] = random.uniform(min_angle, max_angle) 
        return mutated                
    
    def crossover(self):
        r = random.random()
        v1 = random.choice(self.initial_population)
        v2 = random.choice(self.initial_population)
        for _ in range(v1.shape[0]):
            if r < 0.5:
                tmp = v1[0:2]
                v1[0:2] = v2[0:2]
                v2[0:2] = tmp
            else:
                rr = random.random()
                if rr < 0.5:
                    tmp = v1[0]
                    v1[0] = v2[0]
                    v2[0] = tmp 
                else:
                    tmp = v1[1]
                    v1[1] = v2[1]
                    v2[1] = tmp     
                    
        self.c_population.append(v1)                
        self.c_population.append(v2)
    
    def remove_infeasible(self, points):
        feasible_points = []
        for i in range(len(points)):
            # Check for point-cylinder intersection
            p1 = copy.deepcopy(self.turbine_lander_origin)
            p2 = copy.deepcopy(self.turbine_lander_origin)
            p2[2] -= 10
            p3 = np.array([points[i][0], points[i][1], points[i][2]])
            distance_to_obs = np.linalg.norm(np.cross(p2-p1,p1-p3))/np.linalg.norm(p2-p1)

            if distance_to_obs > self.safe_tolerance and p3[2] < (self.turbine_lander_origin[2] - 1.5):
                feasible_points.append(points[i])
        return feasible_points
            
    def viewpoint_selection(self):
        self.cap_fitness = []
        viewpoints = {}
        
        self.sphere_vps = ViewpointList()
        self.selected_view = ViewpointList()
        self.selected_vps = ViewpointList()
        self.generated_vps = ViewpointList()
        
        self.sphere_vps.header.frame_id = 'world'
        self.selected_view.header.frame_id = 'world'
        self.selected_vps.header.frame_id = 'world'
        self.generated_vps.header.frame_id = 'world'
        
        # Step 1: Generate spheres
        for sr in self.sphere_radii:
            viewpoints_sphere = GenerateClouds.generateUniformCloud(self.rob_pose[0],
                                                                    self.rob_pose[1],
                                                                    self.rob_pose[2],
                                                                    sr,
                                                                    numPoints= int(sr * 50))
            
            # viewpoints_sphere = GenerateClouds.generateRandomCloud(self.rob_pose[0],
            #                                                        self.rob_pose[1],
            #                                                        self.rob_pose[2],
            #                                                        sr,
            #                                                        numPoints= int(sr * 50))
            
            viewpoints[sr] = viewpoints_sphere
            
            for v in viewpoints_sphere:
                q = Viewpoint()
                q.x = v[0]
                q.y = v[1]
                q.z = v[2]
                q.yaw = v[4]
                q.cost = sr
                self.sphere_vps.viewpoints.append(q)
        
        # print(self.combined_population.shape)
            
        for i in range(self.combined_population.shape[0]):
            v_i = self.combined_population[i]
            
            # Step 2: Extract spherical cap
            capPoints = SliceCloud.sliceSphericalCap(viewpoints[v_i[2]],
                                                     self.rob_pose[0],
                                                     self.rob_pose[1],
                                                     self.rob_pose[2],
                                                     v_i[2],
                                                     v_i[0],
                                                     v_i[1],
                                                     v_i[3])
            
            capPoints = self.remove_infeasible(capPoints)
            if len(capPoints) == 0:
                continue
            
            # Step 3: Obtain battery usage, segment length and percentage of unexplored area for the capPoints
            vps = ViewpointList()
            vps.header.frame_id = "world"
            for cp in capPoints:
                vp = Viewpoint()
                vp.x = cp[0]
                vp.y = cp[1]
                vp.z = cp[2]
                vp.yaw = cp[4]
                vp.cost = v_i[2]
                vps.viewpoints.append(vp)
                self.generated_vps.viewpoints.append(vp)
            
            vps_map = carve_map_service(True, vps)

            vps_battery = battery_usage_service(True, vps_map)
            
            # print(vps_battery)
                                    
            # Step 4: Evaluate the fitness of the capPoints
            rov_properties = []
            cap_points = []
            for viewpoint in vps_battery.vp_info:
                cap_points.append([viewpoint.x, viewpoint.y, viewpoint.z, viewpoint.yaw,
                                   viewpoint.battery, viewpoint.cost, viewpoint.reward])
                rov_properties.append([viewpoint.battery, -viewpoint.cost, viewpoint.reward])
            
            if len(rov_properties) <= 1:
                continue
                        
            objectiveValues = np.array(rov_properties)
            fitnessValues = ParetoFronts.calcSPFitnessValues(objectiveValues)
            
            # print(fitnessValues)
            
            # Step 5: Determine average fitness of the cap
            avg_fitness = np.mean(fitnessValues)
            self.cap_fitness.append([cap_points, v_i, fitnessValues, avg_fitness])
        
        if len(self.cap_fitness) == 0:
            return None
        else:
            # Step 6: Sort caps based on average fitness
            self.cap_fitness.sort(key=lambda x:x[3])
            self.cap_fitness = np.array(self.cap_fitness)
            
            # Step 7: Select 'N' best caps        
            self.selected_viewpoints = []
            self.selected_ids = []
            
            norm_score = np.linalg.norm(self.cap_fitness[:, 3])
            self.cap_fitness[:, 3] /= norm_score
            
            if self.cap_fitness.shape[0] > self.pop_size:
                for _ in range(self.pop_size):
                    r = random.random()
                    np.random.shuffle(self.cap_fitness)
                    for k in range(self.cap_fitness.shape[0]):
                        if r > self.cap_fitness[k][3]:
                            self.selected_viewpoints.append(self.cap_fitness[k])
                            self.selected_ids.append(k)
                            self.cap_fitness = np.delete(self.cap_fitness, k, axis=0)
                            break
            else:
                self.selected_viewpoints = copy.deepcopy(self.cap_fitness)
                    
            self.selected_viewpoints = np.array(self.selected_viewpoints)
            print(self.selected_viewpoints.shape)        
            
            # Step 8: Extract the cap with the best overall fitness
            placeholder_ids = np.argsort(self.selected_viewpoints[:, 3]).tolist()
            section_idx = placeholder_ids.index(min(placeholder_ids))       
            best_cap = self.selected_viewpoints[section_idx]
            
            # Step 9: Extract the viewpoint with the best overall fitness
            best_cap_fitness = best_cap[2].tolist()
            best_viewpoint_idx = best_cap_fitness.index(min(best_cap_fitness))
            best_viewpoint = best_cap[0][best_viewpoint_idx]
            self.target_viewpoint = copy.deepcopy(best_viewpoint)
            
            q = Viewpoint()
            q.x = best_viewpoint[0]
            q.y = best_viewpoint[1]
            q.z = best_viewpoint[2]
            q.yaw = best_viewpoint[3]
            q.cost = best_viewpoint[5]
            self.selected_view.viewpoints.append(q)
            
            self.sphere_viewpoints_pub.publish(self.sphere_vps)
            self.generated_viewpoints_pub.publish(self.generated_vps)
            self.selected_viewpoints_pub.publish(self.selected_vps)
            self.selected_view_pub.publish(self.selected_view)
            
            waypoint = PoseArray()
            waypoint.header.frame_id = "world"
            waypoint.header.stamp = rospy.Time.now()
            selected_point = Pose()
            selected_point.position.x = best_viewpoint[0]
            selected_point.position.y = best_viewpoint[1]
            selected_point.position.z = best_viewpoint[2]
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, best_viewpoint[3])
            selected_point.orientation.x = q[0]
            selected_point.orientation.y = q[1]
            selected_point.orientation.z = q[2]
            selected_point.orientation.w = q[3]
            waypoint.poses.append(selected_point)
            
            viz_marker = Point()
            viz_marker.x = best_viewpoint[0]
            viz_marker.y = best_viewpoint[1]
            viz_marker.z = best_viewpoint[2]
            
            if len(self.traversed_path) == 0.0:
                current_location = Point()
                current_location.x = self.rob_pose[0]
                current_location.y = self.rob_pose[1]
                current_location.z = self.rob_pose[2]
                self.traversed_path.append(current_location)
            
            self.path_length += best_viewpoint[5]
            self.traversed_path.append(viz_marker)
            self.visualize_path()
            self.best_viewpoint_pub.publish(waypoint)
            self.move_flag = True
            deplete_battery_service(best_viewpoint[5])
            print(best_viewpoint, self.path_length)
            self.battery_remaining -= best_viewpoint[5]
            
            return best_cap[1]
    
    def wait_until_move(self):
        diff_yaw = abs(self.target_viewpoint[3] - self.rob_pose[5])
        diff_dist = np.linalg.norm(np.array(self.rob_pose[0:3]) -  np.array(self.target_viewpoint[0:3]))
        # print(diff_yaw, diff_dist)
        if diff_yaw < self.yaw_tolerance and diff_dist < self.distance_tolerance:
            self.move_flag = False
        
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
    rospy.init_node('peas_node', anonymous=True)
    p = PEAS()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down the node")
