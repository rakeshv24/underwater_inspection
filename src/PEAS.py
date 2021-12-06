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
import numpy as np
import copy
import ParetoFronts


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
        rospy.logerr("Battery usage service call failed %s", e)

        
def deplete_battery_service(distance):
    rospy.wait_for_service('underwater_inspection/deplete_battery')
    try:
        handle = rospy.ServiceProxy('underwater_inspection/deplete_battery', DepleteBattery)
        resp = handle(distance)
        return resp
    except rospy.ServiceException as e:
        rospy.logerr("Deplete battery service call failed %s", e)


def carve_map_service(flag, viewpoints):
    rospy.wait_for_service('underwater_inspection/carve_map_service')
    try:
        handle = rospy.ServiceProxy('underwater_inspection/carve_map_service', CarveMap)
        resp = handle(flag, viewpoints)
        if resp.success == False:
            print("[PEAS][carve_map_service] Not successful")
            return carve_map_service(flag, viewpoints)
        else:
            # print(resp.vps_map)
            return resp.vps_map
    except rospy.ServiceException as e:
        rospy.logerr("Deplete battery service call failed %s", e)
        
        
class PEAS:
    def __init__(self):
        self.rob_pose = None
        self.generations = 1
        self.initial_population = []
        self.mutated_population = []
        self.crossed_population = []
        self.combined_population = []
        self.selected_population = []
        self.pop_size = 5
        self.cap_fitness = []
        # self.avg_cap_fitness = []
        self.mutation_rate = 0.5
        self.sphere_radii = [1.0, 2.0]
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        self.path_length = 0.0
        self.sphere_viewpoints_pub = rospy.Publisher("/rob537/sphere_viewpoints", ViewpointList, queue_size=10)
        self.generated_viewpoints_pub = rospy.Publisher("/rob537/generated_viewpoints", ViewpointList, queue_size=10)
        self.selected_viewpoints_pub = rospy.Publisher("/rob537/selected_viewpoints", ViewpointList, queue_size=10)
        self.selected_view_pub = rospy.Publisher("/rob537/selected_view", ViewpointList, queue_size=10)
        self.odom_sub = rospy.Subscriber("/raven/odom", Odometry, self.odom_callback, queue_size=1)

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

        self.generate_init_pop()
        for i in range(self.generations):
            self.mutated_population = []
            self.crossed_population = []
            for j in range(self.initial_population.shape[0]):
                if random.random() < self.mutation_rate:
                    self.mutated_population.append(self.mutate(j))
                    self.m_population = np.array(self.mutated_population)
                else:
                    self.crossover()
                    self.c_population = np.array(self.crossed_population)
                    
            self.combined_population = np.vstack((self.initial_population, self.m_population, self.c_population))
            self.combined_population = np.unique(self.combined_population, axis=0)

            self.viewpoint_selection()

    def generate_init_pop(self):
        self.initial_population = []

        for i in self.sphere_radii:
            # Front
            self.initial_population.append([self.altitude_angle, self.azimuth_angle, i, 0.5 * i])
            # Left
            self.initial_population.append([self.altitude_angle, self.wrap_angle(
                self.azimuth_angle - np.radians(90)), i, 0.5 * i])
            # Right
            self.initial_population.append([self.altitude_angle, self.wrap_angle(
                self.azimuth_angle + np.radians(90)), i, 0.5 * i])
            # Up
            self.initial_population.append(
                [self.wrap_angle(self.altitude_angle - np.radians(90)), self.azimuth_angle, i, 0.5 * i])
            # Down
            self.initial_population.append(
                [self.wrap_angle(self.altitude_angle + np.radians(90)), self.azimuth_angle, i, 0.5 * i])

        self.initial_population = np.array(self.initial_population)

    def mutate(self, idx):
        r = random.random()
        mutated = copy.deepcopy(self.initial_population[idx, :])
        if r < 0.33:
            mutated[0] = random.uniform(-1.57, 1.57) 
        elif 0.33 <= r < 0.66:
            mutated[1] = random.uniform(-1.57, 1.57) 
        else:
            tmp = mutated[0]
            mutated[0] = mutated[1]
            mutated[1] = tmp
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
                if rr < 0.25:
                    tmp = v1[0]
                    v1[0] = v2[0]
                    v2[0] = tmp 
                elif 0.25 <= rr < 0.5:
                    tmp = v1[1]
                    v1[1] = v2[1]
                    v2[1] = tmp     
                elif 0.5 <= rr < 0.75:
                    tmp = v1[0]
                    v1[0] = v2[1]
                    v2[1] = tmp 
                else:
                    tmp = v1[1]
                    v1[1] = v2[0]
                    v2[0] = tmp
                    
        self.crossed_population.append(v1)                
        self.crossed_population.append(v2)                

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
            viewpoints[sr] = viewpoints_sphere
            for v in viewpoints_sphere:
                q = Viewpoint()
                q.x = v[0]
                q.y = v[1]
                q.z = v[2]
                q.yaw = v[4]
                q.cost = sr
                self.sphere_vps.viewpoints.append(q)
            
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
            # print(capPoints)
            # print(vps_map)
            vps_battery = battery_usage_service(True, vps_map)
            
            # print(vps_battery)
                                    
            # Step 4: Evaluate the fitness of the capPoints
            points = []
            for viewpoint in vps_battery.vp_info:
                rov_properties = [viewpoint.battery, -viewpoint.cost, viewpoint.reward]
                points.append(rov_properties)
            
            print(points)
            
            objectiveValues = np.array(points)
            fitnessValues = ParetoFronts.calcSPFitnessValues(objectiveValues)
            
            print(fitnessValues)
            
            # Step 5: Determine average fitness of the cap
            avg_fitness = np.mean(fitnessValues)
            self.cap_fitness.append([vps_battery, v_i, fitnessValues, avg_fitness])
        
        # Step 6: Sort caps based on average fitness
        self.cap_fitness.sort(key=lambda x:x[3])
        
        print(self.cap_fitness)    
        
        # Step 7: Select 'N' best caps        
        self.selected_population = []
        self.selected_ids = []
        for _ in range(len(self.cap_fitness)):
            self.cap_fitness = np.array(self.cap_fitness)
            norm_score = np.linalg.norm(self.cap_fitness[:, 3])
            self.cap_fitness[:, 3] /= norm_score
            
            for _ in range(self.pop_size):
                r = random.random()
                np.random.shuffle(self.cap_fitness)
                for k in range(self.cap_fitness.shape[0]):
                    if r > self.cap_fitness[k][3]:
                        self.selected_population.append(self.cap_fitness[k])
                        self.selected_ids.append(k)
                        self.cap_fitness = np.delete(self.cap_fitness, k)
                        break
                    
        self.selected_population = np.array(self.selected_population)
        print(self.selected_population)        
        
        # Step 8: Extract the cap with the best overall fitness
        placeholder_ids = np.argsort(self.selected_population[:, 3]).tolist()
        section_idx = placeholder_ids.index(min(placeholder_ids))       
        best_cap = self.selected_population[section_idx]
        
        # Step 9: Extract the viewpoint with the best overall fitness
        best_cap_fitness = best_cap[2]
        best_viewpoint_idx = best_cap_fitness.index(min(best_cap_fitness))
        best_viewpoint = best_cap[0].vp_info[best_viewpoint_idx]
        
        q = Viewpoint()
        q.x = best_viewpoint.x
        q.y = best_viewpoint.y
        q.z = best_viewpoint.z
        q.yaw = best_viewpoint.yaw
        q.cost = best_viewpoint.cost
        self.selected_view.viewpoints.append(q)

        self.sphere_viewpoints_pub.publish(self.sphere_vps)
        self.generated_viewpoints_pub.publish(self.generated_vps)
        self.selected_viewpoints_pub.publish(self.selected_vps)
        self.selected_view_pub.publish(self.selected_view)
    

if __name__ == '__main__':
    rospy.init_node('peas_node', anonymous=True)
    p = PEAS()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down the node")
