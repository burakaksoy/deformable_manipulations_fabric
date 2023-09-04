#!/usr/bin/env python3

import sys

import rospy

import numpy as np
import time


from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, Float32MultiArray

from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Empty, EmptyResponse

import cvxpy as cp

import time 

class Controller:
    def __init__(self, Kp=1.0, Kd=0.0, MAX_TIMESTEP = 0.1):
        # PD gains
        self.Kp = np.array(Kp)
        self.Kd = np.array(Kd)

        self.last_time = None # time.time()

        # Velocity commands will only be considered if they are spaced closer than MAX_TIMESTEP
        self.MAX_TIMESTEP = MAX_TIMESTEP

    def output(self, error, vel):
        # Calculate the output
        output = self.Kp * error - self.Kd * vel

        return output
    
    def get_dt(self):
        current_time = time.time()
        if self.last_time:
            dt = current_time - self.last_time
            self.last_time = current_time
            if dt > self.MAX_TIMESTEP:
                dt = 0.0
                rospy.logwarn("Controller took more time than specified MAX_TIMESTEP duration, resetting to 0.")
            return dt
        else:
            self.last_time = current_time
            return 0.0


class VelocityControllerNode:
    def __init__(self):
        # rospy.sleep(2)

        self.pub_rate_odom = rospy.get_param("~pub_rate_odom", 50)

        try:
            self.id = int(rospy.get_param("~id")) # follower particle id of this controller node controls
        except:
            rospy.logerr("NO FOLLOWER PARTICLE ID IS SPECIFIED FOR THE CONTROLLER NODE")


        self.delta_x = rospy.get_param("/delta_x", 0.1)
        self.delta_y = rospy.get_param("/delta_y", 0.1)
        self.delta_z = rospy.get_param("/delta_z", 0.1)

        # Parameters needed to figure out the max and min allowed distances automatically
        # NOTE: Fabric is assumed to be regular rectangular mesh
        # TODO: For a generic fabric mesh this method needs to be updated.
        self.fabric_x = rospy.get_param("/fabric_x") # 2.0 # fabric x size in meters
        self.fabric_y = rospy.get_param("/fabric_y") # 2.0 # fabric y size in meters
        self.fabric_res = rospy.get_param("/fabric_resolution") # 7 # particle resolution per meter 

        self.particles = None
        self.follower_particles = None
        self.leader_particle = None
        self.odom_topic_prefix = None
        while not self.particles:
            try:
                self.particles = rospy.get_param("/custom_static_particles")
                self.follower_particles = rospy.get_param("/follower_particles") 
                self.leader_particle = rospy.get_param("/leader_particle") 
                self.odom_topic_prefix = rospy.get_param("/custom_static_particles_odom_topic_prefix")
            except:
                rospy.logwarn("No particles obtained from ROS parameters in Controller node.")
                time.sleep(0.5)

        # Find maximum allowed distances between particles
        self.d_maxs = {}
        self.d_mins = {}

        self.find_max_n_min_allowed_distances()

        # SERVICES
        self.enabled = False  # Flag to enable/disable controller
        self.set_enable_server = rospy.Service('~set_enable', SetBool, self.set_enable)
        self.reset_positions_server = rospy.Service('~reset_positions', Empty, self.reset_positions)

        # Subscriber for fabric points to figure out the current particle positions
        self.initial_values_set = False  # Initialization state variable

        self.particle_positions = {}

        self.particle_positions_prev = {} # dicts to store the prev. particle positions, 

        self.initial_relative_position = None

        self.fabric_points_topic_name    = rospy.get_param("~fabric_points_topic_name", "/fabric_points") 

        self.sub_marker = rospy.Subscriber(self.fabric_points_topic_name, Marker, self.marker_callback, queue_size=10)

        # Subscribers to the minimum distances with perturbations
        self.min_distance_topic_prefix    = rospy.get_param("~min_distance_topic_prefix",   "/distance_to_obj") 
        self.min_distance_topic_name    = self.min_distance_topic_prefix
        self.min_distance_dx_topic_name = self.min_distance_topic_name + "_" + str(self.id) + "_x" 
        self.min_distance_dy_topic_name = self.min_distance_topic_name + "_" + str(self.id) + "_y" 
        self.min_distance_dz_topic_name = self.min_distance_topic_name + "_" + str(self.id) + "_z" 

        self.min_distance    = None
        self.min_distance_dx = None
        self.min_distance_dy = None
        self.min_distance_dz = None

        self.sub_min_distance    = rospy.Subscriber(self.min_distance_topic_name,    Float32, self.min_distance_callback,    queue_size=10)
        self.sub_min_distance_dx = rospy.Subscriber(self.min_distance_dx_topic_name, Float32, self.min_distance_dx_callback, queue_size=10)
        self.sub_min_distance_dy = rospy.Subscriber(self.min_distance_dy_topic_name, Float32, self.min_distance_dy_callback, queue_size=10)
        self.sub_min_distance_dz = rospy.Subscriber(self.min_distance_dz_topic_name, Float32, self.min_distance_dz_callback, queue_size=10)


        # Controller gains
        self.kp = np.array(rospy.get_param("~kp", [10.0,10.0,10.0]))
        self.kd = np.array(rospy.get_param("~kd", [0.0,0.0,0.0]))
        self.control_output_safe = np.zeros(3) # initialization for the velocity command

        # Create the nominal controller
        self.controller = Controller(self.kp, self.kd, self.pub_rate_odom*2.0)

        # create a publisher for the controlled particle
        self.odom_publisher = rospy.Publisher(self.odom_topic_prefix + str(self.id), Odometry, queue_size=1)

        # Create the necessary publishers for information topics, 
        # topic names are prefixed with "controller_info_" + str(self.id) 
        # followed by a description of the information topic.
        self.info_error_norm_publisher       = rospy.Publisher("controller_info_"+str(self.id)+"_error_norm",                Float32,           queue_size=1)
        self.info_target_pose_publisher      = rospy.Publisher("controller_info_"+str(self.id)+"_target_pose",               Marker,            queue_size=1)
        self.info_J_publisher                = rospy.Publisher("controller_info_"+str(self.id)+"_J_matrix",                  Float32MultiArray, queue_size=1)
        self.info_h_collision_publisher      = rospy.Publisher("controller_info_"+str(self.id)+"_h_distance_collision",      Float32,           queue_size=1)
        self.info_h_overstretching_publisher = rospy.Publisher("controller_info_"+str(self.id)+"_h_distance_overstretching", Float32MultiArray, queue_size=1)
        self.info_h_too_close_publisher      = rospy.Publisher("controller_info_"+str(self.id)+"_h_distance_too_close",      Float32MultiArray, queue_size=1)

        # Start the control
        self.odom_pub_timer = rospy.Timer(rospy.Duration(1. / self.pub_rate_odom), self.odom_pub_timer_callback)

    def default_distance_between_particles(self, i, j):
        # Calculate number of particles along each dimension
        num_particle_x = int(self.fabric_x * self.fabric_res)
        num_particle_y = int(self.fabric_y * self.fabric_res)
        
        # Create coordinate vectors for the coordinates
        x_coords = np.linspace(-self.fabric_x/2.0, self.fabric_x/2.0, num_particle_x + 1)
        y_coords = np.linspace(-self.fabric_y/2.0, self.fabric_y/2.0, num_particle_y + 1)
        
        # Map the linear index to 2D indices
        i_x, i_y = divmod(i, num_particle_y + 1)
        j_x, j_y = divmod(j, num_particle_y + 1)
        
        # Fetch coordinates of particles i and j
        coord_i = np.array([x_coords[i_x], y_coords[i_y], 0.0])
        coord_j = np.array([x_coords[j_x], y_coords[j_y], 0.0])
        
        # Compute the distance
        distance = np.linalg.norm(coord_i - coord_j)
        
        return distance

    def find_max_n_min_allowed_distances(self):
        for particle in self.particles:
            if particle != self.id:
                l_i = 0.0 # each particle segment length
                l_0_ij = self.default_distance_between_particles(self.id, particle) # default distance between particle i and j (i:self.id, j:other particle)
                delta_norm = np.linalg.norm([self.delta_x,self.delta_y,self.delta_z])

                self.d_maxs[particle] = l_0_ij - l_i - 1.0*delta_norm # convention we follow for max allowed distance
                self.d_mins[particle] = l_i + 1.0*delta_norm # convention we follow for min allowed distance

        rospy.loginfo("Controlled Particle: " + str(self.id) + " maximum allowed distances with other particles: " + str(self.d_maxs))
        rospy.loginfo("Controlled Particle: " + str(self.id) + " minimum allowed distances with other particles: " + str(self.d_mins))


    def odom_pub_timer_callback(self,event):
        # Only publish if enabled
        if self.enabled:
            # Do not proceed until the initial values have been set
            if (not self.initial_values_set) or (not self.is_min_distances_set()):
                return
            
            # for particle in self.particles:
            #     if particle != self.leader_particle:
            target_pose = self.calculate_target_pose()

            # target_pose is geometry_msgs.msg.Pose, publish it for information to visualize in RVIZ 
            # as a line segment and a sphere in the end, and starting from the leader_position = self.particle_positions[self.leader_particle]
            leader_position = self.particle_positions[self.leader_particle]
            self.publish_arrow_marker(leader_position, target_pose)


            # Get current position
            current_pose = Pose()
            current_pose.position = self.particle_positions[self.id]

            # Calculate (relative) error
            error = self.calculate_error(current_pose, target_pose)

            # error is 3D np.array, publish its norm for information
            error_norm = np.linalg.norm(error)
            self.info_error_norm_publisher.publish(Float32(data=error_norm))

            # Update the controller terms with the current error and the last executed command
            # Get control output from the controller
            control_output = self.controller.output(error,self.control_output_safe) # nominal
            # control_output = np.zeros(3) # disable nominal controller to test only the safe controller

            # init_t = time.time()
            # Get safe control output using control barrier functions
            self.control_output_safe = self.calculate_safe_control_output(control_output, 1./self.pub_rate_odom) # safe
            # rospy.logwarn("QP solver calculation time: " + str(1000*(time.time() - init_t)) + " ms.")

            if self.control_output_safe is not None:
                # Scale down the calculated output if its norm is higher than the specified norm max_u
                self.control_output_safe = self.scale_down_vector(self.control_output_safe, max_u=0.15)

                # print(np.linalg.norm(self.control_output_safe - control_output))

                # print(self.control_output_safe - control_output)
                # rospy.logwarn("control_output:" + str(control_output))
                # rospy.logwarn("control_output_safe: "+ str(self.control_output_safe))

                # self.control_output_safe = control_output

                # Prepare Odometry message
                odom = Odometry()
                odom.header.stamp = rospy.Time.now()
                odom.header.frame_id = "map"

                dt_check = self.controller.get_dt() # 
                dt = 1./self.pub_rate_odom

                # Control output is the new position
                odom.pose.pose.position.x =  current_pose.position.x + self.control_output_safe[0]*dt
                odom.pose.pose.position.y =  current_pose.position.y + self.control_output_safe[1]*dt
                odom.pose.pose.position.z =  current_pose.position.z + self.control_output_safe[2]*dt

                self.odom_publisher.publish(odom)
            else:
                self.control_output_safe = np.zeros(3)
            


    def calculate_target_pose(self):
        # Calculate the target position  relative to the leader
        target_pose = Pose()
        leader_position = self.particle_positions[self.leader_particle]

        target_pose.position.x = leader_position.x + self.initial_relative_position.x
        target_pose.position.y = leader_position.y + self.initial_relative_position.y
        target_pose.position.z = leader_position.z + self.initial_relative_position.z

        return target_pose

    def calculate_error(self, current_pose, target_pose):
        
        err_x = target_pose.position.x - current_pose.position.x
        err_y = target_pose.position.y - current_pose.position.y
        err_z = target_pose.position.z - current_pose.position.z

        return np.array([err_x,err_y,err_z])

    def calculate_current_relative_position_vec(self,particle1,particle2):
        p_x = self.particle_positions[particle1].x - self.particle_positions[particle2].x
        p_y = self.particle_positions[particle1].y - self.particle_positions[particle2].y
        p_z = self.particle_positions[particle1].z - self.particle_positions[particle2].z

        return np.array([p_x,p_y,p_z])


    def marker_callback(self, marker):
        # Positions
        if marker.type == Marker.POINTS:
            for particle in self.particles:
                if self.initial_values_set:
                    self.particle_positions_prev[particle] = self.particle_positions[particle]
                self.particle_positions[particle] = marker.points[particle]

            # Calculate initial relative positions
            if (not self.initial_relative_position):
                self.initial_relative_position = Point()
                self.initial_relative_position.x = self.particle_positions[self.id].x - self.particle_positions[self.leader_particle].x
                self.initial_relative_position.y = self.particle_positions[self.id].y - self.particle_positions[self.leader_particle].y
                self.initial_relative_position.z = self.particle_positions[self.id].z - self.particle_positions[self.leader_particle].z

        # After all initial relative positions have been calculated, set the initialization state variable to True
        self.initial_values_set = True
    
    def set_enable(self, request):
        self.enabled = request.data
        return SetBoolResponse(True, 'Successfully set enabled state to {}'.format(self.enabled))
    
    def reset_positions(self, request):
        # Reset the initial relative positions and orientations
        self.initial_relative_position = Point()
        self.initial_relative_position.x = self.particle_positions[self.id].x - self.particle_positions[self.leader_particle].x
        self.initial_relative_position.y = self.particle_positions[self.id].y - self.particle_positions[self.leader_particle].y
        self.initial_relative_position.z = self.particle_positions[self.id].z - self.particle_positions[self.leader_particle].z
                
        return EmptyResponse()


    def min_distance_callback(self, distance):
        t = 0.5
        if self.min_distance is None:
            self.min_distance = distance.data
        else:
            self.min_distance = t*self.min_distance + (1-t)*distance.data

    def min_distance_dx_callback(self, distance):
        t = 0.5
        if self.min_distance_dx is None:
            self.min_distance_dx = distance.data
        else:
            self.min_distance_dx = t*self.min_distance_dx + (1-t)*distance.data

    def min_distance_dy_callback(self, distance):
        t = 0.5
        if self.min_distance_dy is None:
            self.min_distance_dy = distance.data
        else:
            self.min_distance_dy = t*self.min_distance_dy + (1-t)*distance.data

    def min_distance_dz_callback(self, distance):
        t = 0.5
        if self.min_distance_dz is None:
            self.min_distance_dz = distance.data
        else:
            self.min_distance_dz = t*self.min_distance_dz + (1-t)*distance.data

    def is_min_distances_set(self):
        return ((self.min_distance    is not None) and
                (self.min_distance_dx is not None) and  
                (self.min_distance_dy is not None) and 
                (self.min_distance_dz is not None))



    def calculate_safe_control_output(self, nominal_u, dt):
        if not self.is_min_distances_set():
            return

        # ---------------------------------------------------
        # Define optimization variables
        u = cp.Variable(3)

        # Define weights for each control input
        weights = np.array([1.0, 1.0, 0.1])  # Less weight on z control input
        
        # Define cost function with weights
        cost = cp.sum_squares(cp.multiply(weights, u - nominal_u)) / 2.0

        # Initialize the constraints
        constraints = []
        # ---------------------------------------------------

        # ---------------------------------------------------
        # DEFINE COLLISION AVOIDANCE CONTROL BARRRIER CONSTRAINT
        d_offset = 0.1 # offset distance from the obstacles
        h = self.min_distance - d_offset  # Control Barrier Function (CBF)
        alpha_h = self.alpha_collision_avoidance(h)

        # publish h that is the distance to collision for information
        # print("h distance to collision: ",str(h))
        self.info_h_collision_publisher.publish(Float32(data=h))

        Jx = ((self.min_distance_dx - self.min_distance) / self.delta_x) if self.delta_x != 0 else 0
        Jy = ((self.min_distance_dy - self.min_distance) / self.delta_y) if self.delta_y != 0 else 0
        Jz = ((self.min_distance_dz - self.min_distance) / self.delta_z) if self.delta_z != 0 else 0

        J = np.array([[Jx,Jy,Jz]]) # 1x3

        # publish J for information
        # print("J",str(J))
        J_msg = Float32MultiArray(data=np.ravel(J))
        self.info_J_publisher.publish(J_msg)

        J_tolerance = 0.05
        if not np.all(np.abs(J) < J_tolerance):
            # Add collision avoidance to the constraints
            constraints += [J @ u >= -alpha_h]
        else:
            rospy.loginfo("Follower: "+str(self.id)+", ignored J")
        # ---------------------------------------------------

        # ---------------------------------------------------
        h_overstretchs = []
        h_too_closes = []
        for particle in self.particles:
            if particle != self.id:
                # DEFINE OVERSTRETCHING AVOIDANCE CONTROL BARRRIER CONSTRAINT
                P_cur = self.calculate_current_relative_position_vec(self.id, particle)
                P_cur_norm = np.linalg.norm(P_cur) 

                min_norm_tolerance = 1.0e-9
                if P_cur_norm >= min_norm_tolerance:
                    P_cur_unit = P_cur/P_cur_norm

                    d_max = self.d_maxs[particle] # meters # maximum distance between the human held point and the robot controlled point

                    h_overstretch = d_max - P_cur_norm  # Control Barrier Function (CBF)
                    alpha_h = self.alpha_overstretch_avoidance(h_overstretch)

                    # print("h distance to overstretching: ",str(h_overstretch))
                    h_overstretchs.append(h_overstretch)

                    # calculate the velocity of the other particle
                    v_other_x = (self.particle_positions[particle].x - self.particle_positions_prev[particle].x)/dt if dt >= 1.0e-5 else 0.0
                    v_other_y = (self.particle_positions[particle].y - self.particle_positions_prev[particle].y)/dt if dt >= 1.0e-5 else 0.0
                    v_other_z = (self.particle_positions[particle].z - self.particle_positions_prev[particle].z)/dt if dt >= 1.0e-5 else 0.0
                    v_other = np.array([v_other_x,v_other_y,v_other_z])
                    # rospy.loginfo("Follower: "+str(self.id)+", v_other: "+str(v_other)+ " (for particle: " + str(particle) + ")")

                    # Add overstretching avoidance to the constraints
                    constraints += [-P_cur_unit.T @ u + P_cur_unit.T @ v_other >= -alpha_h]
                # ---------------------------------------------------

                    # ---------------------------------------------------
                    # DEFINE GETTING TOO CLOSE AVOIDANCE CONTROL BARRRIER CONSTRAINT
                    d_min = self.d_mins[particle] # meters # maximum distance between the human held point and the robot controlled point

                    h_too_close = P_cur_norm - d_min   # Control Barrier Function (CBF)
                    alpha_h = self.alpha_too_close_avoidance(h_too_close)

                    # print("h distance to agent getting too close: ",str(h))
                    h_too_closes.append(h_too_close)
                    
                    # Add getting too close avoidance to the constraints
                    constraints += [P_cur_unit @ u - P_cur_unit.T @ v_other >= -alpha_h]
                    # ---------------------------------------------------
        
        # publish h that is the distance to overstretching for information
        self.info_h_overstretching_publisher.publish(Float32MultiArray(data=h_overstretchs))

        # publish h that is the distance to agent getting too close for information
        self.info_h_too_close_publisher.publish(Float32MultiArray(data=h_too_closes))


        # Add also limit to the feasible u
        # u_max = 0.15
        # constraints += [cp.norm(u) <= u_max]

        # ---------------------------------------------------
        # Define and solve problem
        problem = cp.Problem(cp.Minimize(cost), constraints)

        try:
            problem.solve()
        except cp.error.SolverError as e:
            rospy.logwarn("Follower: "+str(self.id)+",Could not solve the problem: {}".format(e))
            return None

        # check if problem is infeasible or unbounded
        if problem.status in ["infeasible", "unbounded"]:
            rospy.logwarn("Follower: "+str(self.id)+", The problem is {}.".format(problem.status))
            return None
        
        # Return optimal u
        return u.value
    
    def alpha_collision_avoidance(self,h):
        # calculates the value of extended_class_K function \alpha(h) for COLLISION AVOIDANCE
        # Piecewise Linear function is used

        c1 = 0.8 # 0.01 # 0.01 # 0.00335 # decrease this if you want to start reacting early to be more safe, (but causes less nominal controller following)
        c2 = 3.0 # 0.04 # 0.02 # 0.01 # increase this if you want to remove the offset violation more agressively, (but may cause instability due to the discretization if too agressive)
        alpha_h = c1*(h) if (h) >= 0 else c2*(h)
        return alpha_h
    
    def alpha_overstretch_avoidance(self,h):
        # calculates the value of extended_class_K function \alpha(h) for COLLISION AVOIDANCE
        # Piecewise Linear function is used
        
        c1 = 2.0 # 0.01 # 0.01 # 0.00335 # decrease this if you want to start reacting early to be more safe, (but causes less nominal controller following)
        c2 = 2.0 # 0.04 # 0.02 # 0.01 # increase this if you want to remove the offset violation more agressively, (but may cause instability due to the discretization if too agressive)
        alpha_h = c1*(h) if (h) >= 0 else c2*(h)
        return alpha_h
    
    def alpha_too_close_avoidance(self,h):
        # calculates the value of extended_class_K function \alpha(h) for COLLISION AVOIDANCE
        # Piecewise Linear function is used
        
        c1 = 2.0 # 0.01 # 0.01 # 0.00335 # decrease this if you want to start reacting early to be more safe, (but causes less nominal controller following)
        c2 = 2.0 # 0.04 # 0.02 # 0.01 # increase this if you want to remove the offset violation more agressively, (but may cause instability due to the discretization if too agressive)
        alpha_h = c1*(h) if (h) >= 0 else c2*(h)
        return alpha_h

    def scale_down_vector(self, u, max_u=0.005):
        norm_u = np.linalg.norm(u)

        if norm_u > max_u:
            u = u / norm_u * max_u

        return u

    def publish_arrow_marker(self, leader_position, target_pose):
        # Create a marker for the arrow
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Set the scale of the arrow
        marker.scale.x = 0.015  # Shaft diameter
        marker.scale.y = 0.05  # Head diameter
        marker.scale.z = 0.3  # Head length
        
        # Set the color
        marker.color.a = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        # Set the pose (position and orientation) for the marker
        marker.pose.orientation.w = 1.0  # Orientation (quaternion)
        
        # Set the start and end points of the arrow
        marker.points = []
        start_point = leader_position  # Should be a Point message
        end_point = target_pose.position  # Assuming target_pose is a Pose message
        marker.points.append(start_point)
        marker.points.append(end_point)
        
        # Publish the marker
        self.info_target_pose_publisher.publish(marker)



if __name__ == "__main__":
    rospy.init_node('fabric_velocity_controller_node', anonymous=False)

    node = VelocityControllerNode()

    rospy.spin()
