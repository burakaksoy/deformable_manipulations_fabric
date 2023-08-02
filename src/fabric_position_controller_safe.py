#!/usr/bin/env python3

import sys

import rospy

import numpy as np
import time


from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Empty, EmptyResponse

import cvxpy as cp


class PIDController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0):
        # PID gains
        self.Kp = np.array(Kp)
        self.Ki = np.array(Ki)
        self.Kd = np.array(Kd)

        # Integral and derivative terms
        self.integral_term = np.zeros_like(self.Kp)
        self.derivative_term = np.zeros_like(self.Kp)

        # Error and time step
        self.last_error = np.zeros_like(self.Kp)
        self.last_time = rospy.Time.now().to_sec()

    def update(self, error):
        # Update the integral and derivative terms
        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.last_time

        self.integral_term += error * dt
        self.derivative_term = (error - self.last_error) / dt

        # Update error and time
        self.last_error = error
        self.last_time = current_time

    def output(self):
        # Calculate the output
        output = self.Kp * self.last_error + self.Ki * self.integral_term + self.Kd * self.derivative_term

        return output


class PositionControllerNode:
    def __init__(self):
        self.pub_rate_odom = rospy.get_param("~pub_rate_odom", 100)
        self.leader_particle = rospy.get_param("~leader_particle", 0) # Default leader particle is the first one
        self.binded_particles = rospy.get_param("~binded_particles", [14]) # Default binded particles is an empty list

        self.fabric_points_topic_name    = rospy.get_param("~fabric_points_topic_name", "/fabric_points") 

        self.min_distance_topic_name    = rospy.get_param("~min_distance_dx_topic_name", "/distance_to_obj") 
        self.min_distance_dx_topic_name = rospy.get_param("~min_distance_dx_topic_name", "/distance_to_obj_x") 
        self.min_distance_dy_topic_name = rospy.get_param("~min_distance_dx_topic_name", "/distance_to_obj_y") 
        self.min_distance_dz_topic_name = rospy.get_param("~min_distance_dx_topic_name", "/distance_to_obj_z") 

        self.particles = None
        self.odom_topic_prefix = None
        while not self.particles:
            try:
                self.particles = rospy.get_param("/fabric_simulator_node/custom_static_particles")
                self.odom_topic_prefix = rospy.get_param("/fabric_simulator_node/custom_static_particles_odom_topic_prefix")
            except:
                rospy.logwarn("No particles obtained from ROS parameters. GUI will be empty.")
                time.sleep(0.5)

        self.odom_publishers = {}
        self.particle_positions = {}
        self.initial_relative_positions = {}
        self.pid_controllers = {}

        self.initial_values_set = False  # Initialization state variable

        self.enabled = False  # Flag to enable/disable controller
        self.set_enable_server = rospy.Service('~set_enable', SetBool, self.set_enable)
        self.reset_positions_server = rospy.Service('~reset_positions', Empty, self.reset_positions)

        # PID gains
        self.kp = np.array(rospy.get_param("~kp", [0.1,0.1,0.1]))
        self.ki = np.array(rospy.get_param("~ki", [0.0,0.0,0.0]))
        self.kd = np.array(rospy.get_param("~kd", [0.0,0.0,0.0]))

        self.sub_marker = rospy.Subscriber(self.fabric_points_topic_name, Marker, self.marker_callback, queue_size=1)

        self.min_distance    = None
        self.min_distance_dx = None
        self.min_distance_dy = None
        self.min_distance_dz = None

        self.delta_x = rospy.get_param("~delta_x", 0.1)
        self.delta_y = rospy.get_param("~delta_y", 0.1)
        self.delta_z = rospy.get_param("~delta_z", 0.1)

        self.u_prev = np.zeros(3)  # Initialize previous control input
        
        self.sub_min_distance    = rospy.Subscriber(self.min_distance_topic_name,    Float32, self.min_distance_callback,    queue_size=1)
        self.sub_min_distance_dx = rospy.Subscriber(self.min_distance_dx_topic_name, Float32, self.min_distance_dx_callback, queue_size=1)
        self.sub_min_distance_dy = rospy.Subscriber(self.min_distance_dy_topic_name, Float32, self.min_distance_dy_callback, queue_size=1)
        self.sub_min_distance_dz = rospy.Subscriber(self.min_distance_dz_topic_name, Float32, self.min_distance_dz_callback, queue_size=1)



        for particle in self.particles:
            if particle != self.leader_particle:
                self.odom_publishers[particle] = rospy.Publisher(self.odom_topic_prefix + str(particle), Odometry, queue_size=1)
                self.pid_controllers[particle] = PIDController(self.kp, self.ki, self.kd)

        self.odom_pub_timer = rospy.Timer(rospy.Duration(1. / self.pub_rate_odom), self.odom_pub_timer_callback)


    def odom_pub_timer_callback(self,event):
        # Only publish if enabled
        if self.enabled:
            # Do not proceed until the initial values have been set
            if (not self.initial_values_set) or (not self.is_min_distances_set()):
                return
            
            for particle in self.particles:
                if particle != self.leader_particle:
                    target_pose = self.calculate_target_pose(particle)

                    if particle not in self.binded_particles:
                        # Get current position
                        current_pose = Pose()
                        current_pose.position = self.particle_positions[particle]

                        # Calculate (relative) error
                        error = self.calculate_error(current_pose, target_pose)

                        # Update the PID terms with the current error
                        self.pid_controllers[particle].update(error)

                        # Get control output from PID controller                
                        control_output = self.pid_controllers[particle].output()

                        # Get safe control output using control barrier functions
                        control_output_safe = self.calculate_safe_control_output(control_output) # safe

                        if control_output_safe is not None:
                            # Scale down the calculated output if its norm is higher than the specified norm max_u
                            control_output_safe = self.scale_down_vector(control_output_safe, max_u=0.005)

                            # print(np.linalg.norm(control_output_safe - control_output))

                            # print(control_output_safe - control_output)

                            # control_output_safe = control_output


                            # Prepare Odometry message
                            odom = Odometry()
                            odom.header.stamp = rospy.Time.now()
                            odom.header.frame_id = "map"

                            # Control output is the new position
                            odom.pose.pose.position.x =  current_pose.position.x + control_output_safe[0]
                            odom.pose.pose.position.y =  current_pose.position.y + control_output_safe[1]
                            odom.pose.pose.position.z =  current_pose.position.z + control_output_safe[2]


                            self.odom_publishers[particle].publish(odom)
                    
                    else:
                        # Assuming that the binding particle is directly at the target pose!
                        # In real application this is already done by the human
                        pass
                        # # Prepare Odometry message
                        # odom = Odometry()
                        # odom.header.stamp = rospy.Time.now()
                        # odom.header.frame_id = "map"

                        # # Control output is the new position
                        # odom.pose.pose =  target_pose

                        # self.odom_publishers[particle].publish(odom)


    def calculate_target_pose(self, particle):
        # Calculate the target position  relative to the leader
        target_pose = Pose()
        leader_position = self.particle_positions[self.leader_particle]

        target_pose.position.x = leader_position.x + self.initial_relative_positions[particle].x
        target_pose.position.y = leader_position.y + self.initial_relative_positions[particle].y
        target_pose.position.z = leader_position.z + self.initial_relative_positions[particle].z

        return target_pose


    def calculate_error(self, current_pose, target_pose):
        
        err_x = target_pose.position.x - current_pose.position.x
        err_y = target_pose.position.y - current_pose.position.y
        err_z = target_pose.position.z - current_pose.position.z

        return np.array([err_x,err_y,err_z])


    def marker_callback(self, marker):
        if marker.type == Marker.POINTS:
            for particle in self.particles:
                self.particle_positions[particle] = marker.points[particle]

                # Calculate initial relative positions
                if (particle != self.leader_particle) and (particle not in self.initial_relative_positions):
                    self.initial_relative_positions[particle] = Point()
                    self.initial_relative_positions[particle].x = self.particle_positions[particle].x - self.particle_positions[self.leader_particle].x
                    self.initial_relative_positions[particle].y = self.particle_positions[particle].y - self.particle_positions[self.leader_particle].y
                    self.initial_relative_positions[particle].z = self.particle_positions[particle].z - self.particle_positions[self.leader_particle].z


        # After all initial relative positions have been calculated, set the initialization state variable to True
        self.initial_values_set = True
    
    def set_enable(self, request):
        self.enabled = request.data
        return SetBoolResponse(True, 'Successfully set enabled state to {}'.format(self.enabled))
    
    def reset_positions(self, request):
        # Reset the initial relative positions 
        for particle in self.particles:
            if particle != self.leader_particle:
                self.initial_relative_positions[particle] = Point()
                self.initial_relative_positions[particle].x = self.particle_positions[particle].x - self.particle_positions[self.leader_particle].x
                self.initial_relative_positions[particle].y = self.particle_positions[particle].y - self.particle_positions[self.leader_particle].y
                self.initial_relative_positions[particle].z = self.particle_positions[particle].z - self.particle_positions[self.leader_particle].z
                
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

    def calculate_safe_control_output(self, nominal_u):
        if not self.is_min_distances_set():
            return

        Jx = ((self.min_distance_dx - self.min_distance) / self.delta_x) if self.delta_x != 0 else 0
        Jy = ((self.min_distance_dy - self.min_distance) / self.delta_y) if self.delta_y != 0 else 0
        Jz = ((self.min_distance_dz - self.min_distance) / self.delta_z) if self.delta_z != 0 else 0
        J = np.array([[Jx,Jy,Jz]]) # 1x3

        print("J",str(J))

        h = self.min_distance

        # Define optimization variables
        u = cp.Variable(3)

        # Define weights for each control input
        weights = np.array([1.0, 1.0, 0.1])  # Less weight on z control input
        
        # Define cost function with weights
        cost = cp.sum_squares(cp.multiply(weights, u - nominal_u)) / 2

        alpha_h = self.calculate_extended_class_K_value(h)

        # Define constraints
        constraints = [J @ u >= -alpha_h]

        # # Add control rate constraint
        # delta_u_max = 0.001  # Maximum allowed change in control input
        # constraints += [cp.norm(u - self.u_prev, 2) <= delta_u_max]

        # # Add control constraint
        # u_max = 0.005 # maximum input
        # constraints += [cp.norm(u, 2) <= u_max]

        # Define and solve problem
        problem = cp.Problem(cp.Minimize(cost), constraints)

        try:
            problem.solve()
        except cp.error.SolverError as e:
            rospy.logwarn("Could not solve the problem: {}".format(e))
            # rospy.logwarn("Nominal control input will be used as the control input!")
            # return nominal_u
            return None

        # check if problem is infeasible or unbounded
        if problem.status in ["infeasible", "unbounded"]:
            rospy.logwarn("The problem is {}.".format(problem.status))
            # rospy.logwarn("Nominal control input will be used as the control input!")
            # return nominal_u
            return None
        
        # Update previous control input
        self.u_prev = np.array(u.value)

        # Return optimal u
        return u.value
    
    def calculate_extended_class_K_value(self,h):
        d_offset = 0.1

        # Linear
        # gamma = 1.0
        # alpha_h = gamma*(h - d_offset
        
        # # # Arctan
        # c1 = 0.01
        # c2 = 3.0 # 5.0 # increase this if you want to start reacting early to be more safe, (but causes less nominal controller following)
        # c3 = 1.0 # 3.0 # decrease this if you want to remove the offset violation more agressively, (but may cause instability due to the discretization if too agressive)
        # alpha_h = np.arctan(c1*(h-d_offset))/c2 if (h-d_offset) >= 0 else np.arctan(c1*(h-d_offset))/c3


        # Linear 2
        c1 = 0.01 # 0.01 # 0.00335 # decrease this if you want to start reacting early to be more safe, (but causes less nominal controller following)
        c2 = 0.04 # 0.02 # 0.01 # increase this if you want to remove the offset violation more agressively, (but may cause instability due to the discretization if too agressive)
        alpha_h = c1*(h - d_offset) if (h-d_offset) >= 0 else c2*(h - d_offset)

        return alpha_h

    def scale_down_vector(self, u, max_u=0.005):
        norm_u = np.linalg.norm(u)

        if norm_u > max_u:
            u = u / norm_u * max_u

        return u


if __name__ == "__main__":
    rospy.init_node('fabric_position_controller_node', anonymous=False)

    node = PositionControllerNode()

    rospy.spin()
