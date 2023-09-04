#!/usr/bin/env python3

import sys

import rospy

import numpy as np
import time


from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Empty, EmptyResponse



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
        self.kp = np.array(rospy.get_param("~kp", [0.9,0.9,0.9]))
        self.ki = np.array(rospy.get_param("~ki", [0.0,0.0,0.0]))
        self.kd = np.array(rospy.get_param("~kd", [0.0,0.0,0.0]))

        self.sub_marker = rospy.Subscriber("/fabric_points", Marker, self.marker_callback, queue_size=1)

        for particle in self.particles:
            if particle != self.leader_particle:
                self.odom_publishers[particle] = rospy.Publisher(self.odom_topic_prefix + str(particle), Odometry, queue_size=1)
                self.pid_controllers[particle] = PIDController(self.kp, self.ki, self.kd)

        self.odom_pub_timer = rospy.Timer(rospy.Duration(1. / self.pub_rate_odom), self.odom_pub_timer_callback)


    def odom_pub_timer_callback(self,event):
        # Only publish if enabled
        if self.enabled:
            # Do not proceed until the initial values have been set
            if not self.initial_values_set:
                return
            
            for particle in self.particles:
                if particle != self.leader_particle:
                    target_pose = self.calculate_target_pose(particle)

                    # Get current position 
                    current_pose = Pose()
                    current_pose.position = self.particle_positions[particle]

                    # Calculate (relative) error
                    error = self.calculate_error(current_pose, target_pose)

                    # Update the PID terms with the current error
                    self.pid_controllers[particle].update(error)

                    # Get control output from PID controller                
                    control_output = self.pid_controllers[particle].output()

                    # Prepare Odometry message
                    odom = Odometry()
                    odom.header.stamp = rospy.Time.now()
                    odom.header.frame_id = "map"

                    # Control output is the new position
                    odom.pose.pose.position.x =  current_pose.position.x + control_output[0]
                    odom.pose.pose.position.y =  current_pose.position.y + control_output[1]
                    odom.pose.pose.position.z =  current_pose.position.z + control_output[2]

                    self.odom_publishers[particle].publish(odom)


    def calculate_target_pose(self, particle):
        # Calculate the target position relative to the leader
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


if __name__ == "__main__":
    rospy.init_node('fabric_position_controller_node', anonymous=False)

    node = PositionControllerNode()

    rospy.spin()
