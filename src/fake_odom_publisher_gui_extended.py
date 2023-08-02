#!/usr/bin/env python3

import sys

import rospy

import numpy as np
import time

import PyQt5.QtWidgets as qt_widgets
import PyQt5.QtCore as qt_core
from PyQt5.QtCore import Qt

from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Empty, EmptyResponse


# Velocity commands will only be considered if they are spaced closer than MAX_TIMESTEP
MAX_TIMESTEP = 0.1

class FakeOdomPublisherGUI(qt_widgets.QWidget):
    def __init__(self):
        super(FakeOdomPublisherGUI, self).__init__()
        self.shutdown_timer = qt_core.QTimer()

        self.pub_rate_odom = rospy.get_param("~pub_rate_odom", 100)

        self.delta_x = rospy.get_param("~delta_x", 0.1)
        self.delta_y = rospy.get_param("~delta_y", 0.1)
        self.delta_z = rospy.get_param("~delta_z", 0.1)

        self.initial_values_set = False  # Initialization state variable

        self.particles = None
        self.odom_topic_prefix = None
        while not self.particles:
            try:
                self.particles = rospy.get_param("/fabric_simulator_node/custom_static_particles")
                self.odom_topic_prefix = rospy.get_param("/fabric_simulator_node/custom_static_particles_odom_topic_prefix")
            except:
                rospy.logwarn("No particles obtained from ROS parameters. GUI will be empty.")
                time.sleep(0.5)

        # self.leader_particle = None
        # self.binded_particles = [] # Default binded particles is an empty list
        # while not self.leader_particle:
        #     try:
        #         self.leader_particle = rospy.get_param("/fabric_position_controller_node/leader_particle")
        #         self.binded_particles = rospy.get_param("/fabric_position_controller_node/binded_particles")
        #     except:
        #         rospy.logwarn("No leader particle obtained from ROS parameters from the controller.")
        #         time.sleep(0.5)
        self.leader_particle = rospy.get_param("~leader_particle", 0) # Default leader particle is the first one
        self.binded_particles = rospy.get_param("~binded_particles", [14]) # Default binded particles is an empty list


        self.odom_publishers = {}
        self.odom_publishers_delta_x = {}
        self.odom_publishers_delta_y = {}
        self.odom_publishers_delta_z = {}

        self.particle_positions = {}

        self.createUI()
        
        self.spacenav_twist = Twist() # set it to an empty twist message

        self.sub_twist = rospy.Subscriber("/spacenav/twist", Twist, self.spacenav_twist_callback, queue_size=1)
        self.sub_marker = rospy.Subscriber("/fabric_points", Marker, self.marker_callback, queue_size=1)

        self.last_timestep_requests = {}

        self.odom_pub_timer = rospy.Timer(rospy.Duration(1. / self.pub_rate_odom), self.odom_pub_timer_callback)
        

    def createUI(self):
        self.layout = qt_widgets.QVBoxLayout(self)

        self.buttons = {}
        self.text_inputs = {}

        for particle in self.particles:
            # Create QHBoxLayout for each row
            row_layout = qt_widgets.QHBoxLayout()

            button = qt_widgets.QPushButton()

            button.setText("Manually Control Particle " + str(particle))

            button.setCheckable(True)  # Enables toggle behavior
            button.setChecked(False)

            button.clicked.connect(lambda _, p=particle: self.button_pressed_cb(p))

            # Add button to row layout
            row_layout.addWidget(button)

            # Create LineEdits and Add to row layout
            self.text_inputs[particle] = {}
            for axis in ['x', 'y', 'z']:
                label = qt_widgets.QLabel(axis + ':')
                line_edit = qt_widgets.QLineEdit()
                line_edit.setText(str(rospy.get_param("~initial_position_" + axis, 0.0)))
                row_layout.addWidget(label)
                row_layout.addWidget(line_edit)
                self.text_inputs[particle][axis] = line_edit
            

            # Create Set Position button
            set_pos_button = qt_widgets.QPushButton()
            set_pos_button.setText("Set Position for Particle " + str(particle))
            set_pos_button.clicked.connect(lambda _, p=particle: self.set_position_cb(p))
            row_layout.addWidget(set_pos_button)

            # Add row layout to the main layout
            self.layout.addLayout(row_layout)
            
            self.buttons[particle] = button
            self.odom_publishers[particle] = rospy.Publisher(self.odom_topic_prefix + str(particle), Odometry, queue_size=1)
            self.odom_publishers_delta_x[particle] = rospy.Publisher(self.odom_topic_prefix + "x_" + str(particle), Odometry, queue_size=1)
            self.odom_publishers_delta_y[particle] = rospy.Publisher(self.odom_topic_prefix + "y_" + str(particle), Odometry, queue_size=1)
            self.odom_publishers_delta_z[particle] = rospy.Publisher(self.odom_topic_prefix + "z_" + str(particle), Odometry, queue_size=1)


        # Create Pause and Reset Buttons
        self.pause_button = qt_widgets.QPushButton("Start Fabric Controller", self)
        self.reset_button = qt_widgets.QPushButton("Reset Fabric Positions", self)

        # Connect buttons to slots
        self.pause_button.clicked.connect(self.pause_controller)
        self.reset_button.clicked.connect(self.reset_positions)

        # Set the pause button as checkable
        self.pause_button.setCheckable(True)

        self.layout.addWidget(self.pause_button)
        self.layout.addWidget(self.reset_button)
            
        self.setLayout(self.layout)

        self.shutdown_timer.timeout.connect(self.check_shutdown)
        self.shutdown_timer.start(1000)  # Timer triggers every 1000 ms (1 second)

    def set_position_cb(self, particle):
        pose = Pose()
        pose.position.x = float(self.text_inputs[particle]['x'].text())
        pose.position.y = float(self.text_inputs[particle]['y'].text())
        pose.position.z = float(self.text_inputs[particle]['z'].text())


        # Prepare Odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map" 
        odom.pose.pose = pose

        self.odom_publishers[particle].publish(odom)


    def odom_pub_timer_callback(self,event):
        # Do not proceed until the initial values have been set
        if not self.initial_values_set:
            return
        
        for particle in self.particles:
            frame_id = "map"

            dt = self.get_timestep(particle)   
            # dt = 0.01
            if self.buttons[particle].isChecked():
                # simple time step integration using Twist data
                pose = Pose()
                pose.position.x = self.particle_positions[particle].x + dt*self.spacenav_twist.linear.x
                pose.position.y = self.particle_positions[particle].y + dt*self.spacenav_twist.linear.y
                pose.position.z = self.particle_positions[particle].z + dt*self.spacenav_twist.linear.z

                # Prepare Odometry message
                odom = Odometry()
                odom.header.stamp = rospy.Time.now()
                odom.header.frame_id = frame_id
                odom.pose.pose = pose
                self.odom_publishers[particle].publish(odom)

                if (particle == self.leader_particle) or (particle in self.binded_particles):
                    self.odom_publishers_delta_x[particle].publish(odom)
                    self.odom_publishers_delta_y[particle].publish(odom)
                    self.odom_publishers_delta_z[particle].publish(odom)

            
                for other_particle in self.particles:
                    # if other_particle != particle:
                    if (other_particle != self.leader_particle) and (other_particle not in self.binded_particles):
                        t_now = rospy.Time.now()
                        

                        # Prepare Odometry message
                        odom = Odometry()
                        odom.header.stamp = t_now
                        odom.header.frame_id = frame_id 
                        
                        # ----------------------------------------------------------------------------------------
                        pose_delta_x = Pose()
                        pose_delta_x.position.x = self.particle_positions[other_particle].x + self.delta_x
                        pose_delta_x.position.y = self.particle_positions[other_particle].y 
                        pose_delta_x.position.z = self.particle_positions[other_particle].z 

                        odom.pose.pose = pose_delta_x
                        self.odom_publishers_delta_x[other_particle].publish(odom)
                        # ----------------------------------------------------------------------------------------
                        pose_delta_y = Pose()
                        pose_delta_y.position.x = self.particle_positions[other_particle].x # + self.delta_x
                        pose_delta_y.position.y = self.particle_positions[other_particle].y + self.delta_y
                        pose_delta_y.position.z = self.particle_positions[other_particle].z # + self.delta_z

                        odom.pose.pose = pose_delta_y
                        self.odom_publishers_delta_y[other_particle].publish(odom)
                        # ----------------------------------------------------------------------------------------
                        pose_delta_z = Pose()
                        pose_delta_z.position.x = self.particle_positions[other_particle].x # + self.delta_x
                        pose_delta_z.position.y = self.particle_positions[other_particle].y # + self.delta_y
                        pose_delta_z.position.z = self.particle_positions[other_particle].z + self.delta_z

                        odom.pose.pose = pose_delta_z
                        self.odom_publishers_delta_z[other_particle].publish(odom)


    def spacenav_twist_callback(self, twist):
        self.spacenav_twist = twist

    def marker_callback(self, marker):
        if marker.type == Marker.POINTS:
            for particle in self.particles:
                self.particle_positions[particle] = marker.points[particle]

        self.initial_values_set = True  # Initialization state variable

    def button_pressed_cb(self, particle):
        if self.buttons[particle].isChecked():
            # Button is currently pressed, no need to set it to False
            print(f"Button for particle {particle} is now pressed.")
        else:
            # Button is currently not pressed, no need to set it to True
            print(f"Button for particle {particle} is now NOT pressed.")    

    def get_timestep(self, integrator_name):
        current_time = rospy.Time.now().to_time()
        if integrator_name in self.last_timestep_requests:
            dt = current_time - self.last_timestep_requests[integrator_name]
            self.last_timestep_requests[integrator_name] = current_time
            if dt > MAX_TIMESTEP:
                dt = 0.0
            return dt
        else:
            self.last_timestep_requests[integrator_name] = current_time
            return 0.0

    def check_shutdown(self):
        if rospy.is_shutdown():
            qt_widgets.QApplication.quit()

    # Define slots
    def pause_controller(self):
        # Call pause/resume service
        service_name = '/fabric_position_controller_node/set_enable'

        rospy.wait_for_service(service_name)
        try:
            set_enable_service = rospy.ServiceProxy(service_name, SetBool)
            set_enable_service(self.pause_button.isChecked())  # Pass the button state as the service argument
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def reset_positions(self):
        # Call reset positions service
        service_name = '/fabric_position_controller_node/reset_positions'
        rospy.wait_for_service(service_name)
        try:
            reset_positions_service = rospy.ServiceProxy(service_name, Empty)
            reset_positions_service()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('fake_odom_publisher_gui_node', anonymous=False)

    app = qt_widgets.QApplication(sys.argv)

    gui = FakeOdomPublisherGUI()
    gui.show()

    sys.exit(app.exec_())
