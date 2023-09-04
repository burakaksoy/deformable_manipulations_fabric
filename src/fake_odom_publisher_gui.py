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

import math
# import tf.transformations as transformations

# Velocity commands will only be considered if they are spaced closer than MAX_TIMESTEP
MAX_TIMESTEP = 0.1

class FakeOdomPublisherGUI(qt_widgets.QWidget):
    def __init__(self):
        super(FakeOdomPublisherGUI, self).__init__()
        self.shutdown_timer = qt_core.QTimer()

        self.pub_rate_odom = rospy.get_param("~pub_rate_odom", 50)

        self.initial_values_set = False  # Initialization state variable

        self.particles = None
        self.follower_particles = None
        self.odom_topic_prefix = None
        while (not self.particles) or (not self.follower_particles):
            try:
                self.particles = rospy.get_param("/custom_static_particles")
                self.follower_particles = rospy.get_param("/follower_particles") 
                self.odom_topic_prefix = rospy.get_param("/custom_static_particles_odom_topic_prefix")
            except:
                rospy.logwarn("No particles obtained from ROS parameters. GUI will be empty.")
                time.sleep(0.5)

        self.odom_publishers = {}

        self.particle_positions = {}

        self.createUI()
        
        self.spacenav_twist = Twist() # set it to an empty twist message

        self.sub_twist = rospy.Subscriber("/spacenav/twist", Twist, self.spacenav_twist_callback, queue_size=10)
        self.sub_marker = rospy.Subscriber("/fabric_points", Marker, self.marker_callback, queue_size=10)

        self.last_timestep_requests = {}

        self.odom_pub_timer = rospy.Timer(rospy.Duration(1. / self.pub_rate_odom), self.odom_pub_timer_callback)
        

    def createUI(self):
        self.layout = qt_widgets.QVBoxLayout(self)

        self.buttons = {}
        self.pause_controller_buttons = {}
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

                if axis == 'x':
                    line_edit.setText(str(rospy.get_param("~initial_position_" + axis, 0.0)))
                elif axis == 'y':
                    line_edit.setText(str(rospy.get_param("~initial_position_" + axis, 0.0)))
                elif axis == 'z':
                    line_edit.setText(str(rospy.get_param("~initial_position_" + axis, 2.0)))

                row_layout.addWidget(label)
                row_layout.addWidget(line_edit)
                self.text_inputs[particle][axis] = line_edit
            

            # Create Set Position button
            set_pos_button = qt_widgets.QPushButton()
            set_pos_button.setText("Set Position")
            set_pos_button.clicked.connect(lambda _, p=particle: self.set_position_cb(p))
            row_layout.addWidget(set_pos_button)

            # Add button to reset all trails
            clear_trails_button = qt_widgets.QPushButton()
            clear_trails_button.setText("Clear Trail")
            clear_trails_button.clicked.connect(lambda _, p=particle: self.clear_trails_cb(p))
            row_layout.addWidget(clear_trails_button)

            # Add a separator vertical line here
            separator = qt_widgets.QFrame()
            separator.setFrameShape(qt_widgets.QFrame.VLine)
            separator.setFrameShadow(qt_widgets.QFrame.Sunken)
            row_layout.addWidget(separator)

            # Create Pause Controller Button
            pause_button = qt_widgets.QPushButton()
            pause_button.setText("Start Controller")
            if particle in self.follower_particles:
                pause_button.clicked.connect(lambda _, p=particle: self.pause_controller_cb(p))
                pause_button.setCheckable(True) # Set the pause button as checkable
            else:
                pause_button.setEnabled(False)  # Disable the button
            row_layout.addWidget(pause_button)

            # Create Reset DLO positions button
            reset_button = qt_widgets.QPushButton()
            reset_button.setText("Reset Controller Position")
            if particle in self.follower_particles:
                reset_button.clicked.connect(lambda _, p=particle: self.reset_position_cb(p))
            else:
                reset_button.setEnabled(False)  # Disable the button
            row_layout.addWidget(reset_button)

            # Add row layout to the main layout
            self.layout.addLayout(row_layout)
            self.buttons[particle] = button
            self.pause_controller_buttons[particle] = pause_button
            self.odom_publishers[particle] = rospy.Publisher(self.odom_topic_prefix + str(particle), Odometry, queue_size=10)
            
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

    def reset_position_cb(self, particle):
        # Call reset positions service
        service_name = '/fabric_velocity_controller_' + str(particle)+ '/reset_positions'
        rospy.wait_for_service(service_name, timeout=2.0)
        try:
            reset_positions_service = rospy.ServiceProxy(service_name, Empty)
            reset_positions_service()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def pause_controller_cb(self, particle):
        # Call pause/resume service
        service_name = '/fabric_velocity_controller_' + str(particle)+ '/set_enable'

        rospy.wait_for_service(service_name, timeout=2.0)
        try:
            set_enable_service = rospy.ServiceProxy(service_name, SetBool)
            set_enable_service(self.pause_controller_buttons[particle].isChecked())  # Pass the button state as the service argument
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def clear_trails_cb(self, particle):
        # Call reset positions service
        service_name = '/reset_trail_service_' + str(particle)
        rospy.wait_for_service(service_name, timeout=2.0)
        try:
            reset_trail_service = rospy.ServiceProxy(service_name, Empty)
            reset_trail_service()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def odom_pub_timer_callback(self,event):
        # Do not proceed until the initial values have been set
        if not self.initial_values_set:
            return
        
        for particle in self.particles:
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
                odom.header.frame_id = "map"
                odom.pose.pose = pose
                self.odom_publishers[particle].publish(odom)

    def spacenav_twist_callback(self, twist):
        self.spacenav_twist.linear.x = twist.linear.x 
        self.spacenav_twist.linear.y = twist.linear.y
        self.spacenav_twist.linear.z = twist.linear.z
        self.spacenav_twist.angular.x = 0.0 # twist.angular.x # because we don't care particle orientations
        self.spacenav_twist.angular.y = 0.0 # twist.angular.y # because we don't care particle orientations
        self.spacenav_twist.angular.z = 0.0 # twist.angular.z # because we don't care particle orientations
        
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


if __name__ == "__main__":
    rospy.init_node('fake_odom_publisher_gui_node', anonymous=False)

    app = qt_widgets.QApplication(sys.argv)

    gui = FakeOdomPublisherGUI()
    gui.show()

    sys.exit(app.exec_())
