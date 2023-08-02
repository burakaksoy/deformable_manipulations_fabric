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

"""
Author: Burak Aksoy

The fake_odom_publisher_gui_node simulates the publishing of odometry data for 
a set of particles within a ROS network. Utilizing the PyQt5 library, the node
provides a GUI interface which displays a series of buttons representing each 
particle, allowing the user to manually toggle the publishing state of 
individual particles.

Odometry data, which includes only positional data, is 
published based on the current Twist message received from the 
/spacenav/twist topic and the marker data received from the /fabric_points topic. 

A simple time-step integration technique is employed to calculate the new 
position of the particle in the 3D based on the twist linear velocities. 

The node also implements a rospy Timer to publish odometry messages at a 
predefined rate (pub_rate_odom) for all the particles which are currently 
selected in the GUI. The Timer callback function checks the state of each 
button in the GUI and publishes odometry data if the button (i.e. the particle)
is selected.

The node continuously checks whether the ROS network is still running. If the 
ROS network is shut down, the node also shuts down its GUI interface, ensuring
a clean exit. Lastly, the node maintains a dictionary of the last time-stamp 
requests to ensure that velocity commands are spaced appropriately, and any 
time-step greater than the maximum permissible time-step is ignored.
"""

# Velocity commands will only be considered if they are spaced closer than MAX_TIMESTEP
MAX_TIMESTEP = 0.1

class FakeOdomPublisherGUI(qt_widgets.QWidget):
    def __init__(self):
        super(FakeOdomPublisherGUI, self).__init__()
        self.shutdown_timer = qt_core.QTimer()

        self.pub_rate_odom = rospy.get_param("~pub_rate_odom", 100)

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

        self.createUI()
        
        self.spacenav_twist = Twist() # set it to an empty twist message

        self.sub_twist = rospy.Subscriber("/spacenav/twist", Twist, self.spacenav_twist_callback, queue_size=1)
        self.sub_marker = rospy.Subscriber("/fabric_points", Marker, self.marker_callback, queue_size=1)

        self.last_timestep_requests = {}

        self.odom_pub_timer = rospy.Timer(rospy.Duration(1. / self.pub_rate_odom), self.odom_pub_timer_callback)
        

    def createUI(self):
        self.layout = qt_widgets.QVBoxLayout(self)

        self.buttons = {}
        for particle in self.particles:
            button = qt_widgets.QPushButton()

            button.setText("Manually Control Particle " + str(particle))

            button.setCheckable(True)  # Enables toggle behavior
            button.setChecked(False)

            button.clicked.connect(lambda _, p=particle: self.button_pressed_cb(p))
            
            self.layout.addWidget(button)
            self.buttons[particle] = button
            self.odom_publishers[particle] = rospy.Publisher(self.odom_topic_prefix + str(particle), Odometry, queue_size=1)

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



    def odom_pub_timer_callback(self,event):
        for particle in self.particles:
            if self.buttons[particle].isChecked():
                dt = self.get_timestep(particle)   
                # dt = 0.01     

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
        self.spacenav_twist = twist

    def marker_callback(self, marker):
        if marker.type == Marker.POINTS:
            for particle in self.particles:
                self.particle_positions[particle] = marker.points[particle]


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
