#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

class KeyPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('key_publisher')

        # Initialize publisher
        self.pub = rospy.Publisher('/spacenav/twist', Twist, queue_size=1)

        # Get velocities from parameters
        self.vel_x = rospy.get_param('~vel_x', 1.0)
        self.vel_y = rospy.get_param('~vel_y', 1.0)
        self.vel_z = rospy.get_param('~vel_z', 1.0)

        # Initialize Twist message
        self.twist = Twist()

        # Initialize key press dict
        self.key_press = {"q": 0, "a": 0, "w": 0, "s": 0, "e": 0, "d": 0}

        # Initialize the listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)

    def on_press(self, key):
        try:
            self.key_press[str(key.char)] = 1
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            self.key_press[str(key.char)] = 0
        except AttributeError:
            pass

    def update_twist(self):
        self.twist.linear.x = (self.key_press["q"] - self.key_press["a"]) * self.vel_x
        self.twist.linear.y = (self.key_press["w"] - self.key_press["s"]) * self.vel_y
        self.twist.linear.z = (self.key_press["e"] - self.key_press["d"]) * self.vel_z

    def publish_twist(self):
        self.update_twist()
        self.pub.publish(self.twist)

    def run(self):
        # Start the keyboard listener
        self.listener.start()

        # Set up rate
        rate = rospy.Rate(100) # 10Hz

        while not rospy.is_shutdown():
            self.publish_twist()
            rate.sleep()

        # Stop the keyboard listener
        self.listener.stop()

if __name__ == "__main__":
    try:
        KeyPublisher().run()
    except rospy.ROSInterruptException:
        pass
