#!/usr/bin/env python3

import rospy
import json

def save_parameters_to_file():
    rospy.init_node('save_parameters_node', anonymous=True)

    # Get the list of all parameter names
    all_param_names = rospy.get_param_names()

    # Initialize a dictionary to hold parameter name-value pairs
    params_dict = {}

    # Loop through all parameter names and retrieve their values
    for param_name in all_param_names:
        param_value = rospy.get_param(param_name)
        params_dict[param_name] = param_value

    # Save to a JSON file
    with open('ros_parameters.json', 'w') as f:
        json.dump(params_dict, f, indent=4)

    rospy.loginfo("Saved all parameters to ros_parameters.json")

if __name__ == '__main__':
    try:
        save_parameters_to_file()
    except rospy.ROSInterruptException:
        pass
