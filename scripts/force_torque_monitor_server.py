#!/usr/bin/env python

import rospy
from force_sensor_utils import ForceTorqueMonitor



if __name__ == "__main__":

    rospy.init_node('force_monitoring_action_server')

    action_name =  rospy.get_param('~action_name', 'ForceTorqueMonitoring') 
    wrench_topic = rospy.get_param('~input_topic', '?')

    if wrench_topic == '?':
      rospy.logfatal("No valid `input_topic` parameter, please provide the topic for WrenchStamped messages")
      rospy.signal_shutdown("No input topic given")
    else: 
      monitor = ForceTorqueMonitor(action_name, wrench_topic)
      rospy.loginfo("Monitoring started...")



    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
      rate.sleep()
    rospy.spin()
    
    
