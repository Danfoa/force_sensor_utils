#!/usr/bin/env python

import rospy
import actionlib


from force_sensor_utils import ForceTorqueMonitor

from force_sensor_utils.msg import AxisRange
from force_sensor_utils.msg import ForceTorqueMonitoringAction 
from force_sensor_utils.msg import ForceTorqueMonitoringFeedback
from force_sensor_utils.msg import ForceTorqueMonitoringResult
from force_sensor_utils.msg import ForceTorqueMonitoringGoal

def forceDetected(result):
  rospy.logwarn("Force was detected")


if __name__ == "__main__":

    rospy.init_node('force_monitoring_action_client')

    action_name =  rospy.get_param('~action_name', 'ForceTorqueMonitoring') 

    client = actionlib.SimpleActionClient(action_name, ForceTorqueMonitoringAction )
    rospy.logwarn("Waiting for Action server with action name: " + action_name)
    client.wait_for_server()
    rospy.logwarn("Connected")
    
    # Define detection goal.
    goal = ForceTorqueMonitoringGoal()
    goal.plot = True
    goal.timeout = 15 # [s]
    goal.min_detection_time = 0 # [s] Inmediate detection.

    axis_to_monitor = AxisRange()
    axis_to_monitor.axis_name = AxisRange.EQUIVALENT_FORCE
    axis_to_monitor.max_value = 60
    axis_to_monitor.min_value = AxisRange.DISABLED
    goal.active_axes.append(axis_to_monitor)

    client.send_goal(goal, forceDetected)
    rospy.logwarn("Goal sended")

    client.wait_for_result()

  
    rospy.spin()
    
    
