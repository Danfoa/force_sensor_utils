#!/usr/bin/env python

import rospy
import actionlib
from actionlib.action_client import get_name_of_constant

from force_sensor_utils import ForceTorqueMonitor

from force_sensor_utils.msg import AxisRange
from force_sensor_utils.msg import ForceTorqueMonitoringAction 
from force_sensor_utils.msg import ForceTorqueMonitoringFeedback
from force_sensor_utils.msg import ForceTorqueMonitoringResult
from force_sensor_utils.msg import ForceTorqueMonitoringGoal

def forceDetected(goal_status, result):
  rospy.loginfo("Force torque monitoring result received:\n")
  rospy.loginfo("Goal Status: " + str(goal_status))
  rospy.logwarn("Result: \n" + str(result))


if __name__ == "__main__":

    rospy.init_node('force_monitoring_action_client')

    action_name =  rospy.get_param('~action_name', 'ForceTorqueMonitoring') 

    client = actionlib.SimpleActionClient(action_name, ForceTorqueMonitoringAction )
    rospy.loginfo("Waiting for Action server with action name: " + action_name)
    client.wait_for_server()
    rospy.loginfo("Connected \n")
    
    # Define detection goal.
   
    goal = ForceTorqueMonitoringGoal()
    goal.plot = True
    goal.timeout = 5 # [s]
    goal.min_detection_time = 1 # [s] Inmediate detection.

    # Define detection axes
    # Equivalent force -----------------------------------
    axis_to_monitor = AxisRange()
    axis_to_monitor.axis_name = AxisRange.EQUIVALENT_FORCE
    axis_to_monitor.max_value = 60
    axis_to_monitor.min_value = AxisRange.DISABLED
    goal.active_axes.append(axis_to_monitor)
    # FZ -------------------------------------------------
    axis_to_monitor = AxisRange()
    axis_to_monitor.axis_name = AxisRange.FZ
    axis_to_monitor.max_value = 20
    axis_to_monitor.min_value = -20
    goal.active_axes.append(axis_to_monitor)

    # Send goal 
    client.send_goal(goal, forceDetected)
    rospy.loginfo("Goal sended")

    client.wait_for_result()

    goal.timeout = 10
    goal.min_detection_time = 0
    client.send_goal(goal, forceDetected)
    client.wait_for_result()

    goal.timeout = AxisRange.DISABLED
    goal.min_detection_time = 0
    client.send_goal(goal, forceDetected)
    client.wait_for_result()
  
    rospy.spin()
    
    
