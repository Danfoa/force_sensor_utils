#!/usr/bin/env python
"""--------------------------------------------------------------------
This Module defines the ForceTorqueMonitor class which provides realtime visualization of 
`geometry_msgs/WrenchStamped` messages via a matplotlib plot, and also provides the functionality to
detect if a given force or torque overpassed a given threshold.

The detection of threshold violation is provided through the actionlib
`ForceTorqueMonitoringAction` `Action` 

@author: Daniel Felipe Ordonez Apraez
@email: daniels.ordonez@gmail.com

@copyright Copyright (c) 2019 INVITE GmbH 
   
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the `Software`), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions: 
   
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
   
  THE SOFTWARE IS PROVIDED `AS IS`, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
--------------------------------------------------------------------\
"""
import rospy
import actionlib
import math
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt
from matplotlib import animation

from multiprocessing import Process

from geometry_msgs.msg import WrenchStamped
from force_sensor_utils.msg import AxisRange
from force_sensor_utils.msg import ForceTorqueMonitoringAction 
from force_sensor_utils.msg import ForceTorqueMonitoringFeedback
from force_sensor_utils.msg import ForceTorqueMonitoringResult
from force_sensor_utils.msg import ForceTorqueMonitoringGoal

TIME = 'Time'
FX = AxisRange.FX
FY = AxisRange.FY
FZ = AxisRange.FZ
EQUIVALENT_FORCE = AxisRange.EQUIVALENT_FORCE
MX = AxisRange.MX
MY = AxisRange.MY
MZ = AxisRange.MZ
EQUIVALENT_TORQUE = AxisRange.EQUIVALENT_TORQUE
#
NOT_TRIGGERED = -1



plot_animation = None

class ForceTorqueMonitor(object):

    def __init__(self, action_name, wrench_topic):
        self._action_name = action_name
        self._wrench_topic = wrench_topic
        self._action_server = actionlib.SimpleActionServer(self._action_name,
                                                           ForceTorqueMonitoringAction,
                                                           execute_cb=self.execute_cb,
                                                           auto_start=False)
        self._action_server.register_preempt_callback(self.preempt_cb)
        self._result = ForceTorqueMonitoringResult()
        self._feedback = ForceTorqueMonitoringFeedback()
        self._active_goal = ForceTorqueMonitoringGoal()

        self._data_buffer = pd.DataFrame(columns=[TIME, 
                                    FX, 
                                    FY, 
                                    FZ, 
                                    EQUIVALENT_FORCE, 
                                    MX, 
                                    MY, 
                                    MZ, 
                                    EQUIVALENT_TORQUE])
        self._min_force, self._max_force = 0, 0                      
        self._min_torque, self._max_torque = 0, 0
        self._user_y_limits = None
        self._user_x_limits = None
        rospy.Subscriber(self._wrench_topic, WrenchStamped, self.wrench_data_update)

        # Wait until gripper driver is ready to take commands.
        # watchdog = rospy.Timer(rospy.Duration(15.0), self._connection_timeout, oneshot=True)
        self._start_time = rospy.get_time()
        self._should_plot = True

        # self._plot_process = Process(target = run_graph)
        # self._plot_process.start()
        # self._plot_process.join()
        self._action_server.start()
        rospy.loginfo("Monitor ready for new commands \n - Action Name: %s \n - Wrench Topic: %s" % (action_name, wrench_topic))
        self._run_graph()
        
    def execute_cb(self, new_goal):
        rospy.loginfo("Force/Torque monitoring goal received")
        # Restart data  buffer and initial time 
        # self._data_buffer
        # self._data_buffer = pd.DataFrame(columns=[TIME, FX, FY, FZ, EQUIVALENT_FORCE, MX, MY, MZ, EQUIVALENT_TORQUE])
        # self._start_time = rospy.get_time()
        # Check if new goal is valid 
        if len(new_goal.active_axes) < 1:
            error_msg = "Empty goal received: ignoring and cancelling all previous goals"
            rospy.logwarn(error_msg)
            self._result.threshold_reached = False
            self._action_server.set_aborted(self._result, error_msg)
            return None
        
        # Accept goal and start monitoring 
        for axis in new_goal.active_axes:
            rospy.loginfo(("Monitoring axis %s with limits Min: %.2f Max: %.2f") % 
                            (axis.axis_name, axis.min_value, axis.max_value))
        self._active_goal = new_goal
        self._initial_detection_times = [NOT_TRIGGERED] * len(self._active_goal.active_axes)

        if self._active_goal.timeout > 0:
            watchdog = rospy.Timer(rospy.Duration(self._active_goal.timeout), 
                                    self._timeout_cb, 
                                    oneshot=True)    

        # Wait until goal is aborted or completed.
        rate  = rospy.Rate(4)  # 2 Hz
        while not rospy.is_shutdown() and self._action_server.is_active() and not self._action_server.is_preempt_requested():
            rate.sleep()
        rospy.loginfo("Goal processed")
    
    def preempt_cb(self):
        error_msg = "New goal received: aborting previous goal"
        rospy.logwarn(error_msg)
        self._result.threshold_reached = False
        self._action_server.set_aborted(self._result, error_msg)

    def wrench_data_update(self, sensor_data):
        # rospy.loginfo("Wrench message received")
        

        if self._action_server.is_active():
            for axis_idx in range(len(self._active_goal.active_axes)):
                axis = self._active_goal.active_axes[axis_idx]
                current_value = get_axis_value(sensor_data, axis.axis_name)
                # Save current value for feedback
                axis.current_value = current_value

                axis_triggered = False
                if axis.max_value != AxisRange.DISABLED:
                    if current_value >= axis.max_value:
                        # Current value is above max threshold ! 
                        axis_triggered = True

                if axis.min_value != AxisRange.DISABLED:
                    if current_value <= axis.min_value:
                        # Current value is below min threshold ! 
                        axis_triggered = True

                if axis_triggered:
                    # Check detection time 
                    if self._initial_detection_times[axis_idx] == NOT_TRIGGERED: 
                        # Initial trigger event 
                        self._initial_detection_times[axis_idx] = rospy.get_time()
                    else:
                        # Consecutive trigger events 
                        if rospy.get_time() - self._initial_detection_times[axis_idx] > self._active_goal.min_detection_time:
                            # Monitoring event triggered 
                            rospy.loginfo("Axis triggered, detection time: %.3f" % (rospy.get_time() - self._initial_detection_times[axis_idx]) )
                            self._result.threshold_reached = True
                            self._result.triggered_axis = axis
                            self._action_server.set_succeeded(self._result)
                            break
                else:
                    self._initial_detection_times[axis_idx] = NOT_TRIGGERED
            # Publish feedback 
            self._feedback.active_axes = self._active_goal.active_axes
            self._action_server.publish_feedback(self._feedback)
        
        if self._should_plot:
            # rospy.loginfo("Updating data " + str(len(time_buffer)))
            latest_data = [rospy.get_time() - self._start_time,
                            sensor_data.wrench.force.x,
                            sensor_data.wrench.force.y,
                            sensor_data.wrench.force.z,
                            get_vector_magnitude(sensor_data.wrench.force),
                            sensor_data.wrench.torque.x,
                            sensor_data.wrench.torque.y,
                            sensor_data.wrench.torque.z,
                            get_vector_magnitude(sensor_data.wrench.torque)
                            ]
        
            idx_labels = [TIME, FX, FY, FZ, EQUIVALENT_FORCE, MX, MY, MZ, EQUIVALENT_TORQUE]
            last_reading = pd.Series(latest_data, idx_labels)

            self._data_buffer = self._data_buffer.append(pd.Series(last_reading), ignore_index = True)

    def _run_graph(self):

        fig, (f_ax, m_ax) = plt.subplots(2, 1, sharex = True)
        f_lines = f_ax.plot([0], [0], 'r-',[0], [0], 'g-', [0], [0], 'b-', [0], [0], 'k-') 
        f_ax.legend(f_lines,(FX, FY, FZ, EQUIVALENT_FORCE))
        m_lines = m_ax.plot([0], [0], 'r-', [0], [0], 'g-', [0], [0], 'b-', [0], [0], 'k-') 
        m_ax.legend(m_lines,(MX, MY, MZ, EQUIVALENT_TORQUE))

        rospy.loginfo("Setting up animated graph")
        # Add labels
        fig.canvas.set_window_title(self._wrench_topic)

        subtitle = rospy.get_param('~plot_title', self._wrench_topic)
        fig.suptitle(subtitle, fontsize="x-large")
        
        f_ax.set_title("Force Feedback")
        f_ax.grid()
        m_ax.set_title("Torque Feedback")
        m_ax.grid()
        f_ax.set(ylabel="Force [N]", xlabel="Time [s]")
        m_ax.set(ylabel="Torque [Nm]", xlabel="Time [s]")
        f_ax.set_xlim(0, 25)
        f_ax.set_ylim(-150, 150)
        m_ax.set_xlim(0, 25)
        m_ax.set_ylim(-15, 15)

        def update_plot(data):
            if data is None:
                return []
            last_timestamp = self._data_buffer[TIME].iloc[-1] if self._data_buffer[TIME].size > 0 else 0 
            # Update time axis 
            xmin, xmax = f_ax.get_xlim()
            if xmax > last_timestamp - 5 and (xmax < last_timestamp or xmax > last_timestamp + 20):
                f_ax.set_xlim(xmin, last_timestamp + 10)
                m_ax.set_xlim(xmin, last_timestamp + 10)
                fig.canvas.draw()

            # Update for lines for forces
            # try: 
            f_lines[0].set_data(self._data_buffer[TIME], self._data_buffer[FX])
            f_lines[1].set_data(self._data_buffer[TIME], self._data_buffer[FY])
            f_lines[2].set_data(self._data_buffer[TIME], self._data_buffer[FZ])
            f_lines[3].set_data(self._data_buffer[TIME], self._data_buffer[EQUIVALENT_FORCE])
            # Update for lines for torques 
            m_lines[0].set_data(self._data_buffer[TIME], self._data_buffer[MX])
            m_lines[1].set_data(self._data_buffer[TIME], self._data_buffer[MY])
            m_lines[2].set_data(self._data_buffer[TIME], self._data_buffer[MZ])
            m_lines[3].set_data(self._data_buffer[TIME], self._data_buffer[EQUIVALENT_TORQUE])
            # except:
                # return []
            return f_lines + m_lines
        def get_plot_data():
            # rospy.loginfo("data update")
            # rospy.loginfo("Time buffer: " + str( time_buffer))
            yield self._data_buffer
        
        plot_animation = animation.FuncAnimation(fig, 
                                update_plot, 
                                get_plot_data, 
                                blit=True, 
                                interval=150)
        plt.show()

    def _timeout_cb(self, event):
        # If there is an active goal, set as failed.
        if self._action_server.is_active():
            error_msg = "Force Torque Monitoring goal aborted: Timeout reached"
            rospy.logwarn(error_msg)
            self._result.threshold_reached = False
            self._action_server.set_aborted(self._result, error_msg)
            


def get_axis_value(sensor_data, axis_name):
    if axis_name == FX :
        return sensor_data.wrench.force.x
    elif axis_name == FY :
        return sensor_data.wrench.force.y
    elif axis_name == FZ :
        return sensor_data.wrench.force.z
    elif axis_name == EQUIVALENT_FORCE :
        return get_vector_magnitude(sensor_data.wrench.force)
    elif axis_name == MX :
        return sensor_data.wrench.torque.x
    elif axis_name == MY :
        return sensor_data.wrench.torque.y
    elif axis_name == MZ :
        return sensor_data.wrench.torque.z
    elif axis_name == EQUIVALENT_TORQUE :
        return get_vector_magnitude(sensor_data.wrench.torque)

def get_vector_magnitude(vector):
    return math.sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2))