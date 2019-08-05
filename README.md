# Force Sensor Utils 

This package contains utility functions for the processing and handling of
`geometry_msgs/WrenchStamped` and `geometry_msgs/Wrench` messages coming from any 6DoF force torque
sensor.

The functionalities developed and planned are:

- [x] Real-time plotting of force and torque values (per axis and equivalent e.g. `Fx`, `Fz`, `F_resultant`).
- [x] Force/torque monitoring: Detection of threshold violation of any force/torque data axis.
- [x] Transformation in space of `geometry_msgs/WrenchStamped` messages using `TF` data between tf frames.
- [ ] Addition/subtraction of `geometry_msgs/Wrench` messages.
- [ ] Real-time robot load estimation using URDF links inertia and mass parameters.
- [ ] Provide force/torque (robot agnostic) control functionalities.
- [ ] RQT plugin for Rviz for plot visualization

*New ideas and contributions welcomed*

## Contents

  * [Real-time plotting and monitoring of force/torque](#real-time-plotting-and-monitoring-of-forcetorque)
  * [Space transformation of `WrenchStamped` messages](#space-transformation-of-wrenchstamped-messages)
_________________

## Real-time plotting and monitoring of force/torque


The real-time plotting and threshold monitoring are combined into a single node which once launched
starts an action server for the monitoring functionality on request.

<p align="center" >
  <img  height="500"  src="https://user-images.githubusercontent.com/8356912/62467172-64291400-b793-11e9-9f4c-50d278ab4509.png">
</p>

### Node: `force_torque_monitor_server.py`

This node starts:

1. A matplotlib based real-time plot of the force and torque values of a
`geometry_msgs/WrenchStamped` message and the resultant force and torque values.
2. A `force_sensor_utils/ForceTorqueMonitoring` action server that on request starts monitoring the
   threshold violation of specific axes of the incoming Wrench messages.

### Parameters: 
* `input_topic`: Name of the `geometry_msgs/WrenchStamped` topic to monitor/plot. 
* `action_name`: Name of the `force_sensor_utils/ForceTorqueMonitoring` action to publish and
  connect.
* `plot_title`: (optional) Main title from the force/torque plot, by default is the `input_topic`


### `force_sensor_utils/ForceTorqueMonitoring` action 

The `ForceTorqueMonitoring` action allows for the detection of threshold/bounds violation of force
and torque values. For example, you could detect if the force in the X axis turns greater than 5
Newton or lower than -15 Newtons.

For more information on the action goal, feedback, and result fields see the [ForceTorqueMonitoring.action](https://github.com/invite-robotics/force-sensor-utils/blob/kinetic-devel/action/ForceTorqueMonitoring.action) declaration.

**Goal example**
```
// Create Axis range instance indicating which axis to monitor
force_sensor_utils::AxisRange force_range; 
force_range.axis_name = force_range.EQUIVALENT_FORCE; // Monitor the resultant force 
force_range.max_value = 10 N;                         // Check if resultant force is greater than 10 N          
force_range.min_value = force_range.DISABLED;         // Do not check lower bound/threshold

force_sensor_utils::ForceTorqueMonitoringGoal detection_goal;
detection_goal.active_axes.push_back(force_range);    // Add axis/axes to monitor (can be multiple)
detection_goal.min_detection_time = 0.2;              // Trigger when any of the active axes violates its bounds for more than 0.2 seconds (avoid triggering by noise)
detection_goal.timeout = 10;                          // Monitor resultant force for 10 seconds 

YOUR_CLIENT_INSTANCE->sendGoal(detection_goal, &YOUR_CALLBACK);
```
______________
## Space transformation of `WrenchStamped` messages 

The space transformation of `geometry_msgs/WrenchStamped` messages is done with a `WrenchStamped`
implementation of a
[`tf2_ros::MessageFilter`](http://wiki.ros.org/tf2/Tutorials/Using%20stamped%20datatypes%20with%20tf2%3A%3AMessageFilter)
which uses the `TF` buffer to transform between two monitored `tf axes` (i.g. Your robot links
coordinate frames). For example, you can transform the sensed force in the wrist of your robot to
your world/base frame.

Note: The transformation takes into account only the change in reference frame, i.g. it is
homogenous transformation of the force and torque vectors, no
resultant moment is computed from the translation of the sensed force from the `origin_frame` to the
`target_frame`. (To-Do: Consider if this functionality should be enabled with an optional parameter)

<p align="center" >
  <img  height="450"  src="https://user-images.githubusercontent.com/8356912/62467171-63907d80-b793-11e9-80db-b16d12a512bb.png">
</p>

### Node: `wrench_transformer`

### Subscribed topic:

* `wrench_input` [`geometry_msgs/WrenchStamped`]: Topic containing the
  `geometry_msgs/WrenchStamped` input messages. The origin frame for the transformation is taken
  from the header `frame_id` field of each message.

### Published topic: 

* `wrench_transformed` [`geometry_msgs/WrenchStamped`]: Topic containing the
  `geometry_msgs/WrenchStamped` output/transformed messages, with a stamp `frame_id` equals to the `target_frame`.

### Parameters

* `target_frame`: Name of the TF transformation target frame.
   

