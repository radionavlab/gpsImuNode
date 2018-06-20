# gpsImuNode
Kalman Filter with GPS/INS integration.  Specifically designed to interface with pprx via ppengineros.  IMU messages are formatted based on gbx_ros_bridge_msgs.

# Structure:
The main ROS node creates a filterImu object and subscribers+callbacks for IMU and GPS measurements.  The system is currently designed for the Lynx as mounted on the MG quads.  Measurements are corrected to GPS time from RRT and turned into imuMeas objects.  The filter propagates whenever an imu measurement is recorded.  ROS-side callbacks are in gpsCallbacks.  The main class containing publishers/subscribers is in estimationNode.cpp.

GPS messages come from two different topics.  The callbacks to the two subscribers share data and only create a gpsMeas object once both measurements have been received that is then processed.  A UKF is used for both propagation and measurement.

Currently does not use a buffer for processing out-of-order messages; instead, the message time is used and older messages are rejected.

Time offsets are based on system time compared to GPS time (UTC).  This introduces small errors due to serialization/deserialization and transmission via UDP/TCP from the Lynx to the Snapdragon.  A wired connection should reduce the effects of these issues.  Alternatively, Chrony can be used for clock synchronization.

Depends on gbx_ros_bridge_messages for IMU/GPS message types.  Other dependencies are standard ros packages.