# gpsImuNode
Kalman Filter with GPS/INS integration.  Specifically designed to interface with pprx via ppengineros.  IMU messages are formatted based on gbx_ros_bridge_msgs.

# Structure:
The main ROS node creates a filterImu object and subscribers+callbacks for IMU and GPS measurements.  The system is currently designed for the Lynx as mounted on the MG quads.  Measurements are corrected to GPS time from RRT and turned into imuMeas objects.  The filter propagates whenever an imu measurement is recorded.

GPS messages come from two different topics.  The callbacks to the two subscribers share data and only create a gpsMeas object once both measurements have been received that is then processed.  A UKF is used for both propagation and measurement.

Math helper functions are duplicated in the ROS class containing publishers/subscribers and in the filterImu class.

