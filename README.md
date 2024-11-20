# 陈奕浩作业

## 播放截图

![截图](run-pic.png)

## bag的属性

cyh@LAPTOP-C6VQVV9I:/mnt/c/Users/chenyhao/Desktop/cyh$ rosbag info jiantu2.
bag 
path:        jiantu2.bag
version:     2.0
duration:    1:43s (103s)
start:       Sep 21 2024 06:52:48.60 (1726872768.60)
end:         Sep 21 2024 06:54:32.41 (1726872872.41)
size:        36.9 MB
messages:    91348
compression: none [48/48 chunks]
types:       nav_msgs/Odometry         [cd5e73d190d741a2f92e81eda573aca7]
             sensor_msgs/Imu           [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/LaserScan     [90c7ef2dc6895d81024acba2ac42f369]
             sensor_msgs/MagneticField [2f3b0b43eed0c9501de0fa3ff89a45aa]
topics:      /driver/encoder    4758 msgs    : nav_msgs/Odometry        
             /driver/eul       28635 msgs    : sensor_msgs/Imu          
             /driver/imu       28469 msgs    : sensor_msgs/Imu          
             /driver/mag       28469 msgs    : sensor_msgs/MagneticField
             /driver/scan       1017 msgs    : sensor_msgs/LaserScan
cyh@LAPTOP-C6VQVV9I:/mnt/c/Users/chenyhao/Desktop/cyh$ 

### ROS话题内容细节

cyh@LAPTOP-C6VQVV9I:/mnt/c/Users/chenyhao/Desktop/cyh$ rosmsg show nav_msgs/Odometry
sgs/Imu
rosmsg show sensor_msgs/MagneticField
rosmsg show sensor_msgs/LaserScanstd_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance

cyh@LAPTOP-C6VQVV9I:/mnt/c/Users/chenyhao/Desktop/cyh$ rosmsg show sensor_msgs/Imu
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
  float64 x
  float64 y
  float64 z
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
  float64 x
  float64 y
  float64 z
float64[9] linear_acceleration_covariance

cyh@LAPTOP-C6VQVV9I:/mnt/c/Users/chenyhao/Desktop/cyh$ rosmsg show sensor_msgs/MagneticField
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Vector3 magnetic_field
  float64 x
  float64 y
  float64 z
float64[9] magnetic_field_covariance

cyh@LAPTOP-C6VQVV9I:/mnt/c/Users/chenyhao/Desktop/cyh$ rosmsg show sensor_msgs/LaserScan
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities

cyh@LAPTOP-C6VQVV9I:/mnt/c/Users/chenyhao/Desktop/cyh$ 