import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, MagneticField

def odometry_callback(data):
    rospy.loginfo("Received Odometry message: Position x: %f, y: %f, z: %f",
                  data.pose.pose.position.x,
                  data.pose.pose.position.y,
                  data.pose.pose.position.z)
    # 可以根据需要添加更多字段的打印

def imu_callback(data):
    rospy.loginfo("Received Imu message: Linear acceleration x: %f, y: %f, z: %f",
                  data.linear_acceleration.x,
                  data.linear_acceleration.y,
                  data.linear_acceleration.z)
    # 可以根据需要添加更多字段的打印，例如角速度等

def laserscan_callback(data):
    rospy.loginfo("Received LaserScan message: Number of ranges: %d", len(data.ranges))
    # 可以打印第一个范围或平均值等作为示例
    if data.ranges:
        rospy.loginfo("First range: %f", data.ranges[0])

def magfield_callback(data):
    rospy.loginfo("Received MagneticField message: Magnetic field strength x: %f, y: %f, z: %f",
                  data.magnetic_field.x,
                  data.magnetic_field.y,
                  data.magnetic_field.z)

def listener():
    rospy.init_node('listener_node', anonymous=True)
    rospy.Subscriber("/driver/encoder", Odometry, odometry_callback)
    rospy.Subscriber("/driver/eul", Imu, imu_callback)
    rospy.Subscriber("/driver/imu", Imu, imu_callback)  # 注意：通常不会同时订阅两个IMU话题，除非有特定需求
    rospy.Subscriber("/driver/scan", LaserScan, laserscan_callback)
    rospy.Subscriber("/driver/mag", MagneticField, magfield_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()