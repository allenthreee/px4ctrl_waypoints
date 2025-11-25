#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class PoseToOdometryConverter:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('pose_to_odometry_node', anonymous=True)
        
        # Store latest pose data (thread-safe for 2Hz printing)
        self.latest_pose = None
        self.latest_header = None
        
        # QoS profile matching MAVROS default (Best Effort for vision pose)
        self.pose_sub = rospy.Subscriber(
            '/mavros/vision_pose/pose',
            PoseStamped,
            self.pose_callback,
            queue_size=10
        )
        
        # Odometry publisher
        self.odom_pub = rospy.Publisher(
            '/Odometry',  # Output Odometry topic
            Odometry,
            queue_size=10
        )
        
        # 2Hz timer for printing XYZ + yaw
        self.print_timer = rospy.Timer(
            rospy.Duration(1.0 / 2.0),  # 0.5s interval = 2Hz
            self.print_odometry_data
        )
        
        rospy.loginfo("Pose to Odometry converter started.")

    def pose_callback(self, msg):
        """Callback for /mavros/vision_pose/pose subscription"""
        # Update latest pose data
        self.latest_pose = msg.pose
        self.latest_header = msg.header
        
        # Convert PoseStamped to Odometry
        odom_msg = self.convert_pose_to_odom(msg)
        
        # Publish Odometry message
        self.odom_pub.publish(odom_msg)

    def convert_pose_to_odom(self, pose_stamped):
        """Convert PoseStamped to Odometry message"""
        odom = Odometry()
        
        # Copy header (timestamp and frame ID)
        odom.header = pose_stamped.header
        odom.child_frame_id = "base_link"  # Adjust based on your robot's frame
        
        # Copy position from PoseStamped
        odom.pose.pose.position.x = pose_stamped.pose.position.x
        odom.pose.pose.position.y = pose_stamped.pose.position.y
        odom.pose.pose.position.z = pose_stamped.pose.position.z
        
        # Copy orientation from PoseStamped
        odom.pose.pose.orientation = pose_stamped.pose.orientation
        
        # Set covariance matrices (adjust based on your sensor's accuracy)
        # Position covariance (x, y, z, roll, pitch, yaw)
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.001
        ]
        
        # Velocity covariance (set to zero since vision pose doesn't provide velocity)
        odom.twist.covariance = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ]
        
        # Velocity is zero (vision pose doesn't provide velocity data)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        return odom

    def print_odometry_data(self, event):        
        """Print XYZ position and yaw angle at 2Hz"""
        if self.latest_pose is None:
            rospy.loginfo("Waiting for pose data...")
            return
        
        # Extract XYZ position
        x = self.latest_pose.position.x
        y = self.latest_pose.position.y
        z = self.latest_pose.position.z
        
        # Extract quaternion and convert to Euler angles (roll, pitch, yaw)
        quat = [
            self.latest_pose.orientation.x,
            self.latest_pose.orientation.y,
            self.latest_pose.orientation.z,
            self.latest_pose.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(quat)
        
        # Convert yaw to degrees (optional, comment out for radians)
        yaw_deg = math.degrees(yaw)
        
        # Print with 3 decimal places for readability
        # rospy.loginfo(
        #     f"X={x:.2f}, Y={y:.2f}, Z={z:.2f} | "
        #     f"Yaw: {yaw_deg:.2f}°"
        # )
        # 用 print 替代 rospy.loginfo，无时间戳
        print(
            f"X={x:.2f}, Y={y:.2f}, Z={z:.2f} | "
            f"Yaw: {yaw_deg:.2f}°"
        )

if __name__ == '__main__':
    try:
        converter = PoseToOdometryConverter()
        # Spin to keep node alive
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pose to Odometry converter stopped.")
        pass