#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import math

class OdometryToPoseConverter:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('odometry_to_pose_node', anonymous=True)
        
        # Store latest odometry data (thread-safe for 2Hz printing)
        self.latest_odom = None
        
        # QoS profile matching typical Odometry topics
        self.odom_sub = rospy.Subscriber(
            '/Odometry',  # Input Odometry topic
            Odometry,
            self.odom_callback,
            queue_size=10
        )
        
        # PoseStamped publisher
        self.pose_pub = rospy.Publisher(
            '/mavros/vision_pose/pose',  # Output PoseStamped topic
            PoseStamped,
            queue_size=10
        )
        
        # 2Hz timer for printing XYZ + yaw
        self.print_timer = rospy.Timer(
            rospy.Duration(1.0 / 2.0),  # 0.5s interval = 2Hz
            self.print_pose_data
        )
        
        rospy.loginfo("Odometry to Pose converter started.")

    def odom_callback(self, msg):
        """Callback for /Odometry subscription"""
        # Update latest odometry data
        self.latest_odom = msg
        
        # Convert Odometry to PoseStamped
        pose_msg = self.convert_odom_to_pose(msg)
        
        # Publish PoseStamped message
        self.pose_pub.publish(pose_msg)

    def convert_odom_to_pose(self, odom_msg):
        """Convert Odometry message to PoseStamped"""
        pose = PoseStamped()
        
        # Copy header (timestamp and frame ID)
        pose.header = odom_msg.header
        # You might want to adjust the frame_id if needed
        # pose.header.frame_id = "map"  # or whatever frame your system uses
        
        # Copy position from Odometry
        pose.pose.position.x = odom_msg.pose.pose.position.x
        pose.pose.position.y = odom_msg.pose.pose.position.y
        pose.pose.position.z = odom_msg.pose.pose.position.z
        
        # Copy orientation from Odometry
        pose.pose.orientation = odom_msg.pose.pose.orientation
        
        return pose

    def print_pose_data(self, event):        
        """Print XYZ position and yaw angle at 2Hz"""
        if self.latest_odom is None:
            rospy.loginfo("Waiting for odometry data...")
            return
        
        # Extract XYZ position from odometry pose
        x = self.latest_odom.pose.pose.position.x
        y = self.latest_odom.pose.pose.position.y
        z = self.latest_odom.pose.pose.position.z
        
        # Extract quaternion and convert to Euler angles (roll, pitch, yaw)
        quat = [
            self.latest_odom.pose.pose.orientation.x,
            self.latest_odom.pose.pose.orientation.y,
            self.latest_odom.pose.pose.orientation.z,
            self.latest_odom.pose.pose.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(quat)
        
        # Convert yaw to degrees (optional, comment out for radians)
        yaw_deg = math.degrees(yaw)
        
        # Print with 2 decimal places for readability
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
        converter = OdometryToPoseConverter()
        # Spin to keep node alive
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Odometry to Pose converter stopped.")
        pass