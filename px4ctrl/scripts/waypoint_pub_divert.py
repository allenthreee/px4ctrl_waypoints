#!/usr/bin/env python
import rospy
import math
import sys
from geometry_msgs.msg import Point, Vector3, Quaternion
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion  # To convert quaternion to yaw

# ==============================================================================
# CONFIGURATION - Modify these parameters as needed
# ==============================================================================
# Waypoint list: [(x, y, z, yaw), ...] - ENU coordinates (yaw in radians)
WAYPOINTS = [
    (0.0, 0.0, 1.0, 0.0),    # Waypoint 1 (0 rad = 0 deg)
    (-1.0, 0.0, 1.0, 0.0),    # Waypoint 1 (0 rad = 0 deg)
    (-1.0, 1.0, 1.0, 0.0),  # Waypoint 2 (0.785 rad = 45 deg)
    (0.0, 1.0, 1.0, 0.0),   # Waypoint 3 (1.57 rad = 90 deg)
    (0.0, 0.0, 1.0, 0.0)     # Waypoint 4 (0 rad = 0 deg)
]

# Control parameters
POSITION_TOLERANCE = 0.3  # [m] - 3D position tolerance
YAW_TOLERANCE_DEG = 10.0  # [deg] - Yaw angle tolerance (convert to radians below)
WAYPOINT_HOLD_TIME = 3.0  # [s] - Time to stay at each waypoint
PUBLISH_RATE = 10.0       # [Hz] - Publishing frequency

# Convert yaw tolerance to radians (for math calculations)
YAW_TOLERANCE_RAD = math.radians(YAW_TOLERANCE_DEG)

# ROS Topics
ODOM_TOPIC = "/Odometry"  # Current position/yaw feedback
TARGET_TOPIC = "/cmd"  # Topic for PositionCommand
# ==============================================================================

class WaypointFollower:
    def __init__(self, divert=False):
        # Initialize ROS node
        rospy.init_node('waypoint_follower', anonymous=True)
        
        # Control parameters
        self.divert_mode = divert  # Reverse traversal mode
        self.odom_received = False  # Flag to check if odometry data is received
        
        # Current state variables
        self.current_pos = Point()  # Current position (ENU)
        self.current_yaw = 0.0       # Current yaw (radians, ENU)
        self.current_waypoint_idx = 0
        self.waypoint_reached = False
        self.hold_start_time = None
        self.all_waypoints_completed = False
        
        # Initialize target message
        self.target_msg = self.init_target_message()
        
        # Subscribers
        rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_callback)
        
        # Publisher
        self.pub = rospy.Publisher(TARGET_TOPIC, PositionCommand, queue_size=10)
        
        # Publishing rate
        self.rate = rospy.Rate(PUBLISH_RATE)
        
        # Wait for initial odometry data
        rospy.loginfo("Waiting for initial odometry data...")
        while not rospy.is_shutdown() and not self.odom_received:
            rospy.sleep(0.1)
        
        # Initialize waypoint index based on closest waypoint
        self.initialize_waypoint_index()
        
        # Log initialization
        rospy.loginfo("="*50)
        rospy.loginfo("Waypoint Follower Node Initialized")
        rospy.loginfo(f"Message Type: quadrotor_msgs/PositionCommand")
        rospy.loginfo(f"Number of Waypoints: {len(WAYPOINTS)}")
        rospy.loginfo(f"Position Tolerance: {POSITION_TOLERANCE} m")
        rospy.loginfo(f"Yaw Tolerance: {YAW_TOLERANCE_DEG} deg ({YAW_TOLERANCE_RAD:.2f} rad)")
        rospy.loginfo(f"Waypoint Hold Time: {WAYPOINT_HOLD_TIME} s")
        rospy.loginfo(f"Divert Mode: {self.divert_mode}")
        rospy.loginfo(f"Starting from closest waypoint: {self.current_waypoint_idx + 1}")
        rospy.loginfo("="*50)

    def init_target_message(self):
        """Initialize PositionCommand with default values (all 0 except trajectory_id=1)"""
        msg = PositionCommand()
        msg.header.frame_id = 'local_origin'
        
        # Core fields (set to 0 initially)
        msg.position = Point(0.0, 0.0, 0.0)
        msg.velocity = Vector3(0.0, 0.0, 0.0)
        msg.acceleration = Vector3(0.0, 0.0, 0.0)
        msg.jerk = Vector3(0.0, 0.0, 0.0)
        msg.yaw = 0.0
        msg.yaw_dot = 0.0
        
        # Gain matrices
        msg.kx = [0.0, 0.0, 0.0]
        msg.kv = [0.0, 0.0, 0.0]
        
        # Trajectory metadata
        msg.trajectory_id = 1
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        
        return msg

    def odom_callback(self, msg):
        """Update current position (XYZ) and yaw from odometry"""
        # Update position (XYZ)
        self.current_pos = msg.pose.pose.position
        
        # Update yaw (convert quaternion to Euler angles: roll, pitch, yaw)
        quat = msg.pose.pose.orientation  # Quaternion (x, y, z, w)
        roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.current_yaw = yaw  # Yaw in radians (ENU frame)
        
        # Set flag to indicate odometry data is received
        if not self.odom_received:
            self.odom_received = True
            rospy.loginfo("Initial odometry data received")

    def calculate_position_error(self, pos1, pos2):
        """Calculate 3D Euclidean distance between two positions"""
        return math.sqrt(
            (pos1.x - pos2.x)**2 +
            (pos1.y - pos2.y)**2 +
            (pos1.z - pos2.z)**2
        )

    def calculate_yaw_error(self, yaw1, yaw2):
        """Calculate shortest angular difference between two yaw angles (radians)"""
        # Yaw is periodic (0 to 2π), so compute the minimal difference
        error = math.atan2(math.sin(yaw1 - yaw2), math.cos(yaw1 - yaw2))
        return abs(error)  # Return absolute value (0 to π)

    def is_waypoint_reached(self, target_pos, target_yaw):
        """Check if both position and yaw are within tolerance"""
        pos_error = self.calculate_position_error(self.current_pos, target_pos)
        yaw_error = self.calculate_yaw_error(self.current_yaw, target_yaw)
        
        # Log errors (debug for position, info for yaw to avoid clutter)
        rospy.logdebug(f"Position Error: {pos_error:.2f} m | Yaw Error: {math.degrees(yaw_error):.1f} deg")
        
        # Return True only if both errors are within tolerance
        return (pos_error < POSITION_TOLERANCE) and (yaw_error < YAW_TOLERANCE_RAD)

    def find_closest_waypoint(self):
        """Find the closest waypoint to current position"""
        min_distance = float('inf')
        closest_idx = 0
        
        for i, waypoint in enumerate(WAYPOINTS):
            x, y, z, yaw = waypoint
            temp_point = Point(x, y, z)
            distance = self.calculate_position_error(self.current_pos, temp_point)
            
            if distance < min_distance:
                min_distance = distance
                closest_idx = i
        
        rospy.loginfo(f"Closest waypoint is #{closest_idx + 1}, distance: {min_distance:.2f} m")
        return closest_idx

    def initialize_waypoint_index(self):
        """Initialize the starting waypoint index based on closest waypoint and divert mode"""
        closest_idx = self.find_closest_waypoint()
        
        if self.divert_mode:
            # In divert mode, start from the closest waypoint and go backwards
            self.current_waypoint_idx = closest_idx
            rospy.loginfo(f"Divert mode: Starting from closest waypoint #{closest_idx + 1} and going backwards")
        else:
            # Normal mode: start from the closest waypoint and go forwards
            self.current_waypoint_idx = closest_idx
            rospy.loginfo(f"Normal mode: Starting from closest waypoint #{closest_idx + 1} and going forwards")

    def get_next_waypoint_index(self):
        """Get the next waypoint index based on current mode (normal or divert)"""
        if self.divert_mode:
            # Reverse traversal: decrement index
            next_idx = self.current_waypoint_idx - 1
        else:
            # Normal traversal: increment index
            next_idx = self.current_waypoint_idx + 1
        
        return next_idx

    def is_mission_completed(self):
        """Check if all waypoints have been visited based on current mode"""
        if self.divert_mode:
            # In divert mode, mission completes when we reach the first waypoint (index 0)
            return self.current_waypoint_idx < 0
        else:
            # In normal mode, mission completes when we reach the last waypoint
            return self.current_waypoint_idx >= len(WAYPOINTS)

    def update_target_waypoint(self, waypoint_idx):
        """Update target message with data from the current waypoint"""
        if 0 <= waypoint_idx < len(WAYPOINTS):
            x, y, z, target_yaw = WAYPOINTS[waypoint_idx]
            
            # Update position and yaw
            self.target_msg.position.x = x
            self.target_msg.position.y = y
            self.target_msg.position.z = z
            self.target_msg.yaw = target_yaw
            
            # Update trajectory metadata
            self.target_msg.trajectory_id = waypoint_idx + 1
            self.target_msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            
            # Log waypoint info (convert yaw to degrees for readability)
            direction = "BACKWARD" if self.divert_mode else "FORWARD"
            rospy.loginfo(f"\n=== Moving {direction} to Waypoint {waypoint_idx + 1} ===")
            rospy.loginfo(f"Target: X:{x:.2f}, Y:{y:.2f}, Z:{z:.2f}, yaw:{math.degrees(target_yaw):.1f}°")
            rospy.loginfo(f"Current: X:{self.current_pos.x:.2f}, Y:{self.current_pos.y:.2f}, Z:{self.current_pos.z:.2f}, yaw:{math.degrees(self.current_yaw):.1f}°")

    def run(self):
        """Main control loop"""
        # Load first waypoint immediately
        self.update_target_waypoint(self.current_waypoint_idx)
        
        while not rospy.is_shutdown() and not self.all_waypoints_completed:
            self.target_msg.header.stamp = rospy.Time.now()
            
            # Get current target position and yaw
            target_pos = self.target_msg.position
            target_yaw = self.target_msg.yaw
            
            # Check if waypoint is reached (position + yaw)
            if self.is_waypoint_reached(target_pos, target_yaw):
                if not self.waypoint_reached:
                    # Start hold timer
                    self.waypoint_reached = True
                    self.hold_start_time = rospy.Time.now()
                    direction = "BACKWARD" if self.divert_mode else "FORWARD"
                    rospy.loginfo(f"\nWaypoint {self.current_waypoint_idx + 1} reached!")
                    rospy.loginfo(f"Holding for {WAYPOINT_HOLD_TIME}s before moving {direction}...")
                
                # Check hold time completion
                if (rospy.Time.now() - self.hold_start_time).to_sec() >= WAYPOINT_HOLD_TIME:
                    # Move to next waypoint
                    next_idx = self.get_next_waypoint_index()
                    
                    if not self.is_mission_completed():
                        self.current_waypoint_idx = next_idx
                        self.update_target_waypoint(self.current_waypoint_idx)
                        self.waypoint_reached = False
                    else:
                        # All waypoints completed
                        self.all_waypoints_completed = True
                        self.target_msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_COMPLETED
                        rospy.loginfo("\n" + "="*50)
                        rospy.loginfo("MISSION COMPLETED!")
                        if self.divert_mode:
                            rospy.loginfo("Completed reverse traversal of all waypoints")
                        else:
                            rospy.loginfo("Completed forward traversal of all waypoints")
                        rospy.loginfo("="*50)
            else:
                # Still moving: reset hold flag
                self.waypoint_reached = False
            
            # Publish target message
            self.pub.publish(self.target_msg)
            self.rate.sleep()
        
        # Shutdown node
        rospy.loginfo("Shutting down waypoint follower node...")
        rospy.signal_shutdown("Mission completed successfully")

def parse_arguments():
    """Parse command line arguments"""
    divert = False
    
    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            if arg.lower() == "divert=true":
                divert = True
            elif arg.lower() == "divert=false":
                divert = False
    
    return divert

if __name__ == '__main__':
    try:
        # Parse command line arguments
        divert_mode = parse_arguments()
        
        # Create and run follower
        follower = WaypointFollower(divert=divert_mode)
        follower.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node interrupted by user. Exiting.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {str(e)}")