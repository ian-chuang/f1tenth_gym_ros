
# code adapted from https://github.com/ladavis4/F1Tenth_Final_Project_and_ICRA2022/tree/main/pure_pursuit_pkg
import os

import rclpy
from rclpy.node import Node
from scipy import interpolate
import scipy.ndimage
from scipy.spatial.transform import Rotation as R
import numpy as np
import csv
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import pdb 

class PurePursuit(Node):
    """
    Implement Pure Pursuit on the car
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # declare parameters
        self.declare_parameter("trajectory_csv", "/sim_ws/src/f1tenth_gym_ros/racelines/levine.csv")
        self.declare_parameter("pp_steer_L_fast", 2.5)
        self.declare_parameter("pp_steer_L_slow", 1.8)
        self.declare_parameter("kp_fast", 0.35)
        self.declare_parameter("kp_slow", 0.5)
        self.declare_parameter("L_threshold_speed", 4.0)
        self.declare_parameter("odom_topic", "ego_racecar/odom")
        self.declare_parameter("pure_pursuit_velocity_topic", "pure_pursuit_velocity")
        self.declare_parameter("drive_topic", "drive")
        self.declare_parameter("use_obs_avoid_topic", "use_obs_avoid")
        
        traj_csv = self.get_parameter("trajectory_csv").value
        self.pp_steer_L_fast = self.get_parameter("pp_steer_L_fast").value
        self.pp_steer_L_slow = self.get_parameter("pp_steer_L_slow").value
        self.kp_fast = self.get_parameter("kp_fast").value
        self.kp_slow = self.get_parameter("kp_slow").value
        self.L_threshold_speed = self.get_parameter("L_threshold_speed").value
        odom_topic = self.get_parameter("odom_topic").value
        pure_pursuit_velocity_topic = self.get_parameter("pure_pursuit_velocity_topic").value
        drive_topic = self.get_parameter("drive_topic").value
        use_obs_avoid_topic = self.get_parameter("use_obs_avoid_topic").value

        self.spline_index_car = 0  # Index of the car on the spline

        self.pp_waypoints, self.drive_velocity = load_from_csv(traj_csv)
        self.pp_x_spline = self.pp_waypoints[:, 0]
        self.pp_y_spline = self.pp_waypoints[:, 1]
        self.pp_spline_points = np.vstack((self.pp_x_spline, self.pp_y_spline, np.zeros((len(self.pp_y_spline)))))

        #### Obstacle Avoidance ###
        self.use_obs_avoid = False

        ### ROS PUB/SUB ###
        self.pose_subscriber = self.create_subscription(Odometry, odom_topic, self.pose_callback, 1)
        self.velocity_publisher = self.create_publisher(Float64, pure_pursuit_velocity_topic, 1)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.use_obs_avoid_subscriber = self.create_subscription(Bool, use_obs_avoid_topic, self.use_obs_avoid_callback, 1)

    def pose_callback(self, pose_msg):
        """
        This is the main pure pursuit callback loop
        All parameters are set in the init function
        """
        # parse information from pose_msg
        current_position = pose_msg.pose.pose.position
        current_quat = pose_msg.pose.pose.orientation

        # get the current spline index of the car and goal point
        self.spline_index_car = self.get_closest_point_to_car(current_position, self.pp_spline_points)

        #Determine the speed from the velocity profile
        drive_speed = self.drive_velocity[self.spline_index_car]
        msg = Float64()
        msg.data = drive_speed
        self.velocity_publisher.publish(msg)

        # Calculate goal point
        if drive_speed > self.L_threshold_speed:
            global_goal_point = self.find_goal_point(self.pp_steer_L_fast)
        else:
            global_goal_point = self.find_goal_point(self.pp_steer_L_slow)
        local_goal_point = self.global_2_local(current_quat, current_position, global_goal_point)
        
        # Calculate steer angle
        if drive_speed > self.L_threshold_speed:
            steering_angle = self.calc_steer(local_goal_point, self.kp_fast, self.pp_steer_L_fast)
        else:
            steering_angle = self.calc_steer(local_goal_point, self.kp_slow, self.pp_steer_L_slow)        
        
        if not self.use_obs_avoid:
            msg = AckermannDriveStamped()
            msg.drive.steering_angle = float(steering_angle)
            msg.drive.speed = float(drive_speed)
            self.drive_publisher.publish(msg)

    def calc_steer(self, goal_point_car, kp, L):
        """
        Returns the steering angle from the local goal point
        """
        y = goal_point_car[1]
        steer_dir = np.sign(y)
        r = L ** 2 / (2 * np.abs(y))
        gamma = 1 / r
        steering_angle = (gamma * kp * steer_dir)
        return steering_angle
  
    def use_obs_avoid_callback(self, avoid_msg):
        self.use_obs_avoid = avoid_msg.data

    def global_2_local(self, current_quat, current_position, goal_point_global):
        # Construct transformation matrix from rotation matrix and position
        H_global2car = np.zeros([4, 4]) #rigid body transformation from  the global frame of referce to the car
        H_global2car[3, 3] = 1
        current_rotation_matrix = R.from_quat(np.array([current_quat.x,current_quat.y,current_quat.z,current_quat.w])).as_matrix()
        H_global2car[0:3, 0:3] = np.array(current_rotation_matrix)
        H_global2car[0:3, 3] = np.array([current_position.x, current_position.y, current_position.z])

        # Calculate point
        goal_point_global = np.append(goal_point_global, 1).reshape(4, 1)
        goal_point_car = np.linalg.inv(H_global2car) @ goal_point_global

        return goal_point_car
    
    def get_closest_point_to_car(self, current_position, all_points):
        try:
            current_position=np.array([current_position.x, current_position.y, current_position.z])
        except:
            current_position=np.array([current_position[0], current_position[1], current_position[2]])
        current_position=np.transpose(np.multiply(current_position,np.transpose(np.ones((all_points.shape)))))

        dist = np.linalg.norm(current_position - all_points, axis=0)

        point_index = np.argmin(dist)
        return point_index

    
    def find_goal_point(self, L):
        # Returns the global x,y,z position of the goal point
        points_in_front = np.roll(self.pp_spline_points, - self.spline_index_car, axis=1)
        points_dist = np.linalg.norm(np.roll(points_in_front, 1, axis=1) - points_in_front, axis=0)
        points_dist = np.cumsum(points_dist)
        idx = np.argmin(np.abs(points_dist - L))
        goal_point_car = points_in_front[:, idx]
        return goal_point_car


def load_from_csv(traj_csv, TUM=False, scaler=1):
    # Open csv and read the waypoint data
    points = None
    velocity = None

    with open(traj_csv, 'r') as f:
        lines = (line for line in f if not line.startswith('#'))
        data = np.loadtxt(lines, delimiter=',')
    points = data[:, 0:2] 
    velocity = data[:, 2]

    return points, velocity

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()