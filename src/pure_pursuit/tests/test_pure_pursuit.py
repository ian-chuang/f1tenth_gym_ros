
from pure_pursuit.pure_pursuit import PurePursuit
from pure_pursuit.pure_pursuit import load_from_csv
from std_msgs.msg import Bool
from geometry_msgs.msg import Quaternion, Point
from nav_msgs.msg import Odometry
import numpy as np
import rclpy

'''
- pure_pursuit.tests.test_pure_pursuit test_calc_steer (tests/test_pure_pursuit.py:7)
  <<< failure message
    rclpy.exceptions.NotInitializedException: ('rclpy.init() has not been called', 'cannot create node')
  >>>
'''

rclpy.init()

def test_init():
	pure_pursuit = PurePursuit()
	assert pure_pursuit.pp_steer_L_fast == 2.5
	assert pure_pursuit.pp_steer_L_slow == 1.8
	assert pure_pursuit.kp_fast == 0.35
	assert pure_pursuit.kp_slow == 0.5
	assert pure_pursuit.L_threshold_speed == 4.0
	assert pure_pursuit.use_obs_avoid == False
	pure_pursuit.destroy_node()

def test_calc_steer():
	pure_pursuit = PurePursuit()
	steering_angle = pure_pursuit.calc_steer(goal_point_car=[1, 1], kp=1, L=1)
	assert steering_angle == 2.0
	pure_pursuit.destroy_node()


def test_use_obs_avoid_callback():
    pure_pursuit = PurePursuit()
    avoid_msg = Bool()
    avoid_msg.data = True
    pure_pursuit.use_obs_avoid_callback(avoid_msg)
    assert pure_pursuit.use_obs_avoid == True
    pure_pursuit.destroy_node()
	
def test_global_2_local():
    pure_pursuit = PurePursuit()
    current_quat = Quaternion()
    current_quat.x = 0.0
    current_quat.y = 0.0
    current_quat.z = 0.0
    current_quat.w = 1.0
    current_position = Point()
    current_position.x = 0.0
    current_position.y = 0.0
    current_position.z = 0.0
    goal_point_global = np.array([1.0, 1.0, 0.0])
    goal_point_car = pure_pursuit.global_2_local(current_quat, current_position, goal_point_global)
    assert goal_point_car[0] == 1.0
    assert goal_point_car[1] == 1.0
    assert goal_point_car[2] == 0.0
    pure_pursuit.destroy_node()
    
def test_get_closest_point_to_car():
    pure_pursuit = PurePursuit()
    current_position = Point()
    current_position.x = 1.0
    current_position.y = 1.0
    current_position.z = 0.0
    all_points = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 2.0, 0.0]])
    point_index = pure_pursuit.get_closest_point_to_car(current_position, all_points)
    assert point_index == 2
    pure_pursuit.destroy_node()
    
def test_find_goal_point():
    pure_pursuit = PurePursuit()
    pure_pursuit.spline_index_car = 0
    pure_pursuit.pp_spline_points = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 2.0, 0.0]])
    goal_point_car = pure_pursuit.find_goal_point(1.0)
    assert goal_point_car[0] == 0.0
    assert goal_point_car[1] == 1.0
    assert goal_point_car[2] == 2.0
    pure_pursuit.destroy_node()

def test_load_from_csv():
    traj_csv = "/sim_ws/src/f1tenth_gym_ros/racelines/levine.csv"
    points, velocity = load_from_csv(traj_csv)

    # Test the shape of the loaded data
    N=317
    assert points.shape == (N, 2)  
    assert velocity.shape == (N,)  

    # Test the data type of the loaded data
    assert isinstance(points, np.ndarray)
    assert isinstance(velocity, np.ndarray)
    
def test_pose_callback():
    pure_pursuit = PurePursuit()

    # Set up the pose message
    pose_msg = Odometry()
    pose_msg.pose.pose.position.x = 1.0
    pose_msg.pose.pose.position.y = 1.0
    pose_msg.pose.pose.position.z = 0.0
    pose_msg.pose.pose.orientation.x = 0.0
    pose_msg.pose.pose.orientation.y = 0.0
    pose_msg.pose.pose.orientation.z = 0.0
    pose_msg.pose.pose.orientation.w = 1.0

    # Call the pose_callback method
    pure_pursuit.pose_callback(pose_msg)
    pure_pursuit.destroy_node()