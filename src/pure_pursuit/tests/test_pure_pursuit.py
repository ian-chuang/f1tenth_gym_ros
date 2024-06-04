
from pure_pursuit.pure_pursuit import PurePursuit

def test_math():
	assert 2 + 2 == 4   # This should fail for most mathematical systems

'''
- pure_pursuit.tests.test_pure_pursuit test_calc_steer (tests/test_pure_pursuit.py:7)
  <<< failure message
    rclpy.exceptions.NotInitializedException: ('rclpy.init() has not been called', 'cannot create node')
  >>>
'''
def test_calc_steer():
	node = PurePursuit()
	steering_angle = node.calc_steer(goal_point_car=[1, 1], kp=1, L=1)
	assert steering_angle == 2.0