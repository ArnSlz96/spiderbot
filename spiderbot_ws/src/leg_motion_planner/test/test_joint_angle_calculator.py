import math
import rclpy
import pytest
from leg_motion_planner.calculate_joint_config import joint_angle_calculator
from geometry_msgs.msg import Point

@pytest.fixture(scope="session", autouse=True)
def init_testnode():
    rclpy.init()
    testnode = rclpy.create_node("testnode_joint_angle_calculator")
    yield testnode
    rclpy.shutdown 

@pytest.fixture(scope="class")
def init_joint_angle_calculator():
    ja_calc = joint_angle_calculator()
    return ja_calc


class TestJointAngleCalc:
    def test_class_initialisation(self, init_joint_angle_calculator):
        myJAC = init_joint_angle_calculator
        assert myJAC is not None, "Failed to initialise joint_angle_calculator class."

    def test_direct_kinematic_emptyJoints(self, init_joint_angle_calculator):
        ja_calc = init_joint_angle_calculator

        end_pos_empty = ja_calc._get_endpos_from_joint_angles(0, 0, 0)
        expected_end_pos_empty = [ja_calc.LEG_SEGMENT1_LENGTH + ja_calc.LEG_SEGMENT2_LENGTH, 0, 0]

        assert pytest.approx(end_pos_empty) == expected_end_pos_empty, f"Got wrong end_pos for empty joint angles. Should be {expected_end_pos_empty}, received {end_pos_empty}."

    def test_direct_kinematic_90degreeJoints(self, init_joint_angle_calculator):
        ja_calc = init_joint_angle_calculator

        end_pos_90 = ja_calc._get_endpos_from_joint_angles(math.pi/2, math.pi/2, math.pi/2)
        expected_end_pos_90 = [0, -ja_calc.LEG_SEGMENT1_LENGTH, -ja_calc.LEG_SEGMENT2_LENGTH]

        assert pytest.approx(end_pos_90) == expected_end_pos_90, f"Got wrong end_pos for 90°, 90°, 90° joint angles. Should be {expected_end_pos_90}, received {end_pos_90}."

    def test_inverse_kinematic(self, init_joint_angle_calculator):
        ja_calc = init_joint_angle_calculator

        end_pos = Point()
        end_pos.x = 2
        end_pos.y = 0
        end_pos.z = 0
        hip_yaw, hip_pitch, knee_pitch = ja_calc._calc_hip_knee_pitches(end_pos)

        assert pytest.approx(hip_yaw) == 0
        assert pytest.approx(hip_pitch) == 0
        assert pytest.approx(knee_pitch) == 0
