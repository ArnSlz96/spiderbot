import math
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from geometry_msgs.msg import Point
from mechanical.load_mechanical_params import load_mechanical_params

class joint_angle_calculator():
    def __init__(self):
        # Initialise logger outside of node
        self.roslog = rclpy.logging.get_logger("Calculate_joint_config")

        # Get mechanical parameters from mechanical module
        mech_params = load_mechanical_params()
        self.LEG_SEGMENT1_LENGTH = mech_params["LEG_SEGMENT1_LENGTH"]
        self.LEG_SEGMENT2_LENGTH = mech_params["LEG_SEGMENT2_LENGTH"]

        # Threshold for difference between wanted end_pos and resulting end_pos from joint angles
        # for joint angles to be seen as valid (in mm)
        self.max_valid_endpos_distance =  5


    # --- Primary usage function

    def calculate_joint_angles(self, end_pos: Point) -> list[float]:
        if not self._is_valid_end_pos(end_pos):
            self.roslog.warning(f"Received invalid end_pos {end_pos}")

        # Do calc here
        hip_yaw = self._calc_hip_yaw(end_pos)
        hip_pitch, knee_pitch = self._calc_hip_knee_pitches(end_pos)

        # Do validity check for end_pos resulting from calculated joint angles
        resulting_end_pos = self._get_endpos_from_joint_angles(hip_yaw, hip_pitch, knee_pitch)

        # Check calculated joint configs against range of valid joint configs

        return [hip_yaw, hip_pitch, knee_pitch]
    
    # --- Helper functions

    def _is_valid_end_pos(self, end_pos: Point) -> bool:
        total_leg_length = self.LEG_SEGMENT1_LENGTH + self.LEG_SEGMENT2_LENGTH

        # Check for correct input   
        if end_pos.z>0:
            self.roslog.warning(f"Unexpected Z-value end_pos for joint_angle planner. Should be negative, received {Point.z}.")
            return False
        
        # Check if end_pos is outside of leg range
        distance_from_leg_origin = np.linalg.norm([end_pos.x, end_pos.y, end_pos.z])
        if  distance_from_leg_origin > total_leg_length:
            self.roslog.warning(f"end_pos outside of leg range. Should be less than {total_leg_length}, is {distance_from_leg_origin}")
            return False
        
        return True

    # Calculate hip yaw in x-y plane
    def _calc_hip_yaw(self, end_pos: Point) -> float:
        hip_yaw = math.atan(end_pos.y / end_pos.x)
        return hip_yaw

    def _get_endpos_from_joint_angles(self, hip_yaw: float, hip_pitch: float, knee_pitch:float) -> Point:
        # Define all affine transformation matrices
        hip_yaw_rotation = Rotation.from_euler('z', hip_yaw).as_matrix()
        hip_pitch_rotation = Rotation.from_euler('y', hip_pitch).as_matrix()
        knee_pitch_rotation = Rotation.from_euler('y', knee_pitch).as_matrix()

        T_hip_yaw = np.eye(4)
        T_hip_yaw[:3, :3] = hip_yaw_rotation

        T_hip_pitch = np.eye(4)
        T_hip_pitch[:3, :3] = hip_pitch_rotation

        T_seg1 = np.eye(4)
        T_seg1[0, 3] = self.LEG_SEGMENT1_LENGTH

        T_knee_pitch = np.eye(4)
        T_knee_pitch[:3, :3] = knee_pitch_rotation

        T_seg2 = np.eye(4)
        T_seg2[0, 3] = self.LEG_SEGMENT2_LENGTH

        # Chain transformations
        T = np.eye(4)
        T = T @ T_hip_yaw @ T_hip_pitch @ T_seg1
        T = T @ T_knee_pitch @ T_seg2

        end_position = T[:3, 3]
        end_orientation = T[:3, :3]

        return end_position

