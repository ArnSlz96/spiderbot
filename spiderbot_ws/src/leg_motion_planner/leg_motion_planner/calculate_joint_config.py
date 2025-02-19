import math
import rclpy
from geometry_msgs.msg import Point
from mechanical.mechanical import load_mechanical_params

class joint_angle_calculator():
    def __init__(self):
        # Initialise logger outside of node
        self.roslog = rclpy.logging.get_logger("Calculate_joint_config")

        # Get mechanical parameters from mechanical module
        mech_params = load_mechanical_params()

        LEG_SEGMENT1_LENGTH = mech_params["LEG_SEGMENT1_LENGTH"]
        LEG_SEGMENT2_LENGTH = mech_params["LEG_SEGMENT2_LENGTH"]
        self.LEG_LENGTH = LEG_SEGMENT1_LENGTH + LEG_SEGMENT2_LENGTH

    # --- Primary usage function

    def calculate_joint_angles(self, end_pos: Point) -> list[float]:
        if not self._is_valid_end_pos(end_pos):
            self.roslog.warning(f"Received invalid end_pos {end_pos}")

        # Do calc here
        # Yaw should be easy

        # Pitches a bit more involved

        # TODO figure out geometric relations

        return [hip_yaw, hip_pitch, knee_pitch]
    
    # --- Helper functions

    def _is_valid_end_pos(self, end_pos: Point) -> bool:
        # Check for correct input   
        if Point.z>0:
            self.roslog.warning(f"Unexpected Z-value end_pos for joint_angle planner. Should be negative, received {Point.z}.")
            return False
        
        # Check if end_pos is outside of leg range
        distance_from_leg_origin = np.linalg.norm([end_pos.x, end_pos.y, end_pos.z])
        if  distance_from_leg_origin > self.LEG_LENGTH:
            self.roslog.warning(f"end_pos outside of leg range. Should be less than {self.LEG_LENGTH}, is {distance_from_leg_origin}")
            return False
        
        return True