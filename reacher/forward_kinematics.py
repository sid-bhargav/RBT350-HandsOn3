import math
import numpy as np
import copy

HIP_OFFSET = 0.0335
UPPER_LEG_OFFSET = 0.10  # length of link 1
LOWER_LEG_OFFSET = 0.13  # length of link 2


def rotation_matrix(axis, angle):
    """
    Create a 3x3 rotation matrix which rotates about a specific axis

    Args:
      axis:  Array.  Unit vector in the direction of the axis of rotation
      angle: Number. The amount to rotate about the axis in radians

    Returns:
      3x3 rotation matrix as a numpy array
    """

    # Normalize the axis vector
    axis = axis / np.linalg.norm(axis)

    # Pre-calculate trigonometric values
    c = np.cos(angle)
    s = np.sin(angle)
    t = 1 - c

    # Components of the rotation matrix
    x, y, z = axis

    # Construct the rotation matrix
    rotation_matrix = np.array(
        [
            [t * x * x + c, t * x * y - z * s, t * x * z + y * s],
            [t * x * y + z * s, t * y * y + c, t * y * z - x * s],
            [t * x * z - y * s, t * y * z + x * s, t * z * z + c],
        ]
    )

    return rotation_matrix


def homogenous_transformation_matrix(axis, angle, v_A):
    """
    Create a 4x4 transformation matrix which transforms from frame A to frame B

    Args:
      axis:  Array.  Unit vector in the direction of the axis of rotation
      angle: Number. The amount to rotate about the axis in radians
      v_A:   Vector. The vector translation from A to B defined in frame A

  Returns:
    4x4 transformation matrix as a numpy array
  """

    R = rotation_matrix(axis, angle)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = v_A
    return T


def fk_hip(joint_angles):
    """
    Use forward kinematics equations to calculate the xyz coordinates of the hip
    frame given the joint angles of the robot

  Args:
    joint_angles: numpy array of 3 elements stored in the order [hip_angle, shoulder_angle, 
                  elbow_angle]. Angles are in radians
  Returns:
    4x4 matrix representing the pose of the hip frame in the base frame
  """

    T_hip = homogenous_transformation_matrix(np.array([0, 0, 1]), joint_angles[0], np.array([0, 0, 0]))
    return T_hip


def fk_shoulder(joint_angles):
    """
    Use forward kinematics equations to calculate the xyz coordinates of the shoulder
    joint given the joint angles of the robot

  Args:
    joint_angles: numpy array of 3 elements stored in the order [hip_angle, shoulder_angle, 
                  elbow_angle]. Angles are in radians
  Returns:
    4x4 matrix representing the pose of the shoulder frame in the base frame
  """
    
    T_hip = fk_hip(joint_angles)
    # Shoulder is a translation along the Y-axis from the hip, then rotation about the new Y-axis
    T_shoulder = homogenous_transformation_matrix(np.array([0, 1, 0]), joint_angles[1], np.array([HIP_OFFSET, 0, 0]))
    return T_hip @ T_shoulder


def fk_elbow(joint_angles):
    """
    Use forward kinematics equations to calculate the xyz coordinates of the elbow
    joint given the joint angles of the robot

  Args:
    joint_angles: numpy array of 3 elements stored in the order [hip_angle, shoulder_angle, 
                  elbow_angle]. Angles are in radians
  Returns:
    4x4 matrix representing the pose of the elbow frame in the base frame
  """
    
    T_shoulder = fk_shoulder(joint_angles)
    # Elbow is a translation along the Y-axis from the shoulder, then rotation about the new Y-axis
    T_elbow = homogenous_transformation_matrix(np.array([0, 1, 0]), joint_angles[2], np.array([UPPER_LEG_OFFSET, 0, 0]))
    return T_shoulder @ T_elbow


def fk_foot(joint_angles):
    """
    Use forward kinematics equations to calculate the xyz coordinates of the foot given
    the joint angles of the robot

  Args:
    joint_angles: numpy array of 3 elements stored in the order [hip_angle, shoulder_angle, 
                  elbow_angle]. Angles are in radians
  Returns:
    4x4 matrix representing the pose of the end effector frame in the base frame
  """

    T_elbow = fk_elbow(joint_angles)
    # Foot is a translation along the Y-axis from the elbow
    T_foot = homogenous_transformation_matrix(np.array([0, 0, 1]), 0, np.array([LOWER_LEG_OFFSET, 0, 0]))
    return T_elbow @ T_foot
