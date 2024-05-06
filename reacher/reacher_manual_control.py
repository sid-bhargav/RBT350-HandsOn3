from reacher import forward_kinematics
from reacher import inverse_kinematics
from reacher import reacher_robot_utils
from reacher import reacher_sim_utils
from reacher import vision
import pybullet as p
import time
import contextlib
import numpy as np
import pickle
import cv2
from absl import app
from absl import flags
from pupper_hardware_interface import interface
from sys import platform

flags.DEFINE_bool("run_on_robot", False, "Whether to run on robot or in simulation.")
flags.DEFINE_bool("ik"          , False, "Whether to control arms through cartesian coordinates(IK) or joint angles")
flags.DEFINE_list("set_joint_angles", [], "List of joint angles to set at initialization.")
FLAGS = flags.FLAGS

KP = 5.0  # Amps/rad
KD = 2.0  # Amps/(rad/s)
MAX_CURRENT = 3  # Amps

UPDATE_DT = 0.01  # seconds

HIP_OFFSET = 0.0335  # meters
L1 = 0.08  # meters
L2 = 0.11  # meters


def main(argv):
  run_on_robot = FLAGS.run_on_robot
  reacher = reacher_sim_utils.load_reacher()
  position = [0.1, 0.1, 0.1]

  # Sphere markers for the students' FK solutions
  shoulder_sphere_id = reacher_sim_utils.create_debug_sphere([1, 0, 0, 1])
  elbow_sphere_id    = reacher_sim_utils.create_debug_sphere([0, 1, 0, 1])
  foot_sphere_id     = reacher_sim_utils.create_debug_sphere([0, 0, 1, 1])
  target_sphere_id   = reacher_sim_utils.create_debug_sphere([1, 1, 1, 1], radius=0.01)

  joint_ids = reacher_sim_utils.get_joint_ids(reacher)
  param_ids = reacher_sim_utils.get_param_ids(reacher, FLAGS.ik)
  reacher_sim_utils.zero_damping(reacher)

  p.setPhysicsEngineParameter(numSolverIterations=10)

  # Set up physical robot if we're using it, starting with motors disabled
  if run_on_robot:
    serial_port = reacher_robot_utils.get_serial_port()
    hardware_interface = interface.Interface(serial_port)
    time.sleep(0.25)
    hardware_interface.set_joint_space_parameters(kp=KP, kd=KD, max_current=MAX_CURRENT)
    hardware_interface.deactivate()

  # Whether or not the motors are enabled
  motor_enabled = False
  if run_on_robot:
    mode_text_id = p.addUserDebugText(f"Motor Enabled: {motor_enabled}", [0, 0, 0.2])
  def checkEnableMotors():
    nonlocal motor_enabled, mode_text_id

    # If spacebar (key 32) is pressed and released (0b100 mask), then toggle motors on or off
    if p.getKeyboardEvents().get(32, 0) & 0b100:
      motor_enabled = not motor_enabled
      if motor_enabled:
        hardware_interface.set_activations([0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0])
      else:
        hardware_interface.deactivate()
      p.removeUserDebugItem(mode_text_id)
      mode_text_id = p.addUserDebugText(f"Motor Enabled: {motor_enabled}", [0, 0, 0.2])

  # Control Loop Variables
  p.setRealTimeSimulation(1)
  counter = 0
  last_command = time.time()
  joint_angles = np.zeros(3)
  if flags.FLAGS.set_joint_angles:
    # first the joint angles to 0,0,0
    for idx, joint_id in enumerate(joint_ids):
      p.setJointMotorControl2(
        reacher,
        joint_id,
        p.POSITION_CONTROL,
        joint_angles[idx],
        force=2.
      )
    joint_angles = np.array(flags.FLAGS.set_joint_angles, dtype=np.float32)
    # Set the simulated robot to match the joint angles
    for idx, joint_id in enumerate(joint_ids):
      p.setJointMotorControl2(
        reacher,
        joint_id,
        p.POSITION_CONTROL,
        joint_angles[idx],
        force=2.
      )

  print("\nRobot Status:\n")

  # Read calibration data
  with open('./calibration_sid.pckl', 'rb') as f:
    data = pickle.load(f)

  cameraMatrix, distCoeffs, rvecs, tvecs = data

  # print(uv_to_xy((325, 246), camera_matrix, distortion_matrix))

  # Start capturing video from the webcam
  cap = cv2.VideoCapture(0)

  # Main loop
  while (True):

    # Get video frame
    conn, frame = cap.read()

    # check if camera is connected
    if not conn:
      break

    # Chnage from rgb to hsv colorspace 
    # better for detecting a red hue rather than red color value
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the range of red color in HSV
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([1, 255, 255])
    lower_red_hues = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170, 120, 70])
    upper_red = np.array([180, 255, 255])
    upper_red_hues = cv2.inRange(hsv, lower_red, upper_red)

    # Combine masks for red hues
    mask = lower_red_hues + upper_red_hues

    # filter the frame for only the masked red pixels
    output_hsv = frame
    # output_hsv[np.where(mask==0)] = 0

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Process each contour
    for contour in contours:
      area = cv2.contourArea(contour)
      # Filter small areas to reduce noise
      if area > 100:
        # Calculate circularity to identify dots
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
          continue
        circularity = 4 * np.pi * (area / (perimeter * perimeter))
        # Threshold for circularity can be adjusted, closer to 1 is perfect circle
        if circularity > 0.75:  # Adjust circularity threshold as needed
          # Calculate bounding circle
          (x, y), radius = cv2.minEnclosingCircle(contour)
          center = (int(x), int(y))
          print(center)
          radius = int(radius)
          # Draw the circle
          cv2.circle(output_hsv, center, radius, (0, 255, 0), 2)
          position = vision.CF_to_BF(vision.uv_to_xy(center, cameraMatrix, distCoeffs))
          cv2.putText(output_hsv, str(position), center, cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 1)

    # Display the result
    cv2.imshow('Red Dot Tracker', output_hsv)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

    # Whether or not to send commands to the real robot
    enable = False

    # If interfacing with the real robot, handle those communications now
    if run_on_robot:
      hardware_interface.read_incoming_data()
      checkEnableMotors()

    # Determine the direction of data transfer
    real_to_sim = not motor_enabled and run_on_robot

    # Control loop
    if time.time() - last_command > UPDATE_DT:
      last_command = time.time()
      counter += 1

      # Read the slider values
      try:
        slider_values = np.array([p.readUserDebugParameter(id) for id in param_ids])
      except:
        pass
      if FLAGS.ik:
        xyz = position
        p.resetBasePositionAndOrientation(target_sphere_id, posObj=xyz, ornObj=[0, 0, 0, 1])
      else:
        joint_angles = slider_values
        enable = True

      # If IK is enabled, update joint angles based off of goal XYZ position
      if FLAGS.ik:
          ret = inverse_kinematics.calculate_inverse_kinematics(xyz, joint_angles[:3])
          if ret is not None:
            enable = True
            # Wraps angles between -pi, pi
            joint_angles = np.arctan2(np.sin(ret), np.cos(ret))

            # Double check that the angles are a correct solution before sending anything to the real robot
            pos = forward_kinematics.fk_foot(joint_angles[:3])[:3,3]
            if np.linalg.norm(np.asarray(pos) - xyz) > 0.05:
              joint_angles = np.zeros_like(joint_angles)
              if flags.FLAGS.set_joint_angles:
                joint_angles = np.array(flags.FLAGS.set_joint_angles, dtype=np.float32)
              print("Prevented operation on real robot as inverse kinematics solution was not correct")

      # If real-to-sim, update the joint angles based on the actual robot joint angles
      if real_to_sim:
        joint_angles = hardware_interface.robot_state.position[6:9]
        joint_angles[0] *= -1

      # Set the simulated robot to match the joint angles
      for idx, joint_id in enumerate(joint_ids):
        p.setJointMotorControl2(
          reacher,
          joint_id,
          p.POSITION_CONTROL,
          joint_angles[idx],
          force=2.
        )

      # Set the robot angles to match the joint angles
      if run_on_robot and enable:
        full_actions = np.zeros([3, 4])
        full_actions[:, 2] = joint_angles
        full_actions[0, 2] *= -1

        # Prevent set_actuator_positions from printing to the console
        with contextlib.redirect_stdout(None):
          hardware_interface.set_actuator_postions(full_actions)

      # Get the calculated positions of each joint and the end effector
      shoulder_pos = forward_kinematics.fk_shoulder(joint_angles[:3])[:3,3]
      elbow_pos    = forward_kinematics.fk_elbow(joint_angles[:3])[:3,3]
      foot_pos     = forward_kinematics.fk_foot(joint_angles[:3])[:3,3]

      # Show the bebug spheres for FK
      p.resetBasePositionAndOrientation(shoulder_sphere_id, posObj=shoulder_pos, ornObj=[0, 0, 0, 1])
      p.resetBasePositionAndOrientation(elbow_sphere_id   , posObj=elbow_pos   , ornObj=[0, 0, 0, 1])
      p.resetBasePositionAndOrientation(foot_sphere_id    , posObj=foot_pos    , ornObj=[0, 0, 0, 1])

      # Show the result in the terminal
      if counter % 20 == 0:
        print(f"\rJoint angles: [{', '.join(f'{q: .3f}' for q in joint_angles[:3])}] | Position: ({', '.join(f'{p: .3f}' for p in foot_pos)})", end='')

  # Release the video capture object and close all OpenCV windows
  cap.release()
  cv2.destroyAllWindows()

app.run(main)
