from reacher import forward_kinematics
from reacher import inverse_kinematics
from reacher import reacher_robot_utils
from reacher import reacher_sim_utils
import pybullet as p
import time
import numpy as np
from absl import app
from absl import flags
from pupper_hardware_interface import interface
from sys import platform

flags.DEFINE_bool("run_on_robot", False,
                  "Whether to run on robot or in simulation.")
flags.DEFINE_bool("ik", False, "Whether to control arms through cartesian coordinates(IK) or joint angles")
FLAGS = flags.FLAGS

KP = 5.0  # Amps/rad
KD = 2.0  # Amps/(rad/s)
MAX_CURRENT = 3.0  # Amps

UPDATE_DT = 0.01  # seconds

HIP_OFFSET = 0.0335  # meters
L1 = 0.08  # meters
L2 = 0.11  # meters


def main(argv):
  run_on_robot = FLAGS.run_on_robot
  reacher = reacher_sim_utils.load_reacher()

  # Sphere markers for the students' FK solutions
  shoulder_sphere_id = reacher_sim_utils.create_debug_sphere([1, 0, 0, 1])
  elbow_sphere_id    = reacher_sim_utils.create_debug_sphere([0, 1, 0, 1])
  foot_sphere_id     = reacher_sim_utils.create_debug_sphere([0, 0, 1, 1])
  target_sphere_id   = reacher_sim_utils.create_debug_sphere([1, 1, 1, 1], radius=0.01)

  joint_ids = reacher_sim_utils.get_joint_ids(reacher)
  param_ids = reacher_sim_utils.get_param_ids(reacher, FLAGS.ik)
  reacher_sim_utils.zero_damping(reacher)

  p.setPhysicsEngineParameter(numSolverIterations=10)

  if run_on_robot:
    mode_label_id = p.addUserDebugText("Mode = Real to Sim", [-0.1, -0.1, 0.3])
    serial_port = reacher_robot_utils.get_serial_port()
    hardware_interface = interface.Interface(serial_port)
    time.sleep(0.25)
    hardware_interface.set_joint_space_parameters(kp=KP,
                                                  kd=KD,
                                                  max_current=MAX_CURRENT)

  p.setRealTimeSimulation(1)
  counter = 0
  last_command = time.time()
  enable = True
  joint_angles = np.zeros(6)

  # For switching between control modes on the robot
  last_mode_toggle = time.time()
  SIM_TO_REAL = 0
  REAL_TO_SIM = 1
  mode_names = ["Sim to Real", "Real to Sim"]
  mode = SIM_TO_REAL
  TOGGLE_BUFFER_TIME = 0.3 # seconds

  # Utility function for determining when the spacebar has been pressed down and released
  def isSpacebarPressed() -> bool:
    return (p.getKeyboardEvents().get(32, 0) & 0b100)

  # Main loop
  while (True):

    # Check if we need to switch control modes
    if run_on_robot and time.time() - last_mode_toggle >= TOGGLE_BUFFER_TIME:
      last_mode_toggle = time.time()
      if (isSpacebarPressed()):
        mode = 1 - mode
        p.removeUserDebugItem(mode_label_id)
        mode_label_id = p.addUserDebugText(f"Mode = {mode_names[mode]}", [-0.1, -0.1, 0.3])

    # If interfacing with the real robot, handle those communications now
    if run_on_robot:
      hardware_interface.read_incoming_data()
      if (mode == SIM_TO_REAL):
        hardware_interface.set_activations([0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0])
      else:
        hardware_interface.deactivate()

      # Check if we need to switch control modes
      if time.time() - last_mode_toggle >= TOGGLE_BUFFER_TIME:
        last_mode_toggle = time.time()
        if (isSpacebarPressed()):
          mode = 1 - mode
          p.removeUserDebugItem(mode_label_id)
          mode_label_id = p.addUserDebugText(f"Mode = {mode_names[mode]}", [-0.1, -0.1, 0.3])

    # Control loop
    if time.time() - last_command > UPDATE_DT:
      last_command = time.time()
      counter += 1

      slider_angles = np.zeros_like(joint_angles)
      for i in range(len(param_ids)):
        c = param_ids[i]
        targetPos = p.readUserDebugParameter(c)
        slider_angles[i] = targetPos

      # If IK is enabled, update joint angles based off of goal XYZ position
      if FLAGS.ik:
          xyz = []
          for i in range(3):
            xyz.append(p.readUserDebugParameter(i))
          p.resetBasePositionAndOrientation(target_sphere_id, posObj=xyz, ornObj=[0, 0, 0, 1])

          xyz = np.asarray(xyz)
          ret = inverse_kinematics.calculate_inverse_kinematics(xyz, joint_angles[:3])
          if ret is None:
            enable = False
          else:
            # Wraps angles between -pi, pi
            joint_angles[:3] = np.arctan2(np.sin(ret), np.cos(ret))
            joint_angles[3:] = slider_angles[3:]
            enable = True
      elif mode == SIM_TO_REAL:
        joint_angles = slider_angles
      else:
        joint_angles = hardware_interface.robot_state.position[6:9]

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
        # TODO: Update order & signs for your own robot/motor configuration like below
        # left_angles = [-joint_angles[1], -joint_angles[0], joint_angles[2]]
        left_angles = joint_angles[:3]
        full_actions[:, 2] = left_angles

        hardware_interface.set_actuator_postions(full_actions)

      # Get the positions of each joint and the end effector
      shoulder_pos = forward_kinematics.fk_shoulder(joint_angles[:3])[:3,3]
      elbow_pos    = forward_kinematics.fk_elbow(joint_angles[:3])[:3,3]
      foot_pos     = forward_kinematics.fk_foot(joint_angles[:3])[:3,3]

      # Show the bebug spheres for FK
      p.resetBasePositionAndOrientation(shoulder_sphere_id, posObj=shoulder_pos, ornObj=[0, 0, 0, 1])
      p.resetBasePositionAndOrientation(elbow_sphere_id   , posObj=elbow_pos   , ornObj=[0, 0, 0, 1])
      p.resetBasePositionAndOrientation(foot_sphere_id    , posObj=foot_pos    , ornObj=[0, 0, 0, 1])

      # No, you cannot use this as your Hands-On 3 solution. You have to calculate it yourself
      if (mode == REAL_TO_SIM):
        (_, _, _, _, foot_pos, _) = p.getLinkState(reacher, 3, computeForwardKinematics = True)

      # Show the result in the terminal
      if counter % 20 == 0:
        print(f"\r[MODE = {mode_names[mode]}] | Joint angles: [{', '.join(f'{q: .3f}' for q in joint_angles[:3])}] | Position: ({', '.join(f'{p: .3f}' for p in foot_pos)})", end='')

app.run(main)
