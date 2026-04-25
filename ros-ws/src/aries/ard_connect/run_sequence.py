#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                            run_sequence.py                                   ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  WHAT THIS FILE DOES:                                                        ║
║    This script tells the robot to move through a series of pre-defined       ║
║    poses, one after another — like playing a scripted animation.             ║
║    Each "pose" is a named joint configuration defined in your SRDF file.     ║
║                                                                              ║
║  HOW IT WORKS:                                                               ║
║    1. This script connects to MoveIt2's "/move_action" server                ║
║    2. For each step in SEQUENCE, it sends a goal to MoveIt2                  ║
║    3. MoveIt2 plans a collision-free path and executes it                    ║
║    4. The JointTrajectoryController streams positions to /joint_states       ║
║    5. ros2_to_arduino_bridge.py reads /joint_states and moves the real servos        ║
║    6. After each move finishes, this script waits, then sends the next       ║
║                                                                              ║
║  HOW TO RUN (requires demo.launch.py already running):                       ║
║    Terminal 1:  ros2 launch moveit2 demo.launch.py                           ║
║    Terminal 2:  source ~/.ros2.sh && python3 run_sequence.py                 ║
║                                                                              ║
║  QUICK CUSTOMIZATION GUIDE:                                                  ║
║    • To change the sequence:        edit SEQUENCE list below                 ║
║    • To change pose positions:      edit NAMED_POSES dict below              ║
║    • To change speed globally:      change the 'speed' value in each step    ║
║    • To add a pause between moves:  change 'wait' value in a step            ║
║    • To run sequence in a loop:     see the main() function at the bottom    ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

# ── IMPORTS ────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node

# MoveGroup is the "action" type — think of it like a task request to MoveIt2
# Actions are like function calls that take time and report progress
from moveit_msgs.action import MoveGroup

# These are the building blocks of a motion planning request
from moveit_msgs.msg import (
    MotionPlanRequest,   # The full planning request (group, goal, planner, etc.)
    WorkspaceParameters, # Defines a 3D box where the robot can move
    Constraints,         # A container holding a list of constraints
    JointConstraint      # Specifies one joint should be at a certain angle
)

from action_msgs.msg import GoalStatus  # Status codes (SUCCEEDED, ABORTED, etc.)
from rclpy.action import ActionClient   # Lets us send goals to action servers
from geometry_msgs.msg import Vector3   # 3D vector, used for workspace corners
import time                             # For time.sleep() between poses


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 1: JOINT GROUPS
#  ────────────────────────
#  These lists must exactly match what's defined in your SRDF file.
#  The SRDF divides joints into groups for planning purposes.
# ══════════════════════════════════════════════════════════════════════════════

# Joints in the "arm" planning group (from full.srdf)
ARM_JOINTS = ['jrotate1', 'jlink1', 'jlink2', 'jrotate2', 'jend']

# Joints in the "grippers" planning group (from full.srdf)
# Note: jsgear = connector 1 (gripper), jhorn = connector 2 (magnet)
GRIPPER_JOINTS = ['jsgear', 'jhorn']


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 2: NAMED POSES
#  ───────────────────────
#  These MUST match the group_state definitions in your full.srdf file exactly.
#  If you use MoveIt Setup Assistant to change a pose, update it here too.
#
#  Each pose has:
#    'group'  → which planning group moves ('arm' or 'grippers')
#    'joints' → a dict of {joint_name: angle_in_radians} for the target pose
#
#  HOW TO READ RADIANS:
#    0.0  rad = 0°    (neutral position)
#    1.57 rad ≈ 90°   (quarter turn)
#    3.14 rad ≈ 180°  (half turn)
#   -1.57 rad ≈ -90°  (opposite quarter turn)
# ══════════════════════════════════════════════════════════════════════════════

NAMED_POSES = {

    # ── ARM POSES ─────────────────────────────────────────────────────────────

    # Home: all joints at zero = robot stands upright in its neutral position
    'home': {
        'group': 'arm',
        'joints': {
            'jrotate1': 0.0,
            'jlink1':   0.0,
            'jlink2':   0.0,
            'jrotate2': 0.0,
            'jend':     0.0,
        }
    },

    # Pre-grasp: arm positioned above CONNECTOR 1 (gripper), ready to descend
    'pre_grasp': {
        'group': 'arm',
        'joints': {
            'jrotate1':  0.188,    # slight rotation toward connector 1
            'jlink1':    0.5572,   # arm raised and extended
            'jlink2':    0.292,    # second segment angled down
            'jrotate2':  0.0,
            'jend':     -1.268,    # wrist tilted to align with connector
        }
    },

    # Grasp: arm descended into CONNECTOR 1 — the physical insertion point
    'grasp': {
        'group': 'arm',
        'joints': {
            'jrotate1':  0.188,
            'jlink1':    0.66,     # arm lowered slightly more than pre_grasp
            'jlink2':    0.292,
            'jrotate2':  0.0,
            'jend':     -1.2182,   # wrist adjusted for insertion
        }
    },

    # Post-grasp: arm lifted after attaching CONNECTOR 1 — safe retreat pose
    'post_grasp': {
        'group': 'arm',
        'joints': {
            'jrotate1':  0.188,
            'jlink1':    0.0434,   # arm lifted back up
            'jlink2':   -0.5699,   # second segment pulled back
            'jrotate2':  0.0,
            'jend':     -0.9862,
        }
    },

    # Pre-graspm: arm positioned above CONNECTOR 2 (magnet), ready to descend
    # The 'm' suffix means this is for the MAGNET connector (second end effector)
    # Same vertical pose as pre_grasp but rotated to face connector 2
    'pre_graspm': {
        'group': 'arm',
        'joints': {
            'jrotate1': -0.175,    # rotated to face connector 2 (opposite side)
            'jlink1':    0.5572,
            'jlink2':    0.292,
            'jrotate2':  0.0,
            'jend':     -1.268,
        }
    },

    # Graspm: arm descended into CONNECTOR 2 (magnet)
    'graspm': {
        'group': 'arm',
        'joints': {
            'jrotate1': -0.175,
            'jlink1':    0.66,
            'jlink2':    0.292,
            'jrotate2':  0.0,
            'jend':     -1.2182,
        }
    },

    # Post-graspm: safe retreat after attaching CONNECTOR 2 (magnet)
    'post_graspm': {
        'group': 'arm',
        'joints': {
            'jrotate1': -0.175,
            'jlink1':    0.0434,
            'jlink2':   -0.5699,
            'jrotate2':  0.0,
            'jend':     -0.9862,
        }
    },

    # ── GRIPPER POSES (Connector 1 — jsgear controls) ─────────────────────────
    # Note: jhorn is set to 0.0 here — it doesn't move when using the gripper

    # Gripper open: jsgear rotates to open the gripper fingers
    'gripper_open': {
        'group': 'grippers',
        'joints': {
            'jsgear': -0.8,   # rotated open
            'jhorn':   0.0,   # not used for this connector
        }
    },

    # Gripper close: jsgear returns to zero — fingers closed around object
    'gripper_close': {
        'group': 'grippers',
        'joints': {
            'jsgear':  0.0,   # closed position
            'jhorn':   0.0,   # not used for this connector
        }
    },

    # ── MAGNET POSES (Connector 2 — jhorn controls) ───────────────────────────
    # Note: jsgear is set to 0.0 here — it doesn't move when using the magnet

    # Magnet on: jhorn at zero = magnet is engaged (contact with object)
    'magnet_on': {
        'group': 'grippers',
        'joints': {
            'jsgear':  0.0,   # not used for this connector
            'jhorn':   0.0,   # magnet engaged position
        }
    },

    # Magnet off: jhorn rotates = magnet disengages from object
    'magnet_off': {
        'group': 'grippers',
        'joints': {
            'jsgear':  0.0,   # not used for this connector
            'jhorn':  -0.8,   # magnet released position
        }
    },

}


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 3: THE SEQUENCE
#  ─────────────────────────
#  This is the list of moves the robot will execute, in order.
#  Edit this list to change what the robot does.
#
#  Each step is a Python dictionary with these keys:
#    'pose'  → (required) name of a pose from NAMED_POSES above
#    'speed' → (optional) how fast to move, 0.01 (slowest) to 1.0 (fastest)
#              Default if omitted: 0.3
#              The actual speed = speed × max_velocity from joint_limits.yaml
#              Example: speed=0.1, max_velocity=6.0 rad/s → actual=0.6 rad/s
#    'wait'  → (optional) seconds to pause AFTER this move finishes
#              Default if omitted: 0.5 seconds
#              Use this to give the robot time to settle, or for humans to observe
#
#  SEQUENCE LOGIC:
#    Part 1  (steps 1-5):   Go home, approach/attach Connector 1 (gripper),
#                            lift away, return home
#    Part 2  (steps 6-7):   Test gripper open/close
#    Part 3  (steps 8-11):  Reverse: put Connector 1 back, return home
#    Part 4  (steps 12-14): Go to Connector 2 (magnet), attach, lift away
#    Part 5  (steps 15-16): Test magnet on/off
#    Part 6  (steps 17-21): Reverse: put Connector 2 back, return home
# ══════════════════════════════════════════════════════════════════════════════

SEQUENCE = [
    # ── Part 1: Pick up Connector 1 (gripper) ─────────────────────────────────
    {'pose': 'home',          'speed': 0.3, 'wait': 1.0},  # Step 1 : Start at home
    {'pose': 'pre_grasp',     'speed': 0.2, 'wait': 0.5},  # Step 2 : Approach above connector 1
    {'pose': 'grasp',         'speed': 0.1, 'wait': 1.0},  # Step 3 : Insert into connector 1 (slow!)
    {'pose': 'post_grasp',    'speed': 0.2, 'wait': 1.0},  # Step 4 : Lift connector 1 away
    {'pose': 'home',          'speed': 0.3, 'wait': 1.0},  # Step 5 : Return to home

    # ── Part 2: Test Connector 1 (gripper open/close) ─────────────────────────
    {'pose': 'gripper_open',  'speed': 0.2, 'wait': 1.0},  # Step 6 : Open gripper fingers
    {'pose': 'gripper_close', 'speed': 0.2, 'wait': 1.0},  # Step 7 : Close gripper fingers

    # ── Part 3: Return Connector 1 (reverse of Part 1) ────────────────────────
    {'pose': 'post_grasp',    'speed': 0.2, 'wait': 0.5},  # Step 8 : Move above dock
    {'pose': 'grasp',         'speed': 0.1, 'wait': 1.0},  # Step 9 : Lower into dock (slow!)
    {'pose': 'pre_grasp',     'speed': 0.2, 'wait': 0.5},  # Step 10: Pull up from dock
    {'pose': 'home',          'speed': 0.3, 'wait': 1.0},  # Step 11: Return home

    # ── Part 4: Pick up Connector 2 (magnet) ──────────────────────────────────
    {'pose': 'pre_graspm',    'speed': 0.2, 'wait': 0.5},  # Step 12: Approach above connector 2
    {'pose': 'graspm',        'speed': 0.1, 'wait': 1.0},  # Step 13: Insert into connector 2
    {'pose': 'post_graspm',   'speed': 0.2, 'wait': 1.0},  # Step 14: Lift connector 2 away

    # ── Part 5: Test Connector 2 (magnet on/off) ──────────────────────────────
    {'pose': 'magnet_on',     'speed': 0.2, 'wait': 1.0},  # Step 15: Engage magnet
    {'pose': 'magnet_off',    'speed': 0.2, 'wait': 1.0},  # Step 16: Release magnet

    # ── Part 6: Return Connector 2 (reverse of Part 4) ────────────────────────
    {'pose': 'post_graspm',   'speed': 0.2, 'wait': 0.5},  # Step 17: Move above dock
    {'pose': 'graspm',        'speed': 0.1, 'wait': 1.0},  # Step 18: Lower into dock
    {'pose': 'pre_graspm',    'speed': 0.2, 'wait': 0.5},  # Step 19: Pull up from dock
    {'pose': 'home',          'speed': 0.3, 'wait': 1.0},  # Step 20: Return home
]


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 4: THE EXECUTOR NODE
#  ─────────────────────────────
#  This class handles all communication with MoveIt2.
#  You don't need to edit this — just change NAMED_POSES and SEQUENCE above.
# ══════════════════════════════════════════════════════════════════════════════

class SequenceExecutor(Node):
    """
    A ROS2 node that sends motion goals to MoveIt2 one at a time.

    Think of this like a "boss" that tells MoveIt2 what to do next,
    waits for it to finish, then gives the next instruction.

    Communication uses ROS2 "actions", which are like function calls that:
      - Take time to complete (unlike a regular function call)
      - Send back progress updates while running
      - Return a final result (success or failure)
    """

    def __init__(self):
        super().__init__('sequence_executor')

        # Create an "action client" connected to MoveIt2's move action server
        # Think of this like a phone line to MoveIt2.
        # '/move_action' is the name of MoveIt2's main planning and execution service
        self._client = ActionClient(self, MoveGroup, '/move_action')

        self.get_logger().info('Connecting to MoveIt2 (/move_action)...')
        self._client.wait_for_server()  # Blocks until MoveIt2 is ready to accept goals
        self.get_logger().info('✓ Connected! Ready to execute sequence.')

    # ──────────────────────────────────────────────────────────────────────────
    #  BUILD A JOINT CONSTRAINT LIST
    # ──────────────────────────────────────────────────────────────────────────

    def _build_joint_constraints(self, joints: dict) -> list:
        """
        Converts a simple {joint_name: angle} dict into the JointConstraint
        objects that MoveIt2 expects as its goal.

        A JointConstraint tells MoveIt2: "I want joint X to be at angle Y,
        within a tolerance of ±0.001 radians (±0.057 degrees)."

        The tolerance must be small enough to reach the pose accurately but
        not zero (zero tolerance = impossible to satisfy exactly).
        """
        constraints = []

        for joint_name, target_angle in joints.items():
            c = JointConstraint()
            c.joint_name      = joint_name         # e.g. 'jrotate1'
            c.position        = float(target_angle) # target angle in radians
            c.tolerance_above = 0.001               # allowed error above target
            c.tolerance_below = 0.001               # allowed error below target
            c.weight          = 1.0                 # how important this constraint is (0-1)
            constraints.append(c)

        return constraints

    # ──────────────────────────────────────────────────────────────────────────
    #  MOVE TO ONE POSE
    # ──────────────────────────────────────────────────────────────────────────

    def move_to(self, step: dict) -> bool:
        """
        Sends a single pose goal to MoveIt2 and waits for it to finish.

        Returns True if the move succeeded, False if it failed.

        WHAT HAPPENS INSIDE MOVEIT2 when we call this:
          1. MoveIt2 receives our goal
          2. OMPL (the path planner) finds a collision-free path from
             current position to the target
          3. The path is parameterized (speed/acceleration profiles added)
          4. JointTrajectoryController executes it step by step
          5. FakeSystem publishes positions on /joint_states
          6. MoveIt2 confirms when the goal is reached
        """

        # ── Resolve pose name ─────────────────────────────────────────────────
        pose_name = step.get('pose')  # e.g. 'home' or 'pre_grasp'

        if pose_name:
            if pose_name not in NAMED_POSES:
                self.get_logger().error(
                    f"Unknown pose '{pose_name}'. "
                    f"Available poses: {list(NAMED_POSES.keys())}")
                return False
            group  = NAMED_POSES[pose_name]['group']
            joints = NAMED_POSES[pose_name]['joints']
        else:
            # Allow custom joint positions directly in the step dict
            group  = step['group']
            joints = step['joints']

        # Speed clamped between 1% and 100% of max velocity
        speed = float(step.get('speed', 0.3))
        speed = max(0.01, min(1.0, speed))

        self.get_logger().info(
            f"  ▶ Moving to '{pose_name or 'custom'}' "
            f"(group={group}, speed={speed*100:.0f}%)")

        # ── Build the goal message ─────────────────────────────────────────────
        goal = MoveGroup.Goal()           # The outer goal container
        req  = MotionPlanRequest()        # The actual planning request inside

        # Which joints to plan for
        req.group_name = group            # Must match a group name in full.srdf

        # Define the workspace (a box the robot is allowed to move inside)
        # This helps the planner — making it smaller can speed up planning
        ws = WorkspaceParameters()
        ws.header.frame_id = 'base'       # The reference frame (world origin)
        ws.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)  # 1 meter cube
        ws.max_corner = Vector3(x= 1.0, y= 1.0, z= 1.0)
        req.workspace_parameters = ws

        # Set the target joint positions as constraints
        req.goal_constraints.append(
            Constraints(joint_constraints=self._build_joint_constraints(joints))
        )

        # Planning settings
        req.num_planning_attempts    = 5     # Try up to 5 times if first attempt fails
        req.allowed_planning_time    = 5.0   # Give planner up to 5 seconds
        req.max_velocity_scaling_factor     = speed        # Scale max speed
        req.max_acceleration_scaling_factor = speed * 0.5  # Scale acceleration (half of speed)
        req.pipeline_id = 'ompl'             # Use OMPL (the default planner family)
        req.planner_id  = 'RRTConnect'       # RRTConnect is fast and reliable for arms

        # Execution options
        goal.request = req
        goal.planning_options.plan_only       = False  # Plan AND execute (not just plan)
        goal.planning_options.replan          = True   # Replan if something goes wrong
        goal.planning_options.replan_attempts = 3      # Try replanning up to 3 times
        goal.planning_options.replan_delay    = 0.5    # Wait 0.5s between replan attempts

        # ── Send goal and wait for result ──────────────────────────────────────
        # "Async" means: start the action and give us a "future" (a promise of a result)
        # rclpy.spin_until_future_complete() then waits for that promise to be fulfilled
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        # Check if MoveIt2 accepted our goal at all
        if not goal_handle.accepted:
            self.get_logger().error(f"  ✗ Goal rejected by MoveIt2 for '{pose_name}'")
            return False

        # Wait for the actual execution to complete
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        status = result.status

        # Check the outcome
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"  ✓ '{pose_name}' completed successfully")
            return True
        else:
            err_code = result.result.error_code.val
            # Common error codes: 1=SUCCESS, -1=FAILURE, -3=NO_IK_SOLUTION,
            #                    -6=START_STATE_IN_COLLISION, -12=TIMED_OUT
            self.get_logger().error(
                f"  ✗ '{pose_name}' failed  "
                f"(status={status}, error_code={err_code})\n"
                f"  Hint: If error_code=-12, try increasing 'allowed_planning_time'\n"
                f"  Hint: If error_code=-6, a collision was detected in the start state"
            )
            return False

    # ──────────────────────────────────────────────────────────────────────────
    #  RUN ALL STEPS
    # ──────────────────────────────────────────────────────────────────────────

    def run_sequence(self, sequence: list, stop_on_failure: bool = True) -> bool:
        """
        Executes all steps in a sequence list, one at a time.

        Args:
            sequence: List of step dicts (from SEQUENCE above)
            stop_on_failure: If True (default), the sequence stops when any
                             step fails. If False, it logs the failure and
                             continues to the next step.

        Returns True if all steps succeeded, False if any step failed.
        """
        total = len(sequence)
        self.get_logger().info(
            f'\n{"═"*55}\n'
            f'  STARTING SEQUENCE: {total} steps\n'
            f'{"═"*55}'
        )

        for i, step in enumerate(sequence):
            pose_name = step.get('pose', 'custom_joints')
            self.get_logger().info(f'\nStep {i+1:2d}/{total}: {pose_name}')

            success = self.move_to(step)

            if not success:
                self.get_logger().error(f'Step {i+1} FAILED.')
                if stop_on_failure:
                    self.get_logger().error(
                        'Stopping sequence because stop_on_failure=True.\n'
                        'Fix the failed step and restart.')
                    return False
                else:
                    self.get_logger().warn('Continuing to next step (stop_on_failure=False).')

            # Pause after each step (let robot settle, human observe)
            wait_time = float(step.get('wait', 0.5))
            if wait_time > 0:
                self.get_logger().info(f'  ⏱ Waiting {wait_time}s...')
                time.sleep(wait_time)

        self.get_logger().info(
            f'\n{"═"*55}\n'
            f'  SEQUENCE COMPLETE! All {total} steps done.\n'
            f'{"═"*55}'
        )
        return True


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main():
    """
    Main function: starts ROS2, creates the executor node, runs the sequence.

    To run the sequence in a loop, you could change:
        node.run_sequence(SEQUENCE)
    to:
        while rclpy.ok():
            node.run_sequence(SEQUENCE)
            time.sleep(2.0)   # pause between loops
    """
    rclpy.init()
    node = SequenceExecutor()

    try:
        # Run the full sequence once
        # Change stop_on_failure=False if you want to skip failed poses
        node.run_sequence(SEQUENCE, stop_on_failure=True)

    except KeyboardInterrupt:
        node.get_logger().warn('\nInterrupted by user (Ctrl+C). Stopping.')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
