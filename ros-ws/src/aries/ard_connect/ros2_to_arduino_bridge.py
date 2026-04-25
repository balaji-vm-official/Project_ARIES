#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                           ros2_to_arduino_bridge.py                          ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  WHAT THIS FILE DOES:                                                        ║
║    This is the "translator" between ROS2/MoveIt2 and your Arduino UNO.       ║
║    MoveIt2 thinks in joint names and radians (like -1.57 rad for 90°).       ║
║    Arduino controls servos using degrees (0° to 180°).                       ║
║    This script does the conversion and sends commands over USB cable.        ║
║                                                                              ║
║  DATA FLOW (how information travels):                                        ║
║    MoveIt2 plans a path                                                      ║
║      → JointTrajectoryController executes it step by step                    ║
║        → FakeSystem publishes positions on /joint_states 100x per second     ║
║          → THIS FILE reads /joint_states                                     ║
║            → converts radians to servo degrees                               ║
║              → sends "090,110,110,090,090,090\n" over USB                    ║
║                → Arduino moves the physical servos smoothly                  ║
║                                                                              ║
║  HOW TO RUN:                                                                 ║
║    This file is launched automatically by demo.launch.py after 8 seconds.    ║
║    You don't need to run it manually.                                        ║
║                                                                              ║
║  FINDING YOUR ARDUINO PORT:                                                  ║
║    Plug in Arduino, then run:  ls /dev/tty* | grep -E 'USB|ACM'              ║
║    Update serial_port in demo.launch.py to match (usually /dev/ttyACM0).     ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

# ── IMPORTS ────────────────────────────────────────────────────────────────────
# Think of imports like "loading tools you need before starting work"

import rclpy                        # The main ROS2 Python library
from rclpy.node import Node         # Base class for all ROS2 programs
from sensor_msgs.msg import JointState  # Message type carrying joint positions

import serial                        # PySerial: talks to Arduino over USB
import serial.tools.list_ports       # Lists available USB ports (for debugging)
import math                          # For math.degrees() to convert radians→degrees
import time                          # For time.sleep() to wait on startup
import threading                     # Lets us read Arduino in background


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 1: JOINT CONFIGURATION TABLE
#  ─────────────────────────────────────
#  This is the most important section to understand.
#  When you get a new servo or change hardware, you edit this table.
# ══════════════════════════════════════════════════════════════════════════════

# JOINT_MAP: tells the bridge how to handle each joint
#
# Format:  'joint_name': (serial_index, zero_angle, direction, limit_low, limit_high)
#
# serial_index  — which slot (0 to 5) in the CSV string goes to which servo
#                 Index 0 = first number in "090,110,110,090,090,090"
#                 Index 5 = last number
#
# zero_angle    — the servo DEGREE when ROS2 joint position = 0.0 radians
#                 Most joints: 90° (servo center = robot zero position)
#                 jlink1, jlink2: 110° (these servos were physically installed
#                 at 110° to match the URDF's zero pose)
#
# direction     — +1 means "positive radian = higher degree"
#                 -1 means "positive radian = lower degree" (reversed servo)
#                 If the real robot moves the WRONG WAY compared to RViz,
#                 change +1 to -1 (or vice versa) for that joint.
#
# limit_low/high — joint limits in radians, matching your URDF file
#                  The script clamps values to stay inside these limits
#
# ⚠ IMPORTANT — SHARED PIN (jsgear and jhorn):
#   Both connector end effectors use the SAME Arduino pin 11.
#   You only ever attach ONE end effector at a time (gripper OR magnet).
#   So both joints are given serial_index = 5, and whichever one ROS2
#   commands simply overwrites position 5 in the CSV string.

JOINT_MAP = {
    # Joint Name    (idx, zero°, dir,  low_rad, high_rad)
    # ─────────────────────────────────────────────────────────────────
    # MG996R motor — rotates the whole arm base left/right
    'jrotate1':  (  0,   90,  +1,   -1.57,   1.57  ),

    # MG996R motor — lifts/lowers the first arm segment
    # zero=110 because this servo was physically set to 110° at the URDF zero pose
    'jlink1':    (  1,  110,  +1,   -1.222,  1.92  ),

    # MG996R motor — lifts/lowers the second arm segment
    # same physical situation as jlink1
    'jlink2':    (  2,  110,  +1,   -1.222,  1.92  ),

    # MG90S motor — tilts the end (up/down at wrist)
    'jend':      (  4,   90,  +1,   -1.57,   1.57  ),

    # SG90 motor — CONNECTOR 1 (gripper gear).
    # Shares Arduino pin 11 with jhorn below.
    # When the GRIPPER end effector is attached, this controls it.
    'jsgear':    (  5,   90,  +1,   -1.3,    0.0   ),

    # SG90 motor — CONNECTOR 2 (magnet horn).
    # Shares Arduino pin 11 with jsgear above.
    # When the MAGNET end effector is attached, this controls it.
    # Both use index 5 — last command sent to the Arduino wins.
    'jhorn':     (  5,   90,  +1,   -1.3,    0.0   ),
}

# How many values we send per message. Must match Arduino's #define NUM_JOINTS
NUM_JOINTS = 6

# How much a joint must move before we bother sending an update.
# Without this, we'd flood the Arduino with 100 identical messages per second.
# 0.01 radians ≈ 0.57 degrees — small enough to be smooth, large enough to filter noise.
CHANGE_THRESHOLD = 0.01  # radians


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 2: CONVERSION FUNCTION
#  ────────────────────────────────
#  Pure math: converts radians (ROS2) to degrees (servo hardware)
# ══════════════════════════════════════════════════════════════════════════════

def radians_to_servo(joint_name: str, radians: float) -> int:
    """
    Converts a joint angle from radians (ROS2 format) to servo degrees (0-180).

    The formula is:
        servo_angle = zero_angle + math.degrees(radians) * direction

    WORKED EXAMPLES:
      jrotate1 (zero=90, dir=+1):
        0.0 rad  →  90 + 0°    = 90°  (servo at center)
        1.57 rad →  90 + 90°   = 180° (full forward)
       -1.57 rad →  90 + -90°  = 0°   (full backward)

      jlink1 (zero=110, dir=+1):
        0.0 rad  →  110 + 0°   = 110° (robot's zero pose)
        0.5 rad  →  110 + 28.6° = 138°
       -0.5 rad  →  110 - 28.6° = 81°

    Args:
        joint_name: Key from JOINT_MAP (e.g. 'jrotate1')
        radians:    The angle from ROS2 /joint_states topic

    Returns:
        An integer between 0 and 180 (servo degrees)
    """
    _, zero_servo, direction, low, high = JOINT_MAP[joint_name]

    # Safety clamp: keep within URDF joint limits
    radians = max(low, min(high, radians))

    # The core conversion
    servo_angle = int(zero_servo + math.degrees(radians) * direction)

    # Hardware limit: servos physically cannot go outside 0-180
    return max(0, min(180, servo_angle))


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 3: THE MAIN NODE CLASS
#  ────────────────────────────────
#  In ROS2, every program is a "Node". A Node can subscribe to topics
#  (receive data) and publish to topics (send data).
# ══════════════════════════════════════════════════════════════════════════════

class ArduinoBridge(Node):
    """
    ROS2 Node that reads joint states and sends servo commands to Arduino.

    This class SUBSCRIBES to:   /joint_states     (from FakeSystem/ros2_control)
    This class PUBLISHES to:    /joint_states_real (feedback from Arduino)
    This class WRITES to:       USB serial port    (commands to Arduino)
    This class READS from:      USB serial port    (confirmations from Arduino)
    """

    def __init__(self):
        # Initialize as a ROS2 node named 'ros2_to_arduino_bridge'
        # The name shows up in "ros2 node list" command
        super().__init__('ros2_to_arduino_bridge')

        # ── PARAMETERS ────────────────────────────────────────────────────────
        # Parameters let you change settings at launch time without editing code.
        # In demo.launch.py you can set:
        #   Node(parameters=[{'serial_port': '/dev/ttyACM0', 'baud_rate': 115200}])
        self.declare_parameter('serial_port', '/dev/ttyACM0')  # Change if your port differs eg: ttyACM0,
        self.declare_parameter('baud_rate',   115200)           # Must match Arduino sketch
        self.declare_parameter('dry_run',     False)            # True = test mode, no Arduino

        self.port    = self.get_parameter('serial_port').value
        self.baud    = self.get_parameter('baud_rate').value
        self.dry_run = self.get_parameter('dry_run').value

        # ── STATE MEMORY ──────────────────────────────────────────────────────
        # Remember last sent positions to detect changes (the dead-zone filter)
        # None means "never sent anything yet" — force first send regardless
        self.last_sent = {j: None for j in JOINT_MAP}

        # ── SERIAL SETUP ──────────────────────────────────────────────────────
        self.serial_conn = None

        # threading.Lock() = a "mutex" — ensures only one thread uses serial at once
        # Without this, the main thread (sending commands) and background thread
        # (reading responses) could write/read at the same time and corrupt data
        self._serial_lock = threading.Lock()

        if not self.dry_run:
            self._connect_serial()
        else:
            self.get_logger().warn(
                'DRY RUN mode active — Arduino NOT connected. '
                'Commands will be printed to terminal for testing.')

        # ── SUBSCRIPTION ──────────────────────────────────────────────────────
        # "Subscribe" means: whenever /joint_states publishes a message,
        # automatically call our _joint_state_cb() function.
        # This is the core mechanism — no manual polling needed.
        self.create_subscription(
            JointState,            # The type of message we expect
            '/joint_states',       # The topic we listen to
            self._joint_state_cb,  # The function called on each message
            10                     # Buffer up to 10 messages if we're busy
        )

        # ── PUBLISHER ─────────────────────────────────────────────────────────
        # Publishes what the Arduino confirms back, as ROS2 joint states.
        # Optional but useful for debugging. View with: ros2 topic echo /joint_states_real
        self.real_joint_pub = self.create_publisher(JointState, '/joint_states_real', 10)

        # ── BACKGROUND READER ─────────────────────────────────────────────────
        # Start reading Arduino responses in a background thread.
        # "daemon=True" means this thread will auto-stop when the main program ends.
        if self.serial_conn:
            threading.Thread(target=self._serial_reader, daemon=True).start()

        self.get_logger().info(
            f'\n{"─"*50}\n'
            f'Arduino Bridge READY\n'
            f'  Port     : {self.port}\n'
            f'  Baud rate: {self.baud}\n'
            f'  Dry run  : {self.dry_run}\n'
            f'{"─"*50}'
        )

        # Map ROS2 joint names to the Arduino array index (0 to 5)                                     BY GEMINI
        # Mapping based on your hardware pins:
        # Slot 0 (Pin 3)  -> jrotate1
        # Slot 1 (Pin 5)  -> jlink1
        # Slot 2 (Pin 6)  -> jlink2
        # Slot 3 (Pin 9)  -> (Fixed - No ROS joint mapped)
        # Slot 4 (Pin 10) -> jend
        # Slot 5 (Pin 11) -> jhorn AND jsgear (Shared)
        
        self.joint_mapping = {
            'jrotate1': 0,
            'jlink1': 1,
            'jlink2': 2,
            'jend': 4,
            'jhorn': 5,
            'jsgear': 5
        }

    # ══════════════════════════════════════════════════════════════════════════
    #  CONNECTING TO ARDUINO
    # ══════════════════════════════════════════════════════════════════════════

    def _connect_serial(self):
        """
        Opens the USB serial connection to Arduino.
        If the port doesn't exist, logs a helpful error and continues without it.
        The node still runs — it just won't send commands to the physical robot.
        """
        available = [p.device for p in serial.tools.list_ports.comports()]
        self.get_logger().info(f'Available serial ports: {available}')

        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=1.0  # 1 second read timeout
            )
            # Arduino UNO automatically resets when a serial connection opens.
            # We must wait 2 seconds for it to finish booting and reach setup().
            # Skipping this wait = Arduino not ready = commands get lost.
            self.get_logger().info('Waiting 2s for Arduino to boot...')
            time.sleep(2.0)
            self.get_logger().info(f'✓ Connected to Arduino on {self.port}')

        except serial.SerialException as e:
            self.get_logger().error(
                f'✗ Could not open port {self.port}: {e}\n\n'
                f'  HOW TO FIX:\n'
                f'  1. Check Arduino is plugged in\n'
                f'  2. Run: ls /dev/tty* | grep -E "USB|ACM"\n'
                f'  3. Update serial_port in demo.launch.py\n'
            )
            self.serial_conn = None

    # ══════════════════════════════════════════════════════════════════════════
    #  MAIN CALLBACK — runs every time /joint_states publishes (100 Hz)
    # ══════════════════════════════════════════════════════════════════════════

    def _joint_state_cb(self, msg: JointState):
        """
        Called automatically 100 times per second during MoveIt2 execution.

        The msg looks like this:
            msg.name     = ['jrotate1', 'jlink1', 'jlink2', 'jrotate2', 'jend', 'jsgear', 'jhorn', ...]
            msg.position = [0.0,        0.5572,   0.292,    0.0,        -1.268, -0.8,      0.0,    ...]
                            (these are the current joint angles in RADIANS)

        We zip name and position together, like a zipper joining two lists.
        """

        # STEP 1: Pull out only the joints we know how to handle
        positions = {}
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_MAP:
                positions[name] = pos  # e.g. positions['jrotate1'] = 0.0

        # STEP 2: Check we have all 6 servo slots covered before sending.
        # jsgear and jhorn both cover index 5, so we check unique indices.
        # Check that the 4 controllable arm joints are all present.
        # jrotate2 is a fixed joint now (removed from URDF control) so we
        # don't wait for it — slot 3 is filled with 90° in _send_positions.
        REQUIRED_JOINTS = ['jrotate1', 'jlink1', 'jlink2', 'jend']
        if not all(j in positions for j in REQUIRED_JOINTS):
            return  # Still waiting for arm joints — hold off

        # STEP 3: Dead-zone filter — only send if something changed enough
        # Without this check, we'd send 100 identical messages per second
        # when the robot is sitting still, wasting serial bandwidth.
        changed = any(
            self.last_sent[j] is None or
            abs(positions[j] - self.last_sent[j]) > CHANGE_THRESHOLD
            for j in positions
        )
        if not changed:
            return  # Nothing moved meaningfully, skip this message

        # STEP 4: Send the new positions to Arduino
        self._send_positions(positions)

        # STEP 5: Update our memory of what we sent
        self.last_sent.update(positions)

    # ══════════════════════════════════════════════════════════════════════════
    #  BUILDING AND SENDING THE SERIAL MESSAGE
    # ══════════════════════════════════════════════════════════════════════════

    def _send_positions(self, positions: dict):
        """
        Converts all joint positions to servo degrees and sends to Arduino.

        The Arduino expects exactly this format (6 numbers, 0-180, zero-padded):
            "090,138,118,090,090,162\n"
             J0  J1  J2  J3  J4  J5

        Corresponding to these physical servos:
            J0 = jrotate1 (base)
            J1 = jlink1   (arm segment 1)
            J2 = jlink2   (arm segment 2)
            J3 = jrotate2 (wrist rotation)
            J4 = jend     (wrist tilt)
            J5 = jsgear OR jhorn (whichever end effector is currently attached)
        """

        # Start with default safe positions for all 6 slots
        angles = [90] * NUM_JOINTS  # 90° = center for most servos
        angles[1] = 110             # jlink1 default (calibrated to 110°)
        angles[2] = 110             # jlink2 default (calibrated to 110°)
        angles[3] = 90              # jrotate2: fixed joint in URDF — servo stays at 90° always
                                    # Arduino still expects 6 values, so we fill this slot

        # Now overwrite defaults with actual commanded positions
        for joint_name, pos in positions.items():
            if joint_name in JOINT_MAP:
                idx = JOINT_MAP[joint_name][0]          # Which slot (0-5)
                angles[idx] = radians_to_servo(joint_name, pos)  # Convert
            else:
                # Skip joints that the Arduino doesn't control
                continue

        # Format as "090,110,110,090,090,090\n"
        # f'{a:03d}' means: format as integer, minimum 3 digits, pad with zeros
        msg_str = ','.join(f'{a:03d}' for a in angles) + '\n'

        # Dry run: just print what we would send, don't touch serial port
        if self.dry_run:
            self.get_logger().info(f'[DRY RUN] → {msg_str.strip()}')
            return

        # Send over serial (protected by lock so only one thread at a time)
        with self._serial_lock:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write(msg_str.encode('ascii'))  # String → bytes
                    self.serial_conn.flush()  # Push bytes out immediately (don't buffer)
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')

    # ══════════════════════════════════════════════════════════════════════════
    #  BACKGROUND THREAD: Reading Arduino responses
    # ══════════════════════════════════════════════════════════════════════════

    def _serial_reader(self):
        """
        Runs in a background thread, reading whatever Arduino sends back.

        Arduino sends:
          "READY\n"                  — when it finishes setup()
          "ERR:bad_count\n"          — if it got a message with wrong format
          "090,138,118,090,090,162\n"— current servo positions while moving

        This thread runs independently of the main loop, checking constantly
        without blocking anything else.
        """
        while rclpy.ok():  # Keep running until ROS2 shuts down
            try:
                # Check if Arduino sent anything (non-blocking check)
                with self._serial_lock:
                    has_data = (self.serial_conn and
                                self.serial_conn.in_waiting > 0)

                if has_data:
                    with self._serial_lock:
                        line = self.serial_conn.readline()  # Read until newline

                    # Decode bytes to text, ignore any weird characters
                    text = line.decode('ascii', errors='ignore').strip()
                    self._parse_arduino_response(text)

            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')

            time.sleep(0.01)  # Check 100 times per second

    def _parse_arduino_response(self, line: str):
        """
        Processes one line of text received from Arduino.

        Status messages get logged. Position feedback gets converted
        back to radians and published as /joint_states_real in ROS2.
        """
        if not line:
            return

        # Log startup and error messages from Arduino
        if line.startswith('READY') or line.startswith('ERR'):
            self.get_logger().info(f'Arduino: {line}')
            return

        # Try to parse as 6 servo angles (position feedback)
        try:
            parts = line.split(',')
            if len(parts) != NUM_JOINTS:
                return  # Wrong format — skip silently

            angles = [int(p) for p in parts]

            # Build a ROS2 JointState message with the real servo positions
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()

            # Only use one joint per serial index to avoid duplicates
            # (jsgear and jhorn both map to index 5 — we report as jsgear)
            unique_joints = ['jrotate1', 'jlink1', 'jlink2', 'jend', 'jsgear']  # jrotate2 removed (fixed joint)
            msg.name = unique_joints
            msg.position = []

            for joint_name in unique_joints:
                idx, zero_servo, direction, _, _ = JOINT_MAP[joint_name]
                # Reverse conversion: servo degrees → radians
                rad = math.radians((angles[idx] - zero_servo) * direction)
                msg.position.append(rad)

            self.real_joint_pub.publish(msg)

        except (ValueError, IndexError):
            pass  # Ignore any lines we can't parse

    # ══════════════════════════════════════════════════════════════════════════
    #  CLEANUP
    # ══════════════════════════════════════════════════════════════════════════

    def destroy_node(self):
        """Called automatically when the node shuts down. Closes serial safely."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('Serial port closed cleanly.')
        super().destroy_node()


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT — This runs when you execute the file directly
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)      # Start the ROS2 communications system
    node = ArduinoBridge()     # Create and initialize our bridge node

    try:
        rclpy.spin(node)       # Keep running, processing callbacks as they arrive
    except KeyboardInterrupt:
        pass                   # Graceful exit on Ctrl+C
    finally:
        node.destroy_node()    # Close serial port etc.
        rclpy.shutdown()       # Stop ROS2 communications


if __name__ == '__main__':
    main()