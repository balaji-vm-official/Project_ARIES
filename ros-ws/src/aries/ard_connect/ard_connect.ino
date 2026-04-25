/*
╔══════════════════════════════════════════════════════════════════════════════╗
║                         ros2_to_arduino.ino                                  ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  WHAT THIS FILE DOES:                                                        ║
║    Runs ON the Arduino UNO. Receives servo angle commands from the laptop    ║
║    over USB, then smoothly moves each servo to its target angle.             ║
║                                                                              ║
║  PROTOCOL (how laptop and Arduino talk):                                     ║
║    Laptop → Arduino : "090,110,110,090,090,090\n"                            ║
║                        6 angles (0-180), comma-separated, newline at end     ║
║    Arduino → Laptop : same format echoed back as position feedback           ║
║    Arduino → Laptop : "READY\n" on startup                                   ║
║    Arduino → Laptop : "ERR:bad_count\n" if wrong number of values received   ║
║                                                                              ║
║  SERVO PIN ASSIGNMENTS:                                                      ║
║    Slot 0 → Pin  3  → jrotate1  (base rotation)        — MG996R              ║
║    Slot 1 → Pin  5  → jlink1    (arm segment 1)        — MG996R              ║
║    Slot 2 → Pin  6  → jlink2    (arm segment 2)        — MG996R              ║
║    Slot 3 → Pin  9  → jrotate2  (wrist rotation)       — MG90S               ║
║    Slot 4 → Pin 10  → jend      (wrist tilt)           — MG90S               ║
║    Slot 5 → Pin 11  → jsgear OR jhorn (end effector)   — SG90                ║
║                        Both share this pin — only one attached at a time     ║
║                                                                              ║
║  Valid PWM pins on Arduino UNO: 3, 5, 6, 9, 10, 11 (all used above)          ║
╚══════════════════════════════════════════════════════════════════════════════╝
*/

// ── INCLUDE SERVO LIBRARY ─────────────────────────────────────────────────────
// #include is like "import" in Python — loads extra tools into your sketch
// Servo.h is the built-in Arduino library that handles all PWM signal timing
#include <Servo.h>


// ══════════════════════════════════════════════════════════════════════════════
//  CONFIGURATION  ← edit these to change behaviour
// ══════════════════════════════════════════════════════════════════════════════

// How many servos. Must EXACTLY match NUM_JOINTS in ros2_to_arduino_bridge.py
#define NUM_JOINTS 6

// Smooth movement: how many degrees to move each update tick.
// Lower = smoother but takes longer.  Higher = faster but can be jerky.
// At STEP_SIZE=2 and UPDATE_MS=20 (50 Hz), max speed ≈ 100 degrees/second
#define STEP_SIZE  2

// How many milliseconds between each smooth-movement step.
// 20ms = 50 updates per second = 50 Hz
#define UPDATE_MS  20


// ══════════════════════════════════════════════════════════════════════════════
//  HARDWARE SETUP
// ══════════════════════════════════════════════════════════════════════════════

// Create 6 Servo objects — one remote-control per servo motor
Servo servos[NUM_JOINTS];

// Which Arduino pin each servo slot is wired to.
// The position in this array IS the slot number (array[0] = slot 0 = jrotate1)
// All 6 pins are PWM-capable (required for servo control)
//                    Slot :   0   1   2   3   4    5                               
int servo_pins[NUM_JOINTS] = { 3,  5,  6,  9,  10,  11 };

// Starting angle for each servo at power-on (degrees, 0-180)
// These match the zero_angle values in ros2_to_arduino_bridge.py's JOINT_MAP:
//   jrotate1=90, jlink1=110, jlink2=110, jrotate2=90, jend=90, j6=90
//                        Slot :    0   1    2    3   4   5                         
int default_angles[NUM_JOINTS] = {  90, 145, 110, 90, 90, 90 };


// ══════════════════════════════════════════════════════════════════════════════
//  RUNTIME STATE — these variables change as the program runs
// ══════════════════════════════════════════════════════════════════════════════

// Where each servo SHOULD end up (set by parse_command when ROS2 sends angles)
int target_angles[NUM_JOINTS];

// Where each servo IS right now (updated bit-by-bit each step toward target)
int current_angles[NUM_JOINTS];

// Text buffer to collect serial characters until we receive a complete line
char serial_buf[64];
int  buf_idx = 0;

// Timestamp of the last smooth-movement update (for timing with millis())
unsigned long last_update_time = 0;


// ══════════════════════════════════════════════════════════════════════════════
//  SETUP — runs ONCE when Arduino powers on or resets
// ══════════════════════════════════════════════════════════════════════════════

void setup() {
  // Start serial at 115200 baud. Must match baud_rate in ros2_to_arduino_bridge.py.
  // "Baud" = bits per second.  115200 baud ≈ 11,500 characters per second.
  Serial.begin(115200);

  // Initialize every servo to its starting angle
  for (int i = 0; i < NUM_JOINTS; i++) {
    // .attach() connects the Servo object to the physical PWM pin
    servos[i].attach(servo_pins[i]);

    // Set both current and target to the starting angle
    current_angles[i] = default_angles[i];
    target_angles[i]  = default_angles[i];

    // .write() sends a PWM pulse to actually move the servo to that angle
    servos[i].write(default_angles[i]);
  }

  // Short delay so all servos finish reaching their start positions
  // before we start accepting commands from ROS2
  delay(500);

  // Signal to the laptop that Arduino is ready.
  // ros2_to_arduino_bridge.py watches for this "READY" message before sending commands.
  Serial.println("READY");
}


// ══════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP — runs REPEATEDLY forever, as fast as Arduino can manage
// ══════════════════════════════════════════════════════════════════════════════

void loop() {

  // ── READ INCOMING SERIAL DATA ─────────────────────────────────────────────
  // Serial.available() returns how many characters are waiting to be read.
  // We collect them one-by-one, building up a complete message in serial_buf,
  // until we see a newline ('\n') which marks the end of one command.
  while (Serial.available()) {
    char c = Serial.read();     // Read one character at a time

    if (c == '\n') {
      // End of message — process the complete line we built up
      serial_buf[buf_idx] = '\0';   // Null-terminate to make it a valid C string
      parse_command(serial_buf);    // Parse and apply the command
      buf_idx = 0;                  // Reset buffer for the next incoming message
    }
    else if (buf_idx < 63) {
      // Still receiving the current message — keep adding characters
      // The check buf_idx < 63 prevents buffer overflow (a safety guard)
      serial_buf[buf_idx++] = c;
    }
  }

  // ── SMOOTH SERVO MOVEMENT (rate-limited to UPDATE_MS) ─────────────────────
  // millis() returns the number of milliseconds since Arduino started.
  // We only advance the smooth movement every UPDATE_MS milliseconds.
  // This creates a steady heartbeat of tiny movements toward the target angle.
  unsigned long now = millis();
  if (now - last_update_time >= UPDATE_MS) {
    last_update_time = now;
    move_step();    // Each call moves every servo one STEP_SIZE closer to target
  }
}


// ══════════════════════════════════════════════════════════════════════════════
//  FUNCTION: parse_command
//  Parses a received CSV line and updates target angles
//  Input example: "090,110,110,090,090,090"
// ══════════════════════════════════════════════════════════════════════════════

void parse_command(char* buf) {
  int parsed[NUM_JOINTS];  // Temporary storage for the 6 angles we'll read
  int count = 0;

  // strtok splits a string at delimiter characters (commas here).
  // First call: pass the string → returns pointer to the first token ("090")
  // Later calls: pass NULL → continues where it left off
  char* token = strtok(buf, ",");

  while (token != NULL && count < NUM_JOINTS) {
    int angle = atoi(token);  // atoi = convert text "090" to integer 90

    // constrain() clamps between min and max — prevents unsafe values
    parsed[count++] = constrain(angle, 0, 180);

    token = strtok(NULL, ",");  // Move to next number
  }

  if (count == NUM_JOINTS) {
    // Got exactly 6 values — update all servo targets
    for (int i = 0; i < NUM_JOINTS; i++) {
      target_angles[i] = parsed[i];
    }
    send_current_angles();  // Acknowledge receipt with current positions
  }
  else {
    // Wrong number of values — tell ROS2 something went wrong
    Serial.println("ERR:bad_count");
  }
}


// ══════════════════════════════════════════════════════════════════════════════
//  FUNCTION: move_step
//  Advance every servo ONE step (STEP_SIZE degrees) closer to its target.
//  Called 50 times per second from loop() to create smooth motion.
// ══════════════════════════════════════════════════════════════════════════════

void move_step() {
  bool any_moved = false;

  for (int i = 0; i < NUM_JOINTS; i++) {
    if (current_angles[i] < target_angles[i]) {
      // Too low — step up, but don't go past the target (min() prevents overshoot)
      current_angles[i] = min(current_angles[i] + STEP_SIZE, target_angles[i]);
      any_moved = true;
    }
    else if (current_angles[i] > target_angles[i]) {
      // Too high — step down, don't undershoot (max() prevents undershoot)
      current_angles[i] = max(current_angles[i] - STEP_SIZE, target_angles[i]);
      any_moved = true;
    }
    // If current == target, this servo is already there — nothing to do

    // Write the updated position to the physical servo
    servos[i].write(current_angles[i]);
  }

  // While servos are still moving, send live position feedback back to ROS2.
  // This is optional but lets ros2_to_arduino_bridge.py know the real servo positions.
  if (any_moved) {
    send_current_angles();
  }
}


// ══════════════════════════════════════════════════════════════════════════════
//  FUNCTION: send_current_angles
//  Sends current servo positions to the laptop in CSV format.
//  Output example: "090,110,110,090,090,090\n"
// ══════════════════════════════════════════════════════════════════════════════

void send_current_angles() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (i > 0) Serial.print(",");   // Comma separator between values

    // Zero-pad to 3 digits for consistent message length:
    // 9 → "009",  45 → "045",  180 → "180"
    if (current_angles[i] < 100) Serial.print("0");
    if (current_angles[i] < 10)  Serial.print("0");
    Serial.print(current_angles[i]);
  }
  Serial.print("\n");   // Newline marks the end of the message
}
