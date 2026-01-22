// STM32 G474RE Robot Controller - Velocity-based trajectory execution
//
// Commands:
//   WAYPOINT <j1-4> <v1-4> <time_ms>  - Buffer waypoint (4 positions, 4 velocities, time)
//   CLEAR_TRAJ                        - Clear waypoint buffer
//   START_TRAJ                        - Start trajectory execution
//   END_TRAJ                          - Signal end of waypoints
//   JOINTS <j1> <j2> <j3> <j4>        - Direct position move
//   GRIPPER <angle>                   - Set gripper angle (0-180)
//   HOME                              - Set current position as zero
//   GO_HOME                           - Move to home position
//   STATUS                            - Report current joint positions
//   HOMED                             - Query homed state
//   STOP                              - Emergency stop
//   RESET                             - Reset state (for recovery)
//
// Responses:
//   ACK                         - Command received
//   DONE                        - Motion complete
//   POS <j1> <j2> <j3> <j4>     - Current joint positions
//   HOMED / NOT_HOMED           - Homed state
//   STOPPED                     - Emergency stop executed
//   ERROR <message>             - Error occurred

#include <AccelStepper.h>
#include <Servo.h>

// Pin definitions
#define MOTOR1_STEP PA0
#define MOTOR1_DIR  PA1
#define MOTOR2_STEP PB4
#define MOTOR2_DIR  PB5
#define MOTOR3_STEP PB13
#define MOTOR3_DIR  PB14
#define MOTOR4_STEP PC2
#define MOTOR4_DIR  PC3
#define SERVO_PIN   PB1

// Hardware configuration
#define M1_STEPS_PER_REV 800
#define M1_METERS_PER_REV 0.00508f
#define M1_STEPS_PER_METER (M1_STEPS_PER_REV / M1_METERS_PER_REV)

#define M2_STEPS_PER_REV 3200
#define M2_GEAR_RATIO 25
#define M2_STEPS_PER_OUTPUT_REV (M2_STEPS_PER_REV * M2_GEAR_RATIO)
#define M2_STEPS_PER_RAD (M2_STEPS_PER_OUTPUT_REV / (2.0f * PI))

#define M3_STEPS_PER_REV 3200
#define M3_GEAR_RATIO 25
#define M3_STEPS_PER_OUTPUT_REV (M3_STEPS_PER_REV * M3_GEAR_RATIO)
#define M3_STEPS_PER_RAD (M3_STEPS_PER_OUTPUT_REV / (2.0f * PI))

#define M4_STEPS_PER_REV 3200
#define M4_GEAR_RATIO 25
#define M4_STEPS_PER_OUTPUT_REV (M4_STEPS_PER_REV * M4_GEAR_RATIO)
#define M4_STEPS_PER_RAD (M4_STEPS_PER_OUTPUT_REV / (2.0f * PI))

// Create stepper objects
AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP, MOTOR1_DIR);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP, MOTOR2_DIR);
AccelStepper motor3(AccelStepper::DRIVER, MOTOR3_STEP, MOTOR3_DIR);
AccelStepper motor4(AccelStepper::DRIVER, MOTOR4_STEP, MOTOR4_DIR);

// Servo object
Servo gripper;

// Waypoint buffer
struct Waypoint {
  float joints[4];      // Target positions (meters, radians)
  float velocities[4];  // Target velocities (m/s, rad/s)
  uint32_t time_ms;     // When to reach this waypoint
};

#define BUFFER_SIZE 500
Waypoint buffer[BUFFER_SIZE];
int write_ptr = 0;  // Where to write next waypoint
int read_ptr = 0;   // Where we're currently reading from
int buffer_count = 0;  // How many waypoints in buffer

// ===== Buffer Management =====

// Check if buffer is full
bool bufferFull() {
  return buffer_count >= BUFFER_SIZE;
}

// Check if buffer is empty
bool bufferEmpty() {
  return buffer_count == 0;
}

// Add waypoint to buffer (returns false if full)
bool bufferPush(const Waypoint& wp) {
  if (bufferFull()) return false;
  
  int idx = write_ptr % BUFFER_SIZE;
  buffer[idx] = wp;
  write_ptr++;
  buffer_count++;
  return true;
}

// Get waypoint at offset from read position (0 = current, 1 = next, etc.)
// Returns nullptr if offset is out of range
Waypoint* bufferGet(int offset) {
  if (offset < 0 || offset >= buffer_count) return nullptr;
  int idx = (read_ptr + offset) % BUFFER_SIZE;
  return &buffer[idx];
}

// Consume N waypoints from front of buffer
void bufferConsume(int count) {
  if (count > buffer_count) count = buffer_count;
  read_ptr = (read_ptr + count) % BUFFER_SIZE;
  buffer_count -= count;
}

// Clear entire buffer
void bufferClear() {
  write_ptr = 0;
  read_ptr = 0;
  buffer_count = 0;
}

// Get current buffer count
int bufferCount() {
  return buffer_count;
}

// ===== End Buffer Management =====

// Serial reading limits
#define MAX_CHARS_PER_LOOP 100  // Prevent serial reading from blocking too long

// Rate limiting
#define STATUS_RATE_LIMIT_MS 200  // Max 5Hz for STATUS responses

// State
bool isHomed = false;
bool isExecuting = false;
bool isGoingHome = false;
bool isJogging = false;
bool isFinishing = false;  // Final positioning after trajectory ends
uint32_t traj_start_time = 0;
uint32_t last_status_time = 0;  // Rate limit STATUS responses
String inputBuffer = "";
bool commandReady = false;

// Current joint positions (in real units)
float currentJoints[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// Error handling helper
void sendError(const char* message) {
  Serial.print("ERROR: ");
  Serial.println(message);
}

// Parse space-separated floats from a C string
// Returns the number of values successfully parsed
int parseFloats(const char* str, float* values, int maxCount) {
  int count = 0;
  char* end;
  const char* ptr = str;
  
  while (count < maxCount && *ptr != '\0') {
    // Skip leading spaces
    while (*ptr == ' ') ptr++;
    if (*ptr == '\0') break;
    
    // Parse float
    values[count] = strtof(ptr, &end);
    if (ptr == end) break;  // No conversion happened
    
    count++;
    ptr = end;
  }
  
  return count;
}

// Convert joint positions to motor steps
long jointToSteps1(float meters) { return (long)(meters * M1_STEPS_PER_METER); }
long jointToSteps2(float radians) { return (long)(radians * M2_STEPS_PER_RAD); }
long jointToSteps3(float radians) { return (long)(radians * M3_STEPS_PER_RAD); }
long jointToSteps4(float radians) { return (long)(radians * M4_STEPS_PER_RAD); }

// Convert motor steps to joint positions
float stepsToJoint1(long steps) { return (float)steps / M1_STEPS_PER_METER; }
float stepsToJoint2(long steps) { return (float)steps / M2_STEPS_PER_RAD; }
float stepsToJoint3(long steps) { return (float)steps / M3_STEPS_PER_RAD; }
float stepsToJoint4(long steps) { return (float)steps / M4_STEPS_PER_RAD; }

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  
  // Configure motors
  motor1.setMaxSpeed(4000);
  motor1.setAcceleration(1000);
  motor1.setMinPulseWidth(20);   // CL57T driver needs longer pulses
  motor1.setCurrentPosition(0);
  
  motor2.setMaxSpeed(1000);
  motor2.setAcceleration(500);
  motor2.setCurrentPosition(0);
  
  motor3.setMaxSpeed(1500);
  motor3.setAcceleration(500);
  motor3.setCurrentPosition(0);
  
  motor4.setMaxSpeed(1500);
  motor4.setAcceleration(500);
  motor4.setCurrentPosition(0);
  
  // Attach servo
  gripper.attach(SERVO_PIN);
  
  Serial.println("READY");
}

void loop() {
  // Read serial - read all available characters but limit iterations
  int chars_read = 0;
  while (Serial.available() > 0 && chars_read < MAX_CHARS_PER_LOOP) {
    char c = Serial.read();
    chars_read++;
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        commandReady = true;
        break;  // Process this command before reading more
      }
    } else {
      inputBuffer += c;
    }
  }
  
  // Process command
  if (commandReady) {
    processCommand(inputBuffer);
    inputBuffer = "";
    commandReady = false;
  }
  
  // Execute trajectory
  if (isExecuting && !bufferEmpty()) {
    executeTrajectory();
  }
  
  // Execute GO_HOME
  if (isGoingHome) {
    motor1.run();
    motor2.run();
    motor3.run();
    motor4.run();
    
    // Check if all motors reached home
    if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0 && 
        motor3.distanceToGo() == 0 && motor4.distanceToGo() == 0) {
      isGoingHome = false;
      currentJoints[0] = 0.0f;
      currentJoints[1] = 0.0f;
      currentJoints[2] = 0.0f;
      currentJoints[3] = 0.0f;
      Serial.println("DONE");
    }
  }
  
  // Execute JOINTS
  if (isJogging) {
    motor1.run();
    motor2.run();
    motor3.run();
    motor4.run();
    
    if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0 && 
        motor3.distanceToGo() == 0 && motor4.distanceToGo() == 0) {
      isJogging = false;
      currentJoints[0] = stepsToJoint1(motor1.currentPosition());
      currentJoints[1] = stepsToJoint2(motor2.currentPosition());
      currentJoints[2] = stepsToJoint3(motor3.currentPosition());
      currentJoints[3] = stepsToJoint4(motor4.currentPosition());
      Serial.println("DONE");
    }
  }
  
  // Final positioning after trajectory ends
  if (isFinishing) {
    motor1.run();
    motor2.run();
    motor3.run();
    motor4.run();
    
    if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0 && 
        motor3.distanceToGo() == 0 && motor4.distanceToGo() == 0) {
      isFinishing = false;
      currentJoints[0] = stepsToJoint1(motor1.currentPosition());
      currentJoints[1] = stepsToJoint2(motor2.currentPosition());
      currentJoints[2] = stepsToJoint3(motor3.currentPosition());
      currentJoints[3] = stepsToJoint4(motor4.currentPosition());
      Serial.println("DONE");
    }
  }
}

// Position correction gain - adjusts velocity based on position error
// Units: (steps/sec) per step of error
#define POSITION_CORRECTION_GAIN 10.0f

// Cubic Hermite spline interpolation
// https://blog.demofox.org/2015/08/08/cubic-hermite-interpolation/
// Given two waypoints with positions (p0, p1) and velocities (v0, v1),
// compute smooth position and velocity at parameter t (0 to 1)
// 
// Position: p(t) = h00*p0 + h10*dt*v0 + h01*p1 + h11*dt*v1
// Velocity: p'(t) = (h00'*p0 + h10'*dt*v0 + h01'*p1 + h11'*dt*v1) / dt
//
// where h00, h10, h01, h11 are Hermite basis functions
void cubicHermiteInterpolate(
    float p0, float v0,   // Start position and velocity
    float p1, float v1,   // End position and velocity
    float dt_sec,         // Time delta in seconds
    float t,              // Normalized time (0 to 1)
    float* pos_out,       // Output: interpolated position
    float* vel_out        // Output: interpolated velocity
) {
  // Hermite basis functions for position
  float t2 = t * t;
  float t3 = t2 * t;
  float h00 = 2*t3 - 3*t2 + 1;    // p0 coefficient
  float h10 = t3 - 2*t2 + t;       // v0 coefficient (scaled by dt)
  float h01 = -2*t3 + 3*t2;        // p1 coefficient
  float h11 = t3 - t2;             // v1 coefficient (scaled by dt)
  
  // Hermite basis derivatives for velocity
  float h00_d = 6*t2 - 6*t;        // dp0/dt
  float h10_d = 3*t2 - 4*t + 1;    // dv0/dt (scaled by dt)
  float h01_d = -6*t2 + 6*t;       // dp1/dt
  float h11_d = 3*t2 - 2*t;        // dv1/dt (scaled by dt)
  
  // Interpolated position
  *pos_out = h00*p0 + h10*dt_sec*v0 + h01*p1 + h11*dt_sec*v1;
  
  // Interpolated velocity (derivative of position / dt)
  if (dt_sec > 0.0001f) {
    *vel_out = (h00_d*p0 + h10_d*dt_sec*v0 + h01_d*p1 + h11_d*dt_sec*v1) / dt_sec;
  } else {
    *vel_out = v0;
  }
}

void executeTrajectory() {
  if (bufferEmpty()) {
    isExecuting = false;
    
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor3.setSpeed(0);
    motor4.setSpeed(0);
    
    Serial.println("DONE");
    return;
  }
  
  uint32_t current_time = millis() - traj_start_time;
  
  // Find which waypoints we're between
  int prev_idx = -1;
  int next_idx = -1;
  
  for (int i = 0; i < bufferCount(); i++) {
    Waypoint* wp = bufferGet(i);
    if (wp->time_ms <= current_time) {
      prev_idx = i;
    } else {
      next_idx = i;
      break;
    }
  }
  
  // Advance read_ptr past consumed waypoints (keep 1 for interpolation)
  if (prev_idx > 1) {
    int consume_count = prev_idx - 1;
    bufferConsume(consume_count);
    prev_idx = 1;
    if (next_idx >= 0) next_idx -= consume_count;
  }
  
  // Check if we've reached the last waypoint
  if (next_idx == -1 || prev_idx == bufferCount() - 1) {
    // Switch to position mode for precise final positioning
    Waypoint* last_wp = bufferGet(bufferCount() - 1);
    if (last_wp) {
      motor1.moveTo(jointToSteps1(last_wp->joints[0]));
      motor2.moveTo(jointToSteps2(last_wp->joints[1]));
      motor3.moveTo(jointToSteps3(last_wp->joints[2]));
      motor4.moveTo(jointToSteps4(last_wp->joints[3]));
    }
    
    // Transition to finishing state
    isExecuting = false;
    isFinishing = true;
    bufferClear();
    return;
  }
  
  // Interpolate target position and velocity between waypoints using cubic Hermite spline
  float target_pos[4];
  float target_vel[4];
  
  if (prev_idx < 0) {
    // Before first waypoint - use first waypoint values
    Waypoint* first_wp = bufferGet(0);
    if (first_wp) {
      for (int i = 0; i < 4; i++) {
        target_pos[i] = first_wp->joints[i];
        target_vel[i] = first_wp->velocities[i];
      }
    }
  } else {
    Waypoint* prev_wp = bufferGet(prev_idx);
    Waypoint* next_wp = bufferGet(next_idx);
    
    if (prev_wp && next_wp) {
      float time_delta_ms = next_wp->time_ms - prev_wp->time_ms;
      float time_delta_sec = time_delta_ms / 1000.0f;
      float t = (time_delta_ms > 0) ? (float)(current_time - prev_wp->time_ms) / time_delta_ms : 0;
      
      if (t < 0) t = 0;
      if (t > 1) t = 1;
      
      // Cubic Hermite interpolation for each joint
      for (int i = 0; i < 4; i++) {
        cubicHermiteInterpolate(
          prev_wp->joints[i], prev_wp->velocities[i],
          next_wp->joints[i], next_wp->velocities[i],
          time_delta_sec, t,
          &target_pos[i], &target_vel[i]
        );
      }
    }
  }
  
  // Convert target positions to steps
  long target_steps[4];
  target_steps[0] = jointToSteps1(target_pos[0]);
  target_steps[1] = jointToSteps2(target_pos[1]);
  target_steps[2] = jointToSteps3(target_pos[2]);
  target_steps[3] = jointToSteps4(target_pos[3]);
  
  // Get current positions
  long current_steps[4];
  current_steps[0] = motor1.currentPosition();
  current_steps[1] = motor2.currentPosition();
  current_steps[2] = motor3.currentPosition();
  current_steps[3] = motor4.currentPosition();
  
  // Calculate position errors
  long error[4];
  for (int i = 0; i < 4; i++) {
    error[i] = target_steps[i] - current_steps[i];
  }
  
  // Calculate corrected velocities (spline velocity + position correction)
  float corrected_vel[4];
  corrected_vel[0] = target_vel[0] * M1_STEPS_PER_METER + error[0] * POSITION_CORRECTION_GAIN;
  corrected_vel[1] = target_vel[1] * M2_STEPS_PER_RAD + error[1] * POSITION_CORRECTION_GAIN;
  corrected_vel[2] = target_vel[2] * M3_STEPS_PER_RAD + error[2] * POSITION_CORRECTION_GAIN;
  corrected_vel[3] = target_vel[3] * M4_STEPS_PER_RAD + error[3] * POSITION_CORRECTION_GAIN;
  
  // Apply corrected velocities
  motor1.setSpeed(corrected_vel[0]);
  motor2.setSpeed(corrected_vel[1]);
  motor3.setSpeed(corrected_vel[2]);
  motor4.setSpeed(corrected_vel[3]);
  
  motor1.runSpeed();
  motor2.runSpeed();
  motor3.runSpeed();
  motor4.runSpeed();
}

void resetState() {
  isExecuting = false;
  isGoingHome = false;
  isJogging = false;
  isFinishing = false;
  bufferClear();
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  
  // WAYPOINT <j1> <j2> <j3> <j4> <v1> <v2> <v3> <v4> <time_ms>
  if (cmd.startsWith("WAYPOINT ")) {
    if (!isHomed) {
      sendError("Not homed");
      return;
    }
    
    if (bufferFull()) {
      sendError("Buffer full");
      return;
    }
    
    // Parse 9 values: 4 positions + 4 velocities + time
    float values[9];
    int count = parseFloats(cmd.c_str() + 9, values, 9);
    
    if (count != 9) {
      sendError("WAYPOINT requires 9 values");
      return;
    }
    
    // Create waypoint and add to buffer
    Waypoint wp;
    wp.joints[0] = values[0];
    wp.joints[1] = values[1];
    wp.joints[2] = values[2];
    wp.joints[3] = values[3];
    wp.velocities[0] = values[4];
    wp.velocities[1] = values[5];
    wp.velocities[2] = values[6];
    wp.velocities[3] = values[7];
    wp.time_ms = (uint32_t)values[8];
    
    bufferPush(wp);
  }
  
  // CLEAR_TRAJ - Clear waypoint buffer and reset state
  else if (cmd == "CLEAR_TRAJ") {
    resetState();
    char buf[48];
    snprintf(buf, sizeof(buf), "ACK %d", BUFFER_SIZE);  // Send buffer size
    Serial.println(buf);
  }
  
  // START_TRAJ - Begin trajectory execution
  else if (cmd == "START_TRAJ") {
    if (bufferEmpty()) {
      sendError("No waypoints buffered");
      return;
    }
    
    isExecuting = true;
    traj_start_time = millis();
    
    Serial.println("ACK");
  }
  
  // END_TRAJ - Signal end of waypoints
  else if (cmd == "END_TRAJ") {
    Serial.println("ACK");
  }
  
  // GRIPPER <angle>
  else if (cmd.startsWith("GRIPPER ")) {
    int angle = cmd.substring(8).toInt();
    if (angle < 0 || angle > 180) {
      sendError("Angle must be 0-180");
      return;
    }
    Serial.println("ACK");
    gripper.write(angle);
    Serial.println("DONE");
  }
  
  // HOME - Set current position as zero
  else if (cmd == "HOME") {
    Serial.println("ACK");
    motor1.setCurrentPosition(0);
    motor2.setCurrentPosition(0);
    motor3.setCurrentPosition(0);
    motor4.setCurrentPosition(0);
    currentJoints[0] = 0.0f;
    currentJoints[1] = 0.0f;
    currentJoints[2] = 0.0f;
    currentJoints[3] = 0.0f;
    isHomed = true;
    Serial.println("DONE");
    Serial.flush();  // critical state change
  }
  
  // GO_HOME - Move to home position (all zeros)
  else if (cmd == "GO_HOME") {
    if (!isHomed) {
      sendError("Not homed");
      return;
    }
    
    if (isExecuting || isGoingHome) {
      sendError("Busy");
      return;
    }
    
    Serial.println("ACK");
    
    // Set targets and start non-blocking move
    motor1.moveTo(0);
    motor2.moveTo(0);
    motor3.moveTo(0);
    motor4.moveTo(0);
    isGoingHome = true;
  }
  
  // STATUS - Report current joint positions
  else if (cmd == "STATUS") {
    // Rate limit to avoid overwhelming USB
    if (millis() - last_status_time < STATUS_RATE_LIMIT_MS) return;
    last_status_time = millis();
    
    // Build response string carefully to avoid buffer issues
    Serial.print("POS ");
    Serial.print(currentJoints[0], 4);
    Serial.print(" ");
    Serial.print(currentJoints[1], 4);
    Serial.print(" ");
    Serial.print(currentJoints[2], 4);
    Serial.print(" ");
    Serial.println(currentJoints[3], 4);
  }
  
  // STOP - Emergency stop
  else if (cmd == "STOP") {
    resetState();
    Serial.println("STOPPED");
    Serial.flush();  // critical emergency stop
  }
  
  // HOMED - Query homed state
  else if (cmd == "HOMED") {
    if (isHomed) {
      Serial.println("HOMED");
    } else {
      Serial.println("NOT_HOMED");
    }
  }
  
  // RESET - Reset state without stopping (for recovery)
  else if (cmd == "RESET") {
    resetState();
    Serial.println("ACK");
  }
  
  // JOINTS <j1> <j2> <j3> <j4> - Direct position command (for manual control)
  else if (cmd.startsWith("JOINTS ")) {
    if (!isHomed) {
      sendError("Not homed");
      return;
    }
    
    // Parse 4 float values
    float joints[4];
    int count = parseFloats(cmd.c_str() + 7, joints, 4);
    
    if (count != 4) {
      sendError("JOINTS requires 4 values");
      return;
    }
    
    if (isExecuting || isGoingHome || isJogging) {
      sendError("Busy");
      return;
    }
    
    Serial.println("ACK");
    
    // Set target positions and start non-blocking move
    motor1.moveTo(jointToSteps1(joints[0]));
    motor2.moveTo(jointToSteps2(joints[1]));
    motor3.moveTo(jointToSteps3(joints[2]));
    motor4.moveTo(jointToSteps4(joints[3]));
    isJogging = true;
  }
  
  else {
    String errorMsg = "Unknown command: " + cmd;
    sendError(errorMsg.c_str());
  }
}
