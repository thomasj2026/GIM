#include <Arduino.h>
#include <micro_ros_arduino.h> 
#include <VescUart.h>
#include <sbus.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// You will also need the headers for the specific message types you are using:
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

// DEBUG: Set to true for Serial Monitor testing, false for actual ROS operation
const bool DEBUG_MODE = true; 

// --- USER CONFIGURATION ---
const int POLE_PAIRS = 3;           
const float PROP_RADIUS = 0.05;    
const float WIDTH = 0.5;      
const float MAX_AMPS = 51.0; 
const float SLIP_FACTOR = 1.0;

// --- Hardware ---
VescUart starBoardMotor;
VescUart portMotor;

// SBUS Configuration
bfs::SbusRx sbus(&Serial3);
bfs::SbusData sbus_data;
bool rc_override = false; 

// --- micro-ROS Global Entities ---
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;

// Publishers & Subscribers //
rcl_publisher_t pub_odom;
rcl_publisher_t pub_sb_amps;
rcl_publisher_t pub_p_amps;
rcl_publisher_t pub_sb_esc_temp;
rcl_publisher_t pub_sb_mot_temp;
rcl_publisher_t pub_p_esc_temp;
rcl_publisher_t pub_p_mot_temp;
// --- NEW RPM PUBLISHERS ---
rcl_publisher_t pub_sb_rpm;
rcl_publisher_t pub_p_rpm;
// --- SUBSCRIBER ---
rcl_subscription_t sub_cmd_vel;

// Messages
nav_msgs__msg__Odometry msg_odom;
std_msgs__msg__Float32 msg_f32; 
geometry_msgs__msg__Twist msg_cmd_vel; 

// Safety & State
unsigned long last_time = 0;
unsigned long last_cmd_time = 0; 
float x = 0.0, y = 0.0, theta = 0.0;

bool last_rc_override = false;
unsigned long last_print_time = 0;
const unsigned long WATCHDOG_TIMEOUT = 250; // 0.25 seconds

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100);}}}

float map_sbus_to_amps(int raw_val) {
  if (raw_val > 950 && raw_val < 1034) return 0.0;
  if (raw_val >= 1034) return ((float)(raw_val - 1034) / (1811.0 - 1034.0)) * MAX_AMPS;
  if (raw_val <= 950)  return ((float)(raw_val - 950) / (950.0 - 172.0)) * MAX_AMPS; 
  return 0.0;
}

// --- Callback: Process ROS Commands ---
void cmd_vel_callback(const void * msin) {
  if (rc_override) return; 
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msin;
  last_cmd_time = millis(); 
  float v = msg->linear.x;
  float w = msg->angular.z;
  float current_p = (v - (w * WIDTH / 2.0)) * MAX_AMPS;
  float current_sb = (v + (w * WIDTH / 2.0)) * MAX_AMPS;
  portMotor.setCurrent(constrain(current_p, -MAX_AMPS, MAX_AMPS));
  starBoardMotor.setCurrent(constrain(current_sb, -MAX_AMPS, MAX_AMPS));
}

// --- Callback: Telemetry ---
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  // (Telemetry logic is preserved here for when DEBUG_MODE is false)
  if (starBoardMotor.getVescValues() || portMotor.getVescValues()) {
    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0;
    last_time = now;

    // 1. Convert ERPM to Mechanical RPM
    float rpm_sb = (float)starBoardMotor.data.rpm / POLE_PAIRS;
    float rpm_p = (float)portMotor.data.rpm / POLE_PAIRS;

    // 2. Convert to Mechanical Velocity (m/s) for Odom
    float v_sb = (rpm_sb * 2.0 * PI * PROP_RADIUS * SLIP_FACTOR) / 60.0;
    float v_p = (rpm_p * 2.0 * PI * PROP_RADIUS * SLIP_FACTOR) / 60.0;

    float v_linear = (v_sb + v_p) / 2.0;
    float v_angular = (v_sb - v_p) / WIDTH;

    // 3. Integration (Odometry)
    x += v_linear * cos(theta) * dt;
    y += v_linear * sin(theta) * dt;
    theta += v_angular * dt;

    // 4. Publish Odom
    msg_odom.header.stamp.sec = now / 1000;
    msg_odom.pose.pose.position.x = x;
    msg_odom.pose.pose.position.y = y;
    msg_odom.pose.pose.orientation.z = sin(theta / 2.0);
    msg_odom.pose.pose.orientation.w = cos(theta / 2.0);
    rcl_publish(&pub_odom, &msg_odom, NULL);

    // 5. Publish Starboard Telemetry
    msg_f32.data = starBoardMotor.data.avgMotorCurrent; 
    rcl_publish(&pub_sb_amps, &msg_f32, NULL);
    msg_f32.data = starBoardMotor.data.tempMosfet;
    rcl_publish(&pub_sb_esc_temp, &msg_f32, NULL);
    msg_f32.data = starBoardMotor.data.tempMotor;
    rcl_publish(&pub_sb_mot_temp, &msg_f32, NULL);
    msg_f32.data = rpm_sb; // Mechanical RPM
    rcl_publish(&pub_sb_rpm, &msg_f32, NULL);

    // 6. Publish Port Telemetry
    msg_f32.data = portMotor.data.avgMotorCurrent; 
    rcl_publish(&pub_p_amps, &msg_f32, NULL);
    msg_f32.data = portMotor.data.tempMosfet;
    rcl_publish(&pub_p_esc_temp, &msg_f32, NULL);
    msg_f32.data = portMotor.data.tempMotor;
    rcl_publish(&pub_p_mot_temp, &msg_f32, NULL);
    msg_f32.data = rpm_p; // Mechanical RPM
    rcl_publish(&pub_p_rpm, &msg_f32, NULL);
  }
}

void setup() {
  if (DEBUG_MODE) {
    Serial.begin(115200);
    // Give Serial Monitor time to wake up
    delay(1000); 
    // Initialize SBUS on Serial3
    sbus.Begin();
    // MANUAL OVERRIDE: 
    // This tells the hardware to use SBUS standard (100k baud, Even parity, 2 stop bits)
    // AND inverts the RX pin so it can read the FrSky R9 signal.
    Serial3.begin(100000, SERIAL_8E2_RXINV); 
    Serial.println("\n\n=== TEENSY RC DRY-RUN DEBUGGER ACTIVE ===");
  }

  Serial1.begin(115200); 
  Serial2.begin(115200); 
  starBoardMotor.setSerialPort(&Serial1);
  portMotor.setSerialPort(&Serial2);

    if (!DEBUG_MODE) {
    set_microros_transports();
    delay(2000); 
    
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "ripc_Control", "", &support));

    auto f32_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);
    // Initialize Publishers
    RCCHECK(rclc_publisher_init_default(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "vesc/odom"));
    RCCHECK(rclc_publisher_init_default(&pub_sb_amps, &node, f32_type, "vesc/starboard/amps"));
    RCCHECK(rclc_publisher_init_default(&pub_p_amps, &node, f32_type, "vesc/port/amps"));
    RCCHECK(rclc_publisher_init_default(&pub_sb_esc_temp, &node, f32_type, "vesc/starboard/temp_esc"));
    RCCHECK(rclc_publisher_init_default(&pub_sb_mot_temp, &node, f32_type, "vesc/starboard/temp_motor"));
    RCCHECK(rclc_publisher_init_default(&pub_p_esc_temp, &node, f32_type, "vesc/port/temp_esc"));
    RCCHECK(rclc_publisher_init_default(&pub_p_mot_temp, &node, f32_type, "vesc/port/temp_motor"));
    RCCHECK(rclc_publisher_init_default(&pub_sb_rpm, &node, f32_type, "vesc/starboard/rpm"));
    RCCHECK(rclc_publisher_init_default(&pub_p_rpm, &node, f32_type, "vesc/port/rpm"));
    // Initialize Subscribers
    RCCHECK(rclc_subscription_init_default(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &cmd_vel_callback, ON_NEW_DATA));
    
    last_time = millis();
  }
}

void loop() {
  // Call Read() ONLY ONCE and store the result
  bool has_new_data = sbus.Read();

  if (has_new_data) {
    sbus_data = sbus.data(); // Fetch the data into our struct
    // Check if we are in failsafe
    if (sbus_data.failsafe || sbus_data.lost_frame) {
      rc_override = false;
    } else {
      // Channel 5 (Index 4) is usually Switch A. 
      // If this doesn't work, we'll print all channels to find your switch.
      rc_override = (sbus_data.ch[4] > 1500); 
    }
    // Print status change
    if (DEBUG_MODE && (rc_override != last_rc_override)) {
      if (rc_override) Serial.println("\n>>> MANUAL MODE ON <<<");
      else Serial.println("\n>>> MANUAL MODE OFF <<<");
      last_rc_override = rc_override;
    }
    if (rc_override) {
      last_cmd_time = millis(); // Mark time so watchdog doesn't trigger
      // Mapping logic and Motor commands
      int raw_throttle = sbus_data.ch[2]; 
      int raw_steer = sbus_data.ch[0];    
      
      float throttle_amps = map_sbus_to_amps(raw_throttle);
      float steer_amps = map_sbus_to_amps(raw_steer);

      // Differential Thrust Calculation
      float current_p = constrain(throttle_amps + steer_amps, -MAX_AMPS, MAX_AMPS);
      float current_sb = constrain(throttle_amps - steer_amps, -MAX_AMPS, MAX_AMPS);

      // --- CRITICAL: THE MOTOR COMMANDS ---
      portMotor.setCurrent(current_p);
      starBoardMotor.setCurrent(current_sb);
      
      if (DEBUG_MODE && (millis() - last_print_time > 250)) {
        last_print_time = millis();
        Serial.print("MODE: MANUAL | Port: "); Serial.print(current_p, 1);
        Serial.print("A | Stbd: "); Serial.print(current_sb, 1);
        Serial.println("A");
        Serial.print("CH1: "); Serial.print(sbus_data.ch[0]);
        Serial.print(" | CH3: "); Serial.print(sbus_data.ch[2]);
        Serial.print(" | CH5 (Switch): "); Serial.println(sbus_data.ch[4]);
      }
    }
  } 
  // NOT IN RC MODE DEFAULT TO cmd_vel_callback (ROS Mode)
  if (!DEBUG_MODE) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  }
  // --- THE WATCHDOG ---
  // If we aren't in RC mode, and ROS hasn't sent a command recently...
  if (!rc_override && (millis() - last_cmd_time > WATCHDOG_TIMEOUT)) {
    portMotor.setCurrent(0.0);
    starBoardMotor.setCurrent(0.0);

    if (DEBUG_MODE && (millis() - last_print_time > 2000)) {
      last_print_time = millis();
      Serial.println("[WAITING FOR SAFETY] No RC or ROS signal. Motors Stopped.");
    }
  }

}
