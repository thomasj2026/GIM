#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_BNO055.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/u_int8_multi_array.h>

// --- Hardware ---
SFE_UBLOX_GNSS_SERIAL myGNSS;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// --- ROS 2 Entities ---
rcl_publisher_t pub_gps, pub_imu, pub_rtcm_out;
rcl_subscription_t sub_rtcm_in;

sensor_msgs__msg__NavSatFix msg_gps;
sensor_msgs__msg__Imu msg_imu;
std_msgs__msg__UInt8MultiArray msg_rtcm_in;
std_msgs__msg__UInt8MultiArray msg_rtcm_out;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer_imu;

// --- Buffers & Frames ---
uint8_t rtcm_payload[2048]; 
size_t rtcm_payload_len = 0;
uint8_t rtcm_in_buffer[1024]; 

static char gps_frame[] = "gps_stern";
static char imu_frame[] = "imu_stern";

const float QUAT_SCALE = 1.0 / (1 << 14);
const float ACCEL_SCALE = 1.0 / 100.0;
const float GYRO_SCALE = 1.0 / 900.0;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
states state = WAITING_AGENT;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

// --- RTCM OUT: Handlers ---

// FIXED: This matches the v3 library signature
void rtcm_handler(uint8_t incoming) {
  if (rtcm_payload_len < 2048) {
    rtcm_payload[rtcm_payload_len++] = incoming;
  }
}

void process_rtcm_out() {
  if (state != AGENT_CONNECTED || rtcm_payload_len == 0) return;
  msg_rtcm_out.data.size = rtcm_payload_len;
  msg_rtcm_out.data.data = rtcm_payload;
  rcl_publish(&pub_rtcm_out, &msg_rtcm_out, NULL);
  rtcm_payload_len = 0; 
}

// --- RTCM IN: (Land Station -> Stern GPS) ---
void rtcm_in_callback(const void * msvin) {
  const std_msgs__msg__UInt8MultiArray * msg = (const std_msgs__msg__UInt8MultiArray *)msvin;
  if (msg->data.size > 0) {
    Serial1.write(msg->data.data, msg->data.size); 
  }
}

// --- IMU & GPS Callbacks ---
bool read_imu_mega_burst() {
  uint8_t buf[26];
  Wire.beginTransmission(0x28);
  Wire.write(0x14); 
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(0x28, 26) != 26) return false;
  for(int i=0; i<26; i++) buf[i] = Wire.read();

  msg_imu.angular_velocity.x = ((int16_t)((buf[1]<<8)|buf[0])) * GYRO_SCALE;
  msg_imu.angular_velocity.y = ((int16_t)((buf[3]<<8)|buf[2])) * GYRO_SCALE;
  msg_imu.angular_velocity.z = ((int16_t)((buf[5]<<8)|buf[4])) * GYRO_SCALE;

  msg_imu.orientation.w = ((int16_t)((buf[13]<<8)|buf[12])) * QUAT_SCALE;
  msg_imu.orientation.x = ((int16_t)((buf[15]<<8)|buf[14])) * QUAT_SCALE;
  msg_imu.orientation.y = ((int16_t)((buf[17]<<8)|buf[16])) * QUAT_SCALE;
  msg_imu.orientation.z = ((int16_t)((buf[19]<<8)|buf[18])) * QUAT_SCALE;

  msg_imu.linear_acceleration.x = ((int16_t)((buf[21]<<8)|buf[20])) * ACCEL_SCALE;
  msg_imu.linear_acceleration.y = ((int16_t)((buf[23]<<8)|buf[22])) * ACCEL_SCALE;
  msg_imu.linear_acceleration.z = ((int16_t)((buf[25]<<8)|buf[24])) * ACCEL_SCALE;
  return true;
}

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (state != AGENT_CONNECTED || !read_imu_mega_burst()) return;

  // FIXED: Using rmw_uros_epoch_nanos for Teensy compatibility
  int64_t time_ns = rmw_uros_epoch_nanos();
  msg_imu.header.stamp.sec = time_ns / 1000000000;
  msg_imu.header.stamp.nanosec = time_ns % 1000000000;
  msg_imu.header.frame_id.data = imu_frame;
  msg_imu.header.frame_id.size = strlen(imu_frame);
  
  rcl_publish(&pub_imu, &msg_imu, NULL);
}

void pvtCallback(UBX_NAV_PVT_data_t *ubxDataStruct) {
  if (state != AGENT_CONNECTED) return;
  
  int64_t time_ns = rmw_uros_epoch_nanos();
  msg_gps.header.stamp.sec = time_ns / 1000000000;
  msg_gps.header.stamp.nanosec = time_ns % 1000000000;
  msg_gps.header.frame_id.data = gps_frame;
  msg_gps.header.frame_id.size = strlen(gps_frame);

  msg_gps.latitude = ubxDataStruct->lat / 10000000.0;
  msg_gps.longitude = ubxDataStruct->lon / 10000000.0;
  msg_gps.altitude = ubxDataStruct->hMSL / 1000.0;
  msg_gps.status.status = (ubxDataStruct->flags.bits.gnssFixOK) ? 0 : -1;
  
  rcl_publish(&pub_gps, &msg_gps, NULL);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "localizationSternNode", "", &support));

  RCCHECK(rclc_publisher_init_default(&pub_gps, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/gps_stern/fix"));
  RCCHECK(rclc_publisher_init_default(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu_stern/data"));
  RCCHECK(rclc_publisher_init_default(&pub_rtcm_out, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray), "/rtcm_moving_base"));

  RCCHECK(rclc_subscription_init_default(&sub_rtcm_in, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray), "/rtcm_land"));

  msg_rtcm_in.data.capacity = 1024;
  msg_rtcm_in.data.data = rtcm_in_buffer;

  RCCHECK(rclc_timer_init_default(&timer_imu, &support, RCL_MS_TO_NS(20), imu_timer_callback)); 
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_rtcm_in, &msg_rtcm_in, &rtcm_in_callback, ON_NEW_DATA));

  rmw_uros_sync_session(100);
  return true;
}

void destroy_entities() {
  rcl_publisher_fini(&pub_gps, &node);
  rcl_publisher_fini(&pub_imu, &node);
  rcl_publisher_fini(&pub_rtcm_out, &node);
  rcl_subscription_fini(&sub_rtcm_in, &node);
  rcl_timer_fini(&timer_imu);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  
  while(!myGNSS.begin(Serial1)) delay(100);

  myGNSS.setNavigationFrequency(5); 
  myGNSS.setDynamicModel(DYN_MODEL_SEA);
  myGNSS.setUART1Output(COM_TYPE_UBX | COM_TYPE_RTCM3); 
  myGNSS.setUART1Input(COM_TYPE_UBX | COM_TYPE_RTCM3);
  
  myGNSS.setAutoPVT(true);
  myGNSS.setAutoPVTcallbackPtr(&pvtCallback); 
  
  myGNSS.newCfgValset(VAL_LAYER_RAM);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART1, 1);
  myGNSS.sendCfgValset();

  Wire.begin();
  Wire.setClock(400000); 
  bno.begin();
  
  set_microros_transports();
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) state = AGENT_AVAILABLE;
      break;
    case AGENT_AVAILABLE:
      if (create_entities()) state = AGENT_CONNECTED;
      else state = WAITING_AGENT;
      break;
    case AGENT_CONNECTED:
      static int check_count = 0;
      if (check_count++ > 100) {
        if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) state = AGENT_DISCONNECTED;
        check_count = 0;
      }
      
      myGNSS.checkUblox(); 
      myGNSS.checkCallbacks();
      
      process_rtcm_out(); 
      rclc_executor_spin_some(&executor, 0);
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}