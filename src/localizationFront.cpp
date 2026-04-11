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
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <std_msgs/msg/u_int8_multi_array.h>

// --- Forward Declarations (Fixes "undefined" errors) ---
void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void rtcm_callback(const void * msvin);
bool read_imu_mega_burst();
bool create_entities();
void destroy_entities();

// --- Hardware ---
SFE_UBLOX_GNSS_SERIAL myGNSS;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// --- ROS 2 Entities ---
rcl_publisher_t pub_gps, pub_imu, pub_heading;
rcl_subscription_t sub_rtcm;

sensor_msgs__msg__NavSatFix msg_gps;
sensor_msgs__msg__Imu msg_imu;
geometry_msgs__msg__PoseWithCovarianceStamped msg_heading;
std_msgs__msg__UInt8MultiArray msg_rtcm_in;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer_imu;

static char gps_frame[] = "gps_bow_link";
static char imu_frame[] = "imu_bow_link";

const float QUAT_SCALE = 1.0 / (1 << 14);
const float ACCEL_SCALE = 1.0 / 100.0;
const float GYRO_SCALE = 1.0 / 900.0;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
states state = WAITING_AGENT;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

// --- RTCM Subscriber Callback ---
void rtcm_callback(const void * msvin) {
  const std_msgs__msg__UInt8MultiArray * msg = (const std_msgs__msg__UInt8MultiArray *)msvin;
  if (msg->data.size > 0) {
    Serial1.write(msg->data.data, msg->data.size);
  }
}

// --- IMU Reading Logic ---
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

// --- IMU Timer Callback ---
void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (state != AGENT_CONNECTED || timer == NULL) return;
  if (!read_imu_mega_burst()) return;

  int64_t time_ns = rmw_uros_epoch_nanos();
  msg_imu.header.stamp.sec = time_ns / 1000000000;
  msg_imu.header.stamp.nanosec = time_ns % 1000000000;
  msg_imu.header.frame_id.data = imu_frame;
  msg_imu.header.frame_id.size = strlen(imu_frame);

  rcl_publish(&pub_imu, &msg_imu, NULL);
}

// --- RELPOSNED Callback (Heading) ---
void relposnedCallback(UBX_NAV_RELPOSNED_data_t *ubxDataStruct) {
  if (state != AGENT_CONNECTED) return;
  
  if (ubxDataStruct->flags.bits.gnssFixOK && ubxDataStruct->flags.bits.diffSoln) {
    int64_t time_ns = rmw_uros_epoch_nanos();
    msg_heading.header.stamp.sec = time_ns / 1000000000;
    msg_heading.header.stamp.nanosec = time_ns % 1000000000;
    msg_heading.header.frame_id.data = gps_frame;
    msg_heading.header.frame_id.size = strlen(gps_frame);
    
    double heading_rad = (ubxDataStruct->relPosHeading / 100000.0) * (PI / 180.0);
    msg_heading.pose.pose.orientation.z = sin(heading_rad / 2.0);
    msg_heading.pose.pose.orientation.w = cos(heading_rad / 2.0);
    
    for(int i=0; i<36; i++) msg_heading.pose.covariance[i] = 0.0;
    msg_heading.pose.covariance[35] = 0.0001; 
    
    rcl_publish(&pub_heading, &msg_heading, NULL);
  }
}

// --- GPS PVT Callback ---
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
  msg_gps.status.status = (ubxDataStruct->fixType >= 3) ? 0 : -1;
  rcl_publish(&pub_gps, &msg_gps, NULL);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "localizationFrontNode", "", &support));

  RCCHECK(rclc_publisher_init_default(&pub_gps, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/gps_bow/fix"));
  RCCHECK(rclc_publisher_init_default(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu_bow/data"));
  RCCHECK(rclc_publisher_init_default(&pub_heading, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseWithCovarianceStamped), "/gps_bow/heading"));

  RCCHECK(rclc_subscription_init_default(&sub_rtcm, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray), "/rtcm_moving_base"));

  RCCHECK(rclc_timer_init_default(&timer_imu, &support, RCL_MS_TO_NS(20), imu_timer_callback)); 
  
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_rtcm, &msg_rtcm_in, &rtcm_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&pub_gps, &node);
  rcl_publisher_fini(&pub_imu, &node);
  rcl_publisher_fini(&pub_heading, &node);
  rcl_subscription_fini(&sub_rtcm, &node);
  rcl_timer_fini(&timer_imu);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  Serial1.addMemoryForRead(new uint8_t[1024], 1024);
  Serial1.begin(115200);
  while(!myGNSS.begin(Serial1)) delay(100);

  myGNSS.setUART1Output(COM_TYPE_UBX); 
  myGNSS.setUART1Input(COM_TYPE_UBX | COM_TYPE_RTCM3);
  myGNSS.setNavigationFrequency(10); 
  myGNSS.setDynamicModel(DYN_MODEL_SEA);
  myGNSS.setAutoPVT(true);
  myGNSS.setAutoPVTcallbackPtr(&pvtCallback); 
  myGNSS.setAutoRELPOSNED(true);
  myGNSS.setAutoRELPOSNEDcallbackPtr(&relposnedCallback);
  myGNSS.saveConfiguration();

  Wire.begin();
  Wire.setClock(400000); 
  Wire.setTimeout(3000); 
  while(!bno.begin()) delay(100);
  bno.setExtCrystalUse(true);

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
      if (check_count++ > 50) {
        if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) state = AGENT_DISCONNECTED;
        check_count = 0;
      }
      while (myGNSS.checkUblox()) myGNSS.checkCallbacks();
      rclc_executor_spin_some(&executor, 0);
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}