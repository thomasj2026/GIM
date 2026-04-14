// ============================================================
//  stern_firmware.cpp  –  Localization Rear (Stern) Node
//  Publishes: /gps_stern/fix, /imu_stern/data, /rtcm_moving_base,
//             /gps_stern/velocity
//  Subscribes: /rtcm_land
//
//  Interface strategy:
//    I2C  (Wire, 0x42) → F9P UBX commands + PVT callbacks
//    I2C  (Wire, 0x28) → BNO055 IMU
//    UART1 (Serial1)   → RTCM only:
//                          IN:  land base corrections  (/rtcm_land)
//                          OUT: moving base corrections (/rtcm_moving_base)
//
//  By moving all UBX traffic to I2C, UART1 becomes a clean
//  RTCM-only pipe. Bytes arriving on Serial1 RX are raw RTCM
//  frames from the F9P which we publish directly to ROS.
//  Bytes written to Serial1 TX are land-base RTCM corrections
//  forwarded into the F9P.
// ============================================================
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
#include <geometry_msgs/msg/twist_with_covariance_stamped.h>
#include <std_msgs/msg/u_int8_multi_array.h>

// --- Forward Declarations ---
void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void rtcm_in_callback(const void *msvin);
void pvtCallback(UBX_NAV_PVT_data_t *ubxDataStruct);
bool read_imu_burst();
void process_rtcm_out();
bool create_entities();
void destroy_entities();

// --- Hardware ---
// F9P on I2C at 0x42, BNO055 on I2C at 0x28
// Serial1 is RTCM-only (no UBX)
SFE_UBLOX_GNSS myGNSS;           // I2C version (no _SERIAL suffix)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// --- ROS 2 Entities ---
rcl_publisher_t    pub_gps, pub_imu, pub_rtcm_out, pub_vel;
rcl_subscription_t sub_rtcm_in;
rcl_timer_t        timer_imu;

sensor_msgs__msg__NavSatFix                    msg_gps;
sensor_msgs__msg__Imu                          msg_imu;
geometry_msgs__msg__TwistWithCovarianceStamped msg_vel;
std_msgs__msg__UInt8MultiArray                 msg_rtcm_in;
std_msgs__msg__UInt8MultiArray                 msg_rtcm_out;

rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rclc_executor_t executor;

// --- Buffers ---
static uint8_t rtcm_out_buf[4096];  // RTCM frames read from Serial1 RX
static uint8_t rtcm_in_buf[1024];   // land base RTCM received from ROS

// --- Frame IDs ---
static char gps_frame[]  = "gps_stern_link";
static char imu_frame[]  = "imu_stern_link";
static char base_frame[] = "base_link";

// --- Scaling ---
const float QUAT_SCALE  = 1.0f / (1 << 14);
const float ACCEL_SCALE = 1.0f / 100.0f;
const float GYRO_SCALE  = 1.0f / 900.0f;

// Minimum speed before velocity is published
const float MIN_SPEED_MS = 0.1f;

// --- State Machine ---
enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
states state = WAITING_AGENT;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) return false; }

// ============================================================
//  RTCM OUT – read raw RTCM frames from Serial1 RX and publish
//  The F9P outputs RTCM on UART1 TX → Teensy Serial1 RX.
//  These are moving base Type 4072 frames destined for the bow.
// ============================================================
void process_rtcm_out() {
  if (state != AGENT_CONNECTED) return;

  int available = Serial1.available();
  if (available <= 0) return;

  // Clamp to buffer size
  if (available > (int)sizeof(rtcm_out_buf))
    available = sizeof(rtcm_out_buf);

  int bytesRead = Serial1.readBytes(rtcm_out_buf, available);
  if (bytesRead <= 0) return;

  msg_rtcm_out.data.data = rtcm_out_buf;
  msg_rtcm_out.data.size = bytesRead;
  rcl_publish(&pub_rtcm_out, &msg_rtcm_out, NULL);
}

// ============================================================
//  RTCM IN – receive land base corrections from ROS and
//  forward to F9P via Serial1 TX.
//  The F9P accepts RTCM on UART1 RX for its own RTK fix.
// ============================================================
void rtcm_in_callback(const void *msvin) {
  const std_msgs__msg__UInt8MultiArray *msg =
      (const std_msgs__msg__UInt8MultiArray *)msvin;
  if (msg->data.size > 0)
    Serial1.write(msg->data.data, msg->data.size);
}

// ============================================================
//  IMU – direct I²C burst read from BNO055 (0x28)
// ============================================================
bool read_imu_burst() {
  uint8_t buf[26];
  Wire.beginTransmission(0x28);
  Wire.write(0x14);
  if (Wire.endTransmission() != 0)      return false;
  if (Wire.requestFrom(0x28, 26) != 26) return false;
  for (int i = 0; i < 26; i++) buf[i] = Wire.read();

  msg_imu.angular_velocity.x = ((int16_t)((buf[1]  << 8) | buf[0]))  * GYRO_SCALE;
  msg_imu.angular_velocity.y = ((int16_t)((buf[3]  << 8) | buf[2]))  * GYRO_SCALE;
  msg_imu.angular_velocity.z = ((int16_t)((buf[5]  << 8) | buf[4]))  * GYRO_SCALE;

  msg_imu.orientation.w = ((int16_t)((buf[13] << 8) | buf[12])) * QUAT_SCALE;
  msg_imu.orientation.x = ((int16_t)((buf[15] << 8) | buf[14])) * QUAT_SCALE;
  msg_imu.orientation.y = ((int16_t)((buf[17] << 8) | buf[16])) * QUAT_SCALE;
  msg_imu.orientation.z = ((int16_t)((buf[19] << 8) | buf[18])) * QUAT_SCALE;

  msg_imu.linear_acceleration.x = ((int16_t)((buf[21] << 8) | buf[20])) * ACCEL_SCALE;
  msg_imu.linear_acceleration.y = ((int16_t)((buf[23] << 8) | buf[22])) * ACCEL_SCALE;
  msg_imu.linear_acceleration.z = ((int16_t)((buf[25] << 8) | buf[24])) * ACCEL_SCALE;

  for (int i = 0; i < 9; i++) msg_imu.orientation_covariance[i]         = 0.0;
  for (int i = 0; i < 9; i++) msg_imu.angular_velocity_covariance[i]    = 0.0;
  for (int i = 0; i < 9; i++) msg_imu.linear_acceleration_covariance[i] = 0.0;
  msg_imu.orientation_covariance[0]         = 0.01;
  msg_imu.orientation_covariance[4]         = 0.01;
  msg_imu.orientation_covariance[8]         = 0.01;
  msg_imu.angular_velocity_covariance[0]    = 0.001;
  msg_imu.angular_velocity_covariance[4]    = 0.001;
  msg_imu.angular_velocity_covariance[8]    = 0.001;
  msg_imu.linear_acceleration_covariance[0] = 0.1;
  msg_imu.linear_acceleration_covariance[4] = 0.1;
  msg_imu.linear_acceleration_covariance[8] = 0.1;

  return true;
}

void imu_timer_callback(rcl_timer_t *timer, int64_t /*last_call_time*/) {
  if (state != AGENT_CONNECTED || timer == NULL) return;
  if (!read_imu_burst()) return;

  int64_t t = rmw_uros_epoch_nanos();
  msg_imu.header.stamp.sec     = t / 1000000000;
  msg_imu.header.stamp.nanosec = t % 1000000000;
  msg_imu.header.frame_id.data = imu_frame;
  msg_imu.header.frame_id.size = strlen(imu_frame);

  rcl_publish(&pub_imu, &msg_imu, NULL);
}

// ============================================================
//  GPS PVT Callback – fires via I2C polling in loop()
// ============================================================
void pvtCallback(UBX_NAV_PVT_data_t *ubxDataStruct) {
  if (state != AGENT_CONNECTED) return;

  // Guard: discard until valid GNSS fix
  if (!ubxDataStruct->flags.bits.gnssFixOK) return;

  int64_t t = rmw_uros_epoch_nanos();

  // ----------------------------------------------------------
  //  NavSatFix
  // ----------------------------------------------------------
  msg_gps.header.stamp.sec     = t / 1000000000;
  msg_gps.header.stamp.nanosec = t % 1000000000;
  msg_gps.header.frame_id.data = gps_frame;
  msg_gps.header.frame_id.size = strlen(gps_frame);

  msg_gps.latitude  = ubxDataStruct->lat  / 10000000.0;
  msg_gps.longitude = ubxDataStruct->lon  / 10000000.0;
  msg_gps.altitude  = ubxDataStruct->hMSL / 1000.0;

  uint8_t carrSoln = ubxDataStruct->flags.bits.carrSoln;
  if      (carrSoln == 2) msg_gps.status.status = 2;  // RTK fixed
  else if (carrSoln == 1) msg_gps.status.status = 1;  // RTK float
  else                    msg_gps.status.status = 0;  // standard fix
  msg_gps.status.service = 1;  // SERVICE_GPS

  float hAcc = ubxDataStruct->hAcc / 1000.0f;
  float vAcc = ubxDataStruct->vAcc / 1000.0f;
  for (int i = 0; i < 9; i++) msg_gps.position_covariance[i] = 0.0;
  msg_gps.position_covariance[0] = hAcc * hAcc;
  msg_gps.position_covariance[4] = hAcc * hAcc;
  msg_gps.position_covariance[8] = vAcc * vAcc;
  msg_gps.position_covariance_type = 2;  // DIAGONAL_KNOWN

  rcl_publish(&pub_gps, &msg_gps, NULL);

  // ----------------------------------------------------------
  //  Velocity – NED -> ENU, suppress below MIN_SPEED_MS
  // ----------------------------------------------------------
  float speed = ubxDataStruct->gSpeed / 1000.0f;
  if (speed >= MIN_SPEED_MS) {
    float sAcc  = ubxDataStruct->sAcc / 1000.0f;
    float sAcc2 = sAcc * sAcc;

    msg_vel.header.stamp.sec     = t / 1000000000;
    msg_vel.header.stamp.nanosec = t % 1000000000;
    msg_vel.header.frame_id.data = base_frame;
    msg_vel.header.frame_id.size = strlen(base_frame);

    msg_vel.twist.twist.linear.x  = ubxDataStruct->velE / 1000.0f;  // East  -> X
    msg_vel.twist.twist.linear.y  = ubxDataStruct->velN / 1000.0f;  // North -> Y
    msg_vel.twist.twist.linear.z  = 0.0f;
    msg_vel.twist.twist.angular.x = 0.0f;
    msg_vel.twist.twist.angular.y = 0.0f;
    msg_vel.twist.twist.angular.z = 0.0f;

    for (int i = 0; i < 36; i++) msg_vel.twist.covariance[i] = 0.0;
    msg_vel.twist.covariance[0]  = sAcc2;
    msg_vel.twist.covariance[7]  = sAcc2;
    msg_vel.twist.covariance[14] = 0.1;

    rcl_publish(&pub_vel, &msg_vel, NULL);
  }
}

// ============================================================
//  Entity Lifecycle
// ============================================================
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "localizationSternNode", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_default(&pub_gps,      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,   msg, NavSatFix),                  "/gps_stern/fix"));
  RCCHECK(rclc_publisher_init_default(&pub_imu,      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,   msg, Imu),                        "/imu_stern/data"));
  RCCHECK(rclc_publisher_init_default(&pub_rtcm_out, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,      msg, UInt8MultiArray),            "/rtcm_moving_base"));
  RCCHECK(rclc_publisher_init_default(&pub_vel,      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistWithCovarianceStamped), "/gps_stern/velocity"));

  // Subscription – land base corrections
  RCCHECK(rclc_subscription_init_default(&sub_rtcm_in, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray), "/rtcm_land"));

  // Pre-assign static buffer
  msg_rtcm_in.data.capacity = sizeof(rtcm_in_buf);
  msg_rtcm_in.data.data     = rtcm_in_buf;
  msg_rtcm_in.data.size     = 0;

  RCCHECK(rclc_timer_init_default(&timer_imu, &support, RCL_MS_TO_NS(20), imu_timer_callback));

  // 2 handles: timer + sub_rtcm_in
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_rtcm_in, &msg_rtcm_in,
                                         &rtcm_in_callback, ON_NEW_DATA));

  rmw_uros_sync_session(1000);
  return true;
}

void destroy_entities() {
  rcl_publisher_fini(&pub_gps,      &node);
  rcl_publisher_fini(&pub_imu,      &node);
  rcl_publisher_fini(&pub_rtcm_out, &node);
  rcl_publisher_fini(&pub_vel,      &node);
  rcl_subscription_fini(&sub_rtcm_in, &node);
  rcl_timer_fini(&timer_imu);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ============================================================
//  Setup & Loop
// ============================================================
void setup() {
  // I2C bus – shared by F9P (0x42) and BNO055 (0x28)
  Wire.begin();
  Wire.setClock(400000);

  // Connect to F9P over I2C
  // myGNSS uses Wire by default (SFE_UBLOX_GNSS, not _SERIAL variant)
  while (!myGNSS.begin()) {
    delay(100);  // waits for 0x42 to respond
  }

  myGNSS.factoryDefault();
  delay(3000);

  // UART1: RTCM in (land corrections) + RTCM out (moving base to bow)
  // No UBX on UART1 — all UBX goes over I2C
  Serial1.addMemoryForRead(new uint8_t[4096], 4096);
  Serial1.begin(115200);

  myGNSS.setNavigationFrequency(5);
  myGNSS.setDynamicModel(DYN_MODEL_SEA);

  // I2C port: UBX in/out only
  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.setI2CInput(COM_TYPE_UBX);

  // UART1 port: RTCM in/out only, no UBX
  myGNSS.setUART1Output(COM_TYPE_RTCM3);
  myGNSS.setUART1Input(COM_TYPE_RTCM3);

  // Enable moving base RTCM output messages on UART1
  myGNSS.newCfgValset(VAL_LAYER_RAM);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART1,   1); // GPS MSM7
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART1,   1); // GLONASS MSM7
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART1,   1); // GLONASS biases
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1, 1); // moving base (required)
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART1, 1); // moving base (required)
  myGNSS.sendCfgValset();

  // PVT callback fires when myGNSS.checkUblox() is called in loop()
  myGNSS.setAutoPVT(true);
  myGNSS.setAutoPVTcallbackPtr(&pvtCallback);

  myGNSS.saveConfiguration();

  // BNO055 IMU
  while (!bno.begin()) delay(100);
  bno.setExtCrystalUse(true);

  set_microros_transports();
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) state = AGENT_AVAILABLE;
      break;

    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      break;

    case AGENT_CONNECTED: {
      static int check_count = 0;
      if (++check_count > 100) {
        if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) state = AGENT_DISCONNECTED;
        check_count = 0;
      }

      // Poll F9P over I2C – fires pvtCallback when new PVT data ready
      myGNSS.checkUblox();
      myGNSS.checkCallbacks();

      // Read RTCM frames from Serial1 RX and publish to /rtcm_moving_base
      process_rtcm_out();

      rclc_executor_spin_some(&executor, 0);
      break;
    }

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}