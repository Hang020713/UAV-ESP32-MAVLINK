//IMPORTANT NOTES:
//PLEASE View the library, there's a testsuite.h in the library of MAVLink to get example code
#include <MAVLink.h>
#include <HardwareSerial.h>

// Define MAVLink system parameters
uint8_t system_id = 1;     //Same as ArduPilot System ID
uint8_t component_id = 195;  //Component ID, copied from ArduPilot website
uint8_t target_system = 1;    //Default ArduPilot System ID
uint8_t target_component = 1; //Default ArduPilot Component ID, optional to be 0(Broadcasting)

//MAVLink Serial
HardwareSerial MAVSerial(2); // UART2 on ESP32

//Timing variables
bool debug_mode = false;
unsigned long last_heartbeat_received = 0;
unsigned long heartbeat_outdate_time = 2000;  //2s, if you think heartbeat is too long, change ardupilot parameter list
unsigned long last_debug = 0;
unsigned long debug_period = 100;  //0.3s

//Positioning Data
//IMU: ICM-42688-P
const float MILLI_G_TO_G = 0.001;
const float G_TO_MS2 = 9.81;
const float MILLI_RAD_TO_RAD = 0.001;
mavlink_raw_imu_t imu_info;
float peak_gyro_x = 0, peak_gyro_y = 0, peak_gyro_z = 0;
unsigned long last_peak_reset = 0;
mavlink_attitude_t attitude_info;
mavlink_power_status_t power_info;
mavlink_battery_status_t battery_info;
mavlink_statustext_t statustext;

void setup() {
  Serial.begin(57600);  //Debug console
  delay(1000);          //General Waiting
  Serial.println("\n\nESP MAVLink Program");
  
  //Setup MAVLink baudrate & Pin, GPIO pin 16 as RX, 17 TX
  MAVSerial.begin(57600, SERIAL_8N1, 16, 17);
  delay(1000);  //General Waiting
  sendHeartbeat();  //Optional, just tell fc that ESP is in the town

  //Reset time variable(s)
  last_debug = millis();
  Serial.println("Setup DONE, start receiving MAVLink");
}

void loop() {
  //Check heartbeat
  if((millis() - last_heartbeat_received) >= heartbeat_outdate_time)
  {
    Serial.print("WARNING: Last Received Heartbeat is ");
    Serial.print((millis() - last_heartbeat_received)/1000.0);
    Serial.println("s ago");
  }
  else
  {
    //sendArmCommand();
  }

  //Debug Mode
  if(debug_mode)
  { //IMU Info
    if(millis() - last_debug >= debug_period)
    {
      //printIMUInfo(imu_info);  //IMU info
      //printAttitudeInfo(attitude_info); //Attitude
      printBatteryInfo(battery_info);

      //Last
      last_debug = millis(); //Updated last print time
    }
  }
  
  //Start Receiving
  mavlinkReceive();
  //delay(10);  //General delay
}

void sendHeartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack a heartbeat message
  mavlink_msg_heartbeat_pack(system_id, component_id, &msg,
                            MAV_TYPE_GCS,           // GCS type
                            MAV_AUTOPILOT_INVALID,  // Not an autopilot
                            0,                      // System mode
                            0,                      // Custom mode
                            MAV_STATE_ACTIVE);      // System state
  
  // Convert the message to a buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the buffer
  MAVSerial.write(buf, len);
}

void sendArmCommand() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // MAV_CMD_COMPONENT_ARM_DISARM command
  // param1: 1 = arm, 0 = disarm
  // param2: 21196 = force arm (bypasses some safety checks)
  mavlink_msg_command_long_pack(
    system_id,           // System ID (your ESP)
    component_id,        // Component ID (your ESP) 
    &msg,                // Message buffer
    target_system,       // Target system (ArduPilot)
    target_component,    // Target component (ArduPilot)
    MAV_CMD_COMPONENT_ARM_DISARM,  // Command ID
    0,                   // Confirmation
    1,                   // param1: 1 = ARM
    21196,               // param2: 0 = normal arm (use 21196 for force)
    0, 0, 0, 0, 0        // param3-7: unused
  );
  
  // Send the message
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVSerial.write(buf, len);
  
  Serial.println("ARM command sent!");
}

void sendDisarmCommand() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_command_long_pack(
    system_id, component_id, &msg,
    target_system, target_component,
    MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0,                   // param1: 0 = DISARM
    0,                   // param2: 0 = normal disarm
    0, 0, 0, 0, 0
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVSerial.write(buf, len);
  
  Serial.println("DISARM command sent!");
}

void mavlinkReceive() {
  mavlink_message_t msg;
  mavlink_status_t status;
  
  //Read any available bytes from the flight controller
  //MAVLink data will send all msg in one time, so need available()
  while (MAVSerial.available()) {
    uint8_t c = MAVSerial.read();
    
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) 
    { //Successfully received a complete message
      //Reference: https://mavlink.io/en/messages/common.html
      //Below are possible receiving message
      // 0 HeartBeat
      // 1 UNKNOWN
      // 2 SYSTIME TIME
      // 24 GPW_RAW_INT
      // 27 RAW_IMU
      // 29 SCALED_PRESSURE
      // 30 ATTITUDE
      // 33 GLOBAL_POSITION_INT
      // 36 SERVO_OUTPUT_RAW
      // 42 MISSION_CURRENT
      // 62 NAV_CONTROLLER_OUTPUT
      // 65 RC_CHANNELS
      // 74 VFR_HUD
      // 111 TIMESYNC
      // 125 POWER_STATUS
      // 136 TERRAIN_REPORT
      // 147 BATTERY_STATUS
      // 152 UNKNOWN
      // 163 UNKNOWN
      // 178 MAV_CMD_DO_CHANGE_SPEED
      // 193 MAV_CMD_DO_PAUSE_CONTINUE
      // 241 VIBRATION
      // 253 STATUSTEXT
      switch(msg.msgid)
      {
        case 0: //Heartbeat
          last_heartbeat_received = millis();
          break;
        case 27: //RAW_IMU
          mavlink_msg_raw_imu_decode(&msg, &imu_info);
          break;
        case 30: //ATTITUDE
          mavlink_msg_attitude_decode(&msg, &attitude_info);
          break;
        case 125: //POWER_STATUS
          mavlink_msg_power_status_decode(&msg, &power_info);
          break;
        case 147: //BATTERY_STATUS
          mavlink_msg_battery_status_decode(&msg, &battery_info);
          break;
        case 253: //STATUSTEXT
          mavlink_msg_statustext_decode(&msg, &statustext);
          Serial.printf("FC STATUS: %s\n", statustext.text);
          break;
      }
    }
  }
}

//Debug Function(s)
//IMU
void printIMUInfo(const mavlink_raw_imu_t& imu) {
  //Acceleration
  float x_g = imu.xacc * MILLI_G_TO_G;  //Convert to G-unit
  float y_g = imu.yacc * MILLI_G_TO_G;  
  float z_g = imu.zacc * MILLI_G_TO_G;
  float x_ms2 = x_g * G_TO_MS2;  //Convert to m/s²
  float y_ms2 = y_g * G_TO_MS2;
  float z_ms2 = z_g * G_TO_MS2;
  Serial.printf("IMU[%d] Acc:(%.2f, %.2f, %.2f)m/s² ", imu.id, x_ms2, y_ms2, z_ms2);

  //Gyro, ArduPilot RAW_IMU gyro is in milli-degrees/second
  float x_rads = imu.xgyro * MILLI_RAD_TO_RAD;
  float y_rads = imu.ygyro * MILLI_RAD_TO_RAD;
  float z_rads = imu.zgyro * MILLI_RAD_TO_RAD;
  float x_dps = x_rads * RAD_TO_DEG;
  float y_dps = y_rads * RAD_TO_DEG;
  float z_dps = z_rads * RAD_TO_DEG;
  
  //Track peak values
  if (abs(x_dps) > peak_gyro_x) peak_gyro_x = abs(x_dps);
  if (abs(y_dps) > peak_gyro_y) peak_gyro_y = abs(y_dps);
  if (abs(z_dps) > peak_gyro_z) peak_gyro_z = abs(z_dps);
  
  float total_rotation = sqrt(x_dps*x_dps + y_dps*y_dps + z_dps*z_dps);
  if (total_rotation > 50) {
    Serial.print(" [FAST!]");
  } else if (total_rotation > 20) {
    Serial.print(" [ACTIVE]");
  } else if (total_rotation > 5) {
    Serial.print(" [MOVING]");
  }
  Serial.printf(" Gyro:(%.1f,%.1f,%.1f)°/s", x_dps, y_dps, z_dps);
  // Serial.print(") Mag:(");
  // Serial.print(imu.xmag); Serial.print(",");
  // Serial.print(imu.ymag); Serial.print(",");
  // Serial.print(imu.zmag);

  Serial.print(") Temp:");
  if (imu.temperature == 0) {
    Serial.print("N/A");
  } else {
    Serial.print(imu.temperature / 100.0, 1);
    Serial.print("°C");
  }
  Serial.println();
}

//Attitude
void printAttitudeInfo(const mavlink_attitude_t& attitude) {
  float roll_deg = attitude.roll * 57.2958;
  float pitch_deg = attitude.pitch * 57.2958;
  float yaw_deg = attitude.yaw * 57.2958;
  if (yaw_deg < 0) yaw_deg += 360;
  
  Serial.printf("ATTITUDE: R:%6.1f° P:%6.1f° Y:%6.1f° | Rates R:%5.1f P:%5.1f Y:%5.1f°/s\n",
                roll_deg, pitch_deg, yaw_deg,
                attitude.rollspeed * 57.2958,
                attitude.pitchspeed * 57.2958,
                attitude.yawspeed * 57.2958);
}

//Battery
void printBatteryInfo(const mavlink_battery_status_t& battery) {
  //Voltage (convert from mV to V)
  float voltage = battery.voltages[0] / 1000.0;
  
  //Current (convert from cA to A)
  float current = battery.current_battery / 100.0;
  
  //Temperature (convert from cdegC to C)
  float temp = battery.temperature / 100.0;
  
  //Consumed (already in mAh)
  float consumed = battery.current_consumed;
  
  Serial.printf("BATTERY[%d] %.2fV %.2fA %d%% ", battery.id, voltage, current, battery.battery_remaining);
  
  Serial.printf("Consumed:%.0fmAh ", consumed);
  
  if (battery.temperature != INT16_MAX) {
    Serial.printf("Temp:%.1f°C ", temp);
  } else {
    Serial.print("Temp:N/A ");
  }
  
  if (battery.time_remaining > 0) {
    Serial.printf("Time:%ds", battery.time_remaining);
  } else {
    Serial.print("Time:N/A");
  }
  
  Serial.println();
}