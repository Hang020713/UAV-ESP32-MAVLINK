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
bool debug_mode = true;
unsigned long last_heartbeat_received = 0;
unsigned long heartbeat_outdate_time = 2000;  //2s, if you think heartbeat is too long, change ardupilot parameter list
unsigned long last_debug = 0;
unsigned long debug_period = 300;  //0.3s

//Positioning Data
mavlink_raw_imu_t imu_info;
mavlink_attitude_t attitude_info;

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

  //Debug Mode
  if(debug_mode)
  { //IMU Info
    if(millis() - last_debug >= debug_period)
    {
      //IMU Info
      // typedef struct __mavlink_raw_imu_t {
      // uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
      // int16_t xacc; /*<  X acceleration (raw)*/
      // int16_t yacc; /*<  Y acceleration (raw)*/
      // int16_t zacc; /*<  Z acceleration (raw)*/
      // int16_t xgyro; /*<  Angular speed around X axis (raw)*/
      // int16_t ygyro; /*<  Angular speed around Y axis (raw)*/
      // int16_t zgyro; /*<  Angular speed around Z axis (raw)*/
      // int16_t xmag; /*<  X Magnetic field (raw)*/
      // int16_t ymag; /*<  Y Magnetic field (raw)*/
      // int16_t zmag; /*<  Z Magnetic field (raw)*/
      // uint8_t id; /*<  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)*/
      // int16_t temperature; /*< [cdegC] Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).*/
      // }) mavlink_raw_imu_t;
      
      printRawIMUInfo(imu_info);  //IMU info
      printAttitude(attitude_info); //Attitude

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
      }
    }
  }
}

//Debug Function(s)
//IMU
void printRawIMUInfo(const mavlink_raw_imu_t& imu) {
  Serial.print("IMU[");
  Serial.print(imu.id);
  Serial.print("] Acc:(");
  Serial.print(imu.xacc); Serial.print(",");
  Serial.print(imu.yacc); Serial.print(",");
  Serial.print(imu.zacc);
  Serial.print(") Gyro:(");
  Serial.print(imu.xgyro); Serial.print(",");
  Serial.print(imu.ygyro); Serial.print(",");
  Serial.print(imu.zgyro);
  Serial.print(") Mag:(");
  Serial.print(imu.xmag); Serial.print(",");
  Serial.print(imu.ymag); Serial.print(",");
  Serial.print(imu.zmag);
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
void printAttitude(const mavlink_attitude_t& attitude) {
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