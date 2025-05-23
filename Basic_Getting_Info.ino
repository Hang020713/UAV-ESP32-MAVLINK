#include <MAVLink.h>
#include <HardwareSerial.h>

// Configure which hardware serial port to use
HardwareSerial MAVSerial(2); // UART2 on ESP32

// Define MAVLink system parameters
uint8_t system_id = 1;     // GCS type
uint8_t component_id = 195;  // Component ID

// Target system and component (the flight controller)
uint8_t target_system = 1;    // Default ArduPilot system ID
uint8_t target_component = 1; // Default ArduPilot component ID

// Timing variables
unsigned long last_heartbeat_sent = 0;
unsigned long last_command_sent = 0;
bool heartbeat_received = false;

void setup() {
  // Initialize the serial ports
  Serial.begin(57600);  // Debug console
  delay(1000);
  
  Serial.println("\n\nMAVLink Status Request Test");
  
  // Use YOUR working pin configuration
  MAVSerial.begin(57600, SERIAL_8N1, 17, 16); // RX on GPIO44, TX on GPIO43
}

void loop() {
  // Send heartbeat at 1Hz
  if (millis() - last_heartbeat_sent > 1000) {
    sendHeartbeat();
    last_heartbeat_sent = millis();
    
    Serial.print("Connection status: ");
    Serial.println(heartbeat_received ? "CONNECTED" : "WAITING");
  }
  
  // Send a request every 5 seconds
  if (heartbeat_received && (millis() - last_command_sent > 5000)) {
    // Try different requests
    
    // Request 1: SYS_STATUS message
    //requestMessageInterval(MAVLINK_MSG_ID_SYS_STATUS, 200000); // 5Hz

    //Battery
    requestMessageInterval(MAVLINK_MSG_ID_BATTERY_INFO, 100000);
    
    // Request 2: Direct command to report status
    //requestSystemState();
    
    // Request 3: Direct parameter read
    //requestParameter("SR2_EXTRA1");
    
    Serial.println("Sent status/parameter requests");
    last_command_sent = millis();
  }
  
  // Check for incoming messages
  mavlinkReceive();
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

void requestMessageInterval(uint16_t message_id, uint32_t interval_us) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Command to request message
  mavlink_msg_command_long_pack(system_id, component_id, &msg,
                              target_system, target_component,
                              MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                              message_id,     // Message ID
                              interval_us,    // Interval in microseconds
                              0, 0, 0, 0, 0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVSerial.write(buf, len);
}

void requestSystemState() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Request system state
  mavlink_msg_command_long_pack(system_id, component_id, &msg,
                              target_system, target_component,
                              MAV_CMD_REQUEST_MESSAGE, 0,
                              MAVLINK_MSG_ID_SYS_STATUS,  // Request SYS_STATUS message
                              0, 0, 0, 0, 0, 0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVSerial.write(buf, len);
}

void requestParameter(const char* param_id) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Request a parameter by name
  mavlink_msg_param_request_read_pack(system_id, component_id, &msg,
                                    target_system, target_component,
                                    param_id, -1);  // -1 means lookup by name
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVSerial.write(buf, len);
}

void mavlinkReceive() {
  mavlink_message_t msg;
  mavlink_status_t status;
  
  // Read any available bytes from the flight controller
  while (MAVSerial.available()) {
    uint8_t c = MAVSerial.read();
    
    // Try to parse the byte as part of a MAVLink message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Successfully received a complete message
      
      // Print all received messages for debugging
      Serial.print("Message ID: ");
      Serial.print(msg.msgid);
      
      // Handle specific message types
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          Serial.println(" (HEARTBEAT)");
          
          // Got a heartbeat from the flight controller
          mavlink_heartbeat_t heartbeat;
          mavlink_msg_heartbeat_decode(&msg, &heartbeat);
          
          // Only count heartbeats from the autopilot
          if (msg.sysid == target_system && 
              msg.compid == target_component) {
            
            heartbeat_received = true;
            
            // Print autopilot type and system status
            Serial.print("  Autopilot: ");
            Serial.print(heartbeat.autopilot);
            Serial.print(", System status: ");
            Serial.println(heartbeat.system_status);
          }
          break;
        }
        
        case MAVLINK_MSG_ID_SYS_STATUS: {
          Serial.println(" (SYS_STATUS)");
          
          // Got system status
          mavlink_sys_status_t sys_status;
          mavlink_msg_sys_status_decode(&msg, &sys_status);
          
          // Print key system status information
          Serial.print("  Battery: ");
          Serial.print(sys_status.voltage_battery / 1000.0);
          Serial.print("V, ");
          Serial.print(sys_status.current_battery / 100.0);
          Serial.println("A");
          Serial.print("  CPU Load: ");
          Serial.print(sys_status.load / 10.0);
          Serial.println("%");
          break;
        }
        
        case MAVLINK_MSG_ID_PARAM_VALUE: {
          Serial.println(" (PARAM_VALUE)");
          
          // Got a parameter value
          mavlink_param_value_t param_value;
          mavlink_msg_param_value_decode(&msg, &param_value);
          
          // Extract parameter name
          char param_id[17];
          strncpy(param_id, param_value.param_id, 16);
          param_id[16] = '\0';
          
          Serial.print("  Parameter: ");
          Serial.print(param_id);
          Serial.print(" = ");
          Serial.println(param_value.param_value);
          break;
        }
        
        case MAVLINK_MSG_ID_COMMAND_ACK: {
          Serial.println(" (COMMAND_ACK)");
          
          // Got a command acknowledgement
          mavlink_command_ack_t command_ack;
          mavlink_msg_command_ack_decode(&msg, &command_ack);
          
          Serial.print("  Command: ");
          Serial.print(command_ack.command);
          Serial.print(", Result: ");
          Serial.println(command_ack.result);
          break;
        }
        
        case 111: {
          // TIMESYNC message
          Serial.println(" (TIMESYNC)");
          break;
        }
        
        default: {
          Serial.println();
          break;
        }
      }
    }
  }
}