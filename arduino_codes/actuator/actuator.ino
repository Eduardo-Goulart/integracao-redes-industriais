#include <SPI.h>
#include <mcp2515.h>

#define CS 53

MCP2515 mcp2515(CS);

struct can_frame can_actuator;

const int FAN_PIN = 8;
int pwm_signal = 10;
const word PWM_FREQ_HZ = 25000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SPI.begin();
  
  pinMode(FAN_PIN, OUTPUT);
  analogWrite(FAN_PIN, pwm_signal);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  can_actuator.can_id = 0x033;
  can_actuator.can_dlc = 1;

  Serial.println("CAN Configurated...");
  Serial.println("********************");
}

union cvt {
    float ival;
    byte bval[4];
} as_bytes;


void read_controller_msg(){
      
  if (mcp2515.readMessage(&can_actuator) == MCP2515::ERROR_OK){
    Serial.println("...");
    if (can_actuator.can_id == 0x035){
      int signal_control = can_actuator.data[0];

      analogWrite(FAN_PIN, signal_control);
      Serial.println("****************");
      Serial.print("Received message from CAN ID: ");
      Serial.print("Control signal: ");
      Serial.println(signal_control);
      Serial.print("From CAN id: ");
      Serial.println(can_actuator.can_id, HEX);      
      Serial.print("Frame size: ");
      Serial.println(can_actuator.can_dlc, HEX);      
    }  
  }
}

void keep_alive(){
  can_actuator.can_id = 0x002;
  can_actuator.can_dlc = 1;
  
  can_actuator.data[0] = 1;

  mcp2515.sendMessage(&can_actuator);
}

void loop() {
  // put your main code here, to run repeatedly:

  read_controller_msg();
  
  keep_alive();
  
  delay(500);
}
