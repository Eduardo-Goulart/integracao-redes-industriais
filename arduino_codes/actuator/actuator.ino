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

  can_actuator.can_id = 0x033;
  can_actuator.can_dlc = 1;
  
}

union cvt {
    float ival;
    byte bval[4];
} as_bytes;


void read_controller_msg(){
      
  if (mcp2515.readMessage(&can_actuator) == MCP2515::ERROR_OK){
    if (can_actuator.can_id == 0x035){
      //for(int i = 0; i < 2; i++){
       // as_bytes.bval[i] = can_actuator.data[i];
      //}

      //int signal_control = as_bytes.ival;
      int signal_control = can_actuator.data[0];

      analogWrite(FAN_PIN, signal_control);
      Serial.println("****************");
      Serial.print("Received message from CAN ID: ");
      Serial.println(can_actuator.can_id, HEX);      
    }  
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  read_controller_msg();

  delay(2000);
}
