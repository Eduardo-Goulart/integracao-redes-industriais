#include <SPI.h>
#include <mcp2515.h>

#define CS 53

float setpoint = 29;
float error = 0;
float P = 0, I = 0, D = 0;
float kp = 1.0, ki = 0.3, kd = 0.1;
//float kp = 1.0, ki = 0.5, kd = 0.05;
float PID = 0;
int pwm_control = 0;
float last_temperature;
float last_process = 0;

struct can_frame canMsg;

MCP2515 mcp2515(CS);

void setup() {
  
  Serial.begin(115200);
  SPI.begin();

  mcp2515.reset();

  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("CAN configurated");
  Serial.println("Ready to receive messages...");
  Serial.println();
  
  canMsg.can_id = 0x035;
  canMsg.can_dlc = 2;
}


union cvt {
    float ival;
    byte bval[4];
} as_bytes;

float read_sensor_msg(){
      
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK){
    if (canMsg.can_id == 0x036){
      for(int i = 0; i < 4; i++){
        as_bytes.bval[i] = canMsg.data[i];
      }

      float temperature = as_bytes.ival;
      //float temperature = canMsg.data[0];

      Serial.println("****************");
      Serial.print("Received message from CAN ID: ");
      Serial.println(canMsg.can_id, HEX);
      Serial.print("Received temperature: ");
      Serial.println(temperature);
      Serial.print("Setpoint: ");
      Serial.println(setpoint);
      Serial.println("----------------");
      
      return temperature;
    }  
  }
}

// utilizar union para transmitir os 4 bytes do sensor
void transmit_control_signal(float pwm){
  
   canMsg.can_id = 0x035;
   canMsg.data[0] = pwm;
   Serial.print("Controller sending: ");
   Serial.println(pwm);
   Serial.print("Can id: ");
   Serial.println(canMsg.can_id, HEX);
   Serial.println(canMsg.data[0]);
   Serial.println("*******************");
   mcp2515.sendMessage(&canMsg);

}

void loop() {

  float temperature = read_sensor_msg();

  // Controle
  error = setpoint - temperature;
  
  float timedelta = (millis() - last_process) / 1000.0;
  last_process = millis();
  
  P = error * kp;
  
  I += (error * ki) * timedelta;
  
  D = (last_temperature - temperature) * kd * timedelta;

  PID = P + I + D;
  
  pwm_control = (PID + 5);

  if (pwm_control < 0 ){
    pwm_control = 0;  
  }
  if (pwm_control > 255){
    pwm_control = 255;
  }

  Serial.println("Control parameters: ");
  Serial.print("P = ");
  Serial.println(P);
  Serial.print("I = ");
  Serial.println(I);
  Serial.print("D = ");
  Serial.println(D);
  Serial.print("PID = ");
  Serial.println(PID);
  Serial.print("Control signal: ");
  Serial.println(pwm_control);
  Serial.println("----------------");
      
  last_temperature = temperature; 
  //
  transmit_control_signal(pwm_control);
  
  delay(2000);
}
