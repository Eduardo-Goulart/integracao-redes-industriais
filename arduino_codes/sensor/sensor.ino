#include <SPI.h>
#include <mcp2515.h>

#define CS 53
#define TEMP_PIN A0

struct can_frame can_sensor;

MCP2515 mcp2515(CS);

void setup() {

  Serial.begin(115200);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  can_sensor.can_id = 0x036;
  can_sensor.can_dlc = 4;
  
}

float read_temperature(){
  
  float raw_temp = float(analogRead(TEMP_PIN));
  float voltage = (raw_temp * 5) / 1023;
  float temperature = voltage / 0.010;

  return temperature;
} 


void send_data(float temperature){
  union cvt {
    float ival;
    byte bval[4];
  } as_bytes;
  
  can_sensor.can_id = 0x036;
  can_sensor.can_dlc = 4;
  
  as_bytes.ival = temperature;
  for(int i = 0; i < 4; i++){
    can_sensor.data[i] = as_bytes.bval[i];
  }
  
//  canMsg.data[0] = temperature;
  mcp2515.sendMessage(&can_sensor);

  Serial.print("Sending temperature to controller: ");
  Serial.println(temperature);
  Serial.print("CAN id: ");
  Serial.println(can_sensor.can_id, HEX);
  Serial.println("\n******************\n");

}

int verify_controller(){
  for(int i = 0; i < 50; i++){
    if (mcp2515.readMessage(&can_sensor) == MCP2515::ERROR_OK){
      if (can_sensor.can_id == 0x003){
        return 1;       
      }
    }
    
    delay(250);
  }

  return 0;
}

void keep_alive(){
  can_sensor.can_id = 0x001;
  can_sensor.can_dlc = 1;
  
  can_sensor.data[0] = 1;

  mcp2515.sendMessage(&can_sensor);
}

void loop() {

  float temperature = read_temperature();

  Serial.print("Temperature: ");
  Serial.println(temperature);

  send_data(temperature);

  keep_alive();
  
  delay(500);

}
