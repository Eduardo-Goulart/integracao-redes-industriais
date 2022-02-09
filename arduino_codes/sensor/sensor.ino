#include <SPI.h>
#include <mcp2515.h>

#define CS 53
#define TEMP_PIN A0

struct can_frame canMsg;

MCP2515 mcp2515(CS);

void setup() {

  Serial.begin(9600);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  canMsg.can_id = 0x036;
  canMsg.can_dlc = 1;
  
}


float read_temperature(){
  
  float raw_temp = float(analogRead(TEMP_PIN));
  float voltage = (raw_temp * 5) / 1023;
  float temperature = voltage / 0.010;

  return temperature;
} 


void send_data(float temperature){
  
  canMsg.data[0] = temperature;
  mcp2515.sendMessage(&canMsg);

  Serial.print("Sending temperature to controller: ");
  Serial.println(temperature);
  Serial.print("CAN id: ");
  Serial.println(canMsg.can_id, HEX);
  Serial.println("\n******************\n");

}


void loop() {

  float temperature = read_temperature();
  
  send_data(temperature);

  delay(3000);

}
