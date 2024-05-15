#include <mcp2515.h>
#include <SPI.h>


struct can_frame canMsg;
MCP2515 mcp2515(10);


void setup() {
  Serial.begin(250000);
  pinMode(LED_BUILTIN, OUTPUT);  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.println(canMsg.can_id, HEX); // print ID
    Serial.print("."); 
    Serial.println(canMsg.can_dlc, HEX); // print DLC
    Serial.print(".");
    Serial.print("Inicio da impressão de dados");
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.println(canMsg.data[i],HEX);
      //digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
      //delay(100);                     // wait for a second
      //digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
      //delay(100);   
    }

    Serial.println();      
  }
}