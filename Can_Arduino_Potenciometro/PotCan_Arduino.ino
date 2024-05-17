#include <mcp2515.h>
#include <SPI.h>


struct can_frame canMsg;
MCP2515 mcp2515(10);
int valores[8];

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
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.println("ID "); 
    Serial.println(canMsg.can_dlc); // print DLC
    Serial.println(" DATA LENGHT (em bytes)");
    Serial.println("Inicio da impressão de dados");
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      valores[i] = canMsg.data[i];
      //delay(1000);
      Serial.println(valores[i]); 
    }
    /*
    for(int i = 0 ; i<8; i++){
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(valores[i]);
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (HIGH is the voltage level)
    delay(valores[i]);   
    }
    
    Serial.println("TERMINOU O PISCA PISCA");  
    */    
  }
}