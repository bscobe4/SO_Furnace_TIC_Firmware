#include <Wire.h>

#define CCMM_TX_BUFFER_SIZE  40
#define SLAVE_ADDR  7

byte cmd_reset = 1; //reset command bit 0 set
byte cmd_selfcalibration = 2; // run self-calibration bit 1 set
byte cmd_burnoutdetect = 4; // run thermocouple burnout detect bit 3 set
byte cmd_read = 0;

//int I2CA_0 = 0;
//int I2CA_1 = 1;
//int I2CA_2 = 2;

char c = 0;

unsigned long cntr = 0; //Counter to switch between commands for testing
unsigned long timeref = millis(); //get reference time at start

void setup() {
  
  Wire.begin();           //join i2c bus
  //TWBR = 4;
  //Wire.setClock(50000);
  Serial.begin(9600); //start serial for o/p
  //pinMode(testPin, INPUT);
}

void loop() {    
  unsigned long timenow = millis(); //Get current time
  if (timenow > timeref + 2000){
    switch(cntr){

 case 0 :
    Wire.beginTransmission(SLAVE_ADDR); //write to device 9
    Wire.write(cmd_read);//0x01
    Wire.endTransmission();
    Serial.println("++read (0x00)");
    delay(3000);
    ReadSlave();
    //delay(2000);
    cntr = 1;
    break;
  case 1 :
    Wire.beginTransmission(SLAVE_ADDR); //write to device 9
    Wire.write(cmd_reset); //0x01
    Wire.endTransmission();
    //timeref = millis();
    Serial.println("++reset (0x01)");
    delay(3000);
    ReadSlave();
    //delay(2000);
    cntr = 2;
    break;
  case 2 :    
    Wire.beginTransmission(SLAVE_ADDR); //write to device 9
    Wire.write(cmd_selfcalibration);//0x02
    Wire.endTransmission();
    Serial.println("++self_calibration (0x02)");
    delay(3000);
    ReadSlave();
    //delay(2000);
    cntr = 3;
    break;

  case 3 :
    Wire.beginTransmission(SLAVE_ADDR); //write to device 9
    Wire.write(cmd_burnoutdetect); //0x08
    Wire.endTransmission();
    timeref = millis();
    Serial.println("++burnout (0x08)");
    delay(3000);
    ReadSlave();
    //delay(2000);
    timeref = millis();
    cntr = 0;
    break;
    }
    timeref = millis();
  }
}

void ReadSlave() {
    Wire.requestFrom(SLAVE_ADDR, 32); //Arduino wire library only has 32 byte buffer...
  Serial.print("Response: ");
  while (Wire.available()){
    c = Wire.read();
    Serial.print(c, HEX);
    Serial.print(" ");
  }
  //Unfortunately with this workaround, the 33rd byte in the MSP430's output buffer gets lost
  // and the 33rd byte printed via serial is actually the 34th byte, so this code only requests 39
  // bytes instead of the actual 40. In addition, the last two status bytes get corrupted as 1F FF
  // This is good enough for at-home testing
    Wire.requestFrom(SLAVE_ADDR, 7);
  while (Wire.available()){
    c = Wire.read();
    Serial.print(c, HEX);
    Serial.print(" ");
  }
  Serial.println("");
}
