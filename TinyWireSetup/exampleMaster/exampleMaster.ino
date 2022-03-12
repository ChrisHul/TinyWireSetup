// Bit bang i2c master test program
// Created to work with attiny slave unit
// by ChrisHul
//
#define _USE_WIRE

#ifdef _USE_WIRE
#include <Wire.h>
#else
#include <BitBang_I2C.h>
I2Chandle_t i2c_handle;
#endif

#define SDA_DIRECT EPORTC | 4
#define SCL_DIRECT EPORTC | 5
#define SDA_PIN A4
#define SCL_PIN A5
byte rxBuf[10], txBuf[10], i;
uint16_t echoError = 0;
uint32_t loopCnt = 0;


void setup() {
  // put your setup code here, to run once:
  byte iSDA= SDA_PIN;
  byte iSCL= SDA_PIN;
  Serial.begin(9600);
#ifdef _USE_WIRE
  Wire.begin(); // join i2c bus (address optional for master)
  Wire.setClock(50000);
#else
  i2c_handle.bWire = 0; // use bit bang, not wire library
  i2c_handle.iSDA = SDA_PIN;
  i2c_handle.iSCL = SCL_PIN;
  I2CInit(&i2c_handle,100000L);
#endif
txBuf[0] = 0xAA;
txBuf[1] = 0x92;
txBuf[2] = 0x68;
txBuf[3] = 0x75;
txBuf[4] = 0x3D;

delay(1000); // allow devices to power up
}
byte x = 2, bus_Status;

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Sending bytes - ");

#ifdef _USE_WIRE
    Wire.beginTransmission(4);
    bus_Status = Wire.write(txBuf, 5); 
    Wire.endTransmission();
#else
    bus_Status = I2CWrite( &i2c_handle, 4, txBuf, 5);
#endif
    Serial.print("Write status: ");
    Serial.println(bus_Status);
    delay(50);
#ifdef _USE_WIRE
   bus_Status = Wire.requestFrom( 4, 6, true); // device #4, 6 bytes, stop when done
   i = 0;
   if (bus_Status != 0) {
     while (Wire.available())
     {
        rxBuf[i++] = Wire.read();
     }
     i = 1; // ok
   }
#else
    i = I2CRead( &i2c_handle, 4, rxBuf, 6);
#endif
    if ( i != 1) {
      Serial.println( "Slave does not respond to read operation");
    }
    else {
      Serial.print("READING: Errors Slave reception: ");
      Serial.print(rxBuf[0]);
      Serial.print(". Echo Error: ");
      Serial.print(echoError);
      Serial.print(" - ");
      
      txBuf[5] = txBuf[0];
      for(i = 1; i < 6; i++) {
        if( (rxBuf[i] ^ txBuf[i-1]) != 0xff) echoError++;
        if (rxBuf[i] < 16) Serial.print( "0");
        Serial.print( rxBuf[i], HEX);
        txBuf[i-1] = txBuf[i];
     }
     Serial.print( " - loopCnt: ");
     Serial.println(loopCnt++);
    }

  delay(250);

}
