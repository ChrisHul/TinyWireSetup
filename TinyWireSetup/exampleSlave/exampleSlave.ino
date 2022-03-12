#include <TinyWire.h>

#define I2C_SLAVE_ADDRESS 0x4 // the 7-bit address (remember to change this when adapting this example)
uint8_t i, buf[5];
uint16_t recError = 0;

void requestEvent()
{
  TinyWire.write(recError);
  for( i = 0; i < 5; i++)
  {
    TinyWire.write(buf[i]);
  }
}

void receiveEvent(uint8_t howMany)
{
    if (howMany != 5)
    {
        // Sanity-check
        recError++;
        return;
    }
  for( i = 0; i < 5; i++)
  {
    buf[i] = ~TinyWire.read(); // bitwise negated
  }
  
}

void setup() {
  // put your setup code here, to run once:
    TinyWire.begin(I2C_SLAVE_ADDRESS);
    TinyWire.onReceive(receiveEvent);
    TinyWire.onRequest(requestEvent);
}

void loop() {
  // put your main code here, to run repeatedly:

}
