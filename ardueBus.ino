/*
  Software serial multple serial test
 
 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.
 
 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)
 
 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69
 
 Not all pins on the Leonardo support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
 
 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example
 
 This example code is in the public domain.
 
 */
#include <SoftwareSerial.h>
#include "ebus-common.h"
#include "ebus-decode.h"

SoftwareSerial mySerial(10, 11); // RX, TX

void setup()  
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  Serial.println("eBus mirror");

  mySerial.begin(2400);
}

boolean output = false;
byte incommingByte;

void loop() // run over and over
{  
  if (mySerial.available()) {
    incommingByte = mySerial.read();
    if(incommingByte == EBUS_SYN_ESC_01 || incommingByte == EBUS_SYN_ESC_00 || incommingByte == EBUS_SYN_ESC_A9) 
      output = true;
      
    if(output) {
      Serial.print(incommingByte, HEX);
      Serial.print(" ");
    }
    if(output && incommingByte == EBUS_SYN) {
      Serial.println();
      output = false;
    }
  }
  if (Serial.available())
    mySerial.write(Serial.read());
}
