#include <Arduino.h>
#include "Serial_Receive.h"

EulerAngle angles;
extern boolean newData;
extern char receivedChars[];
extern char tempChars[];

void setup()
{
  Serial.begin(1000000);
}

void loop()
{
  recvWithStartEndMarkers();
  if (newData == true)
  {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData(angles);
    showParsedData(angles);
    newData = false;
    
    // Do motor balancing here, after the new data was obtained
  }
}
