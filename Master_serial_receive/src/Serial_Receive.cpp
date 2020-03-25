#include <Arduino.h>
#include "Serial_Receive.h"

const unsigned char numChars = 32;
char receivedChars[numChars];
char tempChars[numChars]; // temporary array for use when parsing
bool newData;

void recvWithStartEndMarkers()
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '[';
    char endMarker = ']';
    char rc;

    while (Serial.available() > 0 && newData == false)
    {
        rc = Serial.read();

        if (recvInProgress == true)
        {
            if (rc != endMarker)
            {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars)
                {
                    ndx = numChars - 1;
                }
            }
            else
            {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker)
        {
            recvInProgress = true;
        }
    }
}

//============

void parseData(EulerAngle &angles)
{ // split the data into its parts

    char *strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars, ","); // get the first part - the pitch
    angles.roll = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
    angles.pitch = atof(strtokIndx); // convert second part to pitch

    strtokIndx = strtok(NULL, ",");
    angles.yaw = atof(strtokIndx); // convert third part to yaw
}

//============

void showParsedData(const EulerAngle &angles)
{
    Serial.print("R/P/Y: ");
    Serial.print(angles.roll);
    Serial.print(", ");
    Serial.print(angles.pitch);
    Serial.print(", ");
    Serial.println(angles.yaw);
}