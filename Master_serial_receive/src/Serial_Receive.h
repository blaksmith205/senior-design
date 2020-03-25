#ifndef Serial_Receive_h
#define Serial_Receive_h

struct EulerAngle
{
    float roll, pitch, yaw;
};

void recvWithStartEndMarkers();
void parseData(EulerAngle &angles);
void showParsedData(const EulerAngle &angles);

#endif