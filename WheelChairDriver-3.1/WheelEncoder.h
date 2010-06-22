#ifndef WHEELENCODER_H_
#define WHEELENCODER_H_

#include "SerialCom.h"
#include <errno.h>

#define MAX_ENCODER_VALUE 16777216

class WheelEncoder
{
public:
    WheelEncoder(char * port, int rate);
    ~WheelEncoder();
    int GetTicks();

private:
    // serial port descriptor
    SerialCom * Serial;
    struct termios oldtio;
};

#endif /*WHEELENCODER_H_*/
