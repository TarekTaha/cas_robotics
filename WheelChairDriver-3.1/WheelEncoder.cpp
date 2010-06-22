#include "WheelEncoder.h"

WheelEncoder :: WheelEncoder(char * port, int rate) 
{
    Serial = new SerialCom(port,rate);
}

WheelEncoder::~WheelEncoder() 
{
    delete Serial;
    Serial=NULL;
}

int WheelEncoder::GetTicks() 
{
    int Count = 0;
    unsigned int temp;
    int ret;
    unsigned int Position = 0;

    Serial->WriteByte(0x01);

    temp = 0;
    ret = Serial->ReadByte(&temp);
    if (ret > 0) Count++;
    if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
    Position = temp;

    temp = 0;
    ret = Serial->ReadByte(&temp);
    if (ret > 0) Count++;
    if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
    temp = temp << 8;
    Position |= temp;

    temp = 0;
    ret = Serial->ReadByte(&temp);
    if (ret > 0) Count++;
    if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
    temp = temp << 16;
    Position |= temp;

    temp = 0;
    ret = Serial->ReadByte(&temp);
    if (ret > 0) Count++;
    if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));

    //printf("\nReading Encoder.. %d %d, ", (Position), Count);

    if (Count == 4)
        return Position;
    else
        return -1;
}
