#include "accelerometer.h"

Accelerometer::Accelerometer():
connected(false)
{
	// My N95's BTooth's MAC address
	connectBT(5,"00:1A:DC:CF:B8:08");
}

int Accelerometer::connectBT(uint8_t port, char MAC[18])
{
	// allocate a socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	// set the connection parameters (who to connect to)
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = port;
	str2ba( MAC, &addr.rc_bdaddr );
	// connect to server
	status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
	if( status < 0 ) 
	{
		perror("uh oh");
		connected = false;
		return -1;
	}
	else
	{
		printf("\n Nokia Phone control Connected");
		connected = true;
		return 0;
	}
}

int Accelerometer::getX()
{
	return x;
}

int Accelerometer::getY()
{
	return y;
}

int Accelerometer::getZ()
{
	return z;
}

void Accelerometer::readBT()
{
	int n;
	n = read(s,buffer,BUFFERSIZE);
	if(n>0 && connected)
	{
		buffer[n]='\0';
		sscanf(buffer,"%d %d %d",&x,&y,&z);
		//printf("\nDid i extract then properly X:%d Y:%d Z:%d",x,y,z);
	}
}

Accelerometer::~Accelerometer()
{
	close(s);
}
