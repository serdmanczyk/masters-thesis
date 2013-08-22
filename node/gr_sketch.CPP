#include <rxduino.h>
#include "xbee.h"


XBee *xbee;

void setup()
{

    xbee = new XBee();
    xbee->Init();
}

void loop()
{
	xbee->Pulse(millis());
	delay(5);
}