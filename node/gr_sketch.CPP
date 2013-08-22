#include <rxduino.h>
#include "Task.h"
#include "xbee.h"


int PR(unsigned long now);
int Pulse(unsigned long now);

XBee *xbee;
Task *Tasks;

void setup()
{

    xbee = new XBee();
    Tasks = new Task();

    xbee->Init();
    
    Tasks->scheduleFunction(PR, "pr", 0, 5);
    Tasks->scheduleFunction(Pulse, "pulse", 20, 50);

    while(1) {
        Tasks->Run(millis());
        delay(1);
    }
}

void loop(){}

int PR(unsigned long now){xbee->Rx();return 0;}
int Pulse(unsigned long now){xbee->Pulse(now);return 0;}