
#include <SPI.h>
#include "mcp2515_can.h"
#include <TaskScheduler.h>

#define Freq 100
#define SERIAL Serial

//Global variabels from i/o pins
int n_requested;


void t_20Hz_Event();
void t_5Hz_Event();


Task t_20Hz(1000/20, TASK_FOREVER, &t_20Hz_Event);
Task t_5Hz(1000, TASK_FOREVER, &t_5Hz_Event);