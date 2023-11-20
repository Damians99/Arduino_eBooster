
/*
    This is a periodicaly can bus message sender script
*/

#include <SPI.h>
#include "mcp2518fd_can.h"
#include <TaskScheduler.h>


// Callback methods prototypes
void t1100Hz();
void t2Callback();
void t3Callback();
void t4Callback();

//Tasks
Task t1(1000/100, TASK_FOREVER, &t1100Hz);
Task t2(1000/20, TASK_FOREVER, &t2Callback);
Task t3(1000, TASK_FOREVER, &t3Callback);
Task t4(1, TASK_ONCE, &t4Callback);

Scheduler runner;


// Set SPI CS Pin according to your hardware
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
// *****************************************
// For Arduino MCP2515 Hat:
// SPI_CS Pin: D9

const int SPI_CS_PIN_SEND = 9;
const int SPI_CS_PIN_RECEIVE = 10;

mcp2518fd CAN_SEND(SPI_CS_PIN_SEND);
mcp2518fd CAN_RECEIVE(SPI_CS_PIN_RECEIVE);

unsigned char len = 0;
unsigned char buf[8];

//Data of IDs to send

//eBooster control 100 Hz
unsigned char DATA0x06d[8] = {0, 0, 0, 0, 0, 0, 0, 0}; 
//Bat control 20 Hz
unsigned char DATA0x500[8] = {0, 0, 0, 0, 0, 0, 0, 0};




void t1100Hz() {
    CAN_SEND.sendMsgBuf(0x06d, 0, 8, DATA0x06d);
    SERIAL_PORT_MONITOR.println("CAN BUS sendMsgBuf ok!");

    Serial.print("t1: ");
    Serial.println(millis());


    if (CAN_MSGAVAIL == CAN_RECEIVE.checkReceive()) {
    // read data,  len: data length, buf: data buf
      SERIAL_PORT_MONITOR.println("checkReceive");
      CAN_RECEIVE.readMsgBuf(&len, buf);
    // print the data
    for (int i = 0; i < len; i++) {
        SERIAL_PORT_MONITOR.print(buf[i]); SERIAL_PORT_MONITOR.print(" ");
    }
    SERIAL_PORT_MONITOR.println();
    }
    SERIAL_PORT_MONITOR.println("---LOOP END---");
     
}

void t2Callback() {
    Serial.print("t2: ");
    Serial.println(millis());
  
}

void t3Callback() {
    Serial.print("t3: ");
    Serial.println(millis());
  
}

void t4Callback() {
    Serial.print("t4: ");
    Serial.println(millis());
  
}


void setup() {

    Serial.begin(115200);
    Serial.println("Scheduler TEST");
    
    runner.init();
    Serial.println("Initialized scheduler");
    
    runner.addTask(t1);
    Serial.println("added t1");
    
    runner.addTask(t2);
    Serial.println("added t2");

    runner.addTask(t3);
    Serial.println("added t3");
    
    runner.addTask(t4);
    Serial.println("added t4");

    delay(5000);
    
    t1.enable();
    Serial.println("Enabled t1 as 100Hz Event");
    t2.enable();
    Serial.println("Enabled t2 as 10Hz Event");
    t3.enable();   
    Serial.println("Enabled t3 as 1Hz Event");
    t4.enable();
    Serial.println("Enabled t4 as Single Event");

    SERIAL_PORT_MONITOR.begin(115200);
    while(!Serial); // wait for Serial

    if (CAN_SEND.begin(CAN_500K_1M) != 0 || CAN_RECEIVE.begin(CAN_500K_1M) != 0) {
      SERIAL_PORT_MONITOR.println("CAN-BUS initiliased error!");
      while(1);
    }
    
    SERIAL_PORT_MONITOR.println("CAN init ok!");


}



void loop() {

  runner.execute();

}

// END FILE
