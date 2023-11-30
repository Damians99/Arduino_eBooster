
/*
    This is a periodicaly can bus message sender script with Interrupt Reading
*/

#include <SPI.h>
#include "mcp2515_can.h"
#include <TaskScheduler.h>


#define SERIAL Serial


// Callback methods prototypes
void t_Startup_Event();
void t_100Hz_Event();
void t_20Hz_Event();
void t3Callback();


//Tasks
Task t_Startup(1, TASK_ONCE, &t_Startup_Event);
Task t_100Hz(1000/100, TASK_FOREVER, &t_100Hz_Event);
Task t_20Hz(1000/20, TASK_FOREVER, &t_20Hz_Event);
Task t3(1000, TASK_FOREVER, &t3Callback);

Scheduler scheduler;


// Set SPI CS Pin according to your hardware
// For Arduino MCP2515 Hat:
// SPI_CS Pin: D9

const int SPI_CS_PIN = 9;

mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

unsigned char len = 0;
unsigned char buf[8];




//Data of IDs to send

//eBooster control 100 Hz
unsigned char DATA0x06d[8] = {0, 0, 0, 0, 0, 0, 0, 0}; 
//Bat control 20 Hz
unsigned char DATA0x500[8] = {0, 0, 0, 0, 0, 0, 0, 0};





void setup() {

    Serial.begin(115200);
    Serial.println("Scheduler TEST");
    
    scheduler.init();
    Serial.println("Initialized scheduler");
    
    scheduler.addTask(t_100Hz);
    Serial.println("added t_100Hz");
    
    scheduler.addTask(t_20Hz);
    Serial.println("added t_20Hz");

    scheduler.addTask(t3);
    Serial.println("added t3");
    
    scheduler.addTask(t_Startup);
    Serial.println("added t_Startup");

    delay(1000);
    
    t_100Hz.enable();
    Serial.println("Enabled t_100Hz as 100Hz Event");
    t_20Hz.enable();
    Serial.println("Enabled t_20Hz as 10Hz Event");
    t3.enable();   
    Serial.println("Enabled t3 as 1Hz Event");
    t_Startup.enable();
    Serial.println("Enabled t_Startup as Single Event");


    // Initialise and setup CAN-Bus controller
    SERIAL.begin(115200);
    while(!Serial); // wait for Serial

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        SERIAL.println("CAN BUS Shield init fail");
        SERIAL.println(" Init CAN BUS Shield again");
        delay(100);
    }

    /* 
        set mask, set both the mask to 0x3ff
    */
    CAN.init_Mask(0, 0, 0x3ff);                         // there are 2 mask in mcp2515, we need to set both of them
    CAN.init_Mask(1, 0, 0x3ff);


    /*
        set filter for all id's we will recive
    */
    CAN.init_Filt(0, 0, 0x06b);                         // filters the id with eBooster act. Values
    CAN.init_Filt(1, 0, 0x05);                          // there are 6 filter in mcp2515

    CAN.init_Filt(2, 0, 0x06);                          // there are 6 filter in mcp2515
    CAN.init_Filt(3, 0, 0x07);                          // there are 6 filter in mcp2515
    CAN.init_Filt(4, 0, 0x08);                          // there are 6 filter in mcp2515
    CAN.init_Filt(5, 0, 0x09);                          // there are 6 filter in mcp2515
    SERIAL.println("CAN init ok!");
}

    



void loop() {
    if (CAN_MSGAVAIL == CAN.checkReceive())
    {
        /* code */
    }
    
    scheduler.execute();

}




/**************************************************************************/
/*!
    @brief    Read incoming CAN Messages while main loop is interpted
    @param    none
    @returns  none
*/
/**************************************************************************/

void CanRxInterrupt() {
 
  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned char len;
    unsigned char buf[8];
    unsigned long canId = CAN.getCanId();
    CAN.readMsgBuf(&len, buf);

    // Verarbeite die empfangenen Daten
    processIncomingData(buf, len);
  }
}



/**************************************************************************/
/*!
    @brief    Callback method of task2 - explain
    @param    none
    @returns  none
*/
/**************************************************************************/
void t_Startup_Event() {
    Serial.print("t_Startup: ");
    Serial.println(millis());
  
}



/**************************************************************************/
/*!
    @brief    Callback method of task2 - explain
    @param    none
    @returns  none
*/
/**************************************************************************/
void t_100Hz_Event() {
    CAN.sendMsgBuf(0x06d, 0, 8, DATA0x06d);
    SERIAL.println("CAN BUS sendMsgBuf ok!");
    Serial.print("t_100Hz: ");
    Serial.println(millis());
     
}



/**************************************************************************/
/*!
    @brief    Callback method of task t_20Hz_Event - Can id's that will be 
              sent with a Frequence of 20Hz
    @param    none
    @returns  none
*/
/**************************************************************************/
void t_20Hz_Event() {
    CAN.sendMsgBuf(0x500, 0, 8, DATA0x500);
    SERIAL_PORT_MONITOR.println("CAN BUS sendMsgBuf ok!");

    Serial.print("t_20Hz: ");
    Serial.println(millis());

}



/**************************************************************************/
/*!
    @brief    Callback method of task3 - explain
    @param    none
    @returns  none
*/
/**************************************************************************/
void t3Callback() {
    Serial.print("t3: ");
    Serial.println(millis());
  
}



// END FILE
