
/*
    This is a periodicaly can bus message sender script with Interrupt Reading
*/

#include <SPI.h>
#include "mcp2515_can.h"
#include <TaskScheduler.h>



//Global variabels from i/o pins
int n_requested;


// Callback methods prototypes
void t_Startup_Event();
void t_100Hz_Event();
void t_20Hz_Event();
void t_5Hz_Event();


//Tasks
Task t_Startup(1, TASK_ONCE, &t_Startup_Event);
Task t_100Hz(1000/100, TASK_FOREVER, &t_100Hz_Event);
Task t_20Hz(1000/2, TASK_FOREVER, &t_20Hz_Event);
Task t_5Hz(1000/5, TASK_FOREVER, &t_5Hz_Event);

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

//IDs to recive

enum RxId {
    eBooster_h = 0x06b,
    eBooster_l = 0x17b,
    Batt_Data2 = 0x631,
    Batt_Data1 = 0x630,
    Batt_PWR10 = 0x621
};

class ebooster {
    public:
       double I_act;
       double U_act;
       double T_act;
       double n_act;   
       bool Fault;  
};

ebooster eBooster; 

class bat48V {
    public:
       double I_act;
       double I_dschrg_avail;
       double I_chrg_avail;
       double U_cells;
       double U_terminal;
       double T_act;
       double SOC;
       int CB_State;
       bool Fault;  
};

bat48V Bat48V; 


void setup() {

    eBooster.Fault = false;
    Bat48V.Fault = false;


    Serial.begin(115200);
    Serial.println("Scheduler TEST");
    
    scheduler.init();
    Serial.println("Initialized scheduler");
    
    scheduler.addTask(t_100Hz);
    Serial.println("added t_100Hz");
    
    scheduler.addTask(t_20Hz);
    Serial.println("added t_20Hz");

    scheduler.addTask(t_5Hz);
    Serial.println("added t_5Hz");
    
    scheduler.addTask(t_Startup);
    Serial.println("added t_Startup");

    delay(1000);
    
    t_100Hz.enable();
    t_20Hz.enable();
    t_5Hz.enable();   
    t_Startup.enable();
    Serial.println("All Tasks enabled");


    // Initialise and setup CAN-Bus controller
    Serial.begin(115200);
    while(!Serial); // wait for Serial

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }

    /* 
        set mask, set both the mask to 0x3ff
    */
    CAN.init_Mask(0, 0, 0x3ff);                         // there are 2 mask in mcp2515, we need to set both of them to recive all
    CAN.init_Mask(1, 0, 0x3ff);


    /*
        set filter for all id's we will recive
        there are 6 filter in mcp2515
    */
    CAN.init_Filt(0, 0, eBooster_h);                         // eBooster act. Values (RPM, Current....)
    CAN.init_Filt(1, 0, Batt_Data2);                         // Battery Data 2 CB State, Temp....

    CAN.init_Filt(2, 0, Batt_PWR10);                         // Battery 10s available charge/discaharge power 
    CAN.init_Filt(3, 0, Batt_Data1);                         // Battery Data 1 Current, Voltage
    CAN.init_Filt(4, 0, eBooster_l);                         // eBooster Temp and Voltage

    Serial.println("CAN init ok!");


 }

  



void loop() {

    if (CAN_MSGAVAIL == CAN.checkReceive())
    {
        CanRxInterrupt();
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
    

    // Process the recived data
    switch (canId)
    {
    case eBooster_h:
        eBooster.n_act = (((buf[3] & 0x03) << 8) | buf[4]) * 100;
        eBooster.I_act = buf[4];
        eBooster.Fault = buf[0] < 0 || buf[1] < 0;
        Serial.println("RX--------------------------MSG");
        break;
    
    case eBooster_l:
        eBooster.T_act = (buf[2] - 32) * 5 / 9;
        eBooster.U_act = buf[3] * 0.251256;
        Serial.println("RXXXXXXXXXXXXXXXXXXXXXXXXXXXXXMSG");
        break;

    case Batt_Data1:
        Bat48V.U_cells = ((buf[0] << 8) | buf[1]) * 0.001;
        Bat48V.U_terminal = ((buf[2] << 8) | buf[3]) * 0.001;
        Bat48V.I_act = ((buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7]) * 0.001;
        break;
    
    case Batt_Data2:
        int temp1 = ((buf[0] & 0x07) << 8) | buf[1];
        int temp2 = ((buf[2] & 0x07) << 8) | buf[3];
        Bat48V.T_act = max(temp1, temp2) * 0.1;
        Bat48V.SOC = buf[6];
        Bat48V.CB_State = buf[0] & 0xC0; 
        break;

    case Batt_PWR10:
        Bat48V.I_chrg_avail = (buf[0] << 2) | (((buf[1] & 0xC0)) >> 6);
        Bat48V.I_dschrg_avail = (buf[3] << 2) | (((buf[4] & 0xC0)) >> 6);
        break;
    
    default:
        break;
    }
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
    Serial.println("CAN BUS sendMsgBuf ok!");
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
    Serial.println("CAN BUS sendMsgBuf ok!");

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
void t_5Hz_Event() {
    n_requested = analogRead(A0);
    n_requested = n_requested / 1023 * 70000 / 100;       //Read poti value and convert it to requested eBooster RPM
    DATA0x06d[2]  = (n_requested & 0x300) >> 8;
    DATA0x06d[3]  = (n_requested & 0xFF);
}

// END FILE
