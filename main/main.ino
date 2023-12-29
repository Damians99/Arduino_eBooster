
/*
    This is a periodicaly can bus message sender script with Interrupt Reading
*/

#include <SPI.h>
#include "mcp2515_can.h"
#include <TaskScheduler.h>


#define POTI_READ A0
#define DCDC_RELAY A1

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
Task t_20Hz(1000/20, TASK_FOREVER, &t_20Hz_Event);
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
    Batt_Data1 = 0x630,
    Batt_Data2 = 0x631,
    Batt_PWR10 = 0x621
};

struct ebooster {
    float I_act;
    float U_act;
    float T_act;
    long n_act;   
    bool Fault;  
};

ebooster eBooster; 

struct bat48V {
    float I_act;
    float I_dschrg_avail;
    float I_chrg_avail;
    float U_cells;
    float U_terminal;
    float T_act;
    float SOC;
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
        delay(500);
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

    while (CAN_MSGAVAIL == CAN.checkReceive())
    {
        CanRxInterrupt();
        //Serial.println("MSG detectet on RX");
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

void CanRxInterrupt(void) {


    unsigned char len;
    unsigned char buf[8];
    unsigned int canId;


    CAN.readMsgBuf(&len, buf);
    canId = CAN.getCanId();

    //Serial.println(canId, HEX);
    

    // Process the recived data
    switch (canId)
    {
    case eBooster_h:
        const int RPM_Conv_Factor = 100;

        eBooster.n_act = (( (unsigned long)(buf[4] & 0x03) << 8) | (unsigned long)buf[3]) * RPM_Conv_Factor;
        eBooster.I_act = buf[5];
        eBooster.Fault = buf[0] < 0 || buf[1] < 0;

        //Serial.println("n eBooster = ");
        //Serial.println(eBooster.n_act);

        break;
    
    case eBooster_l:
        const float U_Conv_Factor = 0.251256f;

        eBooster.T_act = (buf[2] - 32) * 5 / 9;
        eBooster.U_act = buf[3] * U_Conv_Factor;
        break;

    case Batt_Data1:
        const float UI_Conv_Factor = 0.001f;

        Bat48V.U_cells = ( ((unsigned int)buf[0] << 8) | (unsigned int)buf[1]) * UI_Conv_Factor;
        Bat48V.U_terminal = ( ((unsigned int)buf[2] << 8) | (unsigned int)buf[3]) * UI_Conv_Factor;
        Bat48V.I_act = ( ((unsigned long)buf[4] << 24) | ((unsigned long)buf[5] << 16) | 
                         ((unsigned long)buf[6] << 8)  | (unsigned long)buf[7]) * UI_Conv_Factor;
        break;
    
    case Batt_Data2:
        const float T_Conv_Factor = 0.1f;

        int temp1 = ((unsigned int)(buf[0] & 0x07) << 8) | (unsigned int)buf[1];
        int temp2 = ((unsigned int)(buf[2] & 0x07) << 8) | (unsigned int)buf[3];
        Bat48V.T_act = max(temp1, temp2) * T_Conv_Factor;
        Bat48V.SOC = buf[6];
        Bat48V.CB_State = buf[0] & 0xC0;
        Serial.println("0x631 read");
        break;

    case Batt_PWR10:
        Bat48V.I_chrg_avail = ((unsigned int)buf[0] << 2) | ((unsigned int)(buf[1] & 0xC0) >> 6);
        Bat48V.I_dschrg_avail = ((unsigned int)buf[3] << 2) | ((unsigned int)(buf[4] & 0xC0) >> 6);
        break;
    
    default:
        break;

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
    //Serial.println("CAN BUS sendMsgBuf ok!");
    //Serial.print("t_100Hz: ");
    //Serial.println(millis());
     
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
    //Serial.println("CAN BUS sendMsgBuf ok!");
    //Serial.print("t_20Hz: ");
    //Serial.println(millis());
}



/**************************************************************************/
/*!
    @brief    Callback method of task3 - explain
    @param    none
    @returns  none
*/
/**************************************************************************/
void t_5Hz_Event() {
    
    /*
    const int ANALOG_Max_Value = 1023;
    const int RPM_Max_Value = 72000;
    const int RPM_Conv_Factor = 100;

    int n_requested_raw = analogRead(A0);
    n_requested = round((float)n_requested_raw / ANALOG_Max_Value * RPM_Max_Value / RPM_Conv_Factor); 
    */
    
    int n_requested_raw = analogRead(POTI_READ);
    n_requested = round((float)n_requested_raw / 1023 * 72000 / 100);       //Read poti value and convert it to requested eBooster RPM

    DATA0x06d[2]  = (n_requested) & 0xFF;
    DATA0x06d[3]  = (n_requested >> 8) & 0x03;

    pinMode(DCDC_RELAY, OUTPUT);
    digitalWrite(DCDC_RELAY, Bat48V.CB_State);
    Serial.println(Bat48V.CB_State);

}

// END FILE
