
/*
    This is a periodicaly can bus message sender script with Interrupt Reading
*/
#include <stdint.h>
#include <SPI.h>
#include "mcp2515_can.h"
#include <TaskScheduler.h>
#include <PID_v2.h>


#define POTI_READ_PIN A0
#define DCDC_RELAY_PIN 6

//User defined Macro to send a CAN frame
#define SEND_CAN_MESSAGE(msg) \
    CAN.sendMsgBuf(msg.id, msg.extended, msg.length, msg.data.character)



// Specify the links and initial tuning parameters of voltage controller
double Kp1 = 1.5, Ki1 = 2, Kd1 = 1, Ts1 = 100;
//Battery terminal PID voltage controller
PID_v2 U_Bat_PID(Kp1, Ki1, Kd1, PID::Direct);

// Specify the links and initial tuning parameters of current controller
double Kp2 = 7, Ki2 = 5, Kd2 = 0, Ts2 = 100;
//Battery charching PID current controller
PID_v2 I_Bat_PID(Kp2, Ki2, Kd2, PID::Direct);


//Global variabels from i/o pins
int n_requested;
float i_requested;
long time_notice;


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


// Set SPI CS Pin according to used hardware
// For Arduino MCP2515 Shield:
// SPI_CS Pin: D9

const int SPI_CS_PIN = 9;

mcp2515_can CAN(SPI_CS_PIN); // Set CS pin


// Data of IDs to send
union BytesUnion{
 uint64_t int64;
 uint32_t int32[2];
 uint16_t int16[4];
 uint8_t int8[8];
 unsigned char character[8];
};

 struct  CAN_FRAME{
 uint32_t id;               // 29 bit if ide set, 11 bit otherwise
 uint32_t fid;              // family ID - used internally to library
 uint8_t rtr;               // Remote Transmission Request (1 = RTR, 0 = data frame)
 uint8_t priority;          // Priority but only for TX frames and optional (0-31)
 uint8_t extended;          // Extended ID flag
 uint32_t time;             // CAN timer value when mailbox message was received.
 uint8_t length;            // Number of data bytes
 BytesUnion data;           // 64 bytes - lots of ways to access it.
};

//eBooster control TX Frame (100 Hz)
CAN_FRAME eBooster_VEH_MSG_0x06d;

//Bat control TX Frame (20Hz)
CAN_FRAME Bat48V_VEH_MSG_0x500;

//Bat crash signal TX Frame (20Hz)
CAN_FRAME Bat48V_VEH_CRASH_0x501;

//Custom Diagnostic Data TX Frame (various frequence)
CAN_FRAME Custom_Diag_Data_0x666;


//IDs to recive definition according to DBC
enum RxId {
    eBooster_h = 0x06b,
    eBooster_l = 0x17b,
    Batt_Data1 = 0x630,
    Batt_Data2 = 0x631,
    Batt_PWR10 = 0x621
};

//Define converation factors for incoming CAN Data (see DBC)
    // eBooster_h:
    const int RPM_Conv_Factor = 100;
    // eBooster_l:
    const float U_Conv_Factor = 0.251256f;
    // Batt_Data1
    const float UI_Conv_Factor = 0.001f;
    // Batt_Data2
    const float T_Conv_Factor = 0.1f;
    // Batt_PWR10

//Define Structs to store incomming CAN Data
struct ebooster {
    float I_act;
    float U_act;
    float T_act;
    long n_act;   
    bool Fault;  
};

//Actual values recived from ebooster
ebooster eBooster = {}; 


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

//Actual values recived from 48V Battery
bat48V Bat48V = {};




void setup() {
    Serial.begin(115200);


    eBooster.Fault = false;     //Set all faults to false
    Bat48V.Fault = false;
    
    
    //define initial values of can frame 
    eBooster_VEH_MSG_0x06d.id = 0x06d;
    eBooster_VEH_MSG_0x06d.extended = 0;
    eBooster_VEH_MSG_0x06d.length = 8;

    Bat48V_VEH_MSG_0x500.id = 0x500;
    Bat48V_VEH_MSG_0x500.extended = 0;
    Bat48V_VEH_MSG_0x500.length = 2;

    Bat48V_VEH_CRASH_0x501.id = 0x501;
    Bat48V_VEH_CRASH_0x501.extended = 0;
    Bat48V_VEH_CRASH_0x501.length = 1;
    Bat48V_VEH_CRASH_0x501.data.int8[0] = 0x80;

    Custom_Diag_Data_0x666.id = 0x666;
    Custom_Diag_Data_0x666.extended = 0;
    Custom_Diag_Data_0x666.length = 8;



    //Initialise TaskScheduler
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

    delay(500);
    
    t_100Hz.enable();
    t_20Hz.enable();
    t_5Hz.enable();   
    t_Startup.enable();
    Serial.println("All Tasks enabled");

    // Setup Arduino I/O Pins definitions
    pinMode(DCDC_RELAY_PIN, OUTPUT);
    pinMode(POTI_READ_PIN, INPUT);
    pinMode(A5, INPUT_PULLUP);
    Serial.println("I/O Pins initialized sucessfully");

    // Setup controllers
    U_Bat_PID.Start(Bat48V.U_terminal,              // input
              40,                                   // current output
              42);                                  // setpoint (Voltage at terminal)

    U_Bat_PID.SetOutputLimits(0, 100);
    U_Bat_PID.SetSampleTime(Ts1);

    I_Bat_PID.Start(Bat48V.I_act,                   // input    
                0,                                  // current output
                0);                                 // setpoint (charching current)

    I_Bat_PID.SetOutputLimits(0, 255);
    I_Bat_PID.SetSampleTime(Ts2);


    // Initialise and setup CAN-Bus shield
    Serial.begin(115200);
    while(!Serial); // wait for Serial

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(500);
    }

  
    //set mask for incomming data, set both the mask to 0x3ff
    CAN.init_Mask(0, 0, 0x3ff);                                 // there are 2 mask in mcp2515, we need to set both of them to recive all
    CAN.init_Mask(1, 0, 0x3ff);


    //set filter for all id's we will recive there are 6 filter in mcp2515

    CAN.init_Filt(0, 0, eBooster_h);                            // eBooster act. Values (RPM, Current....)
    CAN.init_Filt(1, 0, Batt_Data2);                            // Battery Data 2 CB State, Temp....

    CAN.init_Filt(2, 0, Batt_PWR10);                            // Battery 10s available charge/discaharge power 
    CAN.init_Filt(3, 0, Batt_Data1);                            // Battery Data 1 Current, Voltage
    CAN.init_Filt(4, 0, eBooster_l);                            // eBooster Temp and Voltage

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
    CAN_FRAME MsgIN;
    
    CAN.readMsgBuf(&MsgIN.length, MsgIN.data.character);
    MsgIN.id = CAN.getCanId();


    //Serial.println(canId, HEX);
    //Serial.println(canId);
    

    // Process the recived data
        switch (MsgIN.id) {

        case eBooster_h:
            eBooster.n_act = (( (unsigned long)(MsgIN.data.int8[4] & 0x03) << 8) | (unsigned long)MsgIN.data.int8[3]) * RPM_Conv_Factor;
            eBooster.I_act = MsgIN.data.int8[5];
            eBooster.Fault = MsgIN.data.int16[0] < 0;

            //Serial.println("0x06b read");
            //Serial.println("n eBooster = ");
            //Serial.println(eBooster.n_act);

            break;

        case eBooster_l:
            eBooster.T_act = (MsgIN.data.int8[2] - 32) * 5 / 9;
            eBooster.U_act = MsgIN.data.int8[3] * U_Conv_Factor;

            //Serial.println("0x17b read");
            break;

        case Batt_Data1:
            Bat48V.U_cells = ( ((unsigned int)MsgIN.data.int8[0] << 8) | (unsigned int)MsgIN.data.int8[1]) * UI_Conv_Factor;
            Bat48V.U_terminal = ( ((unsigned int)MsgIN.data.int8[2] << 8) | (unsigned int)MsgIN.data.int8[3]) * UI_Conv_Factor;
            Bat48V.I_act = (signed long)( ((unsigned long)MsgIN.data.int8[4] << 24) | ((unsigned long)MsgIN.data.int8[5] << 16) | 
                                 ((unsigned long)MsgIN.data.int8[6] << 8)  | (unsigned long)MsgIN.data.int8[7]) * UI_Conv_Factor;

            //Serial.println("0x630 read");
            //Serial.println(Bat48V.I_act);
            break;

        case Batt_Data2:
            int temp1 = ((unsigned int)(MsgIN.data.int8[0] & 0x07) << 8) | (unsigned int)MsgIN.data.int8[1];
            int temp2 = ((unsigned int)(MsgIN.data.int8[2] & 0x07) << 8) | (unsigned int)MsgIN.data.int8[3];
            Bat48V.T_act = max(temp1, temp2) * T_Conv_Factor;
            Bat48V.SOC = MsgIN.data.int8[6];
            Bat48V.CB_State = MsgIN.data.int8[0] >> 6;

            //Serial.println("0x631 read");

            break;

        case Batt_PWR10:
            Bat48V.I_chrg_avail = ((unsigned int)MsgIN.data.int8[0] << 2) | ((unsigned int)(MsgIN.data.int8[1] & 0xC0) >> 6);
            Bat48V.I_dschrg_avail = ((unsigned int)MsgIN.data.int8[3] << 2) | ((unsigned int)(MsgIN.data.int8[4] & 0xC0) >> 6);
            break;

        default:

            Serial.println("RX Error detected: Unknown ID");
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
    @brief    Callback method of task 100_Hz_Event - Can id's that will be 
              sent with a Frequence of 100Hz
    @param    none
    @returns  none
*/
/**************************************************************************/
void t_100Hz_Event() {

    SEND_CAN_MESSAGE(eBooster_VEH_MSG_0x06d);
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

    if ((Bat48V.CB_State != 1) & millis() > 1000)
    {
        Boot_48V_Bat();
    }

    Set_48V_charching_current(i_requested);

    //Only use to tune controller parameters (Simulate step response)
    //int Jump = digitalRead(A5);
    //if(Jump > 0){Bat48V.U_cells = 20;}
    //else{Bat48V.U_cells = 40;}
    //U_Bat_PID.Setpoint(Bat48V.U_cells);
    //const double input = Bat48V.U_terminal;
    //const double output = U_Bat_PID.Run(input);
    //analogWrite(DCDC_RELAY_PIN, output);


    SEND_CAN_MESSAGE(Bat48V_VEH_MSG_0x500);
    SEND_CAN_MESSAGE(Bat48V_VEH_CRASH_0x501);
    SEND_CAN_MESSAGE(Custom_Diag_Data_0x666);
    //Serial.println("CAN BUS sendMsgBuf ok!");
    //Serial.print("t_20Hz: ");
    //Serial.println(millis());
}



/**************************************************************************/
/*!
    @brief    5Hz Task, used to read analog pins and User inputs
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
    
    int n_requested_raw = analogRead(POTI_READ_PIN);
    n_requested = round((float)n_requested_raw / 1023 * 72000 / 100);       //Read poti value and convert it to requested eBooster RPM
    i_requested = exp((float)n_requested_raw / 1023 * 5)-1; 
    Custom_Diag_Data_0x666.data.int8[0] = uint8_t(i_requested*10);

    eBooster_VEH_MSG_0x06d.data.int16[1] = n_requested;
 

    //Serial.println(i_requested);

    
    //Only use to tune controller parameters
    //const float Inte_val = exp((float)n_requested_raw / 1023 * 10)-1;
    //U_Bat_PID.SetTunings(Kp1, Inte_val, Kd1);
    //Serial.println(U_Bat_PID.GetKi1());
    //Serial.println();

    
    //digitalWrite(DCDC_RELAY_PIN, Bat48V.CB_State);
    //Serial.println(Bat48V.CB_State);

}



/**************************************************************************/
/*!
    @brief    Precharge the 48V System and boot the 48V Battery.
              Voltage must be in -+ 2V Range of batery for 500ms to close Circuit Breaker.
    @param    none
    @returns  none
*/
/**************************************************************************/
void Boot_48V_Bat(void){

    U_Bat_PID.Setpoint(Bat48V.U_cells);                 //Calculate new PWM by PID Controller
    const double input = Bat48V.U_terminal;
    const double output = U_Bat_PID.Run(input);
    analogWrite(DCDC_RELAY_PIN, output);

    if (time_notice >= 500)                             //Check if condition was met the last 500ms
    {
        Bat48V_VEH_MSG_0x500.data.int8[0] = 1;
        analogWrite(DCDC_RELAY_PIN, 0);
    }


    if ((Bat48V.U_terminal < (Bat48V.U_cells + 2)) & (Bat48V.U_terminal > (Bat48V.U_cells - 2)) & (Bat48V.U_terminal > 1))
    {
        time_notice += 50;                              //Increment the timer by 50ms while condition met
    }

    else
    {
        time_notice = 0;                                //When out of +- 2V Range set timer to 0 and set open CB command
        Bat48V_VEH_MSG_0x500.data.int8[0] = 0;
    }
}


/**************************************************************************/
/*!
    @brief    Set the charching current, integrated PID Current control
    @param    current: Final charching current
    @returns  none
*/
/**************************************************************************/
void Set_48V_charching_current(const float current){

    //Calculate new PWM by PID Controller
    if (Bat48V.CB_State == 1)
    {   
        //const float current_requested min(current, Bat48V.I_chrg_avail);
        const float current_requested = current;
        I_Bat_PID.Setpoint(current_requested);
        const double input = Bat48V.I_act;
        const double output = I_Bat_PID.Run(input);

        if(current <= 0.5){
            analogWrite(DCDC_RELAY_PIN, 0);
            
            }
        else{analogWrite(DCDC_RELAY_PIN, output);}


        
        

        Custom_Diag_Data_0x666.data.int8[1] = uint8_t(output);
        Serial.println(output);
    }
}



/**************************************************************************/
/*!
    @brief    Stops the ebooster turning(if not allready happen) and resets error states
              (necessary on every startup to start idle or any other shaft speed)
    @param    none
    @returns  none
*/
/**************************************************************************/
void ebooster_reset_fault(void){
    unsigned char DATA0x06d[8] = {0, 0, 0x03, 0x00, 0, 0, 0, 0}; 

    CAN.sendMsgBuf(0x06d, 0, 8, DATA0x06d);
    CAN.sendMsgBuf(0x06d, 0, 8, DATA0x06d);
    CAN.sendMsgBuf(0x06d, 0, 8, DATA0x06d);
}


/**************************************************************************/
/*!
    @brief    n_set of the eBooster will be set to idle speed (normal operation in standby)
    @param    none
    @returns  none
*/
/**************************************************************************/
void ebooster_idle_speed(void){

    eBooster_VEH_MSG_0x06d.data.int8[2] = 0x20;
    eBooster_VEH_MSG_0x06d.data.int8[3] = eBooster_VEH_MSG_0x06d.data.int8[3] & 0b11111100; 
    

    //DATA0x06d[8] = {0, 0, 0x20, 0x00, 0, 0, 0, 0}; 
}


/**************************************************************************/
/*!
    @brief    Sets the requested speed of the eBooster(n_set in RPM (6'000-80'000)) 
    @param    none
    @returns  none
*/
/**************************************************************************/
void ebooster_set_speed(int16_t rpm_speed){
    
    rpm_speed = round((float)rpm_speed / 100);

    eBooster_VEH_MSG_0x06d.data.int16[1] = rpm_speed;
    
    }


/**************************************************************************/
/*!
    @brief    Sets the maximal current the ebooster can use in ampere
    @param    none
    @returns  none
*/
/**************************************************************************/
void ebooster_set_max_current(int8_t I_max){

    eBooster_VEH_MSG_0x06d.data.int16[7] = I_max;
    
    }

// END FILE
