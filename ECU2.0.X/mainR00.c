/*
 * ECU2
 * R00
 */
 

/**
  Section: Included Files
 */
#include "mcc_generated_files/mcc.h"

#include <stdint.h> 
#define FCY 40000000UL // Instruction cycle frequency, Hz - required for __delayXXX() to work
#include <libpic30.h>        // __delayXXX() functions macros defined here

#define RTDS_TIME_MS			2000	//rules require horn (RTDS) for 1-3 seconds
#define APPS2_SCALING			1.33	//this magic number comes from the ratio of the current sources going into APPS1 and APPS2 sensors, APPS1: 1mA, APPS2:1.3 mA, and adjusted to 1.45 for magic reasons

#define APPS1_MIN_RANGE			20		//lowest plausible accelerator position, anything lower indicates an error
#define APPS1_REAL_MIN			91		//this is actually what is the pedal reading when it is not pressed
#define APPS1_ACCEL_START 		190		//this is where the ECU begins to request torque
#define APPS1_WOT 				290 	//point for wide open throttle
#define APPS1_REAL_MAX			370		//this is actually what is the pedal reading when it is fully pressed
#define APPS1_MAX_RANGE 		900 	//highest plausible accelerator position, anything higher indicates an error

#define APPS1_5PERCENT			(APPS1_REAL_MAX-APPS1_REAL_MIN)/20+APPS1_REAL_MIN //for use in BPP
#define APPS1_25PERCENT			(APPS1_REAL_MAX-APPS1_REAL_MIN)/4+APPS1_REAL_MIN //for use in BPP
#define APPS1_50PERCENT         (APPS1_REAL_MAX-APPS1_REAL_MIN)/2+APPS1_REAL_MIN // this is the value the ADC reads when the accelerator is half pushed, for use in determining half-torque mode

#define APPS2_MIN_RANGE			APPS1_MIN_RANGE*APPS2_SCALING //lowest plausible accelerator position, anything lower indicates an error
#define APPS2_MAX_RANGE 		APPS1_MAX_RANGE*APPS2_SCALING //highest plausible accelerator position, anything higher indicates an error
#define APPS2_REAL_MIN			APPS1_REAL_MIN * APPS2_SCALING		//this is actually what is the pedal reading when it is not pressed
#define APPS2_ACCEL_START 		APPS1_ACCEL_START * APPS2_SCALING		//this is where the ECU begins to request torque
#define APPS2_WOT 				APPS1_WOT * APPS2_SCALING 	//point for wide open throttle
#define APPS2_REAL_MAX			APPS1_REAL_MAX * APPS2_SCALING		//this is actually what is the pedal reading when it is fully pressed

#define APPS2_25PERCENT         APPS1_25PERCENT * APPS2_SCALING
#define APPS2_5PERCENT         APPS1_5PERCENT * APPS2_SCALING

//#define TORQUE_RATE_LIMIT         32      //this is the max value by which the torque request can change between samples

//#define ADC_MAX					1024	//10-bit ADC, full scale at 4.096V
#define APPS_10PERCENT_PLAUSIBILITY	APPS1_REAL_MAX / 10	//allows for up to 10% implausibility between sensors

//#define DAC_MIN					5		//minimum output value for the DAC to send accelerator information; this is set higher than 0 to prevent noise on the line from causing the signal to go below 0V and the inverter giving an "accel shorted" fault
#define INVOFFSET               0       //offset for inverter ids for CAN messages

//#define BRK1_MIN                need to initialize these //plaus check brake inputs
//#define BRK1_MAX                
//#define BRK2_MIN
//#define BRK2_MAX

//#define HV_RTD_TIMEOUT			20		//if the ECU does not see HV present for this many sampling periods, the ECU will exit ready to drive (RTD) mode
//#define HV_PRE_TIMEOUT			350	//if the ECU does not see HV present for this many sampling periods, the ECU will exit ready to drive (RTD) mode

//#define REDUCED_TORQUE          127 //use for half-torque mode
#define BRK1_MIN_RANGE          50 //brks outside these ranges = fault condition
#define BRK1_MAX_RANGE          200
#define BRK2_MIN_RANGE          50
#define BRK2_MAX_RANGE          200
#define BRAKING                 50  //to turn brake light on
#define BRAKING_HARD            100 //value of brake input signal when "braking hard" for software BSPD
//#define BMS_TEMP_LIMIT          40 //BMS temp at which to have ECU limit torque request
//#define BMS_SOC_LIMIT           40 //BMS SOC at which to have ECU limit torque request
//#define BMS_DCL_LIMIT           40 //BMS DCL at which to have ECU limit torque request

typedef enum
{
	STARTUP = -1,
	WAIT = 0,
	DRIVE = 1,
	DEBUG_WAIT = 2,
	DEBUG_DRIVE = 3,
}VEHICLE_MODES __attribute__(());

typedef enum
{
    PEDALS = 0,//from pedal A to CAN, bytes 0,1, APPS1, 2,3 APPS2, 4,5, brakes1, 6,7, brakes2
    //BMS_DCL_SOC_I_HITEMP = 1, //2 bytes, Pack DCL, 1 byte, Pack SOC, 2 bytes, Pack Current, 1 byte, High Temperature
    //BMS_CCL; //1 byte, Pack CCL
    //BMS_V;  //2 bytes, Pack Inst. Voltage
    //BMS_R; //1 byte, Pack Resistance
    //BMS_HEALTH; //1 byte, Pack Health
    //BMS_CYCLES; //1 byte, Total Pack Cycles
    //BMS_AVGI; //2 bytes, Average Current
    //BMS_LOTEMP; //1 byte, Low Temperature
    //BMS_AVGTEMP; //1 byte, Average Temperature
    //BMS_LOCELLV, //2 bytes, Low Cell Voltage
    //BMS_LOCELLID; //1 byte, Low Cell Voltage ID
    //BMS_HICELLV, //2 bytes, High Cell Voltage
    //BMS_HICELLID; //1 byte, High Cell Voltage ID
    //MOT_TEMP = 3, //Temperature #3, bytes 4,5 are motor temp
    //INV_TEMPS = 4, //Temperature #1, bytes 0,1 are IGBT A, bytes 2,3 are IGBT B, bytes 4,5 are IBGT C, bytes 6,7 are gate driver board
    //MOT_SPEED; //Motor Position Information, bytes 2,3 are motor speed
    //INV_I; //Current Information, bytes 0,1 phase A, 2,3 phase B, 4,5 phase C, 6,7 DC bus
    //INV _V; //Voltage Information, bytes 0,1 DC voltage, 2,3 output voltage (line-neutral), 4,5 VAB, 6,7 VBC
    //INV_FLUX; //Flux Information, bytes 0,1 flux command, 2,3 flux feedback, 4,5 Id feedback, 6,7 Iq feedback
    //INV_STATE; //Internal States, 4.0 0 = torque mode, 5 0 = CAN mode
    INV_FAULT = 5, //Fault Codes, each bit represents a fault. see pg 22-24 of RMS CAN Protocol (V4_5).pdf)
    //INV_TORQUE; //Torque & Timer Information, byte 0,1 commanded torque, 2,3 torque feedback
    //INV_FLUXWEAKEN; //Modulation Index & Flux Weakening Output Information, byte 4,5 Id command, 6,7 Iq command
}BUFFER_NUM __attribute__(());
        
//variable decs
int8_t APPS_plaus_state = 1, BPP_state = 1; //1 when normal, 0 when in shutdown due to APPS plausibility/BPP issues
VEHICLE_MODES mode = -1; //-1 initially, 0 on normal power up, 1 in normal drive mode, 2 in debug power up (does not require HV for RTD mode), 3 in debug drive mode 
//int16_t hv_loss_count=0; //index to count number of samples when HV is lost
int16_t brk1 = 0, brk2 = 0, torqueTimesTen = 0, rawAPPS1 = 0, rawAPPS2 = 0, scaledAPPS2 = 0;
//int16_t DACtorque_max = 255; //8-bit DAC to torque (set to 255 max)
//int8_t APPS_plaus_disable=0; //this bit is set high to ignore checking the status of the brake and accelerator pedal
//int8_t prev_green_button=0; //tracks previous green button state
int8_t green_button = 0; //tracks green button state

int8_t qSec = 0, Sec = 0, Min = 0; //used for 5 min timer
int8_t transmitState = 0, receiveState = 0;
//int16_t bmsTemp, bmsI, bmsSOC, bmsDCL;
int16_t CANTorqueMax = 230; //max 230 Nm able to be requested from inverter

//function decs
void RTDbuzz(int);
void updateMode(void);
void calcTorqueRequest(void);
void checkAPPS10(void);
void checkBPP(void);

void _ISR _T1Interrupt(void) { //5 min timer for IMD self-testing
    qSec++; // increment the quarter of a second counter
    if (qSec > 4) // 4 quarters of a second
    {
        qSec = 0;
        Sec++; // increment the seconds counter
        if (Sec > 59) // 60 seconds make a minute
        {
            Sec = 0;
            Min++; // increment the minute counter
            if (Min == 5) {
                Min = 0;
                
                uCAN_MSG CANIMDselfTest;
                CANIMDselfTest.frame.idType = CAN_FRAME_STD;

                CANIMDselfTest.frame.id = 0x21;
                CANIMDselfTest.frame.dlc = 4;
                CANIMDselfTest.frame.data0 = 1; //set as 1 to self test all functionality
                CANIMDselfTest.frame.data1 = 0; //set as 0 to self test all functionality
                CANIMDselfTest.frame.data2 = 0; //set as 0 per the datasheet
                CANIMDselfTest.frame.data3 = 0; //set as 0 per the datasheet

                transmitState = CAN1_transmit(CAN_PRIORITY_MEDIUM, &CANIMDselfTest); //send message
            }
        } // minutes
    } // seconds
    _T1IF = 0; //reset interrupt flag
} // T1Interrupt

void _ISR _T2Interrupt(void) { //.02 s timer for inverter torque request
    
    ////////////////////////////////////////////////////////////////////////////
    /*send prev torque request                                                */
    ////////////////////////////////////////////////////////////////////////////
    uCAN_MSG CANtorqueRequest;
    CANtorqueRequest.frame.idType = CAN_FRAME_STD;

    CANtorqueRequest.frame.id = 0x0C0 + INVOFFSET;
    CANtorqueRequest.frame.dlc = 8;
    CANtorqueRequest.frame.data2 = 0; //speed request = 0
    CANtorqueRequest.frame.data3 = 0; //speed request = 0
    CANtorqueRequest.frame.data4 = 1; //direction - 0 for CW and 1 for CCW
    CANtorqueRequest.frame.data6 = ((CANTorqueMax * 10) % 256); //low order byte of torque limit
    CANtorqueRequest.frame.data7 = ((CANTorqueMax * 10) >> 8); //high order byte of torque limit

    if (mode == DRIVE || mode == DEBUG_DRIVE) {
        CANtorqueRequest.frame.data0 = torqueTimesTen % 256; //low order torque request byte
        CANtorqueRequest.frame.data1 = torqueTimesTen >> 8; //high order torque request byte
        CANtorqueRequest.frame.data5 = 0b01; //5.0 = enable, 1 for on. 5.1 = discharge, 0 for disable
    }
    else  { //WAIT, DEBUG_WAIT, or error state
        CANtorqueRequest.frame.data0 = 0; //low order torque request byte = 0
        CANtorqueRequest.frame.data1 = 0; //high order torque request byte = 0
        CANtorqueRequest.frame.data5 = 0b00; //5.0 = enable, 0 for off. 5.1 = discharge, 0 for disable
    }

    transmitState = CAN1_transmit(CAN_PRIORITY_MEDIUM, &CANtorqueRequest); //send message
    LED1_SetLow();
    if(transmitState)
    {
        LED1_SetHigh(); //successful message transmit
    }
    
    ////////////////////////////////////////////////////////////////////////////
    /*grab new torque request                                                 */
    ////////////////////////////////////////////////////////////////////////////
    uCAN_MSG CANpedalsMessage;
    C1FIFObits.FNRB = PEDALS; //set read buffer to the one with the pedal data in it
    receiveState = CAN1_receive(&CANpedalsMessage); //read from pedals buffer
    rawAPPS1 = (CANpedalsMessage.frame.data1 << 8) + CANpedalsMessage.frame.data0; //APPS1 value from received message
    rawAPPS2 = (CANpedalsMessage.frame.data3 << 8) + CANpedalsMessage.frame.data2; //APPS2 value from received message
    brk1 = (CANpedalsMessage.frame.data5 << 8) + CANpedalsMessage.frame.data4; //brk1 value from received message
    brk2 = (CANpedalsMessage.frame.data7 << 8) + CANpedalsMessage.frame.data6; //brk2 value from received message
    
    if(brk1 > BRAKING)
    {
        BRAKE_CTRL_SetHigh();
    }
    else
    {
        BRAKE_CTRL_SetLow();
    }

    LED2_SetLow();
    if(receiveState)
    {
        LED2_SetHigh(); //successful message reception
    }
    
    _T2IF = 0; //reset interrupt flag
} // T2Interrupt

//void _ISR _T3Interrupt(void) { //.413 s timer for BMS CAN message
    
    
//    _T3IF = 0; //reset interrupt flag
//} // T3Interrupt

/*
    Main application
 */
int main(void) {
    // initialize the device
    SYSTEM_Initialize();
    CAN1_TransmitEnable();

    //XXX initialize inverter? set baud rate, clear command message not received fault
    HV_CTRL_SetHigh(); //initially, ECU says we good for HV
    updateMode();

    /*for 5 min timer for IMD self-testing */
    _T1IP = 4; // this is the default value anyway	
    /*for .25 s timer for sending inverter torque request */
    _T2IP = 7; // highest priority

    while (1) {
        switch (mode)
        {
            case WAIT: //normal power up
            updateMode();
            break;
            case DRIVE: //normal drive mode 
                updateMode();
                checkAPPS10();
                checkBPP();
                calcTorqueRequest();
                break;
            case DEBUG_WAIT: //debug power up
                updateMode();
                break;
            case DEBUG_DRIVE: //debug drive mode
                updateMode();
                checkAPPS10();
                checkBPP();
                calcTorqueRequest();
                break;
            default:
                updateMode();
                break;
        }
    }
    return 1;
}

void RTDbuzz(int buzz_num) {
    int8_t i = 0;
    for (i = 0; i < buzz_num; ++i) //buzzes a certain number of times (buzz_num)
    {
        BUZZ_CTRL_SetLow();
        __delay_ms(RTDS_TIME_MS / (buzz_num * 2));
        BUZZ_CTRL_SetHigh();
        __delay_ms(RTDS_TIME_MS / buzz_num);
    }

}

void updateMode(void) {
    switch (mode)
    {
        case STARTUP: //first time starting up
            if (!nSTART_BTN_GetValue()) {
                mode = DEBUG_WAIT; //if start btn, enter debug mode
            } 
            else {
                mode = WAIT; //if not pushing start btn, enter regular mode
            }
            break;
        case WAIT: //normal power up
            PUMP_CTRL_SetLow(); //turn off pump
            if (!nSTART_BTN_GetValue() && brk1 > BRAKING && rawAPPS1 < APPS1_ACCEL_START && rawAPPS1 > APPS1_MIN_RANGE && rawAPPS2 < APPS2_ACCEL_START && rawAPPS2 > APPS2_MIN_RANGE && HV_ON_GetValue()) {
                mode = DRIVE; //if start btn, brake, not accelerator, and HV on: enter regular drive mode
                RTDbuzz(1);//make the buzz noise
            }
            break;
        case DRIVE: //normal drive mode 
            PUMP_CTRL_SetHigh(); //turn on pump
            if (HV_ON_GetValue()) { 
                TMR3 = 0; //reset timer 3
            }
            else
            {
                if(TMR3 >= 6) //wait for 6 * 52 ms = .3 s
                {
                    //HV_CTRL_SetLow(); //kill HV for good. probs don't want to do this cause driver wouldn't be able to reset after hitting e-stop
                    mode = WAIT; //exit drive mode
                }
            }
            break;
        case DEBUG_WAIT: //debug power up
            PUMP_CTRL_SetLow(); //turn off pump
            if (!nSTART_BTN_GetValue() && brk1 > BRAKING && rawAPPS1 < APPS1_ACCEL_START && rawAPPS1 > APPS1_MIN_RANGE && rawAPPS2 < APPS2_ACCEL_START && rawAPPS2 > APPS2_MIN_RANGE) {
                mode = DEBUG_DRIVE; //if start btn, brake, not accelerator: enter debug drive mode
                RTDbuzz(4); //make the buzz buzz buzz buzz noise
            }
            break;
        case DEBUG_DRIVE: //debug drive mode DON'T LET HV COME ON IF YOURE ALREADY IN DRIVE MODE
            PUMP_CTRL_SetHigh(); //turn on pump
            if (!HV_ON_GetValue()) {
                HV_CTRL_SetLow(); //if HV is off, keep it off
            }
            break;
        default: //welcome to the error state
            Nop(); //compiler doesn't like declaration right after label
            int8_t i = 0;
            for (i = 0; i < 10; ++i) {
                RTDbuzz(6); //make unhappy buzz noises
            }
            break;
    }
}

void checkAPPS10(void) //checks 10% plausibility between 2 APPS
{
    if(rawAPPS1 < APPS1_MAX_RANGE && rawAPPS1 > APPS1_MIN_RANGE && rawAPPS2 < APPS2_MAX_RANGE && rawAPPS2 > APPS2_MIN_RANGE) //if we good on range
    {
        scaledAPPS2 = (double) rawAPPS2 / APPS2_SCALING;
        if(rawAPPS1 <= (scaledAPPS2 + APPS_10PERCENT_PLAUSIBILITY) && rawAPPS1 >= (scaledAPPS2 - APPS_10PERCENT_PLAUSIBILITY)) //if we good on plauz
        {
            //then you good fam. drive da racecar
            APPS_plaus_state = 1; //you good 
            return;
        }
        //APPS both in range but not plausible
        LED1_SetHigh();
    }
    //APPS out of range
    LED3_SetHigh();
    APPS_plaus_state = 0; //implausible
    return;
}

void checkBPP(void)
{
    if(brk1 > BRAKING_HARD && (rawAPPS1 > APPS1_25PERCENT || scaledAPPS2 > APPS2_25PERCENT))
    {
        BPP_state = 0; //cut off torque request
        return;
    }
    else if(rawAPPS1 < APPS1_5PERCENT && scaledAPPS2 < APPS2_5PERCENT)
    {
        BPP_state = 1; //you good
    }
    return;
}

void calcTorqueRequest(void)
{
    int16_t APPSavg = (rawAPPS1 + scaledAPPS2) / 2;
    int16_t torque = (double) APPS_plaus_state * BPP_state * ((APPSavg - APPS1_ACCEL_START) / (APPS1_WOT - APPS1_ACCEL_START)) * CANTorqueMax; //scale torque from 0 to TORQUE_MAX
    torqueTimesTen = torque * 10; //torque to send to inverter
}