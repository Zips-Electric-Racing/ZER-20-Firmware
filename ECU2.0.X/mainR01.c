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

#define PEDAL_SIG_LOST              (int8_t) 100 / 8.33
#define TIMER3_100MS            (int8_t) 100 / 8.33

//#define BRK1_MIN                need to initialize these //plaus check brake inputs
//#define BRK1_MAX                
//#define BRK2_MIN
//#define BRK2_MAX

//#define HV_RTD_TIMEOUT			20		//if the ECU does not see HV present for this many sampling periods, the ECU will exit ready to drive (RTD) state
//#define HV_PRE_TIMEOUT			350	//if the ECU does not see HV present for this many sampling periods, the ECU will exit ready to drive (RTD) state

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
#define CANTorqueMax            230  //max 230 Nm able to be requested from inverter
#define ACCEL_MAX               CANTorqueMax
#define AUTOC_MAX               CANTorqueMax
#define ENDUR_MAX               0.5 * CANTorqueMax
#define SKIDP_MAX               0.4 * CANTorqueMax

#define INVOFFSET               0xA0       //offset for inverter ids for CAN messages
#define INV_HEARTBEAT_ID        0x0C0 + INVOFFSET //id of inverter heartbeat message
#define INV_HEARTBEAT_MSG_SIZE            8 //inv heartbeat message is 8 bytes

#define DASH_BTNST_ID           0x0FF //id of button state message from dashboard
#define PEDAL_ID                0x005 //id of pedal data message from A/CAN board



typedef enum
{
	STARTUP = -1,
    STANDBY = 0,
	WAIT = 1,
	DRIVE = 2,
    DEBUG_STANDBY = 3,
	DEBUG_WAIT = 4,
	DEBUG_DRIVE = 5,
}VEHICLE_STATES __attribute__(());

typedef enum
{
	ACCEL = 0,
    AUTOC = 1,
    SKIDP = 2,
    ENDUR = 3,
}VEHICLE_MODES __attribute__(());

typedef enum
{
    PEDALS = 0,//from pedal A to CAN, bytes 0,1, APPS1, 2,3 APPS2, 4,5, brakes1, 6,7, brakes2
    DASH = 1, //from dashboard, byte 0 start btn, byte 1 selector switch, byte 2 misc button
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
VEHICLE_STATES state = -1; //see enumerated type
//int16_t hv_loss_count=0; //index to count number of samples when HV is lost
int16_t brk1 = 0, brk2 = 0, torqueTimesTen = 0, rawAPPS1 = 0, rawAPPS2 = 0, scaledAPPS2 = 0;
//int16_t DACtorque_max = 255; //8-bit DAC to torque (set to 255 max)
//int8_t APPS_plaus_disable=0; //this bit is set high to ignore checking the status of the brake and accelerator pedal
//int8_t prev_green_button=0; //tracks previous green button state
int8_t startBtn, selectorSwitch, miscBtn; //should probs initialize

int8_t qSec = 0, Sec = 0, Min = 0; //used for 5 min timer
int8_t transmitState = 0, receiveState = 0;
//int16_t bmsTemp, bmsI, bmsSOC, bmsDCL;
int8_t mode = 0; //different mode for each event
int16_t APPSTorqueMax = 230;

//function decs
void RTDbuzz(int);
void updateState(void);

////////////////////////////////////////////////////////////////////////////
/*interrupt triggered when CAN message is received                        */
////////////////////////////////////////////////////////////////////////////        
void __attribute__((__interrupt__, no_auto_psv)) _C1Interrupt(void)  
{   
    //CAN MESSAGE WAS PEDAL DATA
    if(((C1RXFUL1 >> PEDALS) & 0x1) == 0x1)
    {
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
            TMR2 = 0;
        }
        if(TMR2 > PEDAL_SIG_LOST)
        {
            torqueTimesTen = 0; //kill torque request
        }
        else
        {
            ////////////////////////////////////////////////////////////////////////////
            /*do APPS plaus checking                                                  */
            ////////////////////////////////////////////////////////////////////////////
            scaledAPPS2 = (double) rawAPPS2 / APPS2_SCALING;
            if(rawAPPS1 > APPS1_MAX_RANGE || rawAPPS1 < APPS1_MIN_RANGE || rawAPPS2 > APPS2_MAX_RANGE || rawAPPS2 < APPS2_MIN_RANGE) //invalid range
            {
                if(TMR3 > TIMER3_100MS)
                {
                    APPS_plaus_state = 0; //cut torque request
                }
            }
            else if(rawAPPS1 >= (scaledAPPS2 + APPS_10PERCENT_PLAUSIBILITY) || rawAPPS1 <= (scaledAPPS2 - APPS_10PERCENT_PLAUSIBILITY))
            {
                if(TMR3 > TIMER3_100MS)
                {
                    APPS_plaus_state = 0; //cut torque request
                }
            }
            else
            {
                TMR3 = 0; //reset timer
                APPS_plaus_state = 1; //you good to drive
            }

            ////////////////////////////////////////////////////////////////////////////
            /*do BPP plaus checking                                                   */
            ////////////////////////////////////////////////////////////////////////////
            if(brk1 > BRAKING_HARD && (rawAPPS1 > APPS1_25PERCENT || scaledAPPS2 > APPS2_25PERCENT)) //invalid range
            {
                BPP_state = 0; //cut off torque request
                return;
            }
            else if(brk1 > BRK1_MAX_RANGE || brk1 < BRK1_MIN_RANGE || brk2 > BRK2_MAX_RANGE || brk2 < BRK2_MIN_RANGE) //25-5 rule
            {
                BPP_state = 0; //cut off torque request
            }
            else if(rawAPPS1 < APPS1_5PERCENT && scaledAPPS2 < APPS2_5PERCENT)
            {
                BPP_state = 1; //you good
            }

            ////////////////////////////////////////////////////////////////////////////
            /*calc torque request                                                     */
            ////////////////////////////////////////////////////////////////////////////
            int16_t APPSavg = (rawAPPS1 + scaledAPPS2) / 2;
            int16_t torque = (double) APPS_plaus_state * BPP_state * ((APPSavg - APPS1_ACCEL_START) / (APPS1_WOT - APPS1_ACCEL_START)) * APPSTorqueMax; //scale torque from 0 to TORQUE_MAX
            torqueTimesTen = torque * 10; //torque to send to inverter
        }
        
        ////////////////////////////////////////////////////////////////////////////
        /*send torque request                                                     */
        ////////////////////////////////////////////////////////////////////////////
        uCAN_MSG CANtorqueRequest;
        CANtorqueRequest.frame.idType = CAN_FRAME_STD;

        CANtorqueRequest.frame.id = INV_HEARTBEAT_ID;
        CANtorqueRequest.frame.dlc = INV_HEARTBEAT_MSG_SIZE;
        CANtorqueRequest.frame.data2 = 0; //speed request = 0
        CANtorqueRequest.frame.data3 = 0; //speed request = 0
        CANtorqueRequest.frame.data4 = 1; //direction - 0 for CW and 1 for CCW
        CANtorqueRequest.frame.data6 = ((CANTorqueMax * 10) % 256); //low order byte of torque limit
        CANtorqueRequest.frame.data7 = ((CANTorqueMax * 10) >> 8); //high order byte of torque limit

        if (state == DRIVE || state == DEBUG_DRIVE) {
            CANtorqueRequest.frame.data0 = torqueTimesTen % 256; //low order torque request byte
            CANtorqueRequest.frame.data1 = torqueTimesTen >> 8; //high order torque request byte
            CANtorqueRequest.frame.data5 = 0b01; //5.0 = enable, 1 for on. 5.1 = discharge, 0 for disable
        }
        else  { //STANDBY, WAIT, DEBUG_WAIT, or error state
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
        
    }
    
    //CAN MESSAGE WAS DASHBOARD DATA
    if(((C1RXFUL1 >> DASH) & 0x1) == 0x1)
    {
        ////////////////////////////////////////////////////////////////////////////
        /*grab new dashboard states                                               */
        ////////////////////////////////////////////////////////////////////////////
        uCAN_MSG CANdashMessage;
        C1FIFObits.FNRB = DASH; //set read buffer to the one with the pedal data in it
        receiveState = CAN1_receive(&CANdashMessage); //read from pedals buffer
        startBtn = CANdashMessage.frame.data0; //start button state from received message
        mode = CANdashMessage.frame.data1; //selector switch state from received message
        miscBtn = CANdashMessage.frame.data2; //misc button state from received message
    }
    
    if (C1INTFbits.ERRIF)
    {
        
        if (C1INTFbits.TXBO == 1)
        {
            CAN1_CallbackBusOff();
            C1INTFbits.TXBO = 0;
        }
        
        if (C1INTFbits.TXBP == 1)
        {
            CAN1_CallbackTxErrorPassive();
            C1INTFbits.TXBP = 0;
        }

        if (C1INTFbits.RXBP == 1)
        {
            CAN1_CallbackRxErrorPassive();
            C1INTFbits.RXBP = 0;
        }

        /* Call error notification function */
        C1INTFbits.ERRIF = 0;  
        
    }    
    
    if(C1INTFbits.RBIF)
    {
        C1INTFbits.RBIF = 0;  //reset interrupt flag
        
        /* Notification function */
        CAN1_CallbackMessageReceived();  
    } 
    
    if(C1INTFbits.WAKIF)
    {
        C1INTFbits.WAKIF = 0;
    }
   
    IFS2bits.C1IF = 0;
}

/*void _ISR _T1Interrupt(void) { //5 min timer for IMD self-testing
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
} // T1Interrupt*/
    
/*
    Main application
 */
int main(void) {
    // initialize the device
    SYSTEM_Initialize();
    CAN1_TransmitEnable();

    //XXX initialize inverter? set baud rate, clear command message not received fault
    HV_CTRL_SetHigh(); //initially, ECU says we good for HV
    updateState();

    while (1) {
        if(mode == ACCEL)
        {
            APPSTorqueMax = ACCEL_MAX;
        }
        else if(mode == AUTOC)
        {
            APPSTorqueMax = AUTOC_MAX;
        }
        else if(mode == ENDUR)
        {
            APPSTorqueMax = ENDUR_MAX;
        }
        else //mode == SKIDP
        {
            APPSTorqueMax = SKIDP_MAX;
        }
        
        updateState();
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

void updateState(void) {
    switch (state)
    {
        case STARTUP: //first time starting up
            if (startBtn) {
                state = DEBUG_STANDBY; //if start btn, enter debug state
            } 
            else {
                state = STANDBY; //if not pushing start btn, enter regular state
            }
            break;
        case STANDBY: //wait for brake and start button to be released
            if(!startBtn && brk1 < BRAKING && brk2 < BRAKING)
            {
                state = WAIT;
            }
        case WAIT: //normal power up
            PUMP_CTRL_SetLow(); //turn off pump
            if (startBtn && brk1 > BRAKING && rawAPPS1 < APPS1_ACCEL_START && rawAPPS1 > APPS1_MIN_RANGE && rawAPPS2 < APPS2_ACCEL_START && rawAPPS2 > APPS2_MIN_RANGE && HV_ON_GetValue()) {
                state = DRIVE; //if start btn, brake, not accelerator, and HV on: enter regular drive state
                RTDbuzz(1);//make the buzz noise
            }
            break;
        case DRIVE: //normal drive state 
            PUMP_CTRL_SetHigh(); //turn on pump
            if (HV_ON_GetValue()) { 
                TMR3 = 0; //reset timer 3
            }
            else
            {
                if(TMR3 >= 6) //wait for 6 * 52 ms = .3 s
                {
                    //HV_CTRL_SetLow(); //kill HV for good. probs don't want to do this cause driver wouldn't be able to reset after hitting e-stop
                    state = WAIT; //exit drive state
                }
            }
            break;
        case DEBUG_STANDBY: //wait for brake and start button to be released
            if(!startBtn && brk1 < BRAKING && brk2 < BRAKING)
            {
                state = DEBUG_WAIT;
            }
        case DEBUG_WAIT: //debug power up
            PUMP_CTRL_SetLow(); //turn off pump
            if (startBtn && brk1 > BRAKING && rawAPPS1 < APPS1_ACCEL_START && rawAPPS1 > APPS1_MIN_RANGE && rawAPPS2 < APPS2_ACCEL_START && rawAPPS2 > APPS2_MIN_RANGE) {
                state = DEBUG_DRIVE; //if start btn, brake, not accelerator: enter debug drive state
                RTDbuzz(4); //make the buzz buzz buzz buzz noise
            }
            break;
        case DEBUG_DRIVE: //debug drive state DON'T LET HV COME ON IF YOURE ALREADY IN DRIVE state
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