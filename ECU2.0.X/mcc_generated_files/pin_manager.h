/**
  PIN MANAGER Generated Driver File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the generated manager file for the PIC24 / dsPIC33 / PIC32MM MCUs device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for PIN MANAGER.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.125
        Device            :  dsPIC33EV256GM106
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.36B
        MPLAB 	          :  MPLAB X v5.20
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#ifndef _PIN_MANAGER_H
#define _PIN_MANAGER_H
/**
    Section: Includes
*/
#include <xc.h>

/**
    Section: Device Pin Macros
*/

//btn 1
#define nBTN1_SetHigh()          _LATA0 = 1
#define nBTN1_SetLow()           _LATA0 = 0
#define nBTN1_Toggle()           _LATA0 ^= 1
#define nBTN1_GetValue()         _RA0
#define nBTN1_SetDigitalInput()  _TRISA0 = 1
#define nBTN1_SetDigitalOutput() _TRISA0 = 0

//HV_ON (precharge done)
#define HV_ON_SetHigh()          _LATA1 = 1
#define HV_ON_SetLow()           _LATA1 = 0
#define HV_ON_Toggle()           _LATA1 ^= 1
#define HV_ON_GetValue()         _RA1
#define HV_ON_SetDigitalInput()  _TRISA1 = 1
#define HV_ON_SetDigitalOutput() _TRISA1 = 0

//rear brake signal
#define BRK_R_SetHigh()          _LATA10 = 1
#define BRK_R_SetLow()           _LATA10 = 0
#define BRK_R_Toggle()           _LATA10 ^= 1
#define BRK_R_GetValue()         _RA10
#define BRK_R_SetDigitalInput()  _TRISA10 = 1
#define BRK_R_SetDigitalOutput() _TRISA10 = 0

//regen okay signal from BMS?
#define nREGEN_OK_SetHigh()          _LATA11 = 1
#define nREGEN_OK_SetLow()           _LATA11 = 0
#define nREGEN_OK_Toggle()           _LATA11 ^= 1
#define nREGEN_OK_GetValue()         _RA11
#define nREGEN_OK_SetDigitalInput()  _TRISA11 = 1
#define nREGEN_OK_SetDigitalOutput() _TRISA11 = 0

//start btn
#define nSTART_BTN_SetHigh()          _LATA12 = 1
#define nSTART_BTN_SetLow()           _LATA12 = 0
#define nSTART_BTN_Toggle()           _LATA12 ^= 1
#define nSTART_BTN_GetValue()         _RA12
#define nSTART_BTN_SetDigitalInput()  _TRISA12 = 1
#define nSTART_BTN_SetDigitalOutput() _TRISA12 = 0

//out 2?
#define OUT2_CTRL_SetHigh()          _LATA4 = 1
#define OUT2_CTRL_SetLow()           _LATA4 = 0
#define OUT2_CTRL_Toggle()           _LATA4 ^= 1
#define OUT2_CTRL_GetValue()         _RA4
#define OUT2_CTRL_SetDigitalInput()  _TRISA4 = 1
#define OUT2_CTRL_SetDigitalOutput() _TRISA4 = 0

//front brake signal
#define BRK_F_SetHigh()          _LATA7 = 1
#define BRK_F_SetLow()           _LATA7 = 0
#define BRK_F_Toggle()           _LATA7 ^= 1
#define BRK_F_GetValue()         _RA7
#define BRK_F_SetDigitalInput()  _TRISA7 = 1
#define BRK_F_SetDigitalOutput() _TRISA7 = 0

//brake light control
#define BRAKE_CTRL_SetHigh()          _LATA8 = 1
#define BRAKE_CTRL_SetLow()           _LATA8 = 0
#define BRAKE_CTRL_Toggle()           _LATA8 ^= 1
#define BRAKE_CTRL_GetValue()         _RA8
#define BRAKE_CTRL_SetDigitalInput()  _TRISA8 = 1
#define BRAKE_CTRL_SetDigitalOutput() _TRISA8 = 0

//RTD buzzzzzzz ctrl
#define BUZZ_CTRL_SetHigh()          _LATA9 = 1
#define BUZZ_CTRL_SetLow()           _LATA9 = 0
#define BUZZ_CTRL_Toggle()           _LATA9 ^= 1
#define BUZZ_CTRL_GetValue()         _RA9
#define BUZZ_CTRL_SetDigitalInput()  _TRISA9 = 1
#define BUZZ_CTRL_SetDigitalOutput() _TRISA9 = 0

//btn 2
#define nBTN2_SetHigh()          _LATB0 = 1
#define nBTN2_SetLow()           _LATB0 = 0
#define nBTN2_Toggle()           _LATB0 ^= 1
#define nBTN2_GetValue()         _RB0
#define nBTN2_SetDigitalInput()  _TRISB0 = 1
#define nBTN2_SetDigitalOutput() _TRISB0 = 0

//WS_RL
#define WS_RL_SetHigh()          _LATB14 = 1
#define WS_RL_SetLow()           _LATB14 = 0
#define WS_RL_Toggle()           _LATB14 ^= 1
#define WS_RL_GetValue()         _RB14
#define WS_RL_SetDigitalInput()  _TRISB14 = 1
#define WS_RL_SetDigitalOutput() _TRISB14 = 0

//WS_RL
#define WS_FR_SetHigh()          _LATB15 = 1
#define WS_FR_SetLow()           _LATB15 = 0
#define WS_FR_Toggle()           _LATB15 ^= 1
#define WS_FR_GetValue()         _RB15
#define WS_FR_SetDigitalInput()  _TRISB15 = 1
#define WS_FR_SetDigitalOutput() _TRISB15 = 0

//HV_CTRL (relay in shutdown loop)
#define HV_CTRL_SetHigh()          _LATB4 = 1
#define HV_CTRL_SetLow()           _LATB4 = 0
#define HV_CTRL_Toggle()           _LATB4 ^= 1
#define HV_CTRL_GetValue()         _RB4
#define HV_CTRL_SetDigitalInput()  _TRISB4 = 1
#define HV_CTRL_SetDigitalOutput() _TRISB4 = 0

//APPS1 analog interface in case CAN bus fails
#define APPS1_SetHigh()          _LATB8 = 1
#define APPS1_SetLow()           _LATB8 = 0
#define APPS1_Toggle()           _LATB8 ^= 1
#define APPS1_GetValue()         _RB8
#define APPS1_SetDigitalInput()  _TRISB8 = 1
#define APPS1_SetDigitalOutput() _TRISB8 = 0

//LV battery measurement
#define LV_BATT_MEAS_SetHigh()          _LATC10 = 1
#define LV_BATT_MEAS_SetLow()           _LATC10 = 0
#define LV_BATT_MEAS_Toggle()           _LATC10 ^= 1
#define LV_BATT_MEAS_GetValue()         _RC10
#define LV_BATT_MEAS_SetDigitalInput()  _TRISC10 = 1
#define LV_BATT_MEAS_SetDigitalOutput() _TRISC10 = 0

//CAN standby
#define CAN_STBY_SetHigh()          _LATC13 = 1
#define CAN_STBY_SetLow()           _LATC13 = 0
#define CAN_STBY_Toggle()           _LATC13 ^= 1
#define CAN_STBY_GetValue()         _RC13
#define CAN_STBY_SetDigitalInput()  _TRISC13 = 1
#define CAN_STBY_SetDigitalOutput() _TRISC13 = 0

//out1? 
#define OUT1_CTRL_SetHigh()          _LATC3 = 1
#define OUT1_CTRL_SetLow()           _LATC3 = 0
#define OUT1_CTRL_Toggle()           _LATC3 ^= 1
#define OUT1_CTRL_GetValue()         _RC3
#define OUT1_CTRL_SetDigitalInput()  _TRISC3 = 1
#define OUT1_CTRL_SetDigitalOutput() _TRISC3 = 0

//pump ctrl
#define PUMP_CTRL_SetHigh()          _LATC4 = 1
#define PUMP_CTRL_SetLow()           _LATC4 = 0
#define PUMP_CTRL_Toggle()           _LATC4 ^= 1
#define PUMP_CTRL_GetValue()         _RC4
#define PUMP_CTRL_SetDigitalInput()  _TRISC4 = 1
#define PUMP_CTRL_SetDigitalOutput() _TRISC4 = 0

//APPS2
#define APPS2_SetHigh()          _LATC5 = 1
#define APPS2_SetLow()           _LATC5 = 0
#define APPS2_Toggle()           _LATC5 ^= 1
#define APPS2_GetValue()         _RC5
#define APPS2_SetDigitalInput()  _TRISC5 = 1
#define APPS2_SetDigitalOutput() _TRISC5 = 0

//LED1
#define LED1_SetHigh()          _LATC6 = 1
#define LED1_SetLow()           _LATC6 = 0
#define LED1_Toggle()           _LATC6 ^= 1
#define LED1_GetValue()         _RC6
#define LED1_SetDigitalInput()  _TRISC6 = 1
#define LED1_SetDigitalOutput() _TRISC6 = 0

//LED2
#define LED2_SetHigh()          _LATC7 = 1
#define LED2_SetLow()           _LATC7 = 0
#define LED2_Toggle()           _LATC7 ^= 1
#define LED2_GetValue()         _RC7
#define LED2_SetDigitalInput()  _TRISC7 = 1
#define LED2_SetDigitalOutput() _TRISC7 = 0

//LED3
#define LED3_SetHigh()          _LATC8 = 1
#define LED3_SetLow()           _LATC8 = 0
#define LED3_Toggle()           _LATC8 ^= 1
#define LED3_GetValue()         _RC8
#define LED3_SetDigitalInput()  _TRISC8 = 1
#define LED3_SetDigitalOutput() _TRISC8 = 0

//LED4
#define LED4_SetHigh()          _LATC9 = 1
#define LED4_SetLow()           _LATC9 = 0
#define LED4_Toggle()           _LATC9 ^= 1
#define LED4_GetValue()         _RC9
#define LED4_SetDigitalInput()  _TRISC9 = 1
#define LED4_SetDigitalOutput() _TRISC9 = 0

//RX for CAN
#define RX_SetHigh()          _LATF0 = 1
#define RX_SetLow()           _LATF0 = 0
#define RX_Toggle()           _LATF0 ^= 1
#define RX_GetValue()         _RF0
#define RX_SetDigitalInput()  _TRISF0 = 1
#define RX_SetDigitalOutput() _TRISF0 = 0

//WS_RR
#define WS_RR_SetHigh()          _LATG6 = 1
#define WS_RR_SetLow()           _LATG6 = 0
#define WS_RR_Toggle()           _LATG6 ^= 1
#define WS_RR_GetValue()         _RG6
#define WS_RR_SetDigitalInput()  _TRISG6 = 1
#define WS_RR_SetDigitalOutput() _TRISG6 = 0

//WS_FL
#define WS_FL_SetHigh()          _LATG7 = 1
#define WS_FL_SetLow()           _LATG7 = 0
#define WS_FL_Toggle()           _LATG7 ^= 1
#define WS_FL_GetValue()         _RG7
#define WS_FL_SetDigitalInput()  _TRISG7 = 1
#define WS_FL_SetDigitalOutput() _TRISG7 = 0

/**
    Section: Function Prototypes
*/
/**
  @Summary
    Configures the pin settings of the dsPIC33EV256GM106
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description
    This is the generated manager file for the PIC24 / dsPIC33 / PIC32MM MCUs device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    void SYSTEM_Initialize(void)
    {
        // Other initializers are called from this function
        PIN_MANAGER_Initialize();
    }
    </code>

*/
void PIN_MANAGER_Initialize (void);


#endif
