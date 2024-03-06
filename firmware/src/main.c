/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "../VCU_BANCADA.X/main.h"

#include <stdbool.h>  // Defines true
#include <stddef.h>   // Defines NULL
#include <stdlib.h>   // Defines EXIT_FAILURE

#include "../../../CANSART/Controller Library/PIC32/Library/cansart.h"
#include "../VCU_BANCADA.X/CAN_utils.h"
#include "../VCU_BANCADA.X/cansart_db_lc.h"
#include "../VCU_BANCADA.X/utils.h"

struct frame10 frames10;

uint32_t status1;
// uint32_t status1 = 0;
//  CAN Messages to send to Inverter
uint8_t SetCurrent[2] = {0, 0};
uint8_t SetBrakeCurrent[2] = {0, 0};
uint8_t SetERPM[4] = {0, 0, 0, 0};
uint8_t SetPosition[2] = {0, 0};
uint8_t SetRelativeCurrent[2] = {0, 0};
uint8_t SetRelativeBrakeCurrent[2] = {0, 0};
uint8_t SetDigitalOutput[4] = {0, 0, 0, 0};
uint8_t SetMaxACCurrent[2] = {0, 0};
uint8_t SetMaxACBrakeCurrent[2] = {0, 0};
uint8_t SetMaxDCCurrent[2] = {0, 0};
uint8_t SetMaxDCBrakeCurrent[2] = {0, 0};
uint8_t DriveEnable[1] = {0};

// CAN Messages to receive from Inverter
int32_t ERPM = 0; // Electrical RPM Equation: ERPM = Motor RPM * number of the motor pole pairs
int16_t DutyCycle = 0; // The controller duty cycle. The sign of this value will represent whether the motor is running(positive) current or regenerating (negative) current
int16_t InputVoltage = 0; // Input voltage is the DC voltage
int16_t ACcurrent = 0; // The motor current. The sign of this value represents whether the motor is running(positive) current or regenerating (negative) current
int16_t DCcurrent = 0; // DC Current: Current on DC side. The sign of this value represents whether the motor is running(positive) current or regenerating (negative) current.
int8_t ControllerTemperature = 0; // Temperature of the inverter semiconductors
int16_t MotorTemperature = 0; // Temperature of the motor measured by the inverter
int8_t Faults = 0; // 0x00 : NO FAULTS
// 0x01 : Overvoltage - The input voltage is higher than the set maximum.
// 0x02 : Undervoltage - The input voltage is lower than the set minimum.
// 0x03 : DRV - Transistor or transistor drive error
// 0x04 : ABS. Overcurrent - The AC current is higher than the set absolute maximum current.
// 0x05 : CTLR Overtemp. - The controller temperature is higher than the set maximum.
// 0x06 : Motor Overtemp. - The motor temperature is higher than the set maximum.
// 0x07 : Sensor wire fault - Something went wrong with the sensor differential signals.
// 0x08 : Sensor general fault - An error occurred while processing the sensor signals
// 0x09 : CAN Command error - CAN message received contains parameter out of boundaries
// 0x0A : Analog input error  Redundant output out of range
int8_t ThrottleSignal = 0; // Throttle signal derived from analog inputs or CAN2
int8_t BrakeSignal = 0; // Brake signal derived from analog inputs or CAN2
bool DigitalInput1 = 0; // 0-1
bool DigitalInput2 = 0; // 0-1
bool DigitalInput3 = 0; // 0-1
bool DigitalInput4 = 0; // 0-1
bool DigitalOutput1 = 0; // 0-1
bool DigitalOutput2 = 0; // 0-1
bool DigitalOutput3 = 0; // 0-1
bool DigitalOutput4 = 0; // 0-1
bool DriveEnabled = 0; // 0-1
bool CapacitorTempLimit = 0; // 0-1
bool DCcurrentLimit = 0; // 0-1
bool DriveEnableLimit = 0; // 0-1
bool IGBTAccelLimit = 0; // 0-1
bool IGBTTempLimit = 0; // 0-1
bool InputVoltageLimit = 0; // 0-1
bool MotorAccelLimit = 0; // 0-1
bool MotorTempLimit = 0; // 0-1
bool RPMMinLimit = 0; // 0-1
bool RPMMaxLimit = 0; // 0-1
bool PowerLimit = 0; // 0-1

// #include "definitions.h"  // SYS function prototypes
//  *****************************************************************************
//  *****************************************************************************
//  Section: Main Entry Point
//  *****************************************************************************
//  *****************************************************************************

// Define a macro DEBUG para ativar ou desativar o debug_printf
#define DEBUG 0
#define LABVIEW 0
#define INVERTED_APPS 0

// Define o debug_printf para ser ativado ou desativado com base na macro DEBUG
#if DEBUG
#define debug_printf(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...)
#endif

#if LABVIEW
#define labview_printf(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define labview_printf(fmt, ...)
#endif

// ############# ADC ########################################

uint8_t message_ADC[64]; // CAN message to send ADC data
__COHERENT uint16_t ADC[64]; // ADC raw data

// ############# MILIS #######################################
unsigned int previousMillis[10] = {};
unsigned int currentMillis[10] = {};
// ############# MILIS #######################################

unsigned int millis() {
    return (unsigned int) (CORETIMER_CounterGet() / (CORE_TIMER_FREQUENCY / 1000));
}

uint16_t VCU_Temp = 0;

// ############# FUNCTIONS ##################################
void Read_ADC(ADCHS_CHANNEL_NUM channel); // Read ADC function
bool APPS_Function(uint16_t APPS1, uint16_t APPS2); // APPS function to calculate average and percentage

void startupSequence(void); // Startup sequence
void PrintToConsole(uint8_t); // Print data to console
void ReadTEMP(void); // Read temperature

// ############# TMR FUNCTIONS ###############################

void TMR1_5ms(uint32_t status, uintptr_t context) { // 200Hz
    // memset(message_CAN_TX, 0, sizeof(message_CAN_TX));
    // SendID_20();  // send data to ID 0x20 in DataBus
    // memset(message_CAN_TX, 0, sizeof(message_CAN_TX));
    // SendID_23();  // send data to ID 0x23 in DataBus
}

void TMR2_100ms(uint32_t status, uintptr_t context) { // 10Hz
    // memset(message_CAN_TX, 0x00, sizeof(message_CAN_TX));
    // SendID_21();  // send data to ID 0x21 in DataBus
    // memset(message_CAN_TX, 0x00, sizeof(message_CAN_TX));
    // SendID_22();  // send data to ID 0x22 in DataBus
}

void TMR4_500ms(uint32_t status, uintptr_t context) { // 2Hz
    GPIO_RC11_Toggle(); // Heartbeat led
}

void TMR5_100ms(uint32_t status, uintptr_t context) {
    // apps_error = APPS_Function(ADC[0], ADC[3]);  // checks if there is an error in the APPS and calculates the average and percentage
}

void TMR6_5ms(uint32_t status, uintptr_t context) {
    APPS_Function(ADC[0], ADC[3]);
    // setSetERPM(250 * APPS_Percentage);  // Send APPS_percent to inverter
    setSetERPM(25 * APPS_Percentage_1000);

    setDriveEnable(1);
}

// ############# ADC CALLBACKS ###############################

void ADCHS_CH0_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    static int samples[4] = {0};
    static int i = 0;

    samples[i] = ADCHS_ChannelResultGet(channel);
    i = (i + 1) % 4;

    int sum = 0;
    for (int j = 0; j < 4; j++) {
        sum += samples[j];
    }
    ADC[channel] = sum / 4;
}

void ADCHS_CH3_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    static int samples[4] = {0};
    static int i = 0;

    samples[i] = ADCHS_ChannelResultGet(channel);
    i = (i + 1) % 4;

    int sum = 0;
    for (int j = 0; j < 4; j++) {
        sum += samples[j];
    }
    ADC[channel] = sum / 4;
}

int main(void) {
    /* Initialize all modules */
    SYS_Initialize(NULL);

    ADCHS_CallbackRegister(ADCHS_CH0, ADCHS_CH0_Callback, (uintptr_t) NULL); // APPS1 callback
    ADCHS_CallbackRegister(ADCHS_CH3, ADCHS_CH3_Callback, (uintptr_t) NULL); // APPS2 callback
    CTMUCONbits.TGEN = 0;
    ADCHS_ChannelConversionStart(ADCHS_CH53);
    TMR1_CallbackRegister(TMR1_5ms, (uintptr_t) NULL); // 200Hz
    TMR2_CallbackRegister(TMR2_100ms, (uintptr_t) NULL); // 10Hz
    TMR4_CallbackRegister(TMR4_500ms, (uintptr_t) NULL); // 2Hz heartbeat led
    TMR5_CallbackRegister(TMR5_100ms, (uintptr_t) NULL); // 10Hz
    TMR6_CallbackRegister(TMR6_5ms, (uintptr_t) NULL); // 200Hz to send data to the inverter

    fflush(stdout);

    TMR1_Start();
    CORETIMER_DelayMs(5);
    TMR2_Start();
    CORETIMER_DelayMs(5);
    TMR3_Start(); // Used trigger source for ADC conversion
    CORETIMER_DelayMs(5);
    TMR4_Start();
    CORETIMER_DelayMs(5);
    TMR5_Start();
    CORETIMER_DelayMs(5);
    TMR6_Start();
    CORETIMER_DelayMs(5);

    APPS_Init(0.3, 3.0, 0.2); // Initialize APPS
    frames10.ID = 10;
    frames10.LENGHT = 8;
    startupSequence(); // led sequence
    while (true) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();
        // frames10.ADC0H = ADC[0] >> 8;
        //  frames10.ADC0L = ADC[0];
        //  frames10.ADC3H = ADC[3] >> 8;
        //  frames10.ADC3L = ADC[3];

        updateDB(&frames10);
        Read_CAN_BUS_1(); // Read DataBus
        //  Read_CAN_2();  // Read PowerTrainBus
        //  Read_CAN_3();  // Read AutonomousBus

        if (UART1_ReceiverIsReady()) {
            // read until gets a \0

            uint8_t data2[8];
            UART1_Read(data2, 8);
            // TODO need to do a little protection on this
            float apps_min = ((data2[0] << 8) | data2[1]) / 10;
            float apps_max = (data2[2] << 8) | data2[3] / 10;
            float apps_error = (data2[4] << 8) | data2[5] / 10;

            APPS_Init(apps_min, apps_max, apps_error);
        }
        ReadTEMP();
        PrintToConsole(33); // Print data to console time in ms
    }
    /* Execution should not come here during normal operation */
    return (EXIT_FAILURE);
}

/**
 * This command sets the target motor AC current (peak, not
 * RMS). When the controller receives this message, it
 * automatically switches to current control mode.
 * This value must not be above the limits of the inverter and must
 * be multiplied by 10 before sending. This is a signed parameter,
 * and the sign represents the direction of the torque which
 * correlates with the motor AC current. (For the correlation, please
 * refer to the motor parameters)
 * @param current The current value to set.
 */
void setSetCurrent(int16_t current) {
    current = current * 10;
    uint8_t temp[2];
    temp[0] = current >> 8;
    temp[1] = current;
    Send_CAN_BUS_1(SetCurrent_ID, temp, 2);
}

/**
 * Targets the brake current of the motor. It will result negative
 * torque relatively to the forward direction of the motor.
 * This value must be multiplied by 10 before sending, only
 * positive currents are accepted.
 * @param brakeCurrent The brake current value to set.
 */
void setSetBrakeCurrent(int16_t current) {
    uint8_t temp[2];
    temp[0] = current >> 8;
    temp[1] = current;
    Send_CAN_BUS_1(SetBrakeCurrent_ID, temp, 2);
}

/**
 * This command enables the speed control of the motor with a
 * target ERPM. This is a signed parameter, and the sign
 * represents the direction of the spinning. For better operation you
 * need to tune the PID of speed control.
 * Equation: ERPM = Motor RPM * number of the motor pole pairs.
 * @param ERPM The ERPM value to set.
 */
void setSetERPM(int32_t erpm) {
    uint8_t temp[4];
    temp[0] = erpm >> 24;
    temp[1] = erpm >> 16;
    temp[2] = erpm >> 8;
    temp[3] = erpm;
    Send_CAN_BUS_1(SetERPM_ID, temp, 4);
}

/**
 * This value targets the desired position of the motor in degrees.
 * This command is used to hold a position of the motor.
 * This feature is enabled only if encoder is used as position
 * sensor. The value has to be multiplied by 10 before sending.
 * @param position The position value to set.
 */
void setSetPosition(int16_t position) {
    position = position * 10;
    SetPosition[0] = position >> 8;
    SetPosition[1] = position;
    Send_CAN_BUS_1(SetPosition_ID, SetPosition, 2);
}

/**
 * This command sets a relative AC current to the minimum and
 * maximum limits set by configuration. This achieves the same
 * function as the �??Set AC current�?? command. Gives you a freedom
 * to send values between -100,0% and 100,0%. You do not need
 * to know the motor limit parameters.
 * This value must be between -100 and 100 and must be
 * multiplied by 10 before sending.
 * @param relativecurrent The relative current value to set.
 */
void setSetRelativeCurrent(int16_t relativecurrent) {
    // This value must be between -100 and 100 and must be multiplied by 10 before sending.
    if (relativecurrent > 100) {
        relativecurrent = 100;
    } else if (relativecurrent < -100) {
        relativecurrent = -100;
    }
    relativecurrent = relativecurrent * 10;
    SetRelativeCurrent[0] = relativecurrent >> 8;
    SetRelativeCurrent[1] = relativecurrent;
    Send_CAN_BUS_1(SetRelativeCurrent_ID, SetRelativeCurrent, 2);
}

/**
 * Targets the relative brake current of the motor. It will result
 * negative torque relatively to the forward direction of the motor.
 * This value must be between 0 and 100 and must be multiplied
 * by 10 before sending Gives you a freedom to send values
 * between 0% and 100,0%. You do not need to know the motor
 * limit parameters.
 * This value must be between 0 and 100 and has to be multiplied
 * by 10 before sending
 * @param relativebrakecurrent The relative brake current value to set.
 */
void setSetRelativeBrakeCurrent(int16_t relativebrakecurrent) {
    // This value must be between 0 and 100 and must be multiplied by 10 before sending
    if (relativebrakecurrent > 100) {
        relativebrakecurrent = 100;
    } else if (relativebrakecurrent < 0) {
        relativebrakecurrent = 0;
    }
    relativebrakecurrent = relativebrakecurrent * 10;
    SetRelativeBrakeCurrent[0] = relativebrakecurrent >> 8;
    SetRelativeBrakeCurrent[1] = relativebrakecurrent;
    Send_CAN_BUS_1(SetRelativeBrakeCurrent_ID, SetRelativeBrakeCurrent, 2);
}
/**
 * Sets the digital output values and sends them over CAN bus.
 * @param digitaloutput1 The first digital output value to set.
 * @param digitaloutput2 The second digital output value to set.
 * @param digitaloutput3 The third digital output value to set.
 * @param digitaloutput4 The fourth digital output value to set.
 */
/*
 void setSetDigitalOutput(bool digitaloutput1, bool digitaloutput2, bool digitaloutput3, bool digitaloutput4) {
     SetDigitalOutput[0] = digitaloutput1;
     SetDigitalOutput[1] = digitaloutput2;
     SetDigitalOutput[2] = digitaloutput3;
     SetDigitalOutput[3] = digitaloutput4;
     Send_CAN_1(SetDigitalOutput_ID, SetDigitalOutput, 4);
 }*/

/**
 * This value determines the maximum allowable drive current on
 * the AC side. With this function you are able maximize the
 * maximum torque on the motor.
 * The value must be multiplied by 10 before sending.
 * @param maxcurrent The maximum AC current value to set.
 */
void setSetMaxACCurrent(int16_t maxcurrent) {
    maxcurrent = maxcurrent * 10;
    SetMaxACCurrent[0] = maxcurrent >> 8;
    SetMaxACCurrent[1] = maxcurrent;
    Send_CAN_BUS_1(SetMaxACCurrent_ID, SetMaxACCurrent, 2);
}

/**
 * This value sets the maximum allowable brake current on the AC side.
 * This value must be multiplied by 10 before sending, only
 * negative currents are accepted.
 * @param maxbrakecurrent The maximum AC brake current value to set.
 */
void setSetMaxACBrakeCurrent(int16_t maxbrakecurrent) {
    // This value must be multiplied by 10 before sending, only negative currents are accepted.
    if (maxbrakecurrent > 0) {
        maxbrakecurrent = 0;
    }
    maxbrakecurrent = maxbrakecurrent * 10;
    SetMaxACBrakeCurrent[0] = maxbrakecurrent >> 8;
    SetMaxACBrakeCurrent[1] = maxbrakecurrent;
    Send_CAN_BUS_1(SetMaxACBrakeCurrent_ID, SetMaxACBrakeCurrent, 2);
}

/**
 * This value determines the maximum allowable drive current on
 * the DC side. With this command the BMS can limit the
 * maximum allowable battery discharge current.
 * The value has to be multiplied by 10 before sending.
 * @param maxdccurrent The maximum DC current value to set.
 */
void setSetMaxDCCurrent(int16_t maxdccurrent) {
    maxdccurrent = maxdccurrent * 10;
    SetMaxDCCurrent[0] = maxdccurrent >> 8;
    SetMaxDCCurrent[1] = maxdccurrent;
    Send_CAN_BUS_1(SetMaxDCCurrent_ID, SetMaxDCCurrent, 2);
}

/**
 * This value determines the maximum allowable brake current
 * on the DC side. With this command the BMS can limit the
 * maximum allowable battery charge current.
 * The value has to be multiplied by 10 before sending. Only
 * negative currents are accepted.
 * @param maxdcbrakecurrent The maximum DC brake current value to set.
 */
void setSetMaxDCBrakeCurrent(int16_t maxdcbrakecurrent) {
    // The value has to be multiplied by 10 before sending. Only negative currents are accepted.
    if (maxdcbrakecurrent > 0) {
        maxdcbrakecurrent = 0;
    }
    maxdcbrakecurrent = maxdcbrakecurrent * 10;
    SetMaxDCBrakeCurrent[0] = maxdcbrakecurrent >> 8;
    SetMaxDCBrakeCurrent[1] = maxdcbrakecurrent;
    Send_CAN_BUS_1(SetMaxDCBrakeCurrent_ID, SetMaxDCBrakeCurrent, 2);
}

/**
 * 0: Drive not allowed
 * 1: Drive allowed
 * Only 0 and 1 values are accepted. Must be sent periodically to
 * be enabled. Refer to chapter 4.3
 * @param driveenable The drive enable value to set.
 */
void setDriveEnable(bool driveenable) {
    DriveEnable[0] = driveenable;
    Send_CAN_BUS_1(DriveEnable_ID, DriveEnable, 1);
}

void startupSequence() {
    /*LEDs Start up sequence*/
    GPIO_RC11_Set();
    CORETIMER_DelayMs(75);
    GPIO_RC2_Set();
    CORETIMER_DelayMs(75);
    GPIO_RC11_Clear();
    CORETIMER_DelayMs(75);
    GPIO_RC2_Clear();
    CORETIMER_DelayMs(75);
    GPIO_RC11_Set();
    CORETIMER_DelayMs(75);
    GPIO_RC2_Set();
    CORETIMER_DelayMs(75);
    GPIO_RC11_Clear();
    CORETIMER_DelayMs(75);
    GPIO_RC2_Clear();
}

void PrintToConsole(uint8_t time) {
    // Print data-----



    currentMillis[3] = millis();
    if (currentMillis[3] - previousMillis[3] >= time) {
        // APPS_PrintValues();

        // printf("APPSA%dAPPSB%dAPPST%dAPPS_ERROR%dAPPS_Perc%dCAN_ERROR%d", ADC[0], ADC[3], APPS_Mean, APPS_Error, APPS_Percentage, status1);
        // printf("APPS_MIN%dAPPS_MAX%dAPPS_TOL%d", APPS_MIN_bits, APPS_MAX_bits, APPS_Tolerance_bits);
        // printf("VCU_TEMP%d", VCU_Temp);
        // printf("\r\n");

        // printf("\r\n");
        previousMillis[3] = currentMillis[3];
    }
}

void ReadTEMP() {
    currentMillis[4] = millis();
    if (currentMillis[4] - previousMillis[4] >= 500) {
        if (CTMUCONbits.EDG1STAT == CTMUCONbits.EDG1STAT) {
            if (ADCHS_ChannelResultIsReady(ADCHS_CH53)) {
                VCU_Temp = ADCHS_ChannelResultGet(ADCHS_CH53);
            }
            ADCHS_ChannelConversionStart(ADCHS_CH53);
            previousMillis[4] = currentMillis[4];
        }
    }
}