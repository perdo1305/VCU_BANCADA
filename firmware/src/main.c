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

#include "main.h"

#include <stdbool.h>  // Defines true
#include <stddef.h>   // Defines NULL
#include <stdlib.h>   // Defines EXIT_FAILURE

#include "definitions.h"  // SYS function prototypes
// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

// Define a macro DEBUG para ativar ou desativar o debug_printf
#define DEBUG 1
#define LABVIEW 0
#define INVERTED_APPS 1

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

// ############# RX CAN FRAME ###############################

CANFD_MSG_RX_ATTRIBUTE msgAttr = CANFD_MSG_RX_DATA_FRAME;  // RX message attribute

static uint8_t rx_message[64] = {};  // CAN message receive buffer
uint32_t rx_messageID = 0;           // RX message ID
uint32_t status = 0;                 // CAN status
uint8_t rx_messageLength = 0;        // RX message length

bool CANRX_ON = 0;  // flag to check if CAN is receiving
bool CANTX_ON = 0;  // flag to check if CAN is transmitting

// ############# TX CAN FRAME ###############################
uint8_t message_CAN_TX[8] = {};  // TX message buffer
uint8_t cantx_message[8] = {};   // TX message buffer

// ############# ADC ########################################

uint8_t message_ADC[64];    // CAN message to send ADC data
uint16_t ADC[64];           // ADC raw data
uint16_t APPS_average = 0;  // average of APPS1 and APPS2
uint16_t APPS_percent = 0;  // 0-100% of average of APPS1 and APPS2
bool apps_error = 0;        // error if APPS1 and APPS2 are 10% apart
bool error_flag = 0;        //  used in apps function to check if there is an error

// ############# MILIS #######################################
unsigned int previousMillis[10] = {};
unsigned int currentMillis[10] = {};
// ############# MILIS #######################################

unsigned int millis(void) {
    return (unsigned int)(CORETIMER_CounterGet() / (CORE_TIMER_FREQUENCY / 1000));
}

// ############# FUNCTIONS ##################################
void Read_ADC(ADCHS_CHANNEL_NUM channel);                    // Read ADC function
void Read_CAN(void);                                         // Read CAN function
bool APPS_Function(uint16_t APPS1, uint16_t APPS2);          // APPS function to calculate average and percentage
void Send_CAN(uint32_t id, uint8_t* message, uint8_t size);  // Send CAN function
void startupSequence(void);                                  // Startup sequence
void PrintToConsole(uint8_t);                                   // Print data to console

// ############# TMR FUNCTIONS ###############################
void TMR1_5ms(uint32_t status, uintptr_t context) {  // 200Hz
    memset(message_CAN_TX, 0, sizeof(message_CAN_TX));
    SendID_20();  // send data to ID 0x20 in DataBus
    memset(message_CAN_TX, 0, sizeof(message_CAN_TX));
    SendID_23();  // send data to ID 0x23 in DataBus
}

void TMR2_100ms(uint32_t status, uintptr_t context) {  // 10Hz
    memset(message_CAN_TX, 0x00, sizeof(message_CAN_TX));
    SendID_21();  // send data to ID 0x21 in DataBus
    memset(message_CAN_TX, 0x00, sizeof(message_CAN_TX));
    SendID_22();  // send data to ID 0x22 in DataBus
}

void TMR4_500ms(uint32_t status, uintptr_t context) {  // 2Hz
    GPIO_RC11_Toggle();                                // Heartbeat led
}

void TMR5_100ms(uint32_t status, uintptr_t context) {
    apps_error = APPS_Function(ADC[0], ADC[3]);  // checks if there is an error in the APPS and calculates the average and percentage
}
void TMR6_5ms(uint32_t status, uintptr_t context) {
    setSetERPM(APPS_percent);  // Send APPS_percent to inverter
}

// ############# ADC CALLBACKS ###############################
void ADCHS_CH0_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    ADC[channel] = ADCHS_ChannelResultGet(channel);
}

void ADCHS_CH3_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    ADC[channel] = ADCHS_ChannelResultGet(channel);
}

int main(void) {
    /* Initialize all modules */
    SYS_Initialize(NULL);

    ADCHS_ModulesEnable(ADCHS_MODULE0_MASK);                                 // APPS1
    ADCHS_ModulesEnable(ADCHS_MODULE3_MASK);                                 // APPS2
    ADCHS_CallbackRegister(ADCHS_CH0, ADCHS_CH0_Callback, (uintptr_t)NULL);  // APPS1 callback
    ADCHS_CallbackRegister(ADCHS_CH3, ADCHS_CH3_Callback, (uintptr_t)NULL);  // APPS2 callback

    TMR1_CallbackRegister(TMR1_5ms, (uintptr_t)NULL);  // 200Hz
    TMR1_Start();
    TMR2_CallbackRegister(TMR2_100ms, (uintptr_t)NULL);  // 10Hz
    TMR2_Start();
    TMR3_Start();                                        // Used trigger source for ADC conversion
    TMR4_CallbackRegister(TMR4_500ms, (uintptr_t)NULL);  // 2Hz heartbeat led
    TMR4_Start();
    TMR5_CallbackRegister(TMR5_100ms, (uintptr_t)NULL);  // 10Hz
    TMR5_Start();
    TMR6_CallbackRegister(TMR6_5ms, (uintptr_t)NULL);  // 200Hz to send data to the inverter
    TMR6_Start();

    printf("██████╗░███████╗░██████╗███████╗████████╗\r\n");
    printf("██╔══██╗██╔════╝██╔════╝██╔════╝╚══██╔══╝\r\n");
    printf("██████╔╝█████╗░░╚█████╗░█████╗░░░░░██║░░░\r\n");
    printf("██╔══██╗██╔══╝░░░╚═══██╗██╔══╝░░░░░██║░░░\r\n");
    printf("██║░░██║███████╗██████╔╝███████╗░░░██║░░░\r\n");
    printf("╚═╝░░╚═╝╚══════╝╚═════╝░╚══════╝░░░╚═╝░░░\r\n");
    printf("\n\n");
    fflush(stdout);

    startupSequence();  // led sequence

    setDriveEnable(1);  // Enable the inverter

    while (true) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();
        Read_CAN();
        PrintToConsole(200);

    }
    /* Execution should not come here during normal operation */
    return (EXIT_FAILURE);
}

/*******************************************************************************
 End of File
 */

void Read_ADC(ADCHS_CHANNEL_NUM channel) {
    ADCHS_ChannelConversionStart(channel);

    if (ADCHS_ChannelResultIsReady(channel)) {
        ADC[channel] = ADCHS_ChannelResultGet(channel);
        printf("ADC%d: %d\r\n", channel, ADC[channel]);
    }
}

void Read_CAN() {
    status = CAN1_ErrorGet();
    // printf("Status: %d\r\n", status);

    if (status == CANFD_ERROR_NONE) {
        memset(rx_message, 0x00, sizeof(rx_message));
        if (CAN1_MessageReceive(&rx_messageID, &rx_messageLength, rx_message, 0, 2, &msgAttr)) {
            /*
            printf(" New Message Received    \r\n");
            uint8_t length = rx_messageLength;
            printf("ID: 0x%x Len: 0x%x ", (unsigned int)rx_messageID, (unsigned int)rx_messageLength);
            printf("Msg: ");

            while (length) {
                printf("%d ", rx_message[rx_messageLength - length--]);
            }
            printf("\r\n");
             */
            CANRX_ON = 1;
            GPIO_RC2_Toggle();
        }
    } else {
        GPIO_RC2_Clear();
        CANRX_ON = 0;
    }
}

void Send_CAN(uint32_t id, uint8_t* message, uint8_t size) {
    if (CAN1_TxFIFOQueueIsFull(0)) {
        // debug_printf("CAN1_TxFIFOQueueIsFull\r\n");
        CANTX_ON = 0;
    } else {
        if (CAN1_MessageTransmit(id, size, message, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME)) {
            // debug_printf("id 0x%x sent successfully \r\n", id);
            //   debug_printf("ID: 0x%x Len: 0x%x DATA:", (unsigned int)id, (unsigned int)size);
            // for (int i = 0; i < size; i++) {s
            //  debug_printf("%d ", message[i]);
            //}
            // debug_printf("\r\n");
            //  GPIO_RC2_Toggle();
            CANTX_ON = 1;
        } else {
            // debug_printf("id 0x%x not sent\r\n", id);
            CANTX_ON = 0;
        }
    }
}

bool APPS_Function(uint16_t APPS1, uint16_t APPS2) {
    /* range :   APPS1 0-4095
                 APPS2 4095-0
                 */

    static unsigned long error_start_time = 0;  // timer to check if there is an error
    unsigned long current_time;                 // used to save millis value
    int diff;                                   // difference between APPS1 and APPS2

#if INVERTED_APPS
    APPS2 = 4095 - APPS2;  // invert APPS2
#endif

    diff = abs(APPS1 - APPS2);  // Check if APPS1 and APPS2 are too far apart (10% = 409)

    if (diff > 409 || (APPS1 < 5 || APPS2 < 5) || (APPS1 > 3995 || APPS2 > 3995)) {  // Error detected
        if (error_flag == 0) {
            error_start_time = millis();  // save the time when the error was detected
            error_flag = 1;
        } else {  // Error already detected, check if timer has expired

            current_time = millis();  // save the current time
            if (current_time - error_start_time > 100) {
                // Error has persisted for more than 100 ms, set error flag
                error_flag = 1;
            }
        }
    } else {  // No error, reset error flag and timer
        error_flag = 0;
        error_start_time = 0;
    }

    APPS_average = (APPS1 + APPS2) / 2;  // calculates the average and percentage of the two APPS
    APPS_percent = (APPS_average * 100) / 4095;

    // debug_printf("APPS_average: %d\r\n", APPS_average);
    // debug_printf("APPSA%d APPSB%d APPST%d APPS_ERROR%d\r", APPS1, APPS2, APPS_percent, error_flag);
    return error_flag;
}
/**
 * Sets the current value and sends it over CAN bus.
 * The current value is multiplied by 10 before sending.
 * @param current The current value to set.
 */
void setSetCurrent(int16_t current) {
    current = current * 10;
    SetCurrent[0] = current >> 8;
    SetCurrent[1] = current;
    Send_CAN(SetCurrent_ID, (uint8_t*)SetCurrent, 2);
}
/**
 * Sets the brake current value and sends it over CAN bus.
 * The brake current value is multiplied by 10 before sending.
 * @param brakeCurrent The brake current value to set.
 */
void setSetBrakeCurrent(int16_t brakeCurrent) {
    brakeCurrent = brakeCurrent * 10;
    SetBrakeCurrent[0] = brakeCurrent >> 8;
    SetBrakeCurrent[1] = brakeCurrent;
    Send_CAN(SetBrakeCurrent_ID, (uint8_t*)SetBrakeCurrent, 2);
}
/**
 * Sets the ERPM value and sends it over CAN bus.
 * @param ERPM The ERPM value to set.
 */
void setSetERPM(int32_t ERPM) {
    SetERPM[0] = ERPM >> 24;
    SetERPM[1] = ERPM >> 16;
    SetERPM[2] = ERPM >> 8;
    SetERPM[3] = ERPM;
    Send_CAN(SetERPM_ID, (uint8_t*)SetERPM, 4);
}

/**
 * Sets the position value and sends it over CAN bus.
 * The position value is multiplied by 10 before sending.
 * @param position The position value to set.
 */
void setSetPosition(int16_t position) {
    position = position * 10;
    SetPosition[0] = position >> 8;
    SetPosition[1] = position;
    Send_CAN(SetPosition_ID, (uint8_t*)SetPosition, 2);
}
/**
 * Sets the relative current value and sends it over CAN bus.
 * The relative current value is multiplied by 10 before sending.
 * The value must be between -100 and 100.
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
    Send_CAN(SetRelativeCurrent_ID, (uint8_t*)SetRelativeCurrent, 2);
}

/**
 * Sets the relative brake current value and sends it over CAN bus.
 * The relative brake current value is multiplied by 10 before sending.
 * The value must be between 0 and 100.
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
    Send_CAN(SetRelativeBrakeCurrent_ID, (uint8_t*)SetRelativeBrakeCurrent, 2);
}
/**
 * Sets the digital output values and sends them over CAN bus.
 * @param digitaloutput1 The first digital output value to set.
 * @param digitaloutput2 The second digital output value to set.
 * @param digitaloutput3 The third digital output value to set.
 * @param digitaloutput4 The fourth digital output value to set.
 */
void setSetDigitalOutput(bool digitaloutput1, bool digitaloutput2, bool digitaloutput3, bool digitaloutput4) {
    SetDigitalOutput[0] = digitaloutput1;
    SetDigitalOutput[1] = digitaloutput2;
    SetDigitalOutput[2] = digitaloutput3;
    SetDigitalOutput[3] = digitaloutput4;
    Send_CAN(SetDigitalOutput_ID, (uint8_t*)SetDigitalOutput, 4);
}
/**
 * Sets the maximum AC current value and sends it over CAN bus.
 * The maximum AC current value is multiplied by 10 before sending.
 * @param maxcurrent The maximum AC current value to set.
 */
void setSetMaxACCurrent(int16_t maxcurrent) {
    maxcurrent = maxcurrent * 10;
    SetMaxACCurrent[0] = maxcurrent >> 8;
    SetMaxACCurrent[1] = maxcurrent;
    Send_CAN(SetMaxACCurrent_ID, (uint8_t*)SetMaxACCurrent, 2);
}
/**
 * Sets the maximum AC brake current value and sends it over CAN bus.
 * The maximum AC brake current value is multiplied by 10 before sending.
 * Only negative currents are accepted.
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
    Send_CAN(SetMaxACBrakeCurrent_ID, (uint8_t*)SetMaxACBrakeCurrent, 2);
}
/**
 * Sets the maximum DC current value and sends it over CAN bus.
 * The maximum DC current value is multiplied by 10 before sending.
 * @param maxdccurrent The maximum DC current value to set.
 */
void setSetMaxDCCurrent(int16_t maxdccurrent) {
    maxdccurrent = maxdccurrent * 10;
    SetMaxDCCurrent[0] = maxdccurrent >> 8;
    SetMaxDCCurrent[1] = maxdccurrent;
    Send_CAN(SetMaxDCCurrent_ID, (uint8_t*)SetMaxDCCurrent, 2);
}
/**
 * Sets the maximum DC brake current value and sends it over CAN bus.
 * The maximum DC brake current value is multiplied by 10 before sending.
 * Only negative currents are accepted.
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
    Send_CAN(SetMaxDCBrakeCurrent_ID, (uint8_t*)SetMaxDCBrakeCurrent, 2);
}
/**
 * Sets the drive enable value and sends it over CAN bus.
 * @param driveenable The drive enable value to set.
 */
void setDriveEnable(bool driveenable) {
    DriveEnable[0] = driveenable;
    Send_CAN(DriveEnable_ID, (uint8_t*)DriveEnable, 1);
}

void SendID_20(void) {
    static uint8_t id = 0x20;
    message_CAN_TX[0] = APPS_percent;    // APPS_percent 0-100%
    message_CAN_TX[1] = Brake_Pressure;  // Brake_Pressure 0-50
    // Target_Power byte 2,3 and 4
    message_CAN_TX[2] = (Target_Power >> 16) & 0xFF;
    message_CAN_TX[3] = (Target_Power >> 8) & 0xFF;
    message_CAN_TX[4] = Target_Power & 0xFF;
    // Current_Power byte 5,6 and 7
    message_CAN_TX[5] = (Current_Power >> 16) & 0xFF;
    message_CAN_TX[6] = (Current_Power >> 8) & 0xFF;
    message_CAN_TX[7] = Current_Power & 0xFF;

    Send_CAN(id, message_CAN_TX, 8);
}

void SendID_21(void) {
    static uint8_t id = 0x21;
    message_CAN_TX[0] = Inverter_Temperature >> 8;
    message_CAN_TX[1] = Inverter_Temperature;
    message_CAN_TX[2] = Motor_Temperature >> 8;
    message_CAN_TX[3] = Motor_Temperature;
    message_CAN_TX[4] = 0;
    message_CAN_TX[5] = 0;
    message_CAN_TX[6] = 0;
    message_CAN_TX[7] = 0;

    Send_CAN(id, message_CAN_TX, 8);
}

void SendID_22(void) {
    static uint8_t id = 0x22;
    message_CAN_TX[0] = Inverter_Faults >> 8;
    message_CAN_TX[1] = Inverter_Faults;
    message_CAN_TX[2] = LMT1;
    message_CAN_TX[3] = LMT2;
    message_CAN_TX[4] = VcuState;
    message_CAN_TX[5] = 0;
    message_CAN_TX[6] = 0;
    message_CAN_TX[7] = 0;

    Send_CAN(id, message_CAN_TX, 8);
}

void SendID_23(void) {
    static uint8_t id = 0x23;
    message_CAN_TX[0] = Inverter_Voltage >> 8;
    message_CAN_TX[1] = Inverter_Voltage;
    message_CAN_TX[2] = RPM >> 8;
    message_CAN_TX[3] = RPM;
    message_CAN_TX[4] = 0;
    message_CAN_TX[5] = 0;
    message_CAN_TX[6] = 0;
    message_CAN_TX[7] = 0;
    Send_CAN(id, message_CAN_TX, 8);
}

void SendID_420(void) {  // for debugging
    static uint16_t id = 0x420;
    message_CAN_TX[0] = ADC[0] >> 8;
    message_CAN_TX[1] = ADC[0];
    message_CAN_TX[2] = ADC[3] >> 8;
    message_CAN_TX[3] = ADC[3];
    message_CAN_TX[4] = APPS_percent;
    message_CAN_TX[5] = apps_error;
    message_CAN_TX[6] = 0;
    message_CAN_TX[7] = 0;
    Send_CAN(id, message_CAN_TX, 8);
}

void startupSequence() {
    // Start up sequence
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

void PrintToConsole(uint8_t ms) {
    // Print data-----
    currentMillis[3] = millis();
    if (currentMillis[3] - previousMillis[3] >= ms) {
        printf("APPS1:%d,APPS2:%d,APPS_percent:%d,APPS_error:,%d,CAN_status:%d,CanRX_ON:%d,CanTX_ON:%d \r\n", ADC[0], ADC[3], APPS_percent, apps_error, status, CANRX_ON, CANTX_ON);
        previousMillis[3] = currentMillis[3];
    }
}