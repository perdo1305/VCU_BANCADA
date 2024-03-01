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

#include "../VCU_BANCADA.X/utils.h"
#include "definitions.h"  // SYS function prototypes
// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

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

// ############# RX CAN FRAME ###############################

CANFD_MSG_RX_ATTRIBUTE msgAttr = CANFD_MSG_RX_DATA_FRAME;  // RX message attribute

static uint8_t rx_message[64] = {};  // CAN message receive buffer
uint32_t rx_messageID = 0;           // RX message ID
uint8_t rx_messageLength = 0;        // RX message length

uint32_t status1 = 0;  // CAN status
uint32_t status2 = 0;  // CAN status
uint32_t status3 = 0;  // CAN status

bool CANRX1_ON = 0;  // flag to check if CAN is receiving
bool CANTX1_ON = 0;  // flag to check if CAN is transmitting

bool CANRX2_ON = 0;  // flag to check if CAN is receiving
bool CANTX2_ON = 0;  // flag to check if CAN is transmitting

bool CANRX3_ON = 0;  // flag to check if CAN is receiving
bool CANTX3_ON = 0;  // flag to check if CAN is transmitting

// ############# TX CAN FRAME ###############################
uint8_t message_CAN_TX[8] = {};  // TX message buffer
uint8_t cantx_message[8] = {};   // TX message buffer

// ############# ADC ########################################

uint8_t message_ADC[64];      // CAN message to send ADC data
__COHERENT uint16_t ADC[64];  // ADC raw data

// ############# MILIS #######################################
unsigned int previousMillis[10] = {};
unsigned int currentMillis[10] = {};
// ############# MILIS #######################################

unsigned int millis() {
    return (unsigned int)(CORETIMER_CounterGet() / (CORE_TIMER_FREQUENCY / 1000));
}

// ############# FUNCTIONS ##################################
void Read_ADC(ADCHS_CHANNEL_NUM channel);            // Read ADC function
bool APPS_Function(uint16_t APPS1, uint16_t APPS2);  // APPS function to calculate average and percentage

void Read_CAN_1(void);                                         // Read CAN 1 function
void Send_CAN_1(uint32_t id, uint8_t* message, uint8_t size);  // Send CAN 1 function
void Read_CAN_2(void);                                         // Read CAN 2 function
void Send_CAN_2(uint32_t id, uint8_t* message, uint8_t size);  // Send CAN 2 function
void Read_CAN_3(void);                                         // Read CAN 3 function
void Send_CAN_3(uint32_t id, uint8_t* message, uint8_t size);  // Send CAN 3 function

void startupSequence(void);    // Startup sequence
void PrintToConsole(uint8_t);  // Print data to console

// ############# TMR FUNCTIONS ###############################
void TMR1_5ms(uint32_t status, uintptr_t context) {  // 200Hz
    // memset(message_CAN_TX, 0, sizeof(message_CAN_TX));
    // SendID_20();  // send data to ID 0x20 in DataBus
    // memset(message_CAN_TX, 0, sizeof(message_CAN_TX));
    // SendID_23();  // send data to ID 0x23 in DataBus
}

void TMR2_100ms(uint32_t status, uintptr_t context) {  // 10Hz
    // memset(message_CAN_TX, 0x00, sizeof(message_CAN_TX));
    // SendID_21();  // send data to ID 0x21 in DataBus
    // memset(message_CAN_TX, 0x00, sizeof(message_CAN_TX));
    // SendID_22();  // send data to ID 0x22 in DataBus
}

void TMR4_500ms(uint32_t status, uintptr_t context) {  // 2Hz
    GPIO_RC11_Toggle();                                // Heartbeat led
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
    // GPIO_RC2_Set();
    // GPIO_RC11_Set();

    ADCHS_CallbackRegister(ADCHS_CH0, ADCHS_CH0_Callback, (uintptr_t)NULL);  // APPS1 callback
    ADCHS_CallbackRegister(ADCHS_CH3, ADCHS_CH3_Callback, (uintptr_t)NULL);  // APPS2 callback

    TMR1_CallbackRegister(TMR1_5ms, (uintptr_t)NULL);    // 200Hz
    TMR2_CallbackRegister(TMR2_100ms, (uintptr_t)NULL);  // 10Hz
    TMR4_CallbackRegister(TMR4_500ms, (uintptr_t)NULL);  // 2Hz heartbeat led
    TMR5_CallbackRegister(TMR5_100ms, (uintptr_t)NULL);  // 10Hz
    TMR6_CallbackRegister(TMR6_5ms, (uintptr_t)NULL);    // 200Hz to send data to the inverter
                                                         /*
                                                             printf("�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??\r\n");
                                                             printf("�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??\r\n");
                                                             printf("�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??\r\n");
                                                             printf("�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??\r\n");
                                                             printf("�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??\r\n");
                                                             printf("�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??�??\r\n");
                                                             printf("\n\n");
                                                            fflush(stdout);
                                                        */

    // config inverter
    // setSetCurrent(20);
    // setSetBrakeCurrent(30);
    // setSetERPM(40);
    // setSetPosition(50);
    // setSetRelativeCurrent(21);
    // setSetRelativeBrakeCurrent(1);
    // setSetDigitalOutput(0, 1, 2, 3);
    // setSetMaxACCurrent(5);
    // setSetMaxACBrakeCurrent(4);
    // setSetMaxDCCurrent(4);
    // setSetMaxDCBrakeCurrent(5);
    // setDriveEnable(1);  // Enable the inverter

    // TMR1_Start();
    CORETIMER_DelayMs(5);
    // TMR2_Start();
    CORETIMER_DelayMs(5);
    TMR3_Start();  // Used trigger source for ADC conversion
    CORETIMER_DelayMs(5);
    TMR4_Start();
    CORETIMER_DelayMs(5);
    TMR5_Start();
    CORETIMER_DelayMs(5);
    TMR6_Start();
    CORETIMER_DelayMs(5);

    // setSetDriveEnable(0);

    // setSetBrakeCurrent(0);
    // setSetPosition(0);
    // setSetRelativeCurrent(0);
    // setSetRelativeBrakeCurrent(0);
    // setSetMaxACCurrent(0);
    // setSetMaxACBrakeCurrent(0);
    // setSetMaxDCCurrent(0);
    // setSetMaxDCBrakeCurrent(0);
    // setSetDigitalOutput(1, 0, 0, 0);

    APPS_Init(0.3, 3.0, 0.2);  // Initialize APPS

    startupSequence();  // led sequence

    while (true) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();

        Read_CAN_1();  // Read DataBus
        //  Read_CAN_2();  // Read PowerTrainBus
        //  Read_CAN_3();  // Read AutonomousBus

        if (UART1_ReceiverIsReady()) {
            uint8_t data = UART1_ReadByte();
            switch (data) {
                case 48:

                    if (UART1_ReceiverIsReady()) {
                        uint8_t buffer[8];
                        int buffer_size = 8;
                        UART1_Read(buffer, buffer_size);

                        float apps_tol= ((buffer[0] << 8) | buffer[1]) / 10;
                        float apps_max= ((buffer[2] << 8) | buffer[3]) / 10;
                        float apps_min= ((buffer[4] << 8) | buffer[5]) / 10;

                        APPS_Init(apps_min,apps_max ,apps_tol);
                        data = 0;
                    }
                    break;

                default:
                    break;
            }
        }

        PrintToConsole(200);  // Print data to console time in ms
    }
    /* Execution should not come here during normal operation */
    return (EXIT_FAILURE);
}

void Read_CAN_1() {
    status1 = CAN1_ErrorGet();
    // printf("Status: %d\r\n", status);

    if (status1 == CANFD_ERROR_NONE) {
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

            CANRX1_ON = 1;
            GPIO_RC2_Toggle();
        }
    } else {
        GPIO_RC2_Clear();
        CANRX1_ON = 0;
    }
}

void Send_CAN_1(uint32_t id, uint8_t* message, uint8_t size) {
    if (CAN1_TxFIFOQueueIsFull(0)) {
        // debug_printf("CAN1_TxFIFOQueueIsFull\r\n");
        CANTX1_ON = 0;
    } else {
        if (CAN1_MessageTransmit(id, size, message, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME)) {
            // debug_printf("id 0x%x sent successfully \r\n", id);
            //   debug_printf("ID: 0x%x Len: 0x%x DATA:", (unsigned int)id, (unsigned int)size);
            // for (int i = 0; i < size; i++) {s
            //  debug_printf("%d ", message[i]);
            //}
            // debug_printf("\r\n");
            //  GPIO_RC2_Toggle();
            CANTX1_ON = 1;
        } else {
            // debug_printf("id 0x%x not sent\r\n", id);
            CANTX1_ON = 0;
        }
    }
}
/*
void Read_CAN_2() {
    status2 = CAN2_ErrorGet();
    if (status2 == CANFD_ERROR_NONE) {
        memset(rx_message, 0x00, sizeof(rx_message));
        if (CAN2_MessageReceive(&rx_messageID, &rx_messageLength, rx_message, 0, 2, &msgAttr)) {
            CANRX2_ON = 1;

            //TODO : Add the rest of the cases
            switch (rx_messageID) {
                case ERPMDutyCycleInputVoltage_ID:
                    ERPM = (rx_message[0] << 8) | rx_message[1];
                    DutyCycle = (rx_message[2] << 8) | rx_message[3];
                    InputVoltage = (rx_message[4] << 8) | rx_message[5];
                    break;
                case ACDCcurrentControllerMotorTemperatureFaults_ID:
                    ACcurrent = (rx_message[0] << 8) | rx_message[1];
                    DCcurrent = (rx_message[2] << 8) | rx_message[3];
                    ControllerTemperature = (rx_message[4] << 8) | rx_message[5];
                    MotorTemperature = (rx_message[6] << 8) | rx_message[7];
                    break;
                case ThrottleBrakeDigitalInput1_2_3_4_ID:
                    ThrottleSignal = rx_message[0];
                    BrakeSignal = rx_message[1];
                    DigitalInput1 = rx_message[2];
                    DigitalInput2 = rx_message[3];
                    DigitalInput3 = rx_message[4];
                    DigitalInput4 = rx_message[5];
                    break;
                default:
                    break;
            }
        }
    } else {
        CANRX2_ON = 0;
    }
}

void Send_CAN_2(uint32_t id, uint8_t* message, uint8_t size) {
    if (CAN2_TxFIFOQueueIsFull(0)) {
        CANTX2_ON = 0;
    } else {
        if (CAN2_MessageTransmit(id, size, message, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME)) {
            CANTX2_ON = 1;
        } else {
            CANTX2_ON = 0;
        }
    }
}

void Read_CAN_3() {
    status3 = CAN3_ErrorGet();
    if (status3 == CANFD_ERROR_NONE) {
        memset(rx_message, 0x00, sizeof(rx_message));
        if (CAN3_MessageReceive(&rx_messageID, &rx_messageLength, rx_message, 0, 2, &msgAttr)) {
            CANRX3_ON = 1;
        }
    } else {
        CANRX3_ON = 0;
    }
}

void Send_CAN_3(uint32_t id, uint8_t* message, uint8_t size) {
    if (CAN3_TxFIFOQueueIsFull(0)) {
        CANTX3_ON = 0;
    } else {
        if (CAN3_MessageTransmit(id, size, message, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME)) {
            CANTX3_ON = 1;
        } else {
            CANTX3_ON = 0;
        }
    }
}
*/

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
    Send_CAN_1(SetCurrent_ID, temp, 2);
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
    Send_CAN_1(SetBrakeCurrent_ID, temp, 2);
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
    Send_CAN_1(SetERPM_ID, temp, 4);
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
    Send_CAN_1(SetPosition_ID, SetPosition, 2);
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
    Send_CAN_1(SetRelativeCurrent_ID, SetRelativeCurrent, 2);
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
    Send_CAN_1(SetRelativeBrakeCurrent_ID, SetRelativeBrakeCurrent, 2);
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
    Send_CAN_1(SetMaxACCurrent_ID, SetMaxACCurrent, 2);
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
    Send_CAN_1(SetMaxACBrakeCurrent_ID, SetMaxACBrakeCurrent, 2);
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
    Send_CAN_1(SetMaxDCCurrent_ID, SetMaxDCCurrent, 2);
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
    Send_CAN_1(SetMaxDCBrakeCurrent_ID, SetMaxDCBrakeCurrent, 2);
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
    Send_CAN_1(DriveEnable_ID, DriveEnable, 1);
}
void SendID_20(void) {
    static uint8_t id = 0x20;
    message_CAN_TX[0] = APPS_Percentage;  // APPS_percent 0-100%
    message_CAN_TX[1] = Brake_Pressure;   // Brake_Pressure 0-50
    // Target_Power byte 2,3 and 4
    message_CAN_TX[2] = (Target_Power >> 16) & 0xFF;
    message_CAN_TX[3] = (Target_Power >> 8) & 0xFF;
    message_CAN_TX[4] = Target_Power & 0xFF;
    // Current_Power byte 5,6 and 7
    message_CAN_TX[5] = (Current_Power >> 16) & 0xFF;
    message_CAN_TX[6] = (Current_Power >> 8) & 0xFF;
    message_CAN_TX[7] = Current_Power & 0xFF;

    Send_CAN_1(id, message_CAN_TX, 8);
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

    Send_CAN_1(id, message_CAN_TX, 8);
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

    Send_CAN_1(id, message_CAN_TX, 8);
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
    Send_CAN_1(id, message_CAN_TX, 8);
}

void SendID_420(void) {  // for debugging
    static uint16_t id = 0x420;
    message_CAN_TX[0] = ADC[0] >> 8;
    message_CAN_TX[1] = ADC[0];
    message_CAN_TX[2] = ADC[3] >> 8;
    message_CAN_TX[3] = ADC[3];
    message_CAN_TX[4] = APPS_Percentage;
    message_CAN_TX[5] = APPS_Error;
    message_CAN_TX[6] = 0;
    message_CAN_TX[7] = 0;
    Send_CAN_1(id, message_CAN_TX, 8);
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

void PrintToConsole(uint8_t time) {
    // Print data-----
    currentMillis[3] = millis();
    if (currentMillis[3] - previousMillis[3] >= time) {
        // APPS_PrintValues();
        printf("APPSA%dAPPSB%dAPPST%dAPPS_ERROR%dAPPS_Perc%dCAN_ERROR%d", ADC[0], ADC[3], APPS_Mean, APPS_Error, APPS_Percentage, status1);
        printf("APPS_MIN%dAPPS_MAX%dAPPS_TOL%d", APPS_MIN_bits, APPS_MAX_bits, APPS_Tolerance_bits);
        printf("\r\n");
        previousMillis[3] = currentMillis[3];
    }
}
