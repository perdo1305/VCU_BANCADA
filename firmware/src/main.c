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

#include <stdbool.h>  // Defines true
#include <stddef.h>   // Defines NULL
#include <stdlib.h>   // Defines EXIT_FAILURE

#include "definitions.h"  // SYS function prototypes

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

// Inverter CAN IDs
#define SetCurrent_ID 0x01 //byte 0-1  
#define SetBrakeCurrent_ID 0x02 //byte 0-1
#define SetERPM_ID 0x03 //byte 0-3
#define SetPosition_ID 0x04 //byte 0-1
#define SetRelativeCurrent_ID 0x05 //byte 0-1
#define SetRelativeBrakeCurrent_ID 0x06 //byte 0-1
#define SetDigitalOutput_ID 0x07    //byte 0-3
#define SetMaxACCurrent_ID 0x08 //byte 0-1
#define SetMaxACBrakeCurrent_ID 0x09 //byte 0-1
#define SetMaxDCCurrent_ID 0x0A //byte 0-1
#define SetMaxDCBrakeCurrent_ID 0x0B //byte 0-1
#define DriveEnable_ID 0x0C //byte 0

//mensagens inverter
int8_t SetCurrent[2] = {0, 0};
int8_t SetBrakeCurrent[2] = {0, 0};
int8_t SetERPM[4] = {0, 0, 0, 0};
int8_t SetPosition[2] = {0, 0};
int8_t SetRelativeCurrent[2] = {0, 0};
int8_t SetRelativeBrakeCurrent[2] = {0, 0};
int8_t SetDigitalOutput[4] = {0, 0, 0, 0};
int8_t SetMaxACCurrent[2] = {0, 0};
int8_t SetMaxACBrakeCurrent[2] = {0, 0};
int8_t SetMaxDCCurrent[2] = {0, 0};
int8_t SetMaxDCBrakeCurrent[2] = {0, 0};
int8_t DriveEnable[1] = {0};



// Define a macro DEBUG para ativar ou desativar o debug_printf
#define DEBUG 1
#define LABVIEW 0

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
static uint8_t rx_message[64] = {};

uint32_t messageID = 0;
uint32_t rx_messageID = 0;
uint32_t status = 0;
uint8_t messageLength = 0;
uint8_t rx_messageLength = 0;
uint8_t count = 0;
uint8_t user_input = 0;
CANFD_MSG_RX_ATTRIBUTE msgAttr = CANFD_MSG_RX_DATA_FRAME;

bool CANRX_ON = 0;  // flag to check if CAN is receiving
bool CANTX_ON = 0;  // flag to check if CAN is transmitting

// ############# TX CAN FRAME ###############################
static uint8_t message_CAN_TX[64];
uint8_t cantx_message[8] = {10, 20, 30, 40, 50, 60, 70, 80};

// ############# ADC ########################################

static uint8_t message_ADC[64];  // CAN message to send ADC data
static uint16_t ADC[64];         // ADC raw data
uint16_t APPS_average = 0;       // average of APPS1 and APPS2
uint16_t APPS_percent = 0;       // 0-100% of average of APPS1 and APPS2
bool apps_error = 0;             // error if APPS1 and APPS2 are 10% apart
bool error_flag = 0;             //  used in apps function to check if there is an error

uint8_t Brake_Pressure = 0;      // 0-50
uint32_t Target_Power = 0;       // 0-85000
uint32_t Current_Power = 0;      // 0-85000
uint16_t Inverter_Voltage = 0;   // 0-620
uint16_t Inverter_Temperature = 0; // 0-300
 
bool error_flag = 0;             // used in apps function to check if there is an error
uint8_t cantx_message[8] = {10, 20, 30, 40, 50, 60, 70, 80};

void Read_ADC(ADCHS_CHANNEL_NUM channel); 
void Read_CAN(void);
bool APPS_Function(uint16_t APPS1, uint16_t APPS2);
void Send_CAN(uint32_t id, uint8_t* message, uint8_t size);

unsigned int previousMillis[10] = {};
unsigned int currentMillis[10] = {};

unsigned int millis(void) {
    return (unsigned int)(CORETIMER_CounterGet() / (CORE_TIMER_FREQUENCY / 1000));
}

int main(void) {
    /* Initialize all modules */
    SYS_Initialize(NULL);

    ADCHS_ModulesEnable(ADCHS_MODULE0_MASK); //APPS1
    ADCHS_ModulesEnable(ADCHS_MODULE3_MASK); //APPS2

    printf("██████╗░███████╗░██████╗███████╗████████╗\r\n");
    printf("██╔══██╗██╔════╝██╔════╝██╔════╝╚══██╔══╝\r\n");
    printf("██████╔╝█████╗░░╚█████╗░█████╗░░░░░██║░░░\r\n");
    printf("██╔══██╗██╔══╝░░░╚═══██╗██╔══╝░░░░░██║░░░\r\n");
    printf("██║░░██║███████╗██████╔╝███████╗░░░██║░░░\r\n");
    printf("╚═╝░░╚═╝╚══════╝╚═════╝░╚══════╝░░░╚═╝░░░\r\n");
    printf("\n\n");
    fflush(stdout);

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

    while (true) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();

        Read_CAN();

        // Heartbeat led blink---------------------------------------
        currentMillis[0] = millis();
        if (currentMillis[0] - previousMillis[0] >= 300) {
            GPIO_RC11_Toggle();
            previousMillis[0] = currentMillis[0];
        }
        // Read ADC---------------------------------------
        currentMillis[1] = millis();
        if (currentMillis[1] - previousMillis[1] >= 100) {
            Read_ADC(ADCHS_CH0);
            Read_ADC(ADCHS_CH3);
            apps_error = APPS_Function(ADC[0], ADC[3]);

            // printf("APPS1: %d APPS2: %d APPS_percent: %d APPS_error: %d\r\n", ADC[0], ADC[3], APPS_percent, apps_error);
            previousMillis[1] = currentMillis[1];
        }
        // Send CAN---------------------------------------

        memset(message_CAN_TX, 0x00, sizeof(message_CAN_TX));
        memset(cantx_message, 0x00, sizeof(cantx_message));
        memset(message_ADC, 0x00, sizeof(message_ADC));

        currentMillis[2] = millis();
        if (currentMillis[2] - previousMillis[2] >= 5) {
            message_CAN_TX[0] = APPS_percent; // APPS_percent 0-100%
            message_CAN_TX[1] = Brake_Pressure; // Brake_Pressure 0-50
            //Target_Power byte 2,3 and 4
            message_CAN_TX[2] = (Target_Power >> 16) & 0xFF;
            message_CAN_TX[3] = (Target_Power >> 8) & 0xFF;
            message_CAN_TX[4] = Target_Power & 0xFF;
            //Current_Power byte 5,6 and 7
            message_CAN_TX[5] = (Current_Power >> 16) & 0xFF;
            message_CAN_TX[6] = (Current_Power >> 8) & 0xFF;
            message_CAN_TX[7] = Current_Power & 0xFF;
            Send_CAN(0x20, message_CAN_TX, 8);


            Send_CAN(0x21, cantx_message, 8);
            Send_CAN(0x22, cantx_message, 8);

            Send_CAN(0x420, cantx_message, 8);

            message_ADC[0] = ADC[0] >> 8;  // APPS1
            message_ADC[1] = ADC[0];
            message_ADC[2] = ADC[3] >> 8;  // APPS2
            message_ADC[3] = ADC[3];
            message_ADC[4] = APPS_percent;  // APPS_percent
            message_ADC[5] = apps_error;    // APPS_error

            Send_CAN(0x69, message_ADC, 8);

            previousMillis[2] = currentMillis[2];
        }
        // Print data---------------------------------------
        currentMillis[3] = millis();
        if (currentMillis[3] - previousMillis[3] >= 100) {
            printf("APPS1: %d APPS2: %d APPS_percent: %d APPS_error: %d CAN_status:%d CanRX_ON:%d CanTX_ON:%d \r\n", ADC[0], ADC[3], APPS_percent, apps_error, status, CANRX_ON, CANTX_ON);
            previousMillis[3] = currentMillis[3];
        }
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
    // APPS1 0-4095
    // APPS2 4095-0

    static unsigned long error_start_time;

    // invert APPS2
    // TODO macro bellow
    APPS2 = 4095 - APPS2;

    // Check if APPS1 and APPS2 are too far apart
    int diff = abs(APPS1 - APPS2);
    // TODO make this into variables to make it easier to debug

    if (diff > 409 || (APPS1 < 5 || APPS2 < 5) || (APPS1 > 3995 || APPS2 > 3995)) {
        if (error_flag == 0) {
            // Error detected, start timer
            error_start_time = millis();
            error_flag = 1;
        } else {
            // Error already detected, check if timer has expired
            unsigned long current_time = millis();
            if (current_time - error_start_time > 100) {
                // Error has persisted for more than 100 ms, set error flag
                error_flag = 1;
            }
        }
    } else {
        // No error, reset error flag and timer
        error_flag = 0;
        error_start_time = 0;
    }

    // calculates the average and percentage of the two APPS
    APPS_average = (APPS1 + APPS2) / 2;
    // debug_printf("APPS_average: %d\r\n", APPS_average);
    APPS_percent = (APPS_average * 100) / 4095;

    // debug_printf("APPSA%d APPSB%d APPST%d APPS_ERROR%d\r", APPS1, APPS2, APPS_percent, error_flag);
    //  debug_printf("APPS1:10\n");
    return error_flag;
}


void setSetCurrent(int16_t current) {
    current = current * 10;
    SetCurrent[0] = current >> 8;
    SetCurrent[1] = current;
    Send_CAN(SetCurrent_ID, SetCurrent, 2);
}

void setSetBrakeCurrent(int16_t brakeCurrent) {
    brakeCurrent = brakeCurrent * 10;
    SetBrakeCurrent[0] = brakeCurrent >> 8;
    SetBrakeCurrent[1] = brakeCurrent;
    Send_CAN(SetBrakeCurrent_ID, SetBrakeCurrent, 2);
}

void setSetERPM(int32_t ERPM) {
    SetERPM[0] = ERPM >> 24;
    SetERPM[1] = ERPM >> 16;
    SetERPM[2] = ERPM >> 8;
    SetERPM[3] = ERPM;
    Send_CAN(SetERPM_ID, SetERPM, 4);
}

void setSetPosition(int16_t position) {
    position = position * 10;
    SetPosition[0] = position >> 8;
    SetPosition[1] = position;
    Send_CAN(SetPosition_ID, SetPosition, 2);
}

void setSetRelativeCurrent(int16_t relativecurrent) {
    //This value must be between -100 and 100 and must be multiplied by 10 before sending.
    if (relativecurrent > 100) {
        relativecurrent = 100;
    } else if (relativecurrent < -100) {
        relativecurrent = -100;
    }
    relativecurrent = relativecurrent * 10;
    SetRelativeCurrent[0] = relativecurrent >> 8;
    SetRelativeCurrent[1] = relativecurrent;
    Send_CAN(SetRelativeCurrent_ID, SetRelativeCurrent, 2);
}

void setSetRelativeBrakeCurrent(int16_t relativebrakecurrent) {
    //This value must be between 0 and 100 and must be multiplied by 10 before sending 
    if (relativebrakecurrent > 100) {
        relativebrakecurrent = 100;
    } else if (relativebrakecurrent < 0) {
        relativebrakecurrent = 0;
    }
    relativebrakecurrent = relativebrakecurrent * 10;
    SetRelativeBrakeCurrent[0] = relativebrakecurrent >> 8;
    SetRelativeBrakeCurrent[1] = relativebrakecurrent;
    Send_CAN(SetRelativeBrakeCurrent_ID, SetRelativeBrakeCurrent, 2);
}

void setSetDigitalOutput(bool digitaloutput1, bool digitaloutput2, bool digitaloutput3, bool digitaloutput4) {
    SetDigitalOutput[0] = digitaloutput1;
    SetDigitalOutput[1] = digitaloutput2;
    SetDigitalOutput[2] = digitaloutput3;
    SetDigitalOutput[3] = digitaloutput4;
    Send_CAN(SetDigitalOutput_ID, SetDigitalOutput, 4);
}

void setSetMaxACCurrent(int16_t maxcurrent) {
    maxcurrent = maxcurrent * 10;
    SetMaxACCurrent[0] = maxcurrent >> 8;
    SetMaxACCurrent[1] = maxcurrent;
    Send_CAN(SetMaxACCurrent_ID, SetMaxACCurrent, 2);
}

void setSetMaxACBrakeCurrent(int16_t maxbrakecurrent) {
    //This value must be multiplied by 10 before sending, only negative currents are accepted.
    if (maxbrakecurrent > 0) {
        maxbrakecurrent = 0;
    }
    maxbrakecurrent = maxbrakecurrent * 10;
    SetMaxACBrakeCurrent[0] = maxbrakecurrent >> 8;
    SetMaxACBrakeCurrent[1] = maxbrakecurrent;
    Send_CAN(SetMaxACBrakeCurrent_ID, SetMaxACBrakeCurrent, 2);
}

void setSetMaxDCCurrent(int16_t maxdccurrent) {
    maxdccurrent = maxdccurrent * 10;
    SetMaxDCCurrent[0] = maxdccurrent >> 8;
    SetMaxDCCurrent[1] = maxdccurrent;
    Send_CAN(SetMaxDCCurrent_ID, SetMaxDCCurrent, 2);
}

void setSetMaxDCBrakeCurrent(int16_t maxdcbrakecurrent) {
    //The value has to be multiplied by 10 before sending. Only negative currents are accepted.
    if (maxdcbrakecurrent > 0) {
        maxdcbrakecurrent = 0;
    }
    maxdcbrakecurrent = maxdcbrakecurrent * 10;
    SetMaxDCBrakeCurrent[0] = maxdcbrakecurrent >> 8;
    SetMaxDCBrakeCurrent[1] = maxdcbrakecurrent;
    Send_CAN(SetMaxDCBrakeCurrent_ID, SetMaxDCBrakeCurrent, 2);
}

void setDriveEnable(bool driveenable) {
    DriveEnable[0] = driveenable;
    Send_CAN(DriveEnable_ID, DriveEnable, 1);
}