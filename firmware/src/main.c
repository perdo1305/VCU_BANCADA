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
    UART1_Initialize();
    CAN1_Initialize();

    ADCHS_ModulesEnable(ADCHS_MODULE0_MASK);
    ADCHS_ModulesEnable(ADCHS_MODULE3_MASK);

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