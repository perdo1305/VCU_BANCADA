

#include <stdbool.h>  // Defines true

#include "definitions.h"
// ############# RX CAN FRAME ###############################
CANFD_MSG_RX_ATTRIBUTE msgAttr = CANFD_MSG_RX_DATA_FRAME;  // RX message attribute

typedef struct {
    uint8_t message[8];
    uint32_t messageID;
    uint8_t messageLength;
} CAN_Buffer;

CAN_Buffer rx = {.message = {0}, .messageID = 0, .messageLength = 0};
CAN_Buffer tx = {.message = {0}, .messageID = 0, .messageLength = 0};

uint32_t status1 = 0;  // CAN status

bool CANRX1_ON = 0;  // flag to check if CAN is receiving
bool CANTX1_ON = 0;  // flag to check if CAN is transmitting

// ############# TX CAN FRAME ###############################
uint8_t message_CAN_TX[8] = {};  // TX message buffer
uint8_t cantx_message[8] = {};   // TX message buffer

void Read_CAN_BUS_1(void);                                         // Read CAN 1 function
void Send_CAN_BUS_1(uint32_t id, uint8_t* message, uint8_t size);  // Send CAN 1 function

void Read_CAN_BUS_1() {
    status1 = CAN1_ErrorGet();
    if (status1 == CANFD_ERROR_NONE) {
        memset(rx.message, 0x00, sizeof(rx.message));
        if (CAN1_MessageReceive(&rx.messageID, &rx.messageLength, rx.message, 0, 2, &msgAttr)) {
            CANRX1_ON = 1;
        }
    } else {
        CANRX1_ON = 0;
    }
}

void Send_CAN_BUS_1(uint32_t id, uint8_t* message, uint8_t size) {
    if (CAN1_TxFIFOQueueIsFull(0)) {
        CANTX1_ON = 0;
    } else {
        if (CAN1_MessageTransmit(id, size, message, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME)) {
            CANTX1_ON = 1;
        } else {
            CANTX1_ON = 0;
        }
    }
}
//TODO ACABAR ISTO
void CANSendDatadb_1() {
}

void CANSendDatadb_2() {
}

void CANSendDatadb_3() {
}

void CANSendDatadb_4() {
}