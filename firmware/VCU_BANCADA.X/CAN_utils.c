

#include "CAN_utils.h"

uint32_t status1 = 0;
// ############# RX CAN FRAME ###############################
CANFD_MSG_RX_ATTRIBUTE msgAttr = CANFD_MSG_RX_DATA_FRAME;  // RX message attribute

typedef struct {
    uint8_t message[8];
    uint32_t messageID;
    uint8_t messageLength;
} CAN_Buffer;

CAN_Buffer rx = {.message =
                     {0},
                 .messageID = 0,
                 .messageLength = 0};
CAN_Buffer tx = {.message =
                     {0},
                 .messageID = 0,
                 .messageLength = 0};

bool CANRX1_ON = 0;  // flag to check if CAN is receiving
bool CANTX1_ON = 0;  // flag to check if CAN is transmitting

// ############# TX CAN FRAME ###############################
uint8_t message_CAN_TX[8] = {};  // TX message buffer
uint8_t cantx_message[8] = {};   // TX message buffer

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

// ############# CAN VARS ###################################
/* ID 0x20 */
uint8_t Throttle = 0;        // 0-100   |Byte 0
uint8_t Brake_Pressure = 0;  // 0-50    |Byte 1
uint32_t Target_Power = 0;   // 0-85000 |Byte 2-4
uint32_t Current_Power = 0;  // 0-85000 |Byte 5-7
/* ID 0x21 */
uint16_t Inverter_Temperature = 0;  // 0-300  |Byte 0-1
uint16_t Motor_Temperature = 0;     // 0 -100 |Byte 2-3
/* ID 0x22 */
uint16_t Inverter_Faults = 0;  // 0-65535  |Byte 0-1
uint8_t LMT1 = 0;              // 0-255    |Byte 2
uint8_t LMT2 = 0;              // 0-255    |Byte 3
uint8_t VcuState = 0;          // 0-255    |Byte 4
/* ID 0x23 */
uint16_t Inverter_Voltage = 0;  // 0-65535 |Byte 0-1
uint16_t RPM = 0;               // 0-65535 |Byte 2-3

// TODO ACABAR ISTO

void CANSendDatadb_1(uint8_t consumed_power, uint8_t target_power, uint8_t brake_pressure, uint8_t throttle_position) {
    MAP_ENCODE_CONSUMED_POWER(tx.message, consumed_power);
    MAP_ENCODE_TARGET_POWER(tx.message, target_power);
    MAP_ENCODE_BRAKE_PRESSURE(tx.message, brake_pressure);
    MAP_ENCODE_THROTTLE_POSITION(tx.message, throttle_position);

    Send_CAN_BUS_1(CAN_VCU_ID_1, tx.message, 8);
}

void CANSendDatadb_2() {
}

void CANSendDatadb_3() {
}

void CANSendDatadb_4() {
}