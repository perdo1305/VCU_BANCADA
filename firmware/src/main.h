/*
 * File:   main.h
 * Author: pedro
 *
 * Created on 19 de Fevereiro de 2024, 19:02
 */
#include <stdbool.h>  // Defines true
#include <stddef.h>   // Defines NULL
#include <stdlib.h>   // Defines EXIT_FAILURE

#include "definitions.h"  // SYS function prototypes

#ifndef MAIN_H
#define MAIN_H

// Inverter CAN IDs
#define SetCurrent_ID 0x01               // byte 0-1
#define SetBrakeCurrent_ID 0x02          // byte 0-1
#define SetERPM_ID 0x03                  // byte 0-3
#define SetPosition_ID 0x04              // byte 0-1
#define SetRelativeCurrent_ID 0x05       // byte 0-1
#define SetRelativeBrakeCurrent_ID 0x06  // byte 0-1
#define SetDigitalOutput_ID 0x07         // byte 0-3
#define SetMaxACCurrent_ID 0x08          // byte 0-1
#define SetMaxACBrakeCurrent_ID 0x09     // byte 0-1
#define SetMaxDCCurrent_ID 0x0A          // byte 0-1
#define SetMaxDCBrakeCurrent_ID 0x0B     // byte 0-1
#define DriveEnable_ID 0x0C              // byte 0

// Inverter CAN messages
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

// ############# CAN VARS ###################################
/* ID 0x20 */
// uint8_t Throttle = 0;     // 0-100   |Byte 0
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

void setSetCurrent(int16_t current);
void setSetBrakeCurrent(int16_t brakeCurrent);
void setSetERPM(int32_t ERPM);
void setSetPosition(int16_t position);
void setSetRelativeCurrent(int16_t relativecurrent);
void setSetRelativeBrakeCurrent(int16_t relativebrakecurrent);
void setSetDigitalOutput(bool digitaloutput1, bool digitaloutput2, bool digitaloutput3, bool digitaloutput4);
void setSetMaxACCurrent(int16_t maxcurrent);
void setSetMaxACBrakeCurrent(int16_t maxbrakecurrent);
void setSetMaxDCCurrent(int16_t maxdccurrent);
void setSetMaxDCBrakeCurrent(int16_t maxdcbrakecurrent);
void setDriveEnable(bool driveenable);

void SendID_20(void);  // 200 hz
void SendID_21(void);  // 10 hz
void SendID_22(void);  // 10 hz
void SendID_23(void);  // 200 hz

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
