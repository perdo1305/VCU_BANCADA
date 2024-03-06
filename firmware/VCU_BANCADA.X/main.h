/*
 * File:   main.h
 * Author: pedro
 *
 * Created on 19 de Fevereiro de 2024, 19:02
 */


#ifndef MAIN_H
#define MAIN_H

#include "APPS.h"
#include <stdbool.h>  // Defines true
#include <stddef.h>   // Defines NULL
#include <stdint.h>
#include <stdlib.h>  // Defines EXIT_FAILURE
#include "definitions.h"  // SYS function prototypes
//#include "CAN_utils.h"


// Inverter CAN IDs
#define SetCurrent_ID 0x01                                   // byte 0-1
#define SetBrakeCurrent_ID 0x02                              // byte 0-1
#define SetERPM_ID 0x394                                      // byte 0-3
#define SetPosition_ID 0x04                                  // byte 0-1
#define SetRelativeCurrent_ID 0x05                           // byte 0-1
#define SetRelativeBrakeCurrent_ID 0x06                      // byte 0-1
#define SetDigitalOutput_ID 0x07                             // byte 0-3
#define SetMaxACCurrent_ID 0x08                              // byte 0-1
#define SetMaxACBrakeCurrent_ID 0x09                         // byte 0-1
#define SetMaxDCCurrent_ID 0x0A                              // byte 0-1
#define SetMaxDCBrakeCurrent_ID 0x0B                         // byte 0-1
#define DriveEnable_ID 0x0C                                  // byte 0
#define ERPMDutyCycleInputVoltage_ID 0x0D                    // byte 0-7
#define ACDCcurrentControllerMotorTemperatureFaults_ID 0x0E  // byte 0-7
#define ThrottleBrakeDigitalInput1_2_3_4_ID 0x0F             // byte 0-3


extern uint32_t status1;  // CAN status

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
