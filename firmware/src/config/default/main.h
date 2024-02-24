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
#define SetCurrent_ID 0x354               // byte 0-1   ID 354
#define SetBrakeCurrent_ID 0x374          // byte 0-1   ID 374
#define SetERPM_ID 0x394                  // byte 0-3   ID 394
#define SetPosition_ID 0x3b4              // byte 0-1   ID 3b4
#define SetRelativeCurrent_ID 0x3d4       // byte 0-1   ID 3d4
#define SetRelativeBrakeCurrent_ID 0x3f4  // byte 0-1   ID 3f4
// #define SetDigitalOutput_ID 0x07                  // byte 0-3   ID
#define SetMaxACCurrent_ID 0x414                  // byte 0-1   ID 414
#define SetMaxACBrakeCurrent_ID 0x434             // byte 0-1   ID 434
#define SetMaxDCCurrent_ID 0x454                  // byte 0-1   ID 454
#define SetMaxDCBrakeCurrent_ID 0x474             // byte 0-1 ID 474
#define DriveEnable_ID 0x494                      // byte 0 ID 494
#define ERPMDutyCycleInputVoltage_ID 0x14         // byte 0-7 ID 14
#define ACDCcurrent 0x34                          // byte 0-7  ID 34
#define ControllerMotorTemperatureFaults_ID 0x54  // byte 0-7 ID 54
#define ThrottleBrakeDigitalInput1_2_3_4_ID 0x94  // byte 0-3 ID 94

// CAN Messages to send to Inverter
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
int32_t ERPM = 0;                  // Electrical RPM Equation: ERPM = Motor RPM * number of the motor pole pairs
int16_t DutyCycle = 0;             // The controller duty cycle. The sign of this value will represent whether the motor is running(positive) current or regenerating (negative) current
int16_t InputVoltage = 0;          // Input voltage is the DC voltage
int16_t ACcurrent = 0;             // The motor current. The sign of this value represents whether the motor is running(positive) current or regenerating (negative) current
int16_t DCcurrent = 0;             // DC Current: Current on DC side. The sign of this value represents whether the motor is running(positive) current or regenerating (negative) current.
int8_t ControllerTemperature = 0;  // Temperature of the inverter semiconductors
int16_t MotorTemperature = 0;      // Temperature of the motor measured by the inverter
int8_t Faults = 0;                 // 0x00 : NO FAULTS
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
int8_t ThrottleSignal = 0;         // Throttle signal derived from analog inputs or CAN2
int8_t BrakeSignal = 0;            // Brake signal derived from analog inputs or CAN2
bool DigitalInput1 = 0;            // 0-1
bool DigitalInput2 = 0;            // 0-1
bool DigitalInput3 = 0;            // 0-1
bool DigitalInput4 = 0;            // 0-1
bool DigitalOutput1 = 0;           // 0-1
bool DigitalOutput2 = 0;           // 0-1
bool DigitalOutput3 = 0;           // 0-1
bool DigitalOutput4 = 0;           // 0-1
bool DriveEnabled = 0;             // 0-1
bool CapacitorTempLimit = 0;       // 0-1
bool DCcurrentLimit = 0;           // 0-1
bool DriveEnableLimit = 0;         // 0-1
bool IGBTAccelLimit = 0;           // 0-1
bool IGBTTempLimit = 0;            // 0-1
bool InputVoltageLimit = 0;        // 0-1
bool MotorAccelLimit = 0;          // 0-1
bool MotorTempLimit = 0;           // 0-1
bool RPMMinLimit = 0;              // 0-1
bool RPMMaxLimit = 0;              // 0-1
bool PowerLimit = 0;               // 0-1

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

//millis
unsigned long millis(void);

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
