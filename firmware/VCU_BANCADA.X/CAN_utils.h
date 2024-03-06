/* 
 * File:   CAN_utils.h
 * Author: pedro
 *
 * Created on 5 de Mar�o de 2024, 21:55
 */

#ifndef CAN_UTILS_H
#define	CAN_UTILS_H

#include <stdint.h> 
#include <stdbool.h>  // Defines true
#include <stddef.h>   // Defines NULL
#include <stdlib.h>   // Defines EXIT_FAILURE

#include "Can-Header-Map/CAN_asdb.h"
#include "Can-Header-Map/CAN_datadb.h"
#include "Can-Header-Map/CAN_pwtdb.h"
#include "definitions.h"



void Read_CAN_BUS_1(void); // Read CAN 1 function
void Send_CAN_BUS_1(uint32_t id, uint8_t* message, uint8_t size); // Send CAN 1 function

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* CAN_UTILS_H */

