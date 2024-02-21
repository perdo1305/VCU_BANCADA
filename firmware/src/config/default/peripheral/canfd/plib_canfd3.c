/*******************************************************************************
  CANFD Peripheral Library Interface Source File

  Company:
    Microchip Technology Inc.

  File Name:
    plib_canfd3.c

  Summary:
    CANFD peripheral library interface.

  Description:
    This file defines the interface to the CANFD peripheral library. This
    library provides access to and control of the associated peripheral
    instance.

  Remarks:
    None.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
//DOM-IGNORE-END
// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include <sys/kmem.h>
#include "plib_canfd3.h"
#include "interrupts.h"


// *****************************************************************************
// *****************************************************************************
// Global Data
// *****************************************************************************
// *****************************************************************************
/* CAN3 Message memory size */
#define CANFD_MESSAGE_RAM_CONFIG_SIZE 2448
/* Number of configured FIFO */
#define CANFD_NUM_OF_FIFO             2U
/* Maximum number of CAN Message buffers in each FIFO */
#define CANFD_FIFO_MESSAGE_BUFFER_MAX 32

#define CANFD_CONFIGURATION_MODE      0x4UL
#define CANFD_OPERATION_MODE          (0x0UL)
#define CANFD_NUM_OF_FILTER           1U
/* FIFO Offset in word (4 bytes) */
#define CANFD_FIFO_OFFSET             0xcU
/* Filter Offset in word (4 bytes) */
#define CANFD_FILTER_OFFSET           0x4U
#define CANFD_FILTER_OBJ_OFFSET       0x8U
/* Acceptance Mask Offset in word (4 bytes) */
#define CANFD_ACCEPTANCE_MASK_OFFSET  0x8U
#define CANFD_MSG_SID_MASK            (0x7FFU)
#define CANFD_MSG_EID_MASK            (0x1FFFFFFFU)
#define CANFD_MSG_DLC_MASK            (0x0000000FU)
#define CANFD_MSG_IDE_MASK            (0x00000010U)
#define CANFD_MSG_RTR_MASK            (0x00000020U)
#define CANFD_MSG_BRS_MASK            (0x00000040U)
#define CANFD_MSG_FDF_MASK            (0x00000080U)
#define CANFD_MSG_SEQ_MASK            (0xFFFFFE00U)
#define CANFD_MSG_TX_EXT_SID_MASK     (0x1FFC0000U)
#define CANFD_MSG_TX_EXT_EID_MASK     (0x0003FFFFU)
#define CANFD_MSG_RX_EXT_SID_MASK     (0x000007FFUL)
#define CANFD_MSG_RX_EXT_EID_MASK     (0x1FFFF800U)
#define CANFD_MSG_FLT_EXT_SID_MASK    (0x1FFC0000UL)
#define CANFD_MSG_FLT_EXT_EID_MASK    (0x0003FFFFU)

static uint8_t __attribute__((coherent, aligned(16))) can_message_buffer[CANFD_MESSAGE_RAM_CONFIG_SIZE];
static const uint8_t dlcToLength[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

/******************************************************************************
Local Functions
******************************************************************************/
static void CANLengthToDlcGet(uint8_t length, uint8_t *dlc)
{
    if (length <= 8U)
    {
        *dlc = length;
    }
    else if (length <= 12U)
    {
        *dlc = 0x9U;
    }
    else if (length <= 16U)
    {
        *dlc = 0xAU;
    }
    else if (length <= 20U)
    {
        *dlc = 0xBU;
    }
    else if (length <= 24U)
    {
        *dlc = 0xCU;
    }
    else if (length <= 32U)
    {
        *dlc = 0xDU;
    }
    else if (length <= 48U)
    {
        *dlc = 0xEU;
    }
    else
    {
        *dlc = 0xFU;
    }
}

static inline void CAN3_ZeroInitialize(volatile void* pData, size_t dataSize)
{
    volatile uint8_t* data = (volatile uint8_t*)pData;
    for (uint32_t index = 0; index < dataSize; index++)
    {
        data[index] = 0U;
    }
}

/* MISRAC 2012 deviation block start */
/* MISRA C-2012 Rule 11.6 deviated 5 times. Deviation record ID -  H3_MISRAC_2012_R_11_6_DR_1 */
/* MISRA C-2012 Rule 5.1 deviated 1 time. Deviation record ID -  H3_MISRAC_2012_R_5_1_DR_1 */

// *****************************************************************************
// *****************************************************************************
// CAN3 PLib Interface Routines
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Function:
    void CAN3_Initialize(void)

   Summary:
    Initializes given instance of the CAN peripheral.

   Precondition:
    None.

   Parameters:
    None.

   Returns:
    None
*/
void CAN3_Initialize(void)
{
    /* Switch the CAN module ON */
    CFD3CON |= _CFD3CON_ON_MASK;

    /* Switch the CAN module to Configuration mode. Wait until the switch is complete */
    CFD3CON = (CFD3CON & ~_CFD3CON_REQOP_MASK) | ((CANFD_CONFIGURATION_MODE << _CFD3CON_REQOP_POSITION) & _CFD3CON_REQOP_MASK);
    while(((CFD3CON & _CFD3CON_OPMOD_MASK) >> _CFD3CON_OPMOD_POSITION) != CANFD_CONFIGURATION_MODE)
    {
        /* Do Nothing */
    }

    /* Set the Data bitrate to 1000 Kbps */
    CFD3DBTCFG = ((2UL << _CFD3DBTCFG_BRP_POSITION) & _CFD3DBTCFG_BRP_MASK)
               | ((31UL << _CFD3DBTCFG_TSEG1_POSITION) & _CFD3DBTCFG_TSEG1_MASK)
               | ((6UL << _CFD3DBTCFG_TSEG2_POSITION) & _CFD3DBTCFG_TSEG2_MASK)
               | ((6UL << _CFD3DBTCFG_SJW_POSITION) & _CFD3DBTCFG_SJW_MASK);

    /* Set the Nominal bitrate to 1000 Kbps */
    CFD3NBTCFG = ((0UL << _CFD3NBTCFG_BRP_POSITION) & _CFD3NBTCFG_BRP_MASK)
               | ((117UL << _CFD3NBTCFG_TSEG1_POSITION) & _CFD3NBTCFG_TSEG1_MASK)
               | ((0UL << _CFD3NBTCFG_TSEG2_POSITION) & _CFD3NBTCFG_TSEG2_MASK)
               | ((0UL << _CFD3NBTCFG_SJW_POSITION) & _CFD3NBTCFG_SJW_MASK);

    /* Set Message memory base address for all FIFOs/Queue */
    CFD3FIFOBA = (uint32_t)KVA_TO_PA(can_message_buffer);

    CFD3CON &= ~_CFD3CON_STEF_MASK;

    /* Tx Queue Configuration */
    CFD3TXQCON = (((32UL - 1UL) << _CFD3TXQCON_FSIZE_POSITION) & _CFD3TXQCON_FSIZE_MASK)
               | ((0x7UL << _CFD3TXQCON_PLSIZE_POSITION) & _CFD3TXQCON_PLSIZE_MASK)
               | ((0x0UL << _CFD3TXQCON_TXPRI_POSITION) & _CFD3TXQCON_TXPRI_MASK);
    CFD3CON |= _CFD3CON_TXQEN_MASK;


    /* Configure CAN FIFOs */
    CFD3FIFOCON1 = (((1UL - 1UL) << _CFD3FIFOCON1_FSIZE_POSITION) & _CFD3FIFOCON1_FSIZE_MASK) | _CFD3FIFOCON1_TXEN_MASK | ((0x0UL << _CFD3FIFOCON1_TXPRI_POSITION) & _CFD3FIFOCON1_TXPRI_MASK) | ((0x0UL << _CFD3FIFOCON1_RTREN_POSITION) & _CFD3FIFOCON1_RTREN_MASK) | ((0x7UL << _CFD3FIFOCON1_PLSIZE_POSITION) & _CFD3FIFOCON1_PLSIZE_MASK);
    CFD3FIFOCON2 = (((1UL - 1UL) << _CFD3FIFOCON2_FSIZE_POSITION) & _CFD3FIFOCON2_FSIZE_MASK) | ((0x7UL << _CFD3FIFOCON2_PLSIZE_POSITION) & _CFD3FIFOCON2_PLSIZE_MASK);

    /* Configure CAN Filters */
    /* Filter 0 configuration */
    CFD3FLTOBJ0 = (0U & CANFD_MSG_SID_MASK);
    CFD3MASK0 = (0U & CANFD_MSG_SID_MASK);
    CFD3FLTCON0 |= (((0x2UL << _CFD3FLTCON0_F0BP_POSITION) & _CFD3FLTCON0_F0BP_MASK)| _CFD3FLTCON0_FLTEN0_MASK);

    /* Switch the CAN module to CANFD_OPERATION_MODE. Wait until the switch is complete */
    CFD3CON = (CFD3CON & ~_CFD3CON_REQOP_MASK) | ((CANFD_OPERATION_MODE << _CFD3CON_REQOP_POSITION) & _CFD3CON_REQOP_MASK);
    while(((CFD3CON & _CFD3CON_OPMOD_MASK) >> _CFD3CON_OPMOD_POSITION) != CANFD_OPERATION_MODE)
    {
        /* Do Nothing */
    }
}

// *****************************************************************************
/* Function:
    bool CAN3_MessageTransmit(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoQueueNum, CANFD_MODE mode, CANFD_MSG_TX_ATTRIBUTE msgAttr)

   Summary:
    Transmits a message into CAN bus.

   Precondition:
    CAN3_Initialize must have been called for the associated CAN instance.

   Parameters:
    id           - 11-bit / 29-bit identifier (ID).
    length       - Length of data buffer in number of bytes.
    data         - Pointer to source data buffer
    fifoQueueNum - If fifoQueueNum is 0 then Transmit Queue otherwise FIFO
    mode         - CAN mode Classic CAN or CAN FD without BRS or CAN FD with BRS
    msgAttr      - Data frame or Remote frame to be transmitted

   Returns:
    Request status.
    true  - Request was successful.
    false - Request has failed.
*/
bool CAN3_MessageTransmit(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoQueueNum, CANFD_MODE mode, CANFD_MSG_TX_ATTRIBUTE msgAttr)
{
    CANFD_TX_MSG_OBJECT *txMessage = NULL;
    uint8_t count = 0;
    uint8_t dlc = 0;
    bool status = false;

    if (fifoQueueNum == 0U)
    {
        if ((CFD3TXQSTA & _CFD3TXQSTA_TXQNIF_MASK) == _CFD3TXQSTA_TXQNIF_MASK)
        {
            txMessage = (CANFD_TX_MSG_OBJECT *)PA_TO_KVA1(CFD3TXQUA);
            status = true;
        }
    }
    else if (fifoQueueNum <= CANFD_NUM_OF_FIFO)
    {
        if ((*(volatile uint32_t *)(&CFD3FIFOSTA1 + ((fifoQueueNum - 1U) * CANFD_FIFO_OFFSET)) & _CFD3FIFOSTA1_TFNRFNIF_MASK) == _CFD3FIFOSTA1_TFNRFNIF_MASK)
        {
            txMessage = (CANFD_TX_MSG_OBJECT *)PA_TO_KVA1(*(volatile uint32_t *)(&CFD3FIFOUA1 + ((fifoQueueNum - 1U) * CANFD_FIFO_OFFSET)));
            status = true;
        }
    }
    else
    {
        /* Do Nothing */
    }

    if (status)
    {
        /* Check the id whether it falls under SID or EID,
         * SID max limit is 0x7FF, so anything beyond that is EID */
        if (id > CANFD_MSG_SID_MASK)
        {
            txMessage->t0 = (((id & CANFD_MSG_TX_EXT_SID_MASK) >> 18) | ((id & CANFD_MSG_TX_EXT_EID_MASK) << 11)) & CANFD_MSG_EID_MASK;
            txMessage->t1 = CANFD_MSG_IDE_MASK;
        }
        else
        {
            txMessage->t0 = id;
            txMessage->t1 = 0;
        }
        if (length > 64U)
        {
            length = 64;
        }

        CANLengthToDlcGet(length, &dlc);

        txMessage->t1 |= ((uint32_t)dlc & (uint32_t)CANFD_MSG_DLC_MASK);

        if(mode == CANFD_MODE_FD_WITH_BRS)
        {
            txMessage->t1 |= CANFD_MSG_FDF_MASK | CANFD_MSG_BRS_MASK;
        }
        else if (mode == CANFD_MODE_FD_WITHOUT_BRS)
        {
            txMessage->t1 |= CANFD_MSG_FDF_MASK;
        }
        else
        {
            /* Do Nothing */
        }
        if (msgAttr == CANFD_MSG_TX_REMOTE_FRAME)
        {
            txMessage->t1 |= CANFD_MSG_RTR_MASK;
        }
        else
        {
            while(count < length)
            {
                txMessage->data[count] = *data;
                count++;
                data++;
            }
        }

        if (fifoQueueNum == 0U)
        {
            /* Request the transmit */
            CFD3TXQCON |= _CFD3TXQCON_UINC_MASK;
            CFD3TXQCON |= _CFD3TXQCON_TXREQ_MASK;
        }
        else
        {
            /* Request the transmit */
            *(volatile uint32_t *)(&CFD3FIFOCON1 + ((fifoQueueNum - 1U) * CANFD_FIFO_OFFSET)) |= _CFD3FIFOCON1_UINC_MASK;
            *(volatile uint32_t *)(&CFD3FIFOCON1 + ((fifoQueueNum - 1U) * CANFD_FIFO_OFFSET)) |= _CFD3FIFOCON1_TXREQ_MASK;
        }
    }
    return status;
}

// *****************************************************************************
/* Function:
    bool CAN3_MessageReceive(uint32_t *id, uint8_t *length, uint8_t *data, uint32_t *timestamp, uint8_t fifoNum, CANFD_MSG_RX_ATTRIBUTE *msgAttr)

   Summary:
    Receives a message from CAN bus.

   Precondition:
    CAN3_Initialize must have been called for the associated CAN instance.

   Parameters:
    id          - Pointer to 11-bit / 29-bit identifier (ID) to be received.
    length      - Pointer to data length in number of bytes to be received.
    data        - Pointer to destination data buffer
    timestamp   - Pointer to Rx message timestamp, timestamp value is 0 if Timestamp is disabled in CFD3TSCON
    fifoNum     - FIFO number
    msgAttr     - Data frame or Remote frame to be received

   Returns:
    Request status.
    true  - Request was successful.
    false - Request has failed.
*/
bool CAN3_MessageReceive(uint32_t *id, uint8_t *length, uint8_t *data, uint32_t *timestamp, uint8_t fifoNum, CANFD_MSG_RX_ATTRIBUTE *msgAttr)
{
    CANFD_RX_MSG_OBJECT *rxMessage = NULL;
    uint8_t count = 0;
    bool status = false;

    if ((fifoNum > CANFD_NUM_OF_FIFO) || (id == NULL))
    {
        return status;
    }

    /* Check if there is a message available in FIFO */
    if ((*(volatile uint32_t *)(&CFD3FIFOSTA1 + ((fifoNum - 1U) * CANFD_FIFO_OFFSET)) & _CFD3FIFOSTA1_TFNRFNIF_MASK) == _CFD3FIFOSTA1_TFNRFNIF_MASK)
    {
        /* Get a pointer to RX message buffer */
        rxMessage = (CANFD_RX_MSG_OBJECT *)PA_TO_KVA1(*(volatile uint32_t *)(&CFD3FIFOUA1 + ((fifoNum - 1U) * CANFD_FIFO_OFFSET)));

        /* Check if it's a extended message type */
        if ((rxMessage->r1 & CANFD_MSG_IDE_MASK) != 0U)
        {
            *id = (((rxMessage->r0 & CANFD_MSG_RX_EXT_SID_MASK) << 18) | ((rxMessage->r0 & CANFD_MSG_RX_EXT_EID_MASK) >> 11)) & CANFD_MSG_EID_MASK;
        }
        else
        {
            *id = rxMessage->r0 & CANFD_MSG_SID_MASK;
        }

        if (((rxMessage->r1 & CANFD_MSG_RTR_MASK) != 0U) && ((rxMessage->r1 & CANFD_MSG_FDF_MASK) == 0U))
        {
            *msgAttr = CANFD_MSG_RX_REMOTE_FRAME;
        }
        else
        {
            *msgAttr = CANFD_MSG_RX_DATA_FRAME;
        }

        *length = dlcToLength[(rxMessage->r1 & CANFD_MSG_DLC_MASK)];

        if (timestamp != NULL)
        {
        }

        /* Copy the data into the payload */
        while (count < *length)
        {
            *data = rxMessage->data[count];
             data++;
             count++;
        }

        /* Message processing is done, update the message buffer pointer. */
        *(volatile uint32_t *)(&CFD3FIFOCON1 + ((fifoNum - 1U) * CANFD_FIFO_OFFSET)) |= _CFD3FIFOCON1_UINC_MASK;

        /* Message is processed successfully, so return true */
        status = true;
    }

    return status;
}

// *****************************************************************************
/* Function:
    void CAN3_MessageAbort(uint8_t fifoQueueNum)

   Summary:
    Abort request for a Queue/FIFO.

   Precondition:
    CAN3_Initialize must have been called for the associated CAN instance.

   Parameters:
    fifoQueueNum - If fifoQueueNum is 0 then Transmit Queue otherwise FIFO

   Returns:
    None.
*/
void CAN3_MessageAbort(uint8_t fifoQueueNum)
{
    if (fifoQueueNum == 0U)
    {
        CFD3TXQCON &= ~_CFD3TXQCON_TXREQ_MASK;
    }
    else if (fifoQueueNum <= CANFD_NUM_OF_FIFO)
    {
        *(volatile uint32_t *)(&CFD3FIFOCON1 + ((fifoQueueNum - 1U) * CANFD_FIFO_OFFSET)) &= ~_CFD3FIFOCON1_TXREQ_MASK;
    }
    else
    {
        /* Do Nothing */
    }
}

// *****************************************************************************
/* Function:
    void CAN3_MessageAcceptanceFilterSet(uint8_t filterNum, uint32_t id)

   Summary:
    Set Message acceptance filter configuration.

   Precondition:
    CAN3_Initialize must have been called for the associated CAN instance.

   Parameters:
    filterNum - Filter number
    id        - 11-bit or 29-bit identifier

   Returns:
    None.
*/
void CAN3_MessageAcceptanceFilterSet(uint8_t filterNum, uint32_t id)
{
    uint32_t filterEnableBit = 0;
    uint8_t filterRegIndex = 0;

    if (filterNum < CANFD_NUM_OF_FILTER)
    {
        filterRegIndex = filterNum >> 2;
        filterEnableBit = _CFD3FLTCON0_FLTEN0_MASK;

        *(volatile uint32_t *)(&CFD3FLTCON0 + (filterRegIndex * CANFD_FILTER_OFFSET)) &= ~filterEnableBit;

        if (id > CANFD_MSG_SID_MASK)
        {
            *(volatile uint32_t *)(&CFD3FLTOBJ0 + (filterNum * CANFD_FILTER_OBJ_OFFSET)) = ((((id & CANFD_MSG_FLT_EXT_SID_MASK) >> 18) | ((id & CANFD_MSG_FLT_EXT_EID_MASK) << 11)) & CANFD_MSG_EID_MASK) | _CFD3FLTOBJ0_EXIDE_MASK;
        }
        else
        {
            *(volatile uint32_t *)(&CFD3FLTOBJ0 + (filterNum * CANFD_FILTER_OBJ_OFFSET)) = id & CANFD_MSG_SID_MASK;
        }
        *(volatile uint32_t *)(&CFD3FLTCON0 + (filterRegIndex * CANFD_FILTER_OFFSET)) |= filterEnableBit;
    }
}

// *****************************************************************************
/* Function:
    uint32_t CAN3_MessageAcceptanceFilterGet(uint8_t filterNum)

   Summary:
    Get Message acceptance filter configuration.

   Precondition:
    CAN3_Initialize must have been called for the associated CAN instance.

   Parameters:
    filterNum - Filter number

   Returns:
    Returns Message acceptance filter identifier
*/
uint32_t CAN3_MessageAcceptanceFilterGet(uint8_t filterNum)
{
    uint32_t id = 0;

    if (filterNum < CANFD_NUM_OF_FILTER)
    {
        if ((*(volatile uint32_t *)(&CFD3FLTOBJ0 + (filterNum * CANFD_FILTER_OBJ_OFFSET)) & _CFD3FLTOBJ0_EXIDE_MASK) != 0U)
        {
            id = (*(volatile uint32_t *)(&CFD3FLTOBJ0 + (filterNum * CANFD_FILTER_OBJ_OFFSET)) & CANFD_MSG_RX_EXT_SID_MASK) << 18;
            id = (id | ((*(volatile uint32_t *)(&CFD3FLTOBJ0 + (filterNum * CANFD_FILTER_OBJ_OFFSET)) & CANFD_MSG_RX_EXT_EID_MASK) >> 11))
               & CANFD_MSG_EID_MASK;
        }
        else
        {
            id = (*(volatile uint32_t *)(&CFD3FLTOBJ0 + (filterNum * CANFD_FILTER_OBJ_OFFSET)) & CANFD_MSG_SID_MASK);
        }
    }
    return id;
}

// *****************************************************************************
/* Function:
    void CAN3_MessageAcceptanceFilterMaskSet(uint8_t acceptanceFilterMaskNum, uint32_t id)

   Summary:
    Set Message acceptance filter mask configuration.

   Precondition:
    CAN3_Initialize must have been called for the associated CAN instance.

   Parameters:
    acceptanceFilterMaskNum - Acceptance Filter Mask number
    id                      - 11-bit or 29-bit identifier

   Returns:
    None.
*/
void CAN3_MessageAcceptanceFilterMaskSet(uint8_t acceptanceFilterMaskNum, uint32_t id)
{
    /* Switch the CAN module to Configuration mode. Wait until the switch is complete */
    CFD3CON = (CFD3CON & ~_CFD3CON_REQOP_MASK) | ((CANFD_CONFIGURATION_MODE << _CFD3CON_REQOP_POSITION) & _CFD3CON_REQOP_MASK);
    while(((CFD3CON & _CFD3CON_OPMOD_MASK) >> _CFD3CON_OPMOD_POSITION) != CANFD_CONFIGURATION_MODE)
    {
        /* Do Nothing */

    }

    if (id > CANFD_MSG_SID_MASK)
    {
        *(volatile uint32_t *)(&CFD3MASK0 + (acceptanceFilterMaskNum * CANFD_ACCEPTANCE_MASK_OFFSET)) = ((((id & CANFD_MSG_FLT_EXT_SID_MASK) >> 18) | ((id & CANFD_MSG_FLT_EXT_EID_MASK) << 11)) & CANFD_MSG_EID_MASK) | _CFD3MASK0_MIDE_MASK;
    }
    else
    {
        *(volatile uint32_t *)(&CFD3MASK0 + (acceptanceFilterMaskNum * CANFD_ACCEPTANCE_MASK_OFFSET)) = id & CANFD_MSG_SID_MASK;
    }

    /* Switch the CAN module to CANFD_OPERATION_MODE. Wait until the switch is complete */
    CFD3CON = (CFD3CON & ~_CFD3CON_REQOP_MASK) | ((CANFD_OPERATION_MODE << _CFD3CON_REQOP_POSITION) & _CFD3CON_REQOP_MASK);
    while(((CFD3CON & _CFD3CON_OPMOD_MASK) >> _CFD3CON_OPMOD_POSITION) != CANFD_OPERATION_MODE)
    {
        /* Do Nothing */

    }
}

// *****************************************************************************
/* Function:
    uint32_t CAN3_MessageAcceptanceFilterMaskGet(uint8_t acceptanceFilterMaskNum)

   Summary:
    Get Message acceptance filter mask configuration.

   Precondition:
    CAN3_Initialize must have been called for the associated CAN instance.

   Parameters:
    acceptanceFilterMaskNum - Acceptance Filter Mask number

   Returns:
    Returns Message acceptance filter mask.
*/
uint32_t CAN3_MessageAcceptanceFilterMaskGet(uint8_t acceptanceFilterMaskNum)
{
    uint32_t id = 0;

    if ((*(volatile uint32_t *)(&CFD3MASK0 + (acceptanceFilterMaskNum * CANFD_ACCEPTANCE_MASK_OFFSET)) & _CFD3MASK0_MIDE_MASK) != 0U)
    {
        id = (*(volatile uint32_t *)(&CFD3MASK0 + (acceptanceFilterMaskNum * CANFD_ACCEPTANCE_MASK_OFFSET)) & CANFD_MSG_RX_EXT_SID_MASK) << 18;
        id = (id | ((*(volatile uint32_t *)(&CFD3MASK0 + (acceptanceFilterMaskNum * CANFD_ACCEPTANCE_MASK_OFFSET)) & CANFD_MSG_RX_EXT_EID_MASK) >> 11))
           & CANFD_MSG_EID_MASK;
    }
    else
    {
        id = (*(volatile uint32_t *)(&CFD3MASK0 + (acceptanceFilterMaskNum * CANFD_ACCEPTANCE_MASK_OFFSET)) & CANFD_MSG_SID_MASK);
    }
    return id;
}


// *****************************************************************************
/* Function:
    CANFD_ERROR CAN3_ErrorGet(void)

   Summary:
    Returns the error during transfer.

   Precondition:
    CAN3_Initialize must have been called for the associated CAN instance.

   Parameters:
    None.

   Returns:
    Error during transfer.
*/
CANFD_ERROR CAN3_ErrorGet(void)
{
    uint32_t error = (uint32_t)CANFD_ERROR_NONE;
    uint32_t errorStatus = CFD3TREC;

    /* Check if error occurred */
    error = ((errorStatus & _CFD3TREC_EWARN_MASK) |
                (errorStatus & _CFD3TREC_RXWARN_MASK) |
                (errorStatus & _CFD3TREC_TXWARN_MASK) |
                (errorStatus & _CFD3TREC_RXBP_MASK) |
                (errorStatus & _CFD3TREC_TXBP_MASK) |
                (errorStatus & _CFD3TREC_TXBO_MASK));

    return (CANFD_ERROR)error;
}

// *****************************************************************************
/* Function:
    void CAN3_ErrorCountGet(uint8_t *txErrorCount, uint8_t *rxErrorCount)

   Summary:
    Returns the transmit and receive error count during transfer.

   Precondition:
    CAN3_Initialize must have been called for the associated CAN instance.

   Parameters:
    txErrorCount - Transmit Error Count to be received
    rxErrorCount - Receive Error Count to be received

   Returns:
    None.
*/
void CAN3_ErrorCountGet(uint8_t *txErrorCount, uint8_t *rxErrorCount)
{
    *txErrorCount = (uint8_t)((CFD3TREC & _CFD3TREC_TERRCNT_MASK) >> _CFD3TREC_TERRCNT_POSITION);
    *rxErrorCount = (uint8_t)(CFD3TREC & _CFD3TREC_RERRCNT_MASK);
}

// *****************************************************************************
/* Function:
    bool CAN3_InterruptGet(uint8_t fifoQueueNum, CANFD_FIFO_INTERRUPT_FLAG_MASK fifoInterruptFlagMask)

   Summary:
    Returns the FIFO Interrupt status.

   Precondition:
    CAN3_Initialize must have been called for the associated CAN instance.

   Parameters:
    fifoQueueNum          - FIFO number
    fifoInterruptFlagMask - FIFO interrupt flag mask

   Returns:
    true - Requested fifo interrupt is occurred.
    false - Requested fifo interrupt is not occurred.
*/
bool CAN3_InterruptGet(uint8_t fifoQueueNum, CANFD_FIFO_INTERRUPT_FLAG_MASK fifoInterruptFlagMask)
{
    if (fifoQueueNum == 0U)
    {
        return ((CFD3TXQSTA & (uint32_t)fifoInterruptFlagMask) != 0x0U);
    }
    else
    {
        return ((*(volatile uint32_t *)(&CFD3FIFOSTA1 + ((fifoQueueNum - 1U) * CANFD_FIFO_OFFSET)) & (uint32_t)fifoInterruptFlagMask) != 0x0U);
    }
}

// *****************************************************************************
/* Function:
    bool CAN3_TxFIFOQueueIsFull(uint8_t fifoQueueNum)

   Summary:
    Returns true if Tx FIFO/Queue is full otherwise false.

   Precondition:
    CAN3_Initialize must have been called for the associated CAN instance.

   Parameters:
    fifoQueueNum - FIFO/Queue number

   Returns:
    true  - Tx FIFO/Queue is full.
    false - Tx FIFO/Queue is not full.
*/
bool CAN3_TxFIFOQueueIsFull(uint8_t fifoQueueNum)
{
    if (fifoQueueNum == 0U)
    {
        return ((CFD3TXQSTA & _CFD3TXQSTA_TXQNIF_MASK) != _CFD3TXQSTA_TXQNIF_MASK);
    }
    else
    {
        return ((*(volatile uint32_t *)(&CFD3FIFOSTA1 + ((fifoQueueNum - 1U) * CANFD_FIFO_OFFSET)) & _CFD3FIFOSTA1_TFNRFNIF_MASK) != _CFD3FIFOSTA1_TFNRFNIF_MASK);
    }
}

// *****************************************************************************
/* Function:
    bool CAN3_AutoRTRResponseSet(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoNum)

   Summary:
    Set the Auto RTR response for remote transmit request.

   Precondition:
    CAN3_Initialize must have been called for the associated CAN instance.
    Auto RTR Enable must be set to 0x1 for the requested Transmit FIFO in MHC configuration.

   Parameters:
    id           - 11-bit / 29-bit identifier (ID).
    length       - Length of data buffer in number of bytes.
    data         - Pointer to source data buffer
    fifoNum      - FIFO Number

   Returns:
    Request status.
    true  - Request was successful.
    false - Request has failed.
*/
bool CAN3_AutoRTRResponseSet(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoNum)
{
    CANFD_TX_MSG_OBJECT *txMessage = NULL;
    uint8_t count = 0;
    bool status = false;

    if (fifoNum <= CANFD_NUM_OF_FIFO)
    {
        if ((*(volatile uint32_t *)(&CFD3FIFOSTA1 + ((fifoNum - 1U) * CANFD_FIFO_OFFSET)) & _CFD3FIFOSTA1_TFNRFNIF_MASK) == _CFD3FIFOSTA1_TFNRFNIF_MASK)
        {
            txMessage = (CANFD_TX_MSG_OBJECT *)PA_TO_KVA1(*(volatile uint32_t *)(&CFD3FIFOUA1 + ((fifoNum - 1U) * CANFD_FIFO_OFFSET)));
            status = true;
        }
    }

    if (status)
    {
        /* Check the id whether it falls under SID or EID,
         * SID max limit is 0x7FF, so anything beyond that is EID */
        if (id > CANFD_MSG_SID_MASK)
        {
            txMessage->t0 = (((id & CANFD_MSG_TX_EXT_SID_MASK) >> 18) | ((id & CANFD_MSG_TX_EXT_EID_MASK) << 11)) & CANFD_MSG_EID_MASK;
            txMessage->t1 = CANFD_MSG_IDE_MASK;
        }
        else
        {
            txMessage->t0 = id;
            txMessage->t1 = 0U;
        }

        /* Limit length */
        if (length > 8U)
        {
            length = 8U;
        }
        txMessage->t1 |= length;

        while(count < length)
        {
            txMessage->data[count] = *data;
            count++;
            data++;
        }

        /* Set UINC to respond to RTR */
        *(volatile uint32_t *)(&CFD3FIFOCON1 + ((fifoNum - 1U) * CANFD_FIFO_OFFSET)) |= _CFD3FIFOCON1_UINC_MASK;
    }
    return status;
}

bool CAN3_BitTimingCalculationGet(CANFD_BIT_TIMING_SETUP *setup, CANFD_BIT_TIMING *bitTiming)
{
    bool status = false;
    uint32_t numOfTimeQuanta;
    uint8_t tseg1;
    float temp1;
    float temp2;

    if ((setup != NULL) && (bitTiming != NULL))
    {
        if (setup->nominalBitTimingSet == true)
        {
            numOfTimeQuanta = CAN3_CLOCK_FREQUENCY / (setup->nominalBitRate * ((uint32_t)setup->nominalPrescaler + 1U));
            if ((numOfTimeQuanta >= 4U) && (numOfTimeQuanta <= 385U))
            {
                if (setup->nominalSamplePoint < 50.0f)
                {
                    setup->nominalSamplePoint = 50.0f;
                }
                temp1 = (float)numOfTimeQuanta;
                temp2 = (temp1 * setup->nominalSamplePoint) / 100.0f;
                tseg1 = (uint8_t)temp2;
                bitTiming->nominalBitTiming.nominalTimeSegment2 = (uint8_t)(numOfTimeQuanta - tseg1 - 1U);
                bitTiming->nominalBitTiming.nominalTimeSegment1 = tseg1 - 2U;
                bitTiming->nominalBitTiming.nominalSJW = bitTiming->nominalBitTiming.nominalTimeSegment2;
                bitTiming->nominalBitTiming.nominalPrescaler = setup->nominalPrescaler;
                bitTiming->nominalBitTimingSet = true;
                status = true;
            }
            else
            {
                bitTiming->nominalBitTimingSet = false;
            }
        }
        if (setup->dataBitTimingSet == true)
        {
            numOfTimeQuanta = CAN3_CLOCK_FREQUENCY / (setup->dataBitRate * ((uint32_t)setup->dataPrescaler + 1U));
            if ((numOfTimeQuanta >= 4U) && (numOfTimeQuanta <= 49U))
            {
                if (setup->dataSamplePoint < 50.0f)
                {
                    setup->dataSamplePoint = 50.0f;
                }
                temp1 = (float)numOfTimeQuanta;
                temp2 = (temp1 * setup->dataSamplePoint) / 100.0f;
                tseg1 = (uint8_t)temp2;
                bitTiming->dataBitTiming.dataTimeSegment2 = (uint8_t)(numOfTimeQuanta - tseg1 - 1U);
                bitTiming->dataBitTiming.dataTimeSegment1 = tseg1 - 2U;
                bitTiming->dataBitTiming.dataSJW = bitTiming->dataBitTiming.dataTimeSegment2;
                bitTiming->dataBitTiming.dataPrescaler = setup->dataPrescaler;
                bitTiming->dataBitTimingSet = true;
                status = true;
            }
            else
            {
                bitTiming->dataBitTimingSet = false;
                status = false;
            }
        }
    }

    return status;
}

bool CAN3_BitTimingSet(CANFD_BIT_TIMING *bitTiming)
{
    bool status = false;
    bool nominalBitTimingSet = false;
    bool dataBitTimingSet = false;

    if ((bitTiming->nominalBitTimingSet == true)
    && (bitTiming->nominalBitTiming.nominalTimeSegment1 >= 0x1U)
    && (bitTiming->nominalBitTiming.nominalTimeSegment2 <= 0x7FU)
    && (bitTiming->nominalBitTiming.nominalSJW <= 0x7FU))
    {
        nominalBitTimingSet = true;
    }

    if  ((bitTiming->dataBitTimingSet == true)
    &&  (bitTiming->dataBitTiming.dataTimeSegment1 <= 0x1FU)
    &&  (bitTiming->dataBitTiming.dataTimeSegment2 <= 0xFU)
    &&  (bitTiming->dataBitTiming.dataSJW <= 0xFU))
    {
        dataBitTimingSet = true;
    }

    if ((nominalBitTimingSet == true) || (dataBitTimingSet == true))
    {
        /* Switch the CAN module to Configuration mode. Wait until the switch is complete */
        CFD3CON = (CFD3CON & ~_CFD3CON_REQOP_MASK) | ((CANFD_CONFIGURATION_MODE << _CFD3CON_REQOP_POSITION) & _CFD3CON_REQOP_MASK);
        while(((CFD3CON & _CFD3CON_OPMOD_MASK) >> _CFD3CON_OPMOD_POSITION) != CANFD_CONFIGURATION_MODE)
        {
            /* Do Nothing */
        }

        if (dataBitTimingSet == true)
        {
            /* Set the Data bitrate */
            CFD3DBTCFG = (((uint32_t)bitTiming->dataBitTiming.dataPrescaler << _CFD3DBTCFG_BRP_POSITION) & _CFD3DBTCFG_BRP_MASK)
                       | (((uint32_t)bitTiming->dataBitTiming.dataTimeSegment1 << _CFD3DBTCFG_TSEG1_POSITION) & _CFD3DBTCFG_TSEG1_MASK)
                       | (((uint32_t)bitTiming->dataBitTiming.dataTimeSegment2 << _CFD3DBTCFG_TSEG2_POSITION) & _CFD3DBTCFG_TSEG2_MASK)
                       | (((uint32_t)bitTiming->dataBitTiming.dataSJW << _CFD3DBTCFG_SJW_POSITION) & _CFD3DBTCFG_SJW_MASK);
        }

        if (nominalBitTimingSet == true)
        {
            /* Set the Nominal bitrate */
            CFD3NBTCFG = (((uint32_t)bitTiming->nominalBitTiming.nominalPrescaler << _CFD3NBTCFG_BRP_POSITION) & _CFD3NBTCFG_BRP_MASK)
                       | (((uint32_t)bitTiming->nominalBitTiming.nominalTimeSegment1 << _CFD3NBTCFG_TSEG1_POSITION) & _CFD3NBTCFG_TSEG1_MASK)
                       | (((uint32_t)bitTiming->nominalBitTiming.nominalTimeSegment2 << _CFD3NBTCFG_TSEG2_POSITION) & _CFD3NBTCFG_TSEG2_MASK)
                       | (((uint32_t)bitTiming->nominalBitTiming.nominalSJW << _CFD3NBTCFG_SJW_POSITION) & _CFD3NBTCFG_SJW_MASK);
        }

        /* Switch the CAN module to CANFD_OPERATION_MODE. Wait until the switch is complete */
        CFD3CON = (CFD3CON & ~_CFD3CON_REQOP_MASK) | ((CANFD_OPERATION_MODE << _CFD3CON_REQOP_POSITION) & _CFD3CON_REQOP_MASK);
        while(((CFD3CON & _CFD3CON_OPMOD_MASK) >> _CFD3CON_OPMOD_POSITION) != CANFD_OPERATION_MODE)
        {
            /* Do Nothing */
        }

        status = true;
    }
    return status;
}


/* MISRAC 2012 deviation block end */
