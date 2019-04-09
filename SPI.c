/*============================================================================*/
/* Project      = AUTOSAR Renesas Xx4 MCAL Components                         */
/* Module       = Spi.c                                                       */
/* Version      = 3.1.10                                                      */
/* Date         = 03-Jun-2015                                                 */
/*============================================================================*/
/*                                  COPYRIGHT                                 */
/*============================================================================*/
/* Copyright (c) 2009-2015 by Renesas Electronics Corporation                 */
/*============================================================================*/
/* Purpose:                                                                   */
/* This file contains the implementations AUTOSAR specified APIs for SPI      */
/* handler.                                                                   */
/*============================================================================*/
/*                                                                            */
/* Unless otherwise agreed upon in writing between your company and           */
/* Renesas Electronics Corporation the following shall apply!                 */
/*                                                                            */
/* Warranty Disclaimer                                                        */
/*                                                                            */
/* There is no warranty of any kind whatsoever granted by Renesas. Any        */
/* warranty is expressly disclaimed and excluded by Renesas, either expressed */
/* or implied, including but not limited to those for non-infringement of     */
/* intellectual property, merchantability and/or fitness for the particular   */
/* purpose.                                                                   */
/*                                                                            */
/* Renesas shall not have any obligation to maintain, service or provide bug  */
/* fixes for the supplied Product(s) and/or the Application.                  */
/*                                                                            */
/* Each User is solely responsible for determining the appropriateness of     */
/* using the Product(s) and assumes all risks associated with its exercise    */
/* of rights under this Agreement, including, but not limited to the risks    */
/* and costs of program errors, compliance with applicable laws, damage to    */
/* or loss of data, programs or equipment, and unavailability or              */
/* interruption of operations.                                                */
/*                                                                            */
/* Limitation of Liability                                                    */
/*                                                                            */
/* In no event shall Renesas be liable to the User for any incidental,        */
/* consequential, indirect, or punitive damage (including but not limited     */
/* to lost profits) regardless of whether such liability is based on breach   */
/* of contract, tort, strict liability, breach of warranties, failure of      */
/* essential purpose or otherwise and even if advised of the possibility of   */
/* such damages. Renesas shall not be liable for any services or products     */
/* provided by third party vendors, developers or consultants identified or   */
/* referred to the User by Renesas in connection with the Product(s) and/or   */
/* the Application.                                                           */
/*                                                                            */
/*============================================================================*/
/* Environment:                                                               */
/*              Devices:        Xx4                                           */
/*============================================================================*/

/*******************************************************************************
**                      Revision Control History                              **
*******************************************************************************/
/*
 * V3.0.0:  15-Oct-2009  : Initial version
 * V3.0.1:  26-Nov-2009  : As per SCR 153, modified the source code for the
 *                         changes found during Dual Buffer mode, TxOnly mode
 *                         and DMA testing. Also updated for NEC comments.
 * V3.0.2:  30-Nov-2009  : As per SCR 156, modified DET section of Spi_ReadIB.
 *                         and Spi_SetupEB.
 * V3.0.3:  14-Jan-2010  : As per SCR 185, modified the source code for the
 *                         changes found during Px4 testing.
 * V3.0.4:  26-Feb-2010  : As per SCR 202, modified the source code for the
 *                         DET condition check in Spi_GetHWUnitStatus. The macro
 *                         name SPI_MAX_HWUNIT is changed to SPI_MAX_HW_UNIT
 * V3.0.5:  14-Jan-2011  : As per SCR 393,The conditions of declaring,
 *                         initializing and setting the variable
 *                         "LucChannelBufferType" is updated in
 *                          Spi_ReadIB API.
 * V3.1.0:  27-Jul-2011  : Update Software version 3.1.0
 * V3.1.1:  04-Oct-2011  : Added comments for the violation of MISRA rule.
 * V3.1.2:  16-Feb-2012  : Merged the fixes done for Fx4L Spi driver
 * V3.1.3:  16-May-2012  : As per SCR 013, Spi_SetupEB API is updated for
 *                         the DET check related to length parameter.
 * V3.1.4:  06-Jun-2012  : As per SCR 024,following changes are made.
 *                         1. API Spi_GetVersionInfo is removed.
 *                         2. Environment section is updated to remove compiler
 *                            name.
 *                         3. Software patch version Information is updated.
 * V3.1.5   26-Aug-2013  : 1. As per mantis #12584, Spi_Cancel API is updated.
                           2. Added Justifications for MISRA Violations.
                           3. Software patch version Information is updated.
                           4. As per mantis #9946 ,Updated Spi_ReadIB.
 * V3.1.6   14-Feb-2014  : As part of Dx4 Maintenance Activity, the following
 *                         changes are made:
 *                         1. SPI_C_SW_PATCH_VERSION is not checked.
 *                            Hence Removed to make it uniform with other
 *                            modules.
 *                         2. QAC warnings are justified.
 * V3.1.7   14-Mar-2014  : 1.As per mantis MNT_12582,
 *                           Updated Spi_Cancel Api.
 *                         2.Reoved additional Misra justifications.
 * V3.1.8   31-Oct-2014  : As part of Dx4 V3.16 maintenance activity,
 *                         1.Updated Spi_AsyncTransmit() as per mantis MNT_18983
 *                         2.MISRA Justifications added after generating QAC
 *                           reports.
 * V3.1.9   17-Nov-2014  : As part of Fx4L V3.02 maintenance activity,
 *                         1.Additional MISRA Justifications are deleted after
 *                           generating QAC reports.
 * V3.1.10  03-Jun-2015  : As part of Fx4 V3.11 maintenance activity,
 *                         following changes are made
 *                         1.As per MNT_0028003, DET implementation is made
 *                           parallel.
 *                         2.As per MNT_0027193,Critical section protection
 *                           is provided for global variables.
 *                         3.As per MNT_0028318,(SPI_LEVEL_DELIVERED ==
 *                           SPI_ONE) is removed from the pre-compiler option
 *                           for Spi_MainFunction_Driving.
 *                         4.As per MNT_0028686,DET check is added for interrupt
 *                           mode in Spi_MainFunction_Driving() API.
 *                         5.MISRA Justifications added after generating QAC
 *                           reports
 *                         6.Updated Copyright Information.
 */
/******************************************************************************/

/*******************************************************************************
**                      Include Section                                       **
*******************************************************************************/
#include "Spi.h"
#include "Spi_Scheduler.h"
#include "Spi_Ram.h"

#if (SPI_DEV_ERROR_DETECT == STD_ON)
#include "Det.h"
#endif

/*******************************************************************************
**                      Version Information                                   **
*******************************************************************************/

/* AUTOSAR SPECIFICATION VERSION INFORMATION */
#define SPI_C_AR_MAJOR_VERSION    SPI_AR_MAJOR_VERSION_VALUE
#define SPI_C_AR_MINOR_VERSION    SPI_AR_MINOR_VERSION_VALUE
#define SPI_C_AR_PATCH_VERSION    SPI_AR_PATCH_VERSION_VALUE

/* FILE VERSION INFORMATION */
#define SPI_C_SW_MAJOR_VERSION  3
#define SPI_C_SW_MINOR_VERSION  1

/*******************************************************************************
**                      Version Check                                         **
*******************************************************************************/

#if (SPI_AR_MAJOR_VERSION != SPI_C_AR_MAJOR_VERSION)
  #error "Spi_Handler.c : Mismatch in Specification Major Version"
#endif

#if (SPI_AR_MINOR_VERSION != SPI_C_AR_MINOR_VERSION)
  #error "Spi_Handler.c : Mismatch in Specification Minor Version"
#endif

#if (SPI_AR_PATCH_VERSION != SPI_C_AR_PATCH_VERSION)
  #error "Spi_Handler.c : Mismatch in Specification Patch Version"
#endif

#if (SPI_SW_MAJOR_VERSION != SPI_C_SW_MAJOR_VERSION)
  #error "Spi_Handler.c : Mismatch in Major Version"
#endif

#if (SPI_SW_MINOR_VERSION != SPI_C_SW_MINOR_VERSION)
  #error "Spi_Handler.c : Mismatch in Minor Version"
#endif

/*******************************************************************************
**                         Global Data                                        **
*******************************************************************************/

/*******************************************************************************
**                      Function Definitions                                  **
*******************************************************************************/

/*******************************************************************************
** Function Name       : Spi_Init
**
** Service ID          : 0x00
**
** Description         : This service performs initialization of the SPI Driver
**                       component
**
** Sync/Async          : Synchronous
**
** Re-entrancy         : Non-Reentrant
**
** Input Parameters    : const Spi_ConfigType *ConfigPtr
**
** InOut Parameters    : None
**
** Output Parameters   : None
**
** Return parameter    : void
**
** Preconditions       : None
**
** Remarks             : Global Variable:
**                       Spi_GddInitStatus
**                       Spi_GddDriverStatus
**                       Spi_GpConfigPtr
**                       Spi_GpFirstChannel
**                       Spi_GpFirstJob
**                       Spi_GpFirstSeq
**                       Spi_GaaJobResult
**                       Spi_GaaSeqResult
**                       Spi_GaaChannelIBWrite
**                       Spi_GaaChannelIBRead
**                       Spi_GaaChannelEBData
**                       Spi_GddAsyncMode
**                       Spi_GddDriverStatus
**
**                       Function Invoked:
**                       Det_ReportError
**                       Spi_InternalWriteIB
**                       Spi_HWInit
**
*******************************************************************************/
#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"

FUNC(void, SPI_PUBLIC_CODE) Spi_Init
(P2CONST(Spi_ConfigType, AUTOMATIC, SPI_APPL_CONST) ConfigPtr)
{
  P2CONST(Tdd_Spi_ChannelLTConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpLTChannelConfig;
  P2CONST(Tdd_Spi_ChannelPBConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpPBChannelConfig;

  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  boolean LblErrorFlag;
  #endif

  #if ((SPI_LEVEL_DELIVERED == SPI_ONE)|| (SPI_LEVEL_DELIVERED == SPI_TWO))
  P2VAR(uint8,AUTOMATIC,SPI_CONFIG_DATA) LpStatusPtr;
  #endif

  #if (SPI_CANCEL_API == STD_ON) || (SPI_LEVEL_DELIVERED == SPI_ONE) || \
                                        (SPI_LEVEL_DELIVERED == SPI_TWO)
  uint8 LucVar;
  #endif

  #if (SPI_EB_CONFIGURED == STD_ON)
  Spi_NumberOfDataType LddNoOfBuffers;
  #endif

  Spi_JobType LddNumJobs;
  Spi_SequenceType LddNumSeq;
  Spi_ChannelType LddNumChannel;
  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  LblErrorFlag = SPI_FALSE;
  /* Check if SPI configuration pointer is a NULL Pointer  */
  if(ConfigPtr == NULL_PTR)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                           SPI_INIT_SID, SPI_E_PARAM_CONFIG);
    LblErrorFlag = SPI_TRUE;
  }
  else
  {
    /* No action required */
  }

  if(Spi_GddDriverStatus != SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                     SPI_INIT_SID, SPI_E_ALREADY_INITIALIZED);
    LblErrorFlag = SPI_TRUE;
  }
  else
  {
    /* No action required */
  }

  /* If no DET errors, continue */
  if(LblErrorFlag == SPI_FALSE)

  #endif /* (SPI_DEV_ERROR_DETECT == STD_ON) */
  {
    /* To check whether database is placed or not */
    /* MISRA Rule     : 1.2                                            */
    /* Message        : Dereferencing pointer value that is apparently */
    /*                  NULL.                                          */
    /* Reason         : This is the pointer passed to this function    */
    /* Verification   : However, this is validated when DET is enabled.*/
    if(ConfigPtr->ulStartOfDbToc != SPI_DBTOC_VALUE)
    {
      #if (SPI_DEV_ERROR_DETECT == STD_ON)
      /* Report to DET */
      Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                          SPI_INIT_SID, SPI_E_INVALID_DATABASE);
      #endif
    }
    else /* ConfigPtr->ulStartOfDbToc != SPI_DBTOC_VALUE */
    {
      /* Load ConfigPtr to Global pointer variable  */
      Spi_GpConfigPtr = ConfigPtr;
      /* Load first channel to Global pointer variable */
      /* MISRA Rule     : 1.2                                            */
      /* Message        : Dereferencing pointer value that is apparently */
      /*                  NULL.                                          */
      /* Reason         : This is the pointer passed to this function    */
      /* Verification   : However, this is validated when DET is enabled.*/
      Spi_GpFirstChannel =
       (P2CONST(Tdd_Spi_ChannelPBConfigType, SPI_DATA, SPI_PRIVATE_CONST))

                                                 Spi_GpConfigPtr->pFirstChannel;
      /* Load first job to Global pointer variable*/
      Spi_GpFirstJob =
       (P2CONST(Tdd_Spi_JobConfigType, SPI_DATA, SPI_PRIVATE_CONST))
                                                 Spi_GpConfigPtr->pFirstJob;
      /*  Load first sequence to Global pointer variable */
      Spi_GpFirstSeq =
        (P2CONST(Tdd_Spi_SequenceConfigType, SPI_DATA, SPI_PRIVATE_CONST))
                                                 Spi_GpConfigPtr->pFirstSeq;

      /*  Load first job to Global pointer variable */
      Spi_GpFirstJobList =
        (P2CONST(Tdd_Spi_JobListType, SPI_DATA, SPI_PRIVATE_CONST))
                                                 Spi_GpConfigPtr->pJobList;

      /* Load the number of jobs configured as zero */
      LddNumJobs = SPI_ZERO;

      while(LddNumJobs < SPI_MAX_JOB)
      {
        /* Update contents of Job result pointer */
        Spi_GaaJobResult[LddNumJobs] = SPI_JOB_OK;
        /* Increment number of jobs configured */
        LddNumJobs++;
      }

      /* Load the number of sequence configured as zero */
      LddNumSeq = SPI_ZERO ;

      while(LddNumSeq < SPI_MAX_SEQUENCE)
      {
        /* Update contents of Sequence result pointer */
        Spi_GaaSeqResult[LddNumSeq] = SPI_SEQ_OK;
        /* Increment number of sequences configured */
        LddNumSeq++;
      }

      /* Get the number of channels configured */
      LddNumChannel = SPI_ZERO;

      /* Get the pointer to the link-time structure of channel */
      LpLTChannelConfig = &Spi_GstChannelLTConfig[LddNumChannel];

      LpPBChannelConfig = Spi_GpFirstChannel;

      while(LddNumChannel < SPI_MAX_CHANNEL)
      {
        #if (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
        if((LpPBChannelConfig->ucChannelBufferType) == SPI_ONE)
        #endif
        {
          #if (SPI_EB_CONFIGURED == STD_ON)

          LddNoOfBuffers = LpLTChannelConfig->ddBufferIndex;

          /* Length of external buffer in RAM area of EB */
          Spi_GaaChannelEBData[LddNoOfBuffers].ddEBLength =
                                         (LpLTChannelConfig->ddNoOfBuffers);

          /* Copy the source pointer to RAM area allocated for the external
             buffer attributes of the channel */
          Spi_GaaChannelEBData[LddNoOfBuffers].pSrcPtr = NULL_PTR;
          /* Copy the destination pointer value to a local pointer
             variable */
          Spi_GaaChannelEBData[LddNoOfBuffers].pDestPtr = NULL_PTR;

          #endif /* (SPI_EB_CONFIGURED == STD_ON) */
        }
        #if (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
        else
        #endif
        {
          #if (SPI_IB_CONFIGURED == STD_ON)
          Spi_InternalWriteIB(LddNumChannel, NULL_PTR);
          #endif /* (SPI_IB_CONFIGURED == STD_ON) */
        }

        /* Increment the pointers to channel */
        /* MISRA Rule   : 17.4                                       */
        /* Message      : Increment or decrement operation performed */
        /*                on pointer                                 */
        /* Reason       : To access these pointers in optimized      */
        /*                way in this function                       */
        /* Verification : However, part of the code is verified      */
        /*                manually and it is not having any impact   */
        LpLTChannelConfig++;

        LpPBChannelConfig++;
        /* Increment the number of channels */
        LddNumChannel++;
      } /* End of while(LddNumChannel < SPI_MAX_CHANNEL) */

      #if(SPI_LEVEL_DELIVERED == SPI_TWO)
      /*Global variable for asynchronous mode is polling mode */
      Spi_GddAsyncMode = SPI_POLLING_MODE;
      #else
      /* Set default asynchronous transmit mode as interrupt mode */
      Spi_GddAsyncMode = SPI_INTERRUPT_MODE;
      #endif

      #if ((SPI_LEVEL_DELIVERED == SPI_ONE) || (SPI_LEVEL_DELIVERED == SPI_TWO))

      Spi_HWInit();

      #endif

      #if (SPI_CANCEL_API == STD_ON)
      /* Initialize the counter as zero */
      LucVar = SPI_ZERO;

      while(LucVar < SPI_MAX_CANCEL_BYTES)
      {
        /* Initialize the cancel status byte as zero */
        Spi_GaaSeqCancel[LucVar] = SPI_ZERO;
        /* Increment number of cancel status bytes */
        LucVar++;
      }
      #endif /* (SPI_CANCEL_API == STD_ON) */

      #if ((SPI_LEVEL_DELIVERED == SPI_ONE)|| (SPI_LEVEL_DELIVERED == SPI_TWO))
      /* Initialize the counter as zero */
      LucVar = SPI_ZERO;
      LpStatusPtr = Spi_GpConfigPtr->pStatusArray;

      while(LucVar < (Spi_GpConfigPtr->ucNoofStatusByte))
      {
        /* Initialize the status byte as zero */
        *LpStatusPtr = SPI_ZERO;

        /* Increment the pointer to status byte */
        /* MISRA Rule   : 17.4                                       */
        /* Message      : Increment or decrement operation performed */
        /*                on pointer                                 */
        /* Reason       : To access these pointers in optimized      */
        /*                way in this function                       */
        /* Verification : However, part of the code is verified      */
        /*                manually and it is not having any impact   */
        LpStatusPtr++;

        /* Decrement the counter */
        LucVar++;
      }

      /* Initialize the queue elements with zero */
      /* Initialize the counter as zero */
      LucVar = SPI_ZERO;

      while(LucVar < SPI_MAX_QUEUE)
      {
        Spi_GblQueueStatus[LucVar] = SPI_QUEUE_EMPTY;
        Spi_GddQueueIndex[LucVar] = SPI_ZERO;

        /* Increment number of jobs configured */
        LucVar++;
      }
      #endif

      #if (SPI_LEVEL_DELIVERED == SPI_TWO)
      Spi_GblSyncTx = SPI_FALSE;
      #endif

      #if (SPI_DIRECT_ACCESS_MODE == STD_ON)
      Spi_GblTxEDL = SPI_FALSE;
      Spi_GblRxEDL = SPI_FALSE;
      #endif

      Spi_GucAllQueueSts = SPI_ZERO;
      /* Global Status variable is SPI_IDLE */
      Spi_GddDriverStatus = SPI_IDLE;
    } /* Else of ConfigPtr->ulStartOfDbToc != SPI_DBTOC_VALUE */

  }
}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

/*******************************************************************************
* Function Name          : Spi_DeInit
*
* Service ID             : 0x01
*
* Description            : This service is used for de-initialization of this
*                          module.
*
* Sync/Async             : Synchronous
*
* Re-entrancy            : Non-Reentrant
*
* Input Parameters       : None
*
* InOut Parameters       : None
*
* Output Parameters      : None
*
* Return parameter       : Std_ReturnType (E_OK/E_NOT_OK)
*
* Preconditions          : None
*
* Remarks                : Global Variable:
*                          Spi_GddDriverStatus
*
*
*                           Function invoked:
*                           Det_ReportError
*                           Spi_HWDeInit
*
*******************************************************************************/

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(Std_ReturnType, SPI_PUBLIC_CODE) Spi_DeInit(void)
{
  Std_ReturnType LenReturnValue;

  LenReturnValue = E_NOT_OK;

  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  /* Check if SPI Driver is initialized */
  if(Spi_GddDriverStatus == SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                     SPI_DEINIT_SID, SPI_E_UNINIT);
  }

  else
  #endif
  {
    /* Check if Global status variable is SPI_BUSY */
    if(Spi_GddDriverStatus != SPI_BUSY)
    {
      Spi_HWDeInit();
      LenReturnValue = E_OK;

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Disable relevant interrupts to protect this critical section */
      SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
      #endif

      /* Update the SPI driver status as uninitialized */
      Spi_GddDriverStatus = SPI_UNINIT;

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Enable relevant interrupts to protect this critical section */
      SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
      #endif
    }
  }

  /* Return the value */
  return(LenReturnValue);
}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */


/*******************************************************************************
* Function Name             : Spi_WriteIB
*
* Service ID                : 0x02
*
* Description               : This service for writing one or more data to an
*                             IB SPI Handler/Driver channel specified
*                             by parameter.
*
* Sync/Async                : Synchronous
*
* Re-entrancy               : Reentrant
*
* Input Parameters          : Channel - Channel ID
*                             DataBufferPtr - Pointer to source data buffer
*
* InOut Parameters          : None
*
* Output Parameters         : None
*
* Return parameter          : Std_ReturnType (E_OK/E_NOT_OK)
*
* Preconditions             : Spi_Init should have been invoked.
*
* Remarks                   : Global Variable:
*                             Spi_GddDriverStatus
*                             Spi_GstChannelLTConfig
*                             Spi_GaaChannelIBWrite
*                             Spi_GpFirstChannel
*
*                             Function invoked:
*                             Det_ReportError
*
*******************************************************************************/
#if ((SPI_CHANNEL_BUFFERS_ALLOWED == SPI_ZERO) || \
     (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO))

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(Std_ReturnType, SPI_PUBLIC_CODE) Spi_WriteIB(Spi_ChannelType Channel,
           P2CONST(Spi_DataType, AUTOMATIC, SPI_APPL_CONST) DataBufferPtr)
{
  #if ((SPI_DEV_ERROR_DETECT == STD_ON) && \
                        (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO))
  P2CONST(Tdd_Spi_ChannelPBConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpPBChannelConfig;
  #endif
  Std_ReturnType LenReturnValue = E_OK;

  #if (SPI_DEV_ERROR_DETECT == STD_ON)

  /* Check if SPI Driver is initialized */
  if(Spi_GddDriverStatus == SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID, SPI_WRITEIB_SID,
                                                          SPI_E_UNINIT);
    LenReturnValue = E_NOT_OK;
  }
  else
  {
    /* No action required */
  }

  /* Check if the channel ID passed, is valid */
  if(Channel >= SPI_MAX_CHANNEL)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                         SPI_WRITEIB_SID, SPI_E_PARAM_CHANNEL);
    LenReturnValue = E_NOT_OK;
  }
  else
  {
    /* No action required */
  }


  #if (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
  /* Check for no DET errors */
  if (LenReturnValue == E_OK)
  {
    /* Get the pointer to the post-build structure of the requested channel */

    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */
    LpPBChannelConfig = Spi_GpFirstChannel + Channel;

    /* Check if the channel is configured with external buffer */
    if((LpPBChannelConfig->ucChannelBufferType) == SPI_ONE)
    {
      /* Report to DET */
      Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                          SPI_WRITEIB_SID, SPI_E_PARAM_CHANNEL);
      LenReturnValue = E_NOT_OK;
    }
    else
    {
      /* No action required */
    }

  }
  #endif /* End of (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO) */

  /* Check if any DET error has occurred */
  if(LenReturnValue == E_OK)
  #endif /* End of SPI_DEV_ERROR_DETECT == STD_ON */
  {
    #if (SPI_IB_CONFIGURED == STD_ON)
    Spi_InternalWriteIB(Channel, DataBufferPtr);
    #endif /* End of (SPI_IB_CONFIGURED == STD_ON) */
  } /* End of all operations if there is no DET error */
  return(LenReturnValue);
}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of ((SPI_CHANNEL_BUFFERS_ALLOWED == SPI_ZERO) ||
            (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)) */

/*******************************************************************************
* Function Name            : Spi_AsyncTransmit
*
* Service ID               : 0x03
*
* Description              : This service for transmitting data asynchronously

* Sync/Async               : Asynchronous.
*
* Re-entrancy              : Reentrant
*
* Input Parameters         : Sequence - Sequence ID
*
* InOut Parameters         : None
*
* Output Parameters        : None
*
* Return parameter         : Std_ReturnType (E_OK/E_NOT_OK)
*
* Preconditions            : Spi_Init should have been invoked.
*
* Remarks                  : Global Variable:
*                            Spi_GddDriverStatus
*                            Spi_GaaSeqResult
*                            Spi_GpFirstSeq
*
*                            Function invoked:
*                            Det_ReportError
*                            Spi_Scheduler
*                            Spi_HWInitiateTx
*
*******************************************************************************/

#if (SPI_LEVEL_DELIVERED == SPI_ONE || SPI_LEVEL_DELIVERED == SPI_TWO)

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(Std_ReturnType, SPI_PUBLIC_CODE) Spi_AsyncTransmit
                                               (Spi_SequenceType Sequence)
{
  P2CONST(Tdd_Spi_JobListType,AUTOMATIC,SPI_CONFIG_CONST) LpJobList;

  P2CONST(Tdd_Spi_SequenceConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpSeqConfig;
  #if (SPI_CSIH_CONFIGURED == STD_ON || (SPI_DEV_ERROR_DETECT == STD_ON \
        && SPI_LEVEL_DELIVERED == SPI_TWO))
  P2CONST(Tdd_Spi_JobConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpJobConfig;
  #endif
  P2CONST(uint8,AUTOMATIC,SPI_CONFIG_CONST) LpStsByteValue;
  P2VAR(uint8,AUTOMATIC,SPI_CONFIG_DATA) LpStsByte;

  Std_ReturnType LenReturnValue;
  #if (SPI_CSIH_CONFIGURED == STD_ON)
  Spi_HWUnitType LddHWUnit;
  #endif
  Spi_JobType LddJobListIndex;
  Spi_JobType LddReqJobListIndex;
  Spi_JobType LddNoOfJobs;
  Spi_JobType LddLowestQueueIndex;

  #if ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON))
  Spi_JobType LddCntrForJobsReq;
  Spi_JobType LddData;
  #endif

  uint8 LucHWMemoryMode;

  #if ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON))
  Spi_SequenceType LddSeqCounterBottom;
  #endif

  uint8 LucVar;
  uint8 LucMask;

  LenReturnValue = E_OK;

  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  /* Check if SPI Driver is initialized */
  if(Spi_GddDriverStatus == SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                     SPI_ASYNCTRANSMIT_SID, SPI_E_UNINIT);
    LenReturnValue = E_NOT_OK;
  }
  else
  {
    /* No action required */
  }

  /* Check if the sequence ID passed, is valid */
  if(Sequence >= SPI_MAX_SEQUENCE)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                     SPI_ASYNCTRANSMIT_SID, SPI_E_PARAM_SEQ);
    LenReturnValue = E_NOT_OK;
  }
  else
  {
    /* No action required */
  }

  /* Check if any DET error has occurred to check other DET errors */
  if(LenReturnValue == E_OK)
  {
    /* Get the pointer to the sequence structure */
    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */
    LpSeqConfig = Spi_GpFirstSeq + Sequence;

    /* Get the job list index of the last job of the sequence */
    LddJobListIndex = LpSeqConfig->ddJobListIndex;

    /* Get the pointer to the sequence structure */
    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */

    /* Get the pointer to the job list */
    LpJobList = Spi_GpFirstJobList + LddJobListIndex;

    #if ((SPI_LEVEL_DELIVERED == SPI_TWO) || (SPI_CSIH_CONFIGURED == STD_ON))

    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */

    /* Get the pointer of the last job linked to this sequence */
    LpJobConfig = Spi_GpFirstJob + (LpJobList->ddJobIndex);
    #endif

    /* Check if the requested sequence already pending */
    if(Spi_GaaSeqResult[Sequence] == SPI_SEQ_PENDING)
    {
      /* Report to DET */
      Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                    SPI_ASYNCTRANSMIT_SID, SPI_E_SEQ_PENDING);
      LenReturnValue = E_NOT_OK;
    }
    else
    {
      /* No action required */
    }

    #if (SPI_LEVEL_DELIVERED == SPI_TWO)
      /* Check if the HW Unit of the job is configured for synchronous */
      /* transmission                                                  */


    if((LpJobConfig->ddHWUnitIndex) == SPI_HW_UNIT_SYNCHRONOUS)
      {
        /* Report to DET */
        Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                 SPI_ASYNCTRANSMIT_SID, SPI_E_PARAM_SEQ);
        LenReturnValue = E_NOT_OK;
      }
   else
      {
        /* No action required */
      }

    #endif
  }
  /* Check if any DET error has occurred */
  if(LenReturnValue == E_OK)
  #endif /* SPI_DEV_ERROR_DETECT == STD_ON */
  {
    #if (SPI_DEV_ERROR_DETECT == STD_OFF)
    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */
    LpSeqConfig = Spi_GpFirstSeq + Sequence;
    #endif

    /* Get the status byte mask for the requested sequence */

    /* MISRA Rule         : 9.1                                            */
    /* Message            : The variable '-identifier-' is apparently      */
    /*                      unset at this point.                           */
    /* Reason             : This variable is initialized at two places     */
    /*                      under                                          */
    /*                      different pre-compile options                  */
    /* Verification       : However, it is manually verified that at least */
    /*                      one of the pre-compile options will be ON and  */
    /*                      hence this variable will be always initialized */
    /*                      Hence,this is not having any impact.           */

    LucMask = LpSeqConfig->ucStsByteMask;

    if(LucMask != SPI_ZERO)
    {
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact.   */
      LucVar =
        *((Spi_GpConfigPtr->pStatusArray) + (LpSeqConfig->usStsCheckByteIdx));

      LucVar &= LucMask;
    }
    /* MISRA Rule         : 9.1                                            */
    /* Message            : The variable '-identifier-' is apparently      */
    /*                      unset at this point.                           */
    /* Reason             : This variable is initialized at two places     */
    /*                      under                                          */
    /*                      different pre-compile options                  */
    /* Verification       : However, it is manually verified that at least */
    /*                      one of the pre-compile options will be ON and  */
    /*                      hence this variable will be always initialized */
    /*                      Hence,this is not having any impact.           */

    /* Check if status byte mask is zero. This is to check if the requested
       sequence shares any job with on-going sequence. If status byte mask
       zero, check if any sequence with shared job in progress */
    if((LucMask != SPI_ZERO) && (LucVar != SPI_ZERO))
    {
      #if (SPI_DEV_ERROR_DETECT == STD_ON)
      /* The sequence sharing job with requested sequence is in progress.
         So, report to DET */
      Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID, SPI_ASYNCTRANSMIT_SID,
                                                  SPI_E_SEQ_PENDING);
      #endif
      LenReturnValue = E_NOT_OK;
    }
    else
    {
      /* Since no errors are reported, accept the sequence for transmission */
      /* Update the sequence result as SPI_SEQ_PENDING */
      Spi_GaaSeqResult[Sequence] = SPI_SEQ_PENDING;

      #if(SPI_CANCEL_API == STD_ON)
      /* Get the cancel byte offset for the requested sequence  */
      LucVar = Spi_GstSeqProcess[Sequence].ucCancelOffset;
      /* Get the cancel byte mask */
      LucMask = Spi_GstSeqProcess[Sequence].ucCancelMask;

      /* Reset the cancel bit array for this sequence */
      /* MISRA Rule         : 12.7                                     */
      /* Message            : Bitwise operators shall not be applied   */
      /*                      to operands whose underlying type is     */
      /*                      signed.                                  */
      /* Reason             : Though the bitwise operation is          */
      /*                      performed on unsigned data, this         */
      /*                      warning is generated by the QAC tool     */
      /*                      V6.2.1 as an indirect result of integral */
      /*                      promotion in complex bitwise operations  */
      /* Verification       : However, this part of the code is        */
      /*                      verified manually and it is not having   */
      /*                      any impact.                              */

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Disable relevant interrupts */
      SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
      #endif

      Spi_GaaSeqCancel[LucVar] &= (~LucMask);

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Enable relevant interrupts */
      SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
      #endif
      #endif /* End loop of (SPI_CANCEL_API == STD_ON) */

      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact.   */

      /* Get the pointer to status bytes ROM value for the sequence */
      LpStsByteValue = (Spi_GpConfigPtr->pStsValueArray) +
             (LpSeqConfig->usStsValueStartByteIdx);
      /* Get the number of status byte ROM value */
      LucVar = LpSeqConfig->ucNoOfStsByteValue;
      /* Get the pointer to the start byte of RAM area */
      LpStsByte = (Spi_GpConfigPtr->pStatusArray) +
                       (LpSeqConfig->usStsUpdateStartByteIdx);

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Disable relevant interrupts */
      SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
      #endif

      /* Update driver status as busy */
      Spi_GddDriverStatus = SPI_BUSY;

      /* Since no errors are reported, accept the sequence for transmission */
      /* Update the sequence result as SPI_SEQ_PENDING */
      Spi_GaaSeqResult[Sequence] = SPI_SEQ_PENDING;

      while(LucVar > SPI_ZERO)
      {
        /* Update the status bit of the requested sequence and the sequences
           that have jobs shared with the requested sequence */
        (*LpStsByte) |= *LpStsByteValue;

        /* MISRA Rule   : 17.4                                       */
        /* Message      : Increment or decrement operation performed */
        /*                on pointer                                 */
        /* Reason       : To access these pointers in optimized      */
        /*                way in this function                       */
        /* Verification : However, part of the code is verified      */
        /*                manually and it is not having any impact   */
        LpStsByte++;

        LpStsByteValue++;
        LucVar--;
      }

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Enable relevant interrupts */
      SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
      #endif

      #if (SPI_DEV_ERROR_DETECT == STD_OFF)
      /* Get the job list index of the last job of the sequence */
      LddJobListIndex = LpSeqConfig->ddJobListIndex;

      /* Get the pointer to the job list */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact.   */
      LpJobList = Spi_GpFirstJobList + LddJobListIndex;
      #endif

      #if (SPI_CSIH_CONFIGURED == STD_ON)

      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact.   */

      /* MISRA Rule         : 9.1                                            */
      /* Message            : The variable '-identifier-' is apparently      */
      /*                      unset at this point.                           */
      /* Reason             : This variable is initialized at two places     */
      /*                      under                                          */
      /*                      different pre-compile options                  */
      /* Verification       : However, it is manually verified that at least */
      /*                      one of the pre-compile options will be ON and  */
      /*                      hence this variable will be always initialized */
      /*                      Hence,this is not having any impact.           */

      /* Get the pointer of the last job linked to this sequence */
      LpJobConfig = Spi_GpFirstJob + (LpJobList->ddJobIndex);
      #endif

      /* Get the number of jobs configured for the requested sequence */
      LddNoOfJobs = LpSeqConfig->ddNoOfJobs;

      /* Get the index of the job list for the first job of the sequence */

      /* MISRA Rule         : 9.1                                            */
      /* Message            : The variable '-identifier-' is apparently      */
      /*                      unset at this point.                           */
      /* Reason             : This variable is initialized at two places     */
      /*                      under                                          */
      /*                      different pre-compile options                  */
      /* Verification       : However, it is manually verified that at least */
      /*                      one of the pre-compile options will be ON and  */
      /*                      hence this variable will be always initialized */
      /*                      Hence,this is not having any impact.           */
      LddReqJobListIndex = LddJobListIndex + (LddNoOfJobs - SPI_ONE);

      #if (SPI_CSIH_CONFIGURED == STD_ON)

      /* Get the HW Unit index of the any of the job in the sequence */

      /* MISRA Rule         : 9.1                                            */
      /* Message            : The variable '-identifier-' is apparently      */
      /*                      unset at this point.                           */
      /* Reason             : This variable is initialized at two places     */
      /*                      under                                          */
      /*                      different pre-compile options                  */
      /* Verification       : However, it is manually verified that at least */
      /*                      one of the pre-compile options will be ON and  */
      /*                      hence this variable will be always initialized */
      /*                      Hence,this is not having any impact.           */
      LddHWUnit = LpJobConfig->ddHWUnitIndex;

      /* Check if the HW Unit is CSIH */
      if(LddHWUnit >= SPI_MAX_NUM_OF_CSIG)
      {
        /* Get the configured memeory mode for this HW Unit */
        LucHWMemoryMode =
             Spi_GpConfigPtr->aaHWMemoryMode[LddHWUnit - SPI_MAX_NUM_OF_CSIG];
      }
      else
      #endif
      {
        /* Since HW Unit is CSIG, memory mode is DIRECT ACCESS by default */
        LucHWMemoryMode = SPI_DIRECT_ACCESS_MODE_CONFIGURED;
      }

      #if (((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON)) \
        && ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON)))
      if(LucHWMemoryMode < SPI_TWO)
      #endif
      {
        #if ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON))
        LddLowestQueueIndex = SPI_ZERO;

        /* MISRA Rule         : 13.7                                          */
        /* Message            : The result of this logical operation is       */
        /*                      always false.                                 */
        /* Reason             : Logical operation performed to check the mode */
        /*                      of the Flash Wrapper                          */
        /* Verification       : However, part of the code is verified manually*/
        /*                      and it is not having any impact.              */

        /* MISRA Rule         : 14.1                                          */
        /* Message            : There shall be no unreachable code.           */
        /* Reason             : The condition will be satisfied in some other */
        /*                      configuration.                                */
        /* Verification       : However, part of the code is verified manually*/
        /*                      and it is not having any impact.              */


        if(LucHWMemoryMode == SPI_FIFO_MODE_CONFIGURED)
        {
          LddLowestQueueIndex = Spi_GpConfigPtr->ddDirectAccessQueueSize;
        }

        /* Copy a counter with number of jobs in the requested sequence */
        LddCntrForJobsReq = LddNoOfJobs;

        /* MISRA Rule         : 21.1                                */
        /* Message            : Indexing array with value that will */
        /*                      apparently be out of bounds.        */
        /* Reason             : It is used for array indexing       */
        /* Verification       : However, part of the code           */
        /*                      is verified manually and            */
        /*                      it is not having any impact         */

        /* Check if the job queue is empty */
        if((Spi_GblQueueStatus[LucHWMemoryMode] == SPI_QUEUE_EMPTY) ||
               ((Spi_GblQueueStatus[LucHWMemoryMode] == SPI_QUEUE_FILLED) &&
                  (Spi_GddQueueIndex[LucHWMemoryMode] == LddLowestQueueIndex)))
        {
           #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
           /* Disable relevant interrupts */
           SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
           #endif

          Spi_GddQueueIndex[LucHWMemoryMode] = LddLowestQueueIndex;

          if(Spi_GblQueueStatus[LucHWMemoryMode] != SPI_QUEUE_EMPTY)
          {
             LddData = Spi_GaaJobQueue[Spi_GddQueueIndex[LucHWMemoryMode]];
          }
          do
          {
            /* MISRA Rule    : 9.1                                            */
            /* Message       : The variable '-identifier-' is apparently      */
            /*                 unset at this point.                           */
            /* Reason        : This variable is initialized at two places     */
            /*                 under                                          */
            /*                 different pre-compile options                  */
            /* Verification  : However, it is manually verified that at least */
            /*                 one of the pre-compile options will be ON and  */
            /*                 hence this variable will be always initialized */
            /*                 Hence,this is not having any impact.           */
            /* Push the job list index into the queue */
            Spi_GaaJobQueue[Spi_GddQueueIndex[LucHWMemoryMode]] =
                                                            LddJobListIndex;

           /* Push the job count into RAM */
            Spi_GaaJobCount[LpJobList->ddJobIndex] = LpJobList->ucJobCount;
            /* Increment the job list index */
            LddJobListIndex++;

            /* MISRA Rule   : 17.4                                       */
            /* Message      : Increment or decrement operation performed */
            /*                on pointer                                 */
            /* Reason       : To access these pointers in optimized      */
            /*                way in this function                       */
            /* Verification : However, part of the code is verified      */
            /*                manually and it is not having any impact   */
            LpJobList++;

            /* Increment the queue index */
            Spi_GddQueueIndex[LucHWMemoryMode]++;
            /* Decrement the number of jobs */
            LddCntrForJobsReq--;
          }while(LddCntrForJobsReq > SPI_ZERO);

            /* MISRA Rule         : 21.1                                */
            /* Message            : Indexing array with value that will */
            /*                      apparently be out of bounds.        */
            /* Reason             : It is used for array indexing       */
            /* Verification       : However, part of the code           */
            /*                      is verified manually and            */
            /*                      it is not having any impact         */
            /* Check if critical section protection is required */
            #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
            /* Enable relevant interrupts */
            SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
            #endif

          if(Spi_GblQueueStatus[LucHWMemoryMode] != SPI_QUEUE_EMPTY)
          {
            /* MISRA Rule    : 9.1                                            */
            /* Message       : The variable '-identifier-' is apparently      */
            /*                 unset at this point.                           */
            /* Reason        : This variable is initialized at two places     */
            /*                 under                                          */
            /*                 different pre-compile options                  */
            /* Verification  : However, it is manually verified that at least */
            /*                 one of the pre-compile options will be ON and  */
            /*                 hence this variable will be always initialized */
            /*                 Hence,this is not having any impact.           */
            #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
            /* Disable relevant interrupts */
            SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
            #endif

            Spi_GaaJobQueue[Spi_GddQueueIndex[LucHWMemoryMode]] = LddData;

            #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
            /* Enable relevant interrupts */
            SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
            #endif
          }
          else
          {
            #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
            /* Disable relevant interrupts */
            SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
            #endif

            Spi_GblQueueStatus[LucHWMemoryMode] = SPI_QUEUE_FILLED;

            /* MISRA Rule         : 12.7                                     */
            /* Message            : Bitwise operators shall not be applied   */
            /*                      to operands whose underlying type is     */
            /*                      signed.                                  */
            /* Reason             : Though the bitwise operation is          */
            /*                      performed on unsigned data, this         */
            /*                      warning is generated by the QAC tool     */
            /*                      V6.2.1 as an indirect result of integral */
            /*                      promotion in complex bitwise operations  */
            /* Verification       : However, this part of the code is        */
            /*                      verified manually and it is not having   */
            /*                      any impact.                              */
            Spi_GucAllQueueSts |= (SPI_ONE<<LucHWMemoryMode);

            /* MISRA Rule         : 21.1                                */
            /* Message            : Indexing array with value that will */
            /*                      apparently be out of bounds.        */
            /* Reason             : It is used for array indexing       */
            /* Verification       : However, part of the code           */
            /*                      is verified manually and            */
            /*                      it is not having any impact         */

            /* Point to the index of the last job pushed to the queue */
            Spi_GddQueueIndex[LucHWMemoryMode]--;

            #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
            /* Enable relevant interrupts */
            SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
            #endif
            /* Initiate the transmission for that sequence */
            Spi_HWInitiateTx(LddReqJobListIndex);
          }
        } /* End of queue is empty */
        else
        {
          Spi_PushToQueue(Sequence, LucHWMemoryMode);
        } /* End of Queue is not empty */

        #endif
      }
      #if (((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON)) \
         && ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON)))
      else
      #endif
      {
        #if ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON))

        /* Check the queue for the dual buffer mode is empty */
        if(Spi_GblQueueStatus[LucHWMemoryMode] == SPI_QUEUE_EMPTY)
        {
          Spi_GddQueueIndex[LucHWMemoryMode] = SPI_ZERO;

          if((LucHWMemoryMode - SPI_TWO) != SPI_ZERO)
          {
            Spi_GddQueueIndex[LucHWMemoryMode]
                      = Spi_GpConfigPtr->ddDualBufferQueueSize;
          }

          /* Change the queue status as FILLED */
          Spi_GblQueueStatus[LucHWMemoryMode] = SPI_QUEUE_FILLED;

          /* MISRA Rule         : 12.7                                     */
          /* Message            : Bitwise operators shall not be applied   */
          /*                      to operands whose underlying type is     */
          /*                      signed.                                  */
          /* Reason             : Though the bitwise operation is          */
          /*                      performed on unsigned data, this         */
          /*                      warning is generated by the QAC tool     */
          /*                      V6.2.1 as an indirect result of integral */
          /*                      promotion in complex bitwise operations  */
          /* Verification       : However, this part of the code is        */
          /*                      verified manually and it is not having   */
          /*                      any impact.                              */
          Spi_GucAllQueueSts |= (SPI_ONE<<LucHWMemoryMode);

          /* Put the sequence in the queue */
          Spi_GaaSeqQueue[Spi_GddQueueIndex[LucHWMemoryMode]] = Sequence;

          /* Initiate the transmission for that sequence */
          Spi_HWInitiateTx(LddReqJobListIndex);
        }
        else
        {
          LddLowestQueueIndex = SPI_ZERO;

          if(LucHWMemoryMode == SPI_TX_ONLY_MODE_CONFIGURED)
          {
            LddLowestQueueIndex = Spi_GpConfigPtr->ddDualBufferQueueSize;
          }

          /* Check if critical section protection is required */
          #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
          /* Disable relevant interrupts to protect this critical */
          /* section */
          SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
          #endif
          /* Place the sequence at the bottom of the queue by shifting */
          /* the sequences already in the queue by one position        */
          LddSeqCounterBottom = Spi_GddQueueIndex[LucHWMemoryMode] + SPI_ONE;
          /* Load back the queue index to the bottom most element */
          Spi_GddQueueIndex[LucHWMemoryMode] = LddSeqCounterBottom;
          do
          {
            /* Move the sequence entry one up in the queue */
            Spi_GaaSeqQueue[LddSeqCounterBottom] =
                           Spi_GaaSeqQueue[LddSeqCounterBottom - SPI_ONE];
            /* Decrement the pointer */
            LddSeqCounterBottom--;
          }while(LddSeqCounterBottom > LddLowestQueueIndex);

          /* Place the requested sequence index at the top */
          Spi_GaaSeqQueue[LddSeqCounterBottom] = Sequence;
          /* Check if critical section protection is required */
          #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
          /* Disable relevant interrupts to protect this critical section */
          SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
          #endif
        }
        #endif
      } /* End of else loop of checking if the job queue is empty */
    } /* End of else loop of checking for sequences sharing jobs */
  } /* End of the check if any DET error has occurred */
  return(LenReturnValue);
}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of (SPI_LEVEL_DELIVERED == SPI_ONE
               || SPI_LEVEL_DELIVERED == SPI_TWO) */

/*******************************************************************************
* Function Name            : Spi_ReadIB
*
* Service ID               : 0x04
*
* Description              : This service for reading one or more data from an
*                            IB SPI Handler/Driver channel specified by
*                            parameter

* Sync/Async               : Synchronous
*
* Re-entrancy              : Reentrant
*
* Input Parameters         : Channel - Channel ID
*                            DataBufferPointer - Pointer to destination data
*                            buffer
*
* InOut Parameters         : None
*
* Output Parameters        : None
*
* Return parameter         : Std_ReturnType (E_OK/E_NOT_OK)
*
* Preconditions            : Spi_Init should have been invoked.
*
* Remarks                  : Global Variable:
*                            Spi_GddDriverStatus
*                            Spi_GstChannelLTConfig
*                            Spi_GaaChannelIBRead
*
*                            Function invoked:
*                            Det_ReportError
*
*******************************************************************************/
#if ((SPI_CHANNEL_BUFFERS_ALLOWED == SPI_ZERO) || \
     (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO))

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

/* MISRA Rule         : 16.7                                                  */
/* Message            : A pointer parameter in a function prototype should be */
/*                      declared as pointer to const if the pointer is not    */
/*                      used to modify the addressed object.                  */
/* Reason             : The received data is not constant always and hence the*/
/*                      pointer is declared as pointer to variable.           */
/* Verification       : However, part of the code is verified manually and    */
/*                      it is not having any impact.                          */

FUNC(Std_ReturnType, SPI_PUBLIC_CODE) Spi_ReadIB
               (Spi_ChannelType Channel, P2VAR(Spi_DataType, AUTOMATIC,
                                          SPI_APPL_DATA) DataBufferPointer)

{
  P2CONST(Tdd_Spi_ChannelPBConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpPBChannelConfig;

  #if(((SPI_INTERNAL_RW_BUFFERS == STD_ON) && \
      ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON) || \
       (SPI_LEVEL_DELIVERED == SPI_ZERO))) || (SPI_TX_ONLY_MODE == STD_ON))
  P2CONST(Tdd_Spi_ChannelLTConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpLTChannelConfig;
  P2VAR(Spi_DataType, AUTOMATIC, SPI_CONFIG_DATA) LpDesPtr;
  Spi_NumberOfDataType LddNoOfBuffers;
  #endif

  #if((SPI_INTERNAL_RW_BUFFERS == STD_ON) && \
     ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON) || \
      (SPI_LEVEL_DELIVERED == SPI_ZERO)))
  P2VAR(Spi_DataType,AUTOMATIC,SPI_CONFIG_DATA) LpChannelIB;
  #endif


  #if (SPI_TX_ONLY_MODE == STD_ON)
  P2VAR(uint16 ,AUTOMATIC, SPI_CONFIG_DATA) LpTxOnlyData;

  #if((SPI_8BIT_DATA_WIDTH == STD_OFF) && (SPI_16BIT_DATA_WIDTH == STD_OFF))
  /*  MISRA Rule         : 18.4                                           */
  /*  Message            : Unions shall not be used.                      */
  /*  Reason             : To access the converted values                 */
  /*  Verification       : However, part of the code is verified manually */
  /*                       and it is not having any impact.               */
  Tun_Spi_DataAccess LunDataAccess1;
  #endif

  Spi_DataType LddData;
  #endif

  #if(((SPI_INTERNAL_RW_BUFFERS == STD_ON) && \
      ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON) || \
      (SPI_LEVEL_DELIVERED == SPI_ZERO))) || \
       (((SPI_CSIH_CONFIGURED == STD_ON)  && (SPI_LEVEL_DELIVERED != SPI_ZERO))\
        &&((SPI_TX_ONLY_MODE == STD_ON) || (SPI_DUAL_BUFFER_MODE == STD_ON))))
  uint8 LucChannelBufferType;
  #endif

  Std_ReturnType LenReturnValue = E_OK;

  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  /* Check if SPI Driver is initialized */
  if(Spi_GddDriverStatus == SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID, SPI_READIB_SID,
                                                         SPI_E_UNINIT);
    LenReturnValue = E_NOT_OK;
  }
  else
  {
    /* No action required */
  }


  /* Check if the channel ID passed, is valid */
  if(Channel >= SPI_MAX_CHANNEL)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                         SPI_READIB_SID, SPI_E_PARAM_CHANNEL);
    LenReturnValue = E_NOT_OK;
  }
  else
  {
    /* No action required */
  }


  /* Check if the data buffer pointer passed, is NULL pointer */
  if(DataBufferPointer == NULL_PTR)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                         SPI_READIB_SID, SPI_E_PARAM_POINTER);
    LenReturnValue = E_NOT_OK;
  }
  else
  {
    /* No action required */
  }


  #if (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
  /* Check for no DET errors */
  if (LenReturnValue == E_OK)
  {
    /* Get the pointer to the post-build structure of the requested channel */

    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */
    LpPBChannelConfig = Spi_GpFirstChannel + Channel;

    /* Check if the channel is configured with external buffer */
    if((LpPBChannelConfig->ucChannelBufferType) == SPI_ONE)
    {
      /* Report to DET */
      Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                          SPI_READIB_SID, SPI_E_PARAM_CHANNEL);
      LenReturnValue = E_NOT_OK;
    }
    else
    {
      /* No action required */
    }

  }
  #endif /* End of (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO) */

  /* Check if any DET error has occurred */
  if(LenReturnValue == E_OK)
  #endif /* End of SPI_DEV_ERROR_DETECT == STD_ON */
  {
    /* Get the pointer to the post-build structure of the channel */
    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */

    LpPBChannelConfig = Spi_GpFirstChannel + Channel;


    #if(((SPI_INTERNAL_RW_BUFFERS == STD_ON) && \
        ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON) || \
        (SPI_LEVEL_DELIVERED == SPI_ZERO))) || \
       (((SPI_CSIH_CONFIGURED == STD_ON)  && (SPI_LEVEL_DELIVERED != SPI_ZERO))\
        &&((SPI_TX_ONLY_MODE == STD_ON) || (SPI_DUAL_BUFFER_MODE == STD_ON))))

    /* Get the the type of the channel */
    LucChannelBufferType = LpPBChannelConfig->ucChannelBufferType;
    #endif

    #if(((SPI_INTERNAL_RW_BUFFERS == STD_ON) && \
        ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON) || \
         (SPI_LEVEL_DELIVERED == SPI_ZERO))) || (SPI_TX_ONLY_MODE == STD_ON))
    /* Get the pointer to the link-time structure of the channel */
    LpLTChannelConfig = &Spi_GstChannelLTConfig[Channel];
    /* Copy the destination pointer value to a local pointer variable */
    LpDesPtr = DataBufferPointer;
    /* Get the number of buffers configured for the requested channel */
    LddNoOfBuffers = LpLTChannelConfig->ddNoOfBuffers;
    #endif

    #if((SPI_INTERNAL_RW_BUFFERS == STD_ON) && \
       ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON) || \
        (SPI_LEVEL_DELIVERED == SPI_ZERO)))
    /* Check if the buffer type is internal buffer and not hardware buffer */
    if(LucChannelBufferType < SPI_TWO)
    {
      /* Get the pointer to the internal buffer of the requested channel */
      LpChannelIB = &Spi_GaaChannelIBRead[LpLTChannelConfig->ddBufferIndex];

      do
      {
        /* Copy the data from the internal buffer to the destination data
           buffer */
        /* MISRA Rule     : 1.2                                            */
        /* Message        : Dereferencing pointer value that is apparently */
        /*                  NULL.                                          */
        /* Reason         : This is the pointer passed to this function    */
        /* Verification   : However, this is validated when DET is enabled.*/

        *LpDesPtr = *LpChannelIB;

        /* Increment the internal buffer pointer */
        /* MISRA Rule     : 17.4                                           */
        /* Message        : Increment or decrement operation performed on  */
        /*                : pointer.                                       */
        /* Reason         : To access these pointers in optimized          */
        /*                  way in this function.                          */
        /* Verification   : However, part of the code is verified manually */
        /*                  and it is not having any impact.               */
        LpChannelIB++;

        /* Increment the destination pointer */
        LpDesPtr++;
        /* Decrement the counter for number of buffers */
        LddNoOfBuffers--;
      } while(LddNoOfBuffers > SPI_ZERO);
    } /* End of operations if the buffer type is 'internal buffer' */
    else
    #endif
    {
      #if ((SPI_CSIH_CONFIGURED == STD_ON) && (SPI_LEVEL_DELIVERED != SPI_ZERO))
      #if ((SPI_TX_ONLY_MODE == STD_ON) || (SPI_DUAL_BUFFER_MODE == STD_ON))
      /* The value of ucChannelBufferType should be more than four when the  */
      /* CSIH channel configured as SPI_TX_ONLY_MODE or SPI_DUAL_BUFFER_MODE */
      if((Spi_GpConfigPtr->aaHWMemoryMode[LucChannelBufferType - SPI_FOUR])
                                        == SPI_TX_ONLY_MODE_CONFIGURED)
      {
        #if (SPI_TX_ONLY_MODE == STD_ON)
        /* Get the pointer to the internal buffer of the requested channel */
        LpTxOnlyData = &Spi_GaaTxOnlyRead[LpLTChannelConfig->ddBufferIndex];

        do
        {
          #if(SPI_8BIT_DATA_WIDTH == STD_ON)
          /* Data width is maximum 8-bit. Hence, Receive the data from the */
          /* Rx register to local union variable */
          LddData = (uint8)(*LpTxOnlyData);

          #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
          LddData = *LpTxOnlyData;

          #else
          /* Data width is maximum 32-bit, check if the the data width of */
          /* requested channel is more than 16 bits */
          LddData = *LpTxOnlyData;

          if(LpPBChannelConfig->blEDLEnabled == SPI_TRUE)
          {
            /* MISRA Rule   : 17.4                                       */
            /* Message      : Increment or decrement operation performed */
            /*                on pointer                                 */
            /* Reason       : To access these pointers in optimized      */
            /*                way in this function                       */
            /* Verification : However, part of the code is verified      */
            /*                manually and it is not having any impact   */
            LpTxOnlyData++;

            LddNoOfBuffers--;

            /* Check if the configured data direction is LSB first */
            if(LpPBChannelConfig->blDirection == SPI_TRUE)
            {
              /* Take a local union variable to construct the value from RX0W */
              /* register */
              LunDataAccess1.usRegData5[0] = (uint16)LddData;
              LunDataAccess1.usRegData5[1] = *LpTxOnlyData;
            }
            else
            {
              /* Take a local union variable to construct the value from RX0W */
              /* register */
              LunDataAccess1.usRegData5[1] = (uint16)LddData;
              LunDataAccess1.usRegData5[0] = *LpTxOnlyData;


              LunDataAccess1.usRegData5[0] = LunDataAccess1.usRegData5[0] << \
                            (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
              LunDataAccess1.ulRegData = LunDataAccess1.ulRegData >> \
                         (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));


            } /* End of if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE) */
            LddData = LunDataAccess1.ulRegData;
          }
          #endif /* End of (SPI_8BIT_DATA_WIDTH == STD_ON)*/

          /* MISRA Rule     : 1.2                                            */
          /* Message        : Dereferencing pointer value that is apparently */
          /*                  NULL.                                          */
          /* Reason         : This is the pointer passed to this function    */
          /* Verification   : However, this is validated when DET is enabled.*/
          *LpDesPtr = LddData;

          /* Increment the internal buffer pointer */

          /* MISRA Rule     : 17.4                                           */
          /* Message        : Increment or decrement operation performed on  */
          /*                : pointer.                                       */
          /* Reason         : To access these pointers in optimized          */
          /*                  way in this function.                          */
          /* Verification   : However, part of the code is verified manually */
          /*                  and it is not having any impact.               */
          LpTxOnlyData++;

          /* Increment the destination pointer */
          LpDesPtr++;
          /* Decrement the counter for number of buffers */
          LddNoOfBuffers--;
        } while(LddNoOfBuffers > SPI_ZERO);
        #endif
      }
      #if (SPI_DUAL_BUFFER_MODE == STD_ON)
      else
      {
        /* Memory mode is 'Dual Buffer Mode' */
        Spi_HWReadIB(Channel, DataBufferPointer);
      }
      #endif
      #endif /* End of ((SPI_TX_ONLY_MODE == STD_ON) ||    */
             /*         (SPI_DUAL_BUFFER_MODE == STD_ON))  */
      #endif /* End of ((SPI_CSIH_CONFIGURED == STD_ON) &&
                                      (SPI_LEVEL_DELIVERED != SPI_ZERO))*/
    }
  }
  return(LenReturnValue);
}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of ((SPI_CHANNEL_BUFFERS_ALLOWED == SPI_ZERO) ||
                     (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)) */

/*******************************************************************************
* Function Name            : Spi_SetupEB
*
* Service ID               : 0x05
*
* Description              : This service for setting the buffers and the length
*                            of data for the external buffer of the channel
*                            specified.

* Sync/Async               : Synchronous
*
* Re-entrancy              : Reentrant
*
* Input Parameters         : Channel - Channel ID
*                            SrcDataBufferPtr - Pointer to source data buffer
*                            DesDataBufferPtr - Pointer to destination data
*                            buffer in RAM Length - Length of the data
*
* InOut Parameters         : None
*
* Output Parameters        : None
*
* Return parameter         : Std_ReturnType (E_OK/E_NOT_OK)
*
* Preconditions            : Spi_Init should have been invoked.
*
* Remarks                  : Global Variable:
*                            Spi_GddDriverStatus
*                            Spi_GddDriverStatus
*                            Spi_GpConfigPtr
*                            Spi_GaaChannelEBData
*                            Spi_GstChannelLTConfig
*
*                            Function invoked:
*                            Det_ReportError
*
*******************************************************************************/

#if ((SPI_CHANNEL_BUFFERS_ALLOWED == SPI_ONE) || \
     (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO))

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(Std_ReturnType, SPI_PUBLIC_CODE) Spi_SetupEB
    (Spi_ChannelType Channel,
     P2CONST(Spi_DataType, AUTOMATIC, SPI_APPL_DATA) SrcDataBufferPtr,
     P2VAR(Spi_DataType, AUTOMATIC, SPI_APPL_DATA) DesDataBufferPtr,
     Spi_NumberOfDataType Length)
{
  #if ((SPI_DEV_ERROR_DETECT == STD_ON) || \
        ((SPI_EB_CONFIGURED == STD_ON) && (SPI_DEV_ERROR_DETECT == STD_OFF)))
  P2CONST(Tdd_Spi_ChannelLTConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpLTChannelConfig;
  #endif

  #if((SPI_DEV_ERROR_DETECT == STD_ON) && \
                                 (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO))
  P2CONST(Tdd_Spi_ChannelPBConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpPBChannelConfig;
  #endif
  #if (SPI_EB_CONFIGURED == STD_ON)
  P2VAR(Tdd_Spi_EBData,AUTOMATIC,SPI_CONFIG_DATA) LpChannelEB;
  #endif

  Std_ReturnType LenReturnValue = E_OK;

  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  /* Check if SPI Driver is initialized */
  if(Spi_GddDriverStatus == SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID, SPI_SETUPEB_SID,
                                                            SPI_E_UNINIT);
    LenReturnValue = E_NOT_OK;
  }
  else
  {
    /* No action required */
  }

  /* Check if the channel ID passed, is valid */
  if(Channel >= SPI_MAX_CHANNEL)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                      SPI_SETUPEB_SID, SPI_E_PARAM_CHANNEL);
    LenReturnValue = E_NOT_OK;
  }
  /* If it is valid channel ID, check if the length passed is valid */
  else
  {
    /* Get the pointer to the channel link-time structure */
    LpLTChannelConfig = &Spi_GstChannelLTConfig[Channel];

    /* Check if the 'length' parameter is greater than configured length
       or equal to ZERO */
    if ((Length > (LpLTChannelConfig->ddNoOfBuffers)) || \
                                               (Length == SPI_ZERO))
    {
      /* Report to DET */
      Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                         SPI_SETUPEB_SID, SPI_E_PARAM_LENGTH);
      LenReturnValue = E_NOT_OK;
    }
     else
    {
     /* No action required */
    }

    #if (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
    if(LenReturnValue == E_OK)
    {
      /* Get the pointer to the post-build structure of the requested channel */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact.   */
      LpPBChannelConfig = Spi_GpFirstChannel + Channel;

      /* Check if the requested channel is configured for internal buffer */

      if((LpPBChannelConfig->ucChannelBufferType) != SPI_ONE)
      {
        /* Report to DET */
        Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                          SPI_SETUPEB_SID, SPI_E_PARAM_CHANNEL);
        LenReturnValue = E_NOT_OK;
      }
    }
    #endif /* (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO) */
  }

  /* Check if any DET error has occurred */
  if(LenReturnValue == E_OK)
  #endif /* SPI_DEV_ERROR_DETECT == STD_ON */
  {
    #if (SPI_EB_CONFIGURED == STD_ON)

    #if (SPI_DEV_ERROR_DETECT == STD_OFF)
    /* Get the pointer to the requested structure */
    LpLTChannelConfig = &Spi_GstChannelLTConfig[Channel];
    #endif

    /* Get the pointer to the buffer of the requested channel              */

    /* MISRA Rule         : 9.1                                            */
    /* Message            : The variable '-identifier-' is apparently      */
    /*                      unset at this point.                           */
    /* Reason             : This variable is initialized at two places     */
    /*                      under                                          */
    /*                      different pre-compile options                  */
    /* Verification       : However, it is manually verified that atleast  */
    /*                      one of the pre-compile options will be ON and  */
    /*                      hence this variable will be always initialized */
    /*                      Hence,this is not having any impact.           */

    LpChannelEB = &Spi_GaaChannelEBData[LpLTChannelConfig->ddBufferIndex];
    /* Copy the source pointer to RAM area allocated for the external
       buffer attributes of the channel */
    LpChannelEB->pSrcPtr = SrcDataBufferPtr;
    /* Copy the destination pointer value to a local pointer variable */
    LpChannelEB->pDestPtr = DesDataBufferPtr;
    /* Copy length for that channel */
    LpChannelEB->ddEBLength = Length;

    #endif  /* End of (SPI_EB_CONFIGURED == STD_ON) */
  } /* End of all operations if there is no DET error */
  return(LenReturnValue);
}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of (SPI_CHANNEL_BUFFERS_ALLOWED == SPI_ONE ||
                    SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO) */

/*******************************************************************************
* Function Name            : Spi_GetStatus
*
* Service ID               : 0x06
*
* Description              : This service is for getting the status of SPI
*                            Driver Component

* Sync/Async               : Synchronous
*
* Re-entrancy              : Reentrant
*
* Input Parameters         : None
*
* InOut Parameters         : None
*
* Output Parameters        : None
*
* Return parameter         : Spi_StatusType (SPI_UNINIT/SPI_IDLE/SPI_BUSY)
*
* Preconditions            : None
*
* Remarks                  : Global Variable:
*                            Spi_GddDriverStatus
*
*                            Function Invoked:
*                            None
*
*******************************************************************************/

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(Spi_StatusType, SPI_PUBLIC_CODE) Spi_GetStatus(void)
{
  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  /* Check if SPI Driver is initialized */
  if(Spi_GddDriverStatus == SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID, SPI_GETSTATUS_SID,
                                                             SPI_E_UNINIT);
  }
  else
  {
    /* No action required */
  }
  #endif /* SPI_DEV_ERROR_DETECT == STD_ON */
  return(Spi_GddDriverStatus);
}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

/*******************************************************************************
* Function Name          : Spi_GetJobResult
*
* Service ID             : 0x07
*
* Description            : This service is for getting result of the
*                          specified job

* Sync/Async             : Synchronous
*
* Re-entrancy            : Reentrant
*
* Input Parameters       : Job - Job ID
*
* InOut Parameters       : None
*
* Output Parameters      : None
*
* Return parameter       : Spi_JobResultType
*                          (SPI_JOB_OK/SPI_JOB_PENDING/SPI_JOB_FAILED)
*
* Preconditions          : Spi_Init should have been invoked.
*
* Remarks                : Global Variable:
*                          Spi_GddDriverStatus
*                          Spi_GaaJobResult
*
*                          Function invoked:
*                          Det_ReportError
*
*******************************************************************************/

/* MISRA Rule         : 1.1                                                   */
/* Message            : Number of macros defined within the                   */
/*                      translation exceeds 1024.                             */
/* Reason             : This macro is required for memory section definition. */
/* Verification       : Not necessarily an indication of incorrect code.      */
/*                      However, part of the code is verified manually and    */
/*                      it is not having any impact.                          */

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(Spi_JobResultType, SPI_PUBLIC_CODE)
                       Spi_GetJobResult(Spi_JobType Job)
{
  Spi_JobResultType LddJobResult;

  LddJobResult = SPI_JOB_OK;

  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  /* Check if SPI Driver is initialized */
  if(Spi_GddDriverStatus == SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                     SPI_GETJOBRESULT_SID, SPI_E_UNINIT);
    LddJobResult = SPI_JOB_FAILED;
  }
  else
  {
    /* No action required */
  }

  /* Check if the job ID passed, is valid */
  if(Job >= SPI_MAX_JOB)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                     SPI_GETJOBRESULT_SID, SPI_E_PARAM_JOB);
    LddJobResult = SPI_JOB_FAILED;
  }
  else
  {
    /* No action required */
  }


  if(LddJobResult == SPI_JOB_OK)
  #endif /* SPI_DEV_ERROR_DETECT == STD_ON */
  {
    LddJobResult = Spi_GaaJobResult[Job];
  }
  return(LddJobResult);
}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

/*******************************************************************************
* Function Name         : Spi_GetSequenceResult
*
* Service ID            : 0x08
*
* Description           : This service is for getting result of the specified
*                         sequence

* Sync/Async            : Synchronous
*
* Re-entrancy           : Reentrant
*
* Input Parameters      : Sequence - Sequence ID
*
* InOut Parameters      : None
*
* Output Parameters     : None
*
* Return parameter      : Spi_SeqResultType
*                         (SPI_SEQ_OK/SPI_SEQ_PENDING/
*                          SPI_SEQ_FAILED/SPI_SEQ_CANCELLED)
*
* Preconditions         : Spi_Init should have been invoked.
*
* Remarks               : Global Variable:
*                         Spi_GddDriverStatus
*                         Spi_GaaSeqResult
*
*                         Function invoked:
*                         Det_ReportError
*
*******************************************************************************/

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(Spi_SeqResultType, SPI_PUBLIC_CODE)
                       Spi_GetSequenceResult(Spi_SequenceType Sequence)
{
  Spi_JobResultType LddSeqResult;

  LddSeqResult = SPI_SEQ_OK;

  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  /* Check if SPI Driver is initialized */
  if(Spi_GddDriverStatus == SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                  SPI_GETSEQUENCERESULT_SID, SPI_E_UNINIT);
    LddSeqResult = SPI_SEQ_FAILED;
  }
  else
  {
    /* No action required */
  }

  /* Check if the Sequence ID passed, is valid */
  if(Sequence >= SPI_MAX_SEQUENCE)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                  SPI_GETSEQUENCERESULT_SID, SPI_E_PARAM_SEQ);
    LddSeqResult = SPI_SEQ_FAILED;
  }
  else
  {
    /* No action required */
  }


  if(LddSeqResult == SPI_SEQ_OK)
  #endif /* SPI_DEV_ERROR_DETECT == STD_ON */
  {
    LddSeqResult = Spi_GaaSeqResult[Sequence];
  }
  return(LddSeqResult);
}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

/*******************************************************************************
* Function Name          : Spi_SyncTransmit
*
* Service ID             : 0x0A
*
* Description            : This service is for transmitting data synchronously

* Sync/Async             : Synchronous.
*
* Re-entrancy            : Reentrant
*
* Input Parameters       : Sequence - Sequence ID
*
* InOut Parameters       : None
*
* Output Parameters      : None
*
* Return parameter       : Std_ReturnType (E_OK/E_NOT_OK)
*
* Preconditions          : Spi_Init should have been invoked.
*
* Remarks                : Global Variable:
*                          Spi_GddDriverStatus
*                          Spi_GpFirstSeq
*                          Spi_GaaSeqResult
*                          Spi_GpFirstJob
*                          Spi_GaaJobResult
*
*                          Function invoked:
*                          Det_ReportError
*                          Spi_HWTransmitSyncJob
*
*******************************************************************************/

#if ((SPI_LEVEL_DELIVERED == SPI_ZERO)||(SPI_LEVEL_DELIVERED == SPI_TWO))

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(Std_ReturnType, SPI_PUBLIC_CODE) Spi_SyncTransmit
                                                (Spi_SequenceType Sequence)
{
  P2CONST(Tdd_Spi_SequenceConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpSeqConfig;
  P2CONST(Tdd_Spi_JobConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpJobConfig;
  P2CONST(Tdd_Spi_JobListType,AUTOMATIC,SPI_CONFIG_CONST) LpJobList;
  Spi_JobType LddNoOfJobs;
  Spi_JobType LddJobIndex;
  Spi_JobType LddJobListIndex;

  Std_ReturnType LenReturnValue;
  uint8 LucVar;

  LenReturnValue = E_OK;

  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  /* Check if SPI Driver is initialized */
  if(Spi_GddDriverStatus == SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                     SPI_SYNCTRANSMIT_SID, SPI_E_UNINIT);
    LenReturnValue = E_NOT_OK;
  }
  else
  {
    /* No action required */
  }

  /* Check if the sequence ID passed, is valid */
  if(Sequence >= SPI_MAX_SEQUENCE)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                     SPI_SYNCTRANSMIT_SID, SPI_E_PARAM_SEQ);
    LenReturnValue = E_NOT_OK;
  }
  else
  {
    /* No action required */
  }


  if(LenReturnValue == E_OK)
  {
    /* Get the pointer to the sequence structure */
    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */
    LpSeqConfig = Spi_GpFirstSeq + Sequence;

    LpJobList = Spi_GpFirstJobList + (LpSeqConfig->ddJobListIndex);
    LddJobIndex = LpJobList->ddJobIndex;

    /* Get the pointer of the first job linked to this sequence */
    LpJobConfig = Spi_GpFirstJob + LddJobIndex;

    #if (SPI_LEVEL_DELIVERED == SPI_TWO)
    if((LpJobConfig->ddHWUnitIndex) != SPI_HW_UNIT_SYNCHRONOUS)
    {
      /* Report to DET */
      Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                       SPI_SYNCTRANSMIT_SID, SPI_E_PARAM_SEQ);
      LenReturnValue = E_NOT_OK;
    }
    else
    {
      /* No action required */
    }
    #endif
  }

  /* Check if the SPI Driver is transmitting a sequence */
  if(Spi_GddDriverStatus == SPI_BUSY)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                    SPI_SYNCTRANSMIT_SID, SPI_E_SEQ_IN_PROCESS);
    LenReturnValue = E_NOT_OK;
  }
  else
  {
    /* No action required */
  }


  if(LenReturnValue == E_OK)
  #endif /* SPI_DEV_ERROR_DETECT == STD_ON */
  {
    #if (SPI_DEV_ERROR_DETECT == STD_OFF)

    /* Get the pointer to the requested structure */
    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */
    LpSeqConfig = Spi_GpFirstSeq + Sequence;

    #endif

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts to protect this critical section */
    SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
    #endif

    /* Update the Global status variable */
    Spi_GddDriverStatus = SPI_BUSY;

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Enable relevant interrupts to protect this critical section */
    SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
    #endif
    #if (SPI_LEVEL_DELIVERED == SPI_TWO)
    /* Set the Synchronous transmit flag */
    Spi_GblSyncTx = SPI_TRUE;
    #endif

    /* Update the sequence result variable to SPI_SEQ_PENDING */
    Spi_GaaSeqResult[Sequence] = SPI_SEQ_PENDING;

    /* Get the number of jobs */

    /* MISRA Rule         : 9.1                                            */
    /* Message            : The variable '-identifier-' is apparently      */
    /*                      unset at this point.                           */
    /* Reason             : This variable is initialized at two places     */
    /*                      under                                          */
    /*                      different pre-compile options                  */
    /* Verification       : However, it is manually verified that at least */
    /*                      one of the pre-compile options will be ON and  */
    /*                      hence this variable will be always initialized */
    /*                      Hence,this is not having any impact.           */
    LddNoOfJobs = LpSeqConfig->ddNoOfJobs;

    /* Get the index of the job list for the requested sequence */
    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */
    LddJobListIndex =
            (LpSeqConfig->ddJobListIndex) + (LddNoOfJobs - SPI_ONE);
    LpJobList = Spi_GpFirstJobList + LddJobListIndex;


    while(LddNoOfJobs > SPI_ZERO)
    {
      LddJobIndex = LpJobList->ddJobIndex;

      LucVar = LpJobList->ucJobCount;
      LucVar++;

      /* Get the pointer to the job of the sequence */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact.   */
      LpJobConfig = Spi_GpFirstJob + LddJobIndex;

      /* Update the job result variable */
      Spi_GaaJobResult[LddJobIndex] = SPI_JOB_PENDING;

      do
      {
        /* Invoke the lower layer function to synchronously transmit */
        LenReturnValue = Spi_HWTransmitSyncJob(LpJobConfig);

        if(LenReturnValue != E_OK)
        {
          /* Update the job result variable */
          Spi_GaaJobResult[LddJobIndex] = SPI_JOB_FAILED;
          /* Break the loops */
          LucVar = SPI_ZERO;
          LddNoOfJobs = SPI_ONE;
        }
        else
        {
          /* Decrement the job count */
          LucVar--;
        }
      }while(LucVar > SPI_ZERO);


      if(Spi_GaaJobResult[LddJobIndex] != SPI_JOB_FAILED)
      {
        /* Update the job result variable */
        Spi_GaaJobResult[LddJobIndex] = SPI_JOB_OK;
      }

      /* Increment the pointer to the job list */
      /* MISRA Rule   : 17.4                                       */
      /* Message      : Increment or decrement operation performed */
      /*                on pointer                                 */
      /* Reason       : To access these pointers in optimized      */
      /*                way in this function                       */
      /* Verification : However, part of the code is verified      */
      /*                manually and it is not having any impact   */
      LpJobList--;

      /* Decrement the number of jobs */
      LddNoOfJobs--;
    }

    if(LenReturnValue == E_OK)
    {
      /* Update the sequence result variable to SPI_SEQ_OK */
      Spi_GaaSeqResult[Sequence] = SPI_SEQ_OK;
    }
    else
    {
      /* Update the sequence result variable to SPI_SEQ_FAILED */
      Spi_GaaSeqResult[Sequence] = SPI_SEQ_FAILED;
      /* Raise production error */

      Dem_ReportErrorStatus(SPI_E_SEQ_FAILED, DEM_EVENT_STATUS_FAILED);
    }

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts to protect this critical section */
    SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
    #endif

    Spi_GddDriverStatus = SPI_IDLE;

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Enable relevant interrupts to protect this critical section */
    SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
    #endif
    #if (SPI_LEVEL_DELIVERED == SPI_TWO)
    /* Reset the Synchronous transmit flag */
    Spi_GblSyncTx = SPI_FALSE;
    #endif
  }
   return(LenReturnValue);
}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of (SPI_LEVEL_DELIVERED == SPI_ZERO ||
                     SPI_LEVEL_DELIVERED == SPI_TWO) */

/*******************************************************************************
* Function Name            : Spi_GetHWUnitStatus
*
* Service ID               : 0x0B
*
* Description              : This service is getting the status of the SPI
*                            Hardware microcontroller peripheral

* Sync/Async               : Synchronous
*
* Re-entrancy              : Reentrant
*
* Input Parameters         : HWUnit - ID of CSIG/CSIH Hardware Unit
*
* InOut Parameters         : None
*
* Output Parameters        : None
*
* Return parameter         : Spi_StatusType (SPI_UNINIT/SPI_IDLE/SPI_BUSY)
*
* Preconditions            : Spi_Init should have been invoked.
*
* Remarks                  : Global Variable:
*                            Spi_GddDriverStatus
*                            Spi_GaaHWIndexMap
*
*                           Function invoked:
*                           Det_ReportError
*                           Spi_HWUnitStatus
*
*******************************************************************************/

#if (SPI_HW_STATUS_API == STD_ON)

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(Spi_StatusType, SPI_PUBLIC_CODE) Spi_GetHWUnitStatus
                                                 (Spi_HWUnitType HWUnit)
{
  Spi_StatusType LddHWUnitSts;
  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  uint8 LucHardwareIndex;
  #endif

  LddHWUnitSts = SPI_IDLE;

  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  /* Check if SPI Driver is initialized */
  if(Spi_GddDriverStatus == SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                     SPI_GETHWUNIITSTATUS_SID, SPI_E_UNINIT);
    LddHWUnitSts = SPI_UNINIT;
  }
  else
  {
    /* No action required */
  }

  /* Get the index of the HW Unit */
  LucHardwareIndex = Spi_GaaHWIndexMap[HWUnit];
  if((HWUnit >= SPI_MAX_HW_UNIT) ||
     (LucHardwareIndex == SPI_INVALID_HWUNIT))
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                    SPI_GETHWUNIITSTATUS_SID, SPI_E_PARAM_UNIT);
    LddHWUnitSts = SPI_UNINIT;
  }
  else
  {
    /* No action required */
  }


  if(LddHWUnitSts == SPI_IDLE)
  #endif /* SPI_DEV_ERROR_DETECT == STD_ON */
  {
    LddHWUnitSts = Spi_HWUnitStatus(HWUnit);
  }
  return(LddHWUnitSts);
}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of (SPI_HW_STATUS_API == STD_ON) */

/*******************************************************************************
* Function Name          : Spi_Cancel
*
* Service ID             : 0x0C
*
* Description            : This service is for canceling a on-going sequence

* Sync/Async             : Asynchronous
*
* Re-entrancy            : Reentrant
*
* Input Parameters       : Sequence - Sequence ID
*
* InOut Parameters       : None
*
* Output Parameters      : None
*
* Return parameter       : None
*
* Preconditions          : Spi_Init should have been invoked.
*
* Remarks                : Global Variable:
*                          Spi_GddDriverStatus
*                          Spi_GaaSeqCancel
*
*                          Function invoked:
*                          Det_ReportError
*
*******************************************************************************/

#if (SPI_CANCEL_API == STD_ON)

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PUBLIC_CODE) Spi_Cancel(Spi_SequenceType Sequence)
{
    #if ((SPI_LEVEL_DELIVERED == SPI_TWO) && (SPI_DEV_ERROR_DETECT == STD_ON))
  P2CONST(Tdd_Spi_SequenceConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpSeqConfig;
  P2CONST(Tdd_Spi_JobConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpJobConfig;
  P2CONST(Tdd_Spi_JobListType,AUTOMATIC,SPI_CONFIG_CONST) LpJobList;
  Spi_JobType LddJobListIndex;
  #endif

  uint8 LucStatusOffset;
  uint8 LucStatusMask;

  #if (((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON)) && \
        (SPI_CANCEL_API == STD_ON))
  uint8 LucIndex;
  #endif

  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  boolean LblErrorValue = SPI_FALSE;

  /* Check if SPI Driver is initialized */
  if(Spi_GddDriverStatus == SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                     SPI_CANCEL_SID, SPI_E_UNINIT);
    LblErrorValue = SPI_TRUE;
  }
  else
  {
    /* No action required */
  }

  /* Check if the sequence ID passed, is valid */
  if(Sequence >= SPI_MAX_SEQUENCE)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                                     SPI_CANCEL_SID, SPI_E_PARAM_SEQ);
    LblErrorValue = SPI_TRUE;
  }
  else
  {
    /* No action required */
  }


  #if (SPI_LEVEL_DELIVERED == SPI_TWO)
  if(LblErrorValue == SPI_FALSE)
  {
    /* Get the pointer to the sequence structure */
    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */
    LpSeqConfig = Spi_GpFirstSeq + Sequence;

    /* Get the job list index of the last job of the sequence */
    LddJobListIndex = LpSeqConfig->ddJobListIndex;

    /* Get the pointer to the job list */
    LpJobList = Spi_GpFirstJobList + LddJobListIndex;

    /* Get the pointer of the last job linked to this sequence */
    LpJobConfig = Spi_GpFirstJob + (LpJobList->ddJobIndex);

    /* Check if the HW Unit of the job is configured for synchronous */
    /* transmission                                                  */
    if((LpJobConfig->ddHWUnitIndex) == SPI_HW_UNIT_SYNCHRONOUS)
    {
      /* Report to DET */
      Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                               SPI_CANCEL_SID, SPI_E_PARAM_SEQ);

      LblErrorValue = SPI_TRUE;
    }
   else
    {
      /* No action required */
    }

  }
  #endif

  if(LblErrorValue == SPI_FALSE)
  #endif /* End of SPI_DEV_ERROR_DETECT == STD_ON */
  {
    /* Get the cancel byte offset for the requested sequence */
    LucStatusOffset = Spi_GstSeqProcess[Sequence].ucCancelOffset;
    /* Get the cancel byte mask */
    LucStatusMask = Spi_GstSeqProcess[Sequence].ucCancelMask;
    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts to protect this critical */
    /* section */
    SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
    #endif

    /* Update the cancel bit array for this sequence */
    Spi_GaaSeqCancel[LucStatusOffset] |= LucStatusMask;

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Enable relevant interrupts to protect this critical */
    /* section */
    SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
    #endif

    #if (((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON)) && \
        (SPI_CANCEL_API == STD_ON))
    LucIndex = Spi_GaaSeqCurrentHWUnit[Sequence];
    if((LucIndex >= SPI_MAX_NUM_OF_CSIG) &&
       (Spi_GpConfigPtr->aaHWMemoryMode[LucIndex - SPI_MAX_NUM_OF_CSIG]
        > SPI_ONE))
    {
      Spi_HWCancel(LucIndex);
    }
    #endif
  }
}
#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif  /* End of (SPI_CANCEL_API == STD_ON) */
/*******************************************************************************
* Function Name          : Spi_SetAsyncMode
*
* Service ID             : 0x0D
*
* Description            : This service is for setting the asynchronous mode

* Sync/Async             : Synchronous
*
* Re-entrancy            : Non-Reentrant
*
* Input Parameters       : Mode - New Mode Required
*
* InOut Parameters       : None
*
* Output Parameters      : None
*
* Return parameter       : Std_ReturnType (E_OK/E_NOT_OK)
*
* Preconditions          : Spi_Init should have been invoked.
*
* Remarks                : Global Variable:
*                          Spi_GddDriverStatus
*                          Spi_GddAsyncMode
*
*                          Function invoked:
*                          Det_ReportError
*
*******************************************************************************/

#if (SPI_LEVEL_DELIVERED == SPI_TWO)

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(Std_ReturnType, SPI_PUBLIC_CODE) Spi_SetAsyncMode
                                                  (Spi_AsyncModeType Mode)
{
  Std_ReturnType LenReturnValue;

  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  /* Check if SPI Driver is initialized */
  if(Spi_GddDriverStatus == SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID, SPI_SETAYNCMODE_SID,
                                                            SPI_E_UNINIT);
    LenReturnValue = E_NOT_OK;
  }
  else
  #endif /* SPI_DEV_ERROR_DETECT == STD_ON */
  {
    if((Spi_GddDriverStatus == SPI_BUSY) && (Spi_GblSyncTx == SPI_FALSE))
    {
      LenReturnValue = E_NOT_OK;
    }
    else
    {
      Spi_GddAsyncMode = Mode;
      LenReturnValue = E_OK;
    }
  }
  return(LenReturnValue);
}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif  /* End of (SPI_LEVEL_DELIVERED == SPI_TWO) */


/*******************************************************************************
* Function Name         : Spi_MainFunction_Driving
*
* Service ID            : 0x0E
*
* Description           : This function is to be invoked in the scheduler
*                         loop for asynchronous transmission in polling mode

* Sync/Async            : Synchronous
*
* Re-entrancy           : Non-Reentrant
*
* Input Parameters      : None
*
* InOut Parameters      : None
*
* Output Parameters     : None
*
* Return parameter      : void
*
* Preconditions         : This function should be invoked only when polling
*                         mechanism is selected by Spi_SetAsyncMode API
*
* Remarks               : Global Variable:
*                         Spi_GddDriverStatus
*
*                         Function Invoked:
*                         Det_ReportError
*                         Spi_HWMainFunction_Driving
*
*******************************************************************************/

#if (SPI_LEVEL_DELIVERED == SPI_TWO)

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PUBLIC_CODE)Spi_MainFunction_Driving (void)
{
  #if (SPI_DEV_ERROR_DETECT == STD_ON)
  boolean LblErrorValue = SPI_FALSE;

  /* Check if SPI Driver is initialized */
  if(Spi_GddDriverStatus == SPI_UNINIT)
  {
    /* Report to DET */
    Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                          SPI_MAINFUNCTIONDRIVING_SID, SPI_E_UNINIT);
    LblErrorValue = SPI_TRUE;
  }
  else
  {
      /* No action required */
  }
  if (SPI_INTERRUPT_MODE == Spi_GddAsyncMode)
  {
    /* Report to DET */
    (void)Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                          SPI_MAINFUNCTIONDRIVING_SID, SPI_E_INVALID_MODE );
    LblErrorValue = SPI_TRUE;

  }
  else
  {
      /* No action required */
  }
  if(LblErrorValue == SPI_FALSE)
  #endif /* SPI_DEV_ERROR_DETECT == STD_ON */
  {
    Spi_HWMainFunction_Driving();
  }
}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of ((SPI_LEVEL_DELIVERED == SPI_ONE) ||
                    (SPI_LEVEL_DELIVERED == SPI_TWO))*/

/*******************************************************************************
**                          End of File                                       **
*******************************************************************************/
