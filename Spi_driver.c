/*============================================================================*/
/* Project      = AUTOSAR Renesas Xx4 MCAL Components                         */
/* Module       = Spi_Driver.c                                                */
/* Version      = 3.1.13                                                      */
/* Date         = 03-Jun-2015                                                 */
/*============================================================================*/
/*                                  COPYRIGHT                                 */
/*============================================================================*/
/* Copyright (c) 2009-2015 by Renesas Electronics Corporation                 */
/*============================================================================*/
/* Purpose:                                                                   */
/* This file contains Low level driver function definitions of the SPI        */
/* Driver                                                                     */
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
 * V3.0.1:  05-Nov-2009  : As per SCR 115, I/O structure is updated to have
 *                         separate base address for USER and OS registers.
 * V3.0.2:  10-Nov-2009  : As per SCR 136, the initialization of HW User
 *                         register, CSIG User and OS registers are updated.
 * V3.0.3:  26-Nov-2009  : As per SCR 153, modified the source code for the
 *                         changes found during Dual Buffer mode, TxOnly mode
 *                         and DMA testing.
 * V3.0.4:  14-Jan-2010  : As per SCR 185, modified the source code for the
 *                         changes found during Px4 testing.
 * V3.0.5:  26-Feb-2010  : As per SCR 202, following changes are made:
 *                         1. The value with configured polarity of all
 *                            chip selects is written to CSIH control register.
 *                         2. Check for CSRI bit is added before call to
 *                            Spi_HWDeActivateCS.
 *                         3. Check for the Hw unit whether synchronous or not
 *                            is added in Spi_HWMainFunction_Driving function.
 *                         4. The macro name SPI_MAX_HWUNIT is changed to
 *                            SPI_MAX_HW_UNIT.
 * V3.0.6:  18-Mar-2010  : As per SCR 229, Spi_TxDmaConfig and Spi_RxDmaConfig
 *                         are modified for keeping access to aaHWMemoryMode
 *                         under SPI_CSIH_CONFIGURED macro.
 * V3.0.7:  10-Jul-2010  : As per SCR 298, following changes are done:
 *                         1. The interrupts are enabled/disabled using
 *                            IMR registers.
 *                         2. Values of control registers are updated
 *                            taking care of cautions.
 * V3.0.8:  29-Jul-2010  : As per SCR 322, in Spi_HWTransmitSyncJob function,
 *                         PWR is reset instead of clearing full CTL0 register.
 * V3.0.9:  27-Oct-2010  : As per SCR 378, By masking CSRI bit before loading
 *                         control register.
 * V3.0.10: 14-Jan-2011  : As per SCR 393, Spi_HWDeInit is updated
 *                         to deactivate chip select for CSIG HW unit.
 * V3.0.11: 14-Apr-2011  : As per SCR 432, following changes are made:
 *                         1. The APIs Spi_ProcessChannel and
 *                            Spi_TxDmaConfig are updated to implement DMA with
 *                            software trigger for DIRECT ACCESS MODE if the
 *                            number of buffers is one.
 *                         2. Unwanted blank lines are removed.
 * V3.1.0:  27-Jul-2011  : Update Software version 3.1.0
 * V3.1.1:  04-Oct-2011  : Added comments for the violation of MISRA rule 19.1.
 * V3.1.2:  16-Feb-2012  : Merged the fixes done for Fx4L Spi driver
 * V3.1.3:  16-May-2012  : As per SCR 013, following changes are made:
 *                         1. Updated the function Spi_ProcessChannel with
 *                            blIsChannelPropSame to check if all channel
 *                            properties are same or different in the sequence
 *                         2. Added Spi_GblChannelSameFlag in the function
 *                            Spi_HWInitiateTx to implement for checking the
 *                            channel properties are same or different for CSIH.
 * V3.1.4:  06-Jun-2012  : As per SCR 024,following changes are made:
 *                         1. Environment section is updated to remove compiler
 *                            name.
 *                         2. Software patch version Information is updated.
 * V3.1.5:  24-Oct-2012  : Spi_AsyncTransmit is updated to fix transmit issue
 *                         in Polling mode. (trac #1341)
 * V3.1.5a: 19-Feb-2013  : Merged Sx4-H V3.00 as below.
 *                         As per MANTIS 5059, Spi_ComErrorISR is modified to
 *                         clear error status of STCR0.
 * V3.1.6 : 19-Mar-2013  : As per SCR 082 and mantis #9946 The APIs
 *                         Spi_ProcessChannel, Spi_TransmitISR and Spi_HWWriteIB
 *                         is updated for 24 bit data length transmission.
 * V3.1.7 : 26-Aug-2013  : As per MNT_9946 ,the API Spi_HWTransmitSyncJob,
 *                         Spi_TransmitISR, Spi_ReceiveISR, Spi_HWReadIB are
 *                         updated for 24 bit data lenght transmission.
 * V3.1.8 : 14-Feb-2014  : As part of Dx4 Maintenance Activity, the following
 *                         changes are made:
 *                         1. As per MNT_0015422, Spi_HWInit function is
 *                            modified.
 *                         2. SPI_DRIVER_C_SW_PATCH_VERSION is not checked.
 *                            Hence Removed to make it uniform with
 *                            other modules.
 *                         3. As per MNT_0018648, order of function calls are
 *                            modified as Spi_RxDmaConfig() first and then
 *                            Spi_TxDmaConfig(), in the private function
 *                            Spi_ProcessChannel().
 *                         4. QAC warnings are justified.
 * V3.1.9 : 14-Mar-2014  : As per mantis MNT_12582,
 *                         1.Spi_TransmitCancelISR and Spi_ProcessSequence
 *                           functions updated.
 *                         2.Justification added for misra rule 3.1 to fix the
 *                           QAC comments.
 * V3.1.10 : 31-Oct-2014  : As part of Dx4 V3.16 Maintenance activity,
 *                          1.Justification added for MISRA violations after
 *                           generating QAC reports.
 *                          2.Modified the lines which are more than 80
 *                            characters.
 * V3.1.11 : 05-Dec-2014  : As per MNT_25569, In Spi_HWTransmitSyncJob function
 *                          the variable LddNoOfBuffers is set to "SPI_ONE"
 *                          instead of "SPI_ZERO" in case any problem to
 *                          interrupt the transfer.
 * V3.1.12 : 21-Jan-2015  : As part of Fx4 V3.10 maintenance activity,
 *                          following changes are made:
 *                          1. As per MNT_0019282,In TransmitISR API
 *                             upper 16 bits of CSIHnTX0W is masked
 *                             inorder to retain upper 16 bits.
 *                          2. As per MNT_0024090, variable 'LpPBChannelConfig'
 *                             is initialised before being used
 *                             in Spi_ProcessChannel API.
 *                          3. Copyright information updated.
 * V3.1.13 : 03-Jun-2015  : As part of Fx4 V3.11 maintenance activity,
 *                          following changes are made
 *                          1. As per MNT_0028014,'Spi_ComErrorISR' function
 *                             is modified to remove parity error checking.
 *                             Pre compile check is added for
 *                             'SPI_DATA_CONSISTENCY_CHECK'.
 *                             Renamed 'SPI_PARITY_DCC_ERR' to
 *                             'SPI_DATA_CONSISTENCY_ERROR' and
 *                             'SPI_PARITY_DCC_ERR_CLR' to
 *                             'SPI_DATA_CONSISTENCY_ERROR_CLR'.
 *                          2. As per MNT_0028003, DET implementation is made
 *                             parallel.
 *                          3. As per MNT_0028005,CTL1 for CSIG HW unit
 *                             is set to zero.
 *                          4. As per MNT_0028318,(SPI_LEVEL_DELIVERED ==
 *                             SPI_ONE) is removed from the pre-compiler option
 *                             for Spi_HWMainFunction_Driving API.
 *                          5. As per MNT_0027193,Critical section protection
 *                             is provided around read-modify-write process of
 *                             IMR registers,DMA registers and global variables.
 *                          6. As per MNT_0028347,Disabling of
 *                             CSIHnCTL1.CSIHnCSRI for CSIH is removed and
 *                             enabling of CSIHnCTL1.CSIHnCSRI in
 *                             Spi_HWTransmitSyncJob function is modified.
 *                          7. MISRA Justifications added after generating QAC
 *                             reports.
 */
/******************************************************************************/

/*******************************************************************************
**                      Include Section                                       **
*******************************************************************************/

#include "Spi_Scheduler.h"

/*******************************************************************************
**                      Version Information                                   **
*******************************************************************************/

/* AUTOSAR SPECIFICATION VERSION INFORMATION */
#define SPI_DRIVER_C_AR_MAJOR_VERSION  SPI_AR_MAJOR_VERSION_VALUE
#define SPI_DRIVER_C_AR_MINOR_VERSION  SPI_AR_MINOR_VERSION_VALUE
#define SPI_DRIVER_C_AR_PATCH_VERSION  SPI_AR_PATCH_VERSION_VALUE

/* FILE VERSION INFORMATION */
#define SPI_DRIVER_C_SW_MAJOR_VERSION  3
#define SPI_DRIVER_C_SW_MINOR_VERSION  1

/*******************************************************************************
**                      Version Check                                         **
*******************************************************************************/
#if (SPI_DRIVER_AR_MAJOR_VERSION != SPI_DRIVER_C_AR_MAJOR_VERSION)
  #error "Spi_Driver.c : Mismatch in Specification Major Version"
#endif

#if (SPI_DRIVER_AR_MINOR_VERSION != SPI_DRIVER_C_AR_MINOR_VERSION)
  #error "Spi_Driver.c : Mismatch in Specification Minor Version"
#endif

#if (SPI_DRIVER_AR_PATCH_VERSION != SPI_DRIVER_C_AR_PATCH_VERSION)
  #error "Spi_Driver.c : Mismatch in Specification Patch Version"
#endif

#if (SPI_SW_MAJOR_VERSION != SPI_DRIVER_C_SW_MAJOR_VERSION)
  #error "Spi_Driver.c : Mismatch in Major Version"
#endif

#if (SPI_SW_MINOR_VERSION != SPI_DRIVER_C_SW_MINOR_VERSION)
  #error "Spi_Driver.c : Mismatch in Minor Version"
#endif

/*******************************************************************************
**                         Global Data                                        **
*******************************************************************************/

/*******************************************************************************
**                      Function Definitions                                  **
*******************************************************************************/

/*******************************************************************************
** Function Name      : Spi_HWInit
**
** Service ID         : Not Applicable
**
** Description        : This service initializes the HW Unit
**
** Sync/Async         : Synchronous.
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : None
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : None
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      Spi_GaaHWIndexMap
**
**                      Function invoked:
**                      None
**
*******************************************************************************/
#if ((SPI_LEVEL_DELIVERED == SPI_ONE) || (SPI_LEVEL_DELIVERED == SPI_TWO))
#define SPI_START_SEC_PRIVATE_CODE
  /* MISRA Rule         : 19.1                            */
  /* Message            : #include statements in a file   */
  /*                      should only be preceded by other*/
  /*                      preprocessor directives or      */
  /*                      comments.                       */
  /* Reason             : For code optimization.          */
  /* Verification       : However, part of the code is    */
  /*                      verified manually and it is not */
  /*                      having any impact.              */
#include "MemMap.h"

FUNC(void, SPI_PRIVATE_CODE) Spi_HWInit(void)
{
  P2VAR(uint8,AUTOMATIC,SPI_CONFIG_DATA) LpIntCntlAddress;

  #if(SPI_CSIH_CONFIGURED == STD_ON)
  P2CONST(Tdd_Spi_HWUnitInfo, AUTOMATIC, SPI_PRIVATE_CONST) LpHWUnitInfo;
  P2VAR(Tdd_Spi_MainUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpMainUserBaseAddr;
  P2VAR(Tdd_Spi_MainOsRegs,AUTOMATIC,SPI_CONFIG_DATA) LpMainOsBaseAddr;
  #endif

  #if(SPI_DMA_MODE_ENABLE == STD_ON)
  P2CONST(Tdd_Spi_DmaUnitConfig, AUTOMATIC, SPI_CONFIG_DATA) LpDmaConfig;
  P2VAR(Tdd_Spi_DmaAddrRegs, AUTOMATIC, SPI_CONFIG_DATA) LpDmaRegisters;
  P2VAR(uint16, AUTOMATIC, SPI_CONFIG_DATA) LpDmaTrigFactor;
  #endif

  #if(SPI_CSIH_CONFIGURED == STD_ON)
  /*  MISRA Rule         : 18.4                                           */
  /*  Message            : Unions shall not be used.                      */
  /*  Reason             : To access the converted values                 */
  /*  Verification       : However, part of the code is verified manually */
  /*                       and it is not having any impact.               */
  Tun_Spi_DataAccess LunDataAccess1;
  #endif

  Spi_HWUnitType LddHWUnit;

  #if(SPI_DMA_MODE_ENABLE == STD_ON)
  uint8 LucLoopCount;
  uint8 LucMaxDmaChannels;
  #endif

  uint8 LucIndex;

  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Disable relevant interrupts */
  SchM_Enter_Spi(SPI_DMA_PROTECTION);
  #endif
  #if (SPI_DMA_MODE_ENABLE == STD_ON)
  Spi_GpDmaUnitConfig = Spi_GpConfigPtr->pDMAConfiguration;
  LucMaxDmaChannels = Spi_GpConfigPtr->ucMaxDmaChannels;
  for(LucLoopCount = SPI_ZERO; LucLoopCount <
                           LucMaxDmaChannels; LucLoopCount++)
  {
    LpDmaConfig = &Spi_GpDmaUnitConfig[LucLoopCount];
    LpDmaRegisters = LpDmaConfig->pDmaCntlRegBase;
    /* Clear the DTS bit */
    LpDmaRegisters->ucDTSn &= SPI_DMA_DISABLE;
    if(LpDmaConfig->blComDmaChannel == SPI_ONE)
    {
      /* DMA channel for reception. Hence load the source address register */
      LpDmaRegisters->ulDSAn = LpDmaConfig->ulTxRxRegAddress;
      #if (SPI_8BIT_DATA_WIDTH == STD_ON)
      /* Load the transfer control register */
      LpDmaRegisters->usDTCTn = SPI_DMA_8BIT_RX_SETTINGS;
      #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
      /* Load the transfer control register */
      LpDmaRegisters->usDTCTn = SPI_DMA_16BIT_RX_SETTINGS;
      #endif
    }
    else
    {
      /* DMA channel for transmission. Hence load the destination address */
      /* register */
      LpDmaRegisters->ulDDAn = LpDmaConfig->ulTxRxRegAddress;
      #if (SPI_8BIT_DATA_WIDTH == STD_ON)
      /* Load the transfer control register */
      LpDmaRegisters->usDTCTn = SPI_DMA_8BIT_TX_SETTINGS;
      #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
      /* Load the transfer control register */
      LpDmaRegisters->usDTCTn = SPI_DMA_16BIT_TX_SETTINGS;
      #endif
    }
  #if(SPI_CPU_CORE == SPI_E2M)
    /* Load the source chip select register */
    LpDmaRegisters->usDSCn = SPI_DMA_SRC_SELECT;
    /* Set NSAV bit to 0 in higher address byte */
    LpDmaRegisters->usDNSAnH &= SPI_DMA_CLEAR_NEXT;
    /* Load the destination chip select register */
    LpDmaRegisters->usDDCn = SPI_DMA_DEST_SELECT;
    /* Set NDAV bit to 0 in higher address byte */
    LpDmaRegisters->usDNDAnH &= SPI_DMA_CLEAR_NEXT;
    /* Load the transfer request select register */
    LpDmaRegisters->usDTRSn = SPI_DMA_TRANSFER;
    /* Set NTCV bit to 0 in higher count register */
    LpDmaRegisters->usDNTCn &= SPI_DMA_CLEAR_NEXT;
  #endif
    LpDmaTrigFactor = LpDmaConfig->pDmaDTFRRegAddr;
    /* Load the triger factor configured */
    *LpDmaTrigFactor = LpDmaConfig->usDmaDtfrRegValue;
  }
  #endif /* End of #if (SPI_DMA_MODE_ENABLE == STD_ON) */
  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Disable relevant interrupts */
  SchM_Exit_Spi(SPI_DMA_PROTECTION);
  #endif
  /* Initialize number of HW Unit as zero */
  LddHWUnit = SPI_ZERO;
  /* Initialize each HW Unit */
  while(LddHWUnit < SPI_MAX_HW_UNIT)
  {
    /* Get the index of the hardware */
    LucIndex = Spi_GaaHWIndexMap[LddHWUnit];

    /* Check if the HW Unit configured */
    if(LucIndex != SPI_INVALID_HWUNIT)
    {
      #if(SPI_CSIH_CONFIGURED == STD_ON)
      /* Check if the HW Unit is CSIH */
      if(LddHWUnit >= SPI_MAX_NUM_OF_CSIG)
      {
        /* Get the pointer to the structure of HW Unit information */
        LpHWUnitInfo = &Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[LddHWUnit]];

        /* Get the main user base address */
        LpMainUserBaseAddr = LpHWUnitInfo->pHwMainUserBaseAddress;
        /* Get the main os base address */
        LpMainOsBaseAddr = LpHWUnitInfo->pHwMainOsBaseAddress;
        /* Check if critical section protection is required */
        #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
        /* Disable relevant interrupts */
        SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
        #endif
        /* Reset the PWR bit in main control register 0 */
        LpMainUserBaseAddr->ucMainCTL0 = SPI_ZERO;

        LunDataAccess1.ulRegData = (LpMainOsBaseAddr->ulMainCTL1 &
                                  SPI_CSIH_CS_MASK);

        /* MISRA Rule         : 21.1                                */
        /* Message            : Indexing array with value that will */
        /*                      apparently be out of bounds.        */
        /* Reason             : It is used for array indexing       */
        /* Verification       : However, part of the code           */
        /*                      is verified manually and            */
        /*                      it is not having any impact         */
        LunDataAccess1.ucRegData4[1] =
          Spi_GpConfigPtr->aaChipSelect[LddHWUnit - SPI_MAX_NUM_OF_CSIG];

        /* Configure all chip selects with configured polarity */
        LpMainOsBaseAddr->ulMainCTL1 = LunDataAccess1.ulRegData;
        /* Check if critical section protection is required */
        #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
        /* Disable relevant interrupts */
        SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
        #endif
      }
      else
      {
        /* No action required */
      }
      #endif

      LpIntCntlAddress = Spi_GstHWUnitInfo[LucIndex].pRxImrAddress;
      /* MISRA Rule         : 12.7                                         */
      /* Message            : Bitwise operations on the signed data will   */
      /*                      give implementation defined results          */
      /* Reason             : Though the bitwise operation is performed on */
      /*                      unsigned data, this warning is generated by  */
      /*                      the QAC tool V6.2.1 as an indirect result of */
      /*                      integral promotion in complex bitwise        */
      /*                      operations.                                  */
      /* Verification       : However, this part of the code is verified   */
      /*                      manually and it is not having any impact.    */

      /* Disable interrupt processing */
      *LpIntCntlAddress |= ~(Spi_GstHWUnitInfo[LucIndex].ucRxImrMask);

      LpIntCntlAddress = Spi_GstHWUnitInfo[LucIndex].pTxImrAddress;
      /* MISRA Rule         : 12.7                                         */
      /* Message            : Bitwise operations on the signed data will   */
      /*                      give implementation defined results          */
      /* Reason             : Though the bitwise operation is performed on */
      /*                      unsigned data, this warning is generated by  */
      /*                      the QAC tool V6.2.1 as an indirect result of */
      /*                      integral promotion in complex bitwise        */
      /*                      operations.                                  */
      /* Verification       : However, this part of the code is verified   */
      /*                      manually and it is not having any impact.    */

      /* Disable interrupt processing */
      *LpIntCntlAddress |= ~(Spi_GstHWUnitInfo[LucIndex].ucTxImrMask);

      LpIntCntlAddress = Spi_GstHWUnitInfo[LucIndex].pErrorImrAddress;
      /* MISRA Rule         : 12.7                                         */
      /* Message            : Bitwise operations on the signed data will   */
      /*                      give implementation defined results          */
      /* Reason             : Though the bitwise operation is performed on */
      /*                      unsigned data, this warning is generated by  */
      /*                      the QAC tool V6.2.1 as an indirect result of */
      /*                      integral promotion in complex bitwise        */
      /*                      operations.                                  */
      /* Verification       : However, this part of the code is verified   */
      /*                      manually and it is not having any impact.    */

      /* Disable interrupt processing */
      *LpIntCntlAddress |= ~(Spi_GstHWUnitInfo[LucIndex].ucErrorImrMask);

      #if (((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON)) \
            && (SPI_CANCEL_API == STD_ON))
      LpIntCntlAddress = Spi_GstHWUnitInfo[LucIndex].pTxCancelImrAddress;
      /* MISRA Rule         : 12.7                                         */
      /* Message            : Bitwise operations on the signed data will   */
      /*                      give implementation defined results          */
      /* Reason             : Though the bitwise operation is performed on */
      /*                      unsigned data, this warning is generated by  */
      /*                      the QAC tool V6.2.1 as an indirect result of */
      /*                      integral promotion in complex bitwise        */
      /*                      operations.                                  */
      /* Verification       : However, this part of the code is verified   */
      /*                      manually and it is not having any impact.    */

      /* Disable interrupt processing */
      *LpIntCntlAddress |= ~(Spi_GstHWUnitInfo[LucIndex].ucTxCancelImrMask);
      #endif
    }

    /* Increment HW Unit index */
    LddHWUnit++;
  } /* End of while(LddHWUnit < SPI_MAX_HW_UNIT) */
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* #if ((SPI_LEVEL_DELIVERED == SPI_ONE)
            || (SPI_LEVEL_DELIVERED == SPI_TWO)) */

/*******************************************************************************
** Function Name      : Spi_HWDeInit
**
** Service ID         : Not Applicable
**
** Description        : This service de-initializes the HW Unit
**
** Sync/Async         : Synchronous.
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : None
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : None
**
** Preconditions      : Spi_Init should have been invoked.
**
** Remarks            : Global Variable:
**                      Spi_GaaHWIndexMap
**                      Spi_GstHWUnitInfo
**
**                      Function invoked:
**                      None
**
*******************************************************************************/
#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_HWDeInit(void)
{
  #if(SPI_DMA_MODE_ENABLE == STD_ON)
  P2CONST(Tdd_Spi_DmaUnitConfig, AUTOMATIC, SPI_CONFIG_DATA) LpDmaConfig;
  P2VAR(Tdd_Spi_DmaAddrRegs, AUTOMATIC, SPI_CONFIG_DATA) LpDmaRegisters;
  #endif

  #if (SPI_CSIG_CONFIGURED == STD_ON)
  P2CONST(Tdd_Spi_JobConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpJobConfig;
  Spi_JobType LddNumJobs;
  #endif

  P2VAR(Tdd_Spi_MainUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpHwMainUserBaseAddress;
  P2VAR(Tdd_Spi_MainOsRegs, AUTOMATIC, SPI_CONFIG_DATA) LpHwMainOsBaseAddress;
  Spi_HWUnitType LddHWUnit;

  uint8 LucIndex;

  #if(SPI_DMA_MODE_ENABLE == STD_ON)
  uint8 LucLoopCount;
  uint8 LucMaxDmaChannels;
  #endif

  /* Set the maximum number of zero */
  LddHWUnit = SPI_ZERO;

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
    #endif
  /* Initialize each HW Unit */
  while(LddHWUnit < SPI_MAX_HW_UNIT)
  {
    LucIndex = Spi_GaaHWIndexMap[LddHWUnit];
    /* Check if the CSIG HW Unit configured */
    if(LucIndex != SPI_INVALID_HWUNIT)
    {
      LpHwMainUserBaseAddress =
      Spi_GstHWUnitInfo[LucIndex].pHwMainUserBaseAddress;
       /* Initialize control register 0 to zero */
      LpHwMainUserBaseAddress->ucMainCTL0 = SPI_ZERO;
      LpHwMainOsBaseAddress = \
      Spi_GstHWUnitInfo[LddHWUnit].pHwMainOsBaseAddress;
       /* Initialize control register 1 to zero */
      LpHwMainOsBaseAddress->ulMainCTL1 = SPI_CTL1_ZERO;
    }
    LddHWUnit++;
  } /* (LddHWUnit < SPI_MAX_HW_UNIT) */
    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
    #endif
   /* Deactivate chip select  */
  #if (SPI_CSIG_CONFIGURED == STD_ON)
  LddNumJobs = SPI_ZERO;
  /* Get the pointer of the first job linked to this sequence */
  LpJobConfig = Spi_GpFirstJob;

  while(LddNumJobs < SPI_MAX_JOB)
  {
    if(LpJobConfig->pPortGrpRegAddress != NULL_PTR)
    {
      Spi_HWDeActivateCS(LpJobConfig);
    }
    else
    {
      /* No action required */
    }
    /* MISRA Rule         : 17.4                                          */
    /* Message            : Increment or decrement operation performed on */
    /*                    : pointer                                       */
    /* Reason             : To access the pointer in optimized            */
    /*                      way in this function                          */
    /* Verification       : However, part of the code is verified         */
    /*                      manually and it is not having any impact      */

    LpJobConfig++;
    LddNumJobs++;
  }/* (LddNumJobs < SPI_MAX_JOB) */
  #endif /* #if (SPI_CSIG_CONFIGURED == STD_ON) */

  #if (SPI_DMA_MODE_ENABLE == STD_ON)
  Spi_GpDmaUnitConfig = Spi_GpConfigPtr->pDMAConfiguration;
  LucMaxDmaChannels = Spi_GpConfigPtr->ucMaxDmaChannels;
  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Disable relevant interrupts */
  SchM_Enter_Spi(SPI_DMA_PROTECTION);
  #endif
  for(LucLoopCount = SPI_ZERO; LucLoopCount <
                           LucMaxDmaChannels; LucLoopCount++)
  {
    LpDmaConfig = &Spi_GpDmaUnitConfig[LucLoopCount];
    LpDmaRegisters = LpDmaConfig->pDmaCntlRegBase;
    /* Clear the DTS bit */
    LpDmaRegisters->ucDTSn &= SPI_DMA_DISABLE;
  }
   /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
   /* Disable relevant interrupts */
   SchM_Exit_Spi(SPI_DMA_PROTECTION);
  #endif
  #endif /* #if (SPI_DMA_MODE_ENABLE == STD_ON) */
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */


/*******************************************************************************
** Function Name      : Spi_InternalWriteIB
**
** Service ID         : Not Applicable
**
** Description        : This service writes the data into HW Buffer
**
** Sync/Async         : Synchronous.
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Channel - Channel ID
**                      DataBufferPtr - Pointer to source data buffer
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      None
**
**                      Function Invoked:
**                      Spi_HWWriteIB
**
*******************************************************************************/
#if (SPI_IB_CONFIGURED == STD_ON)

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_InternalWriteIB(Spi_ChannelType Channel,
           P2CONST(Spi_DataType, AUTOMATIC, SPI_APPL_CONST) LpDataBufferPtr)
{
  P2CONST(Tdd_Spi_ChannelPBConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpPBChannelConfig;

  #if((SPI_INTERNAL_RW_BUFFERS == STD_ON) && \
     ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON) || \
      (SPI_LEVEL_DELIVERED == SPI_ZERO)))
  P2CONST(Tdd_Spi_ChannelLTConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpLTChannelConfig;
  P2CONST(Spi_DataType, AUTOMATIC, SPI_APPL_CONST) LpSrcPtr;
  P2VAR(Spi_DataType,AUTOMATIC,SPI_CONFIG_DATA) LpChannelIB;
  Spi_NumberOfDataType LddNoOfBuffers;
  Spi_DataType LddData;
  #endif

  /* Get the pointer to the post-build structure of the channel */

  /* MISRA Rule         : 17.4                           */
  /* Message            : Performing pointer arithmetic  */
  /* Reason             : It is used to achieve          */
  /*                      better throughput.             */
  /* Verification       : However, part of the code      */
  /*                      is verified manually and       */
  /*                      it is not having any impact.   */

  LpPBChannelConfig = Spi_GpFirstChannel + Channel;

  /* Check if the buffer type is internal buffer (not hardware buffer) */
  if((LpPBChannelConfig->ucChannelBufferType) == SPI_ZERO)
  {
    #if((SPI_INTERNAL_RW_BUFFERS == STD_ON) && \
       ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON) || \
        (SPI_LEVEL_DELIVERED == SPI_ZERO)))
    /* Get the pointer to the link-time structure of the channel */
    LpLTChannelConfig = &Spi_GstChannelLTConfig[Channel];
    /* Get the local reference to the source buffer */
    LpSrcPtr = LpDataBufferPtr;
    /* Get the pointer to the internal buffer of the requested channel */
    LpChannelIB = &Spi_GaaChannelIBWrite[LpLTChannelConfig->ddBufferIndex];
    /* Get the number of buffers configured for the requested channel */
    LddNoOfBuffers = LpLTChannelConfig->ddNoOfBuffers;

    /* Check if the source pointer is NULL */
    if(LpSrcPtr == NULL_PTR)
    {
      /* Get the configured default value */
      LddData = LpPBChannelConfig->ddDefaultData;

      do
      {
        /* Copy the default value to the internal buffer */
        *LpChannelIB = LddData;

        /* Increment the source pointer */
        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        LpChannelIB++;

        /* Decrement the counter for number of buffers */
        LddNoOfBuffers--;
      } while(LddNoOfBuffers > SPI_ZERO);
    } /* End of data copy if source pointer is NULL pointer */
    else
    {
      do
      {
        /* Copy the data from the source buffer to the internal buffer */
        *LpChannelIB = *LpSrcPtr;

        /* Increment the source pointer */
        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        LpChannelIB++;

        /* Increment the source pointer */
        LpSrcPtr++;
        /* Decrement the counter for number of buffers */
        LddNoOfBuffers--;
      } while(LddNoOfBuffers > SPI_ZERO);
    } /* End of data copy if source pointer is not NULL pointer  */
    #endif
  } /* End of operations if the buffer type is 'internal buffer' */
  #if ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON))
  else
  {
    /* Buffer type is 'hardware buffer' */
    Spi_HWWriteIB(Channel, LpDataBufferPtr);
  }
  #endif
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif

/*******************************************************************************
** Function Name      : Spi_HWWriteIB
**
** Service ID         : Not Applicable
**
** Description        : This service writes the data into HW Buffer
**
** Sync/Async         : Synchronous.
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Channel - Channel ID
**                      DataBufferPtr - Pointer to source data buffer
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      None
**
**                      Function Invoked:
**                      None
**
*******************************************************************************/
#if ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON))

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_HWWriteIB(Spi_ChannelType Channel,
           P2CONST(Spi_DataType, AUTOMATIC, SPI_APPL_CONST) LpDataBufferPtr)
{
  P2CONST(Tdd_Spi_ChannelLTConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpLTChannelConfig;
  P2CONST(Tdd_Spi_ChannelPBConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpPBChannelConfig;
  P2CONST(Tdd_Spi_HWUnitInfo, AUTOMATIC, SPI_PRIVATE_CONST)LpHWUnitInfo;
  P2VAR(Tdd_Spi_CsihUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsihUserBaseAddr;

  /*  MISRA Rule         : 18.4                                           */
  /*  Message            : Unions shall not be used.                      */
  /*  Reason             : To access the converted values                 */
  /*  Verification       : However, part of the code is verified manually */
  /*                       and it is not having any impact.               */
  Tun_Spi_DataAccess LunDataAccess1;

  #if((SPI_8BIT_DATA_WIDTH == STD_OFF) && (SPI_16BIT_DATA_WIDTH == STD_OFF))
  Tun_Spi_DataAccess LunDataAccess2;
  #endif

  Spi_NumberOfDataType LddNoOfBuffers;
  Spi_HWUnitType LddHWUnit;
  Spi_DataType LddData;

  /* Get the pointer to the link-time structure of the channel*/
  LpLTChannelConfig = &Spi_GstChannelLTConfig[Channel];

  /* Get the pointer to the post-build structure of the channel */

  /* MISRA Rule         : 17.4                           */
  /* Message            : Performing pointer arithmetic  */
  /* Reason             : It is used to achieve          */
  /*                      better throughput.             */
  /* Verification       : However, part of the code      */
  /*                      is verified manually and       */
  /*                      it is not having any impact.   */
  LpPBChannelConfig = Spi_GpFirstChannel + Channel;

  /* Get the index of the HW Unit */
  LddHWUnit = (Spi_HWUnitType)
                         ((LpPBChannelConfig->ucChannelBufferType) + SPI_FOUR);

  /* Get the pointer to the structure of HW Unit information */
  LpHWUnitInfo = &Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[LddHWUnit]];

  /* Get the base address of the HW Unit */
  /* MISRA Rule         : 11.3                                            */
  /* Message            : A cast should not be performed between a        */
  /*                      pointer type and an integral type.              */
  /* Reason             : This is to access the hardware registers.       */
  /* Verification       : However, this part of the code is verified      */
  /*                      manually and it is not having any impact.       */
  LpCsihUserBaseAddr =
  (Tdd_Spi_CsihUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;

  /* Read the existing value from the Read-Write pointer register to */
  /* a local variable */
  LunDataAccess1.ulRegData = LpCsihUserBaseAddr->ulCSIHMRWP0;

  /* Load the local variable with the value of Tx-Buffer Read/Write Pointer */
  LunDataAccess1.usRegData5[0] = LpLTChannelConfig->ddBufferIndex;

  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Disable relevant interrupts */
  SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
  #endif
  /* Load back the value of the local variable to Read-Write pointer register */
  LpCsihUserBaseAddr->ulCSIHMRWP0 = LunDataAccess1.ulRegData;

  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Disable relevant interrupts */
  SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
  #endif
  /* Get the number of buffers configured for the requested channel */
  LddNoOfBuffers = LpLTChannelConfig->ddNoOfBuffers;

  do
  {
    /* Take a local union variable to construct the value for TX0W register */
    LunDataAccess1.ulRegData = SPI_ZERO;

    LunDataAccess1.Tst_ByteAccess.ucRegData2 = LpPBChannelConfig->ucCSInfo;

    /* Check if the source pointer is NULL pointer */
    if(LpDataBufferPtr == NULL_PTR)
    {
      /* get the configured default value */
      LddData = LpPBChannelConfig->ddDefaultData;
    }
    else
    {
      /* Get the value from the source pointer */
      LddData = *LpDataBufferPtr;

      /* Increment the source pointer */
      /* MISRA Rule         : 17.4                                          */
      /* Message            : Increment or decrement operation performed on */
      /*                    : pointer                                       */
      /* Reason             : To access the pointer in optimized            */
      /*                      way in this function                          */
      /* Verification       : However, part of the code is verified         */
      /*                      manually and it is not having any impact      */
      LpDataBufferPtr++;
    }

    #if(SPI_8BIT_DATA_WIDTH == STD_ON)
    /* Data width is maximum 8-bit. Hence, load Tx data portion of the local */
    /* variable with the 8-bit data to be transmitted */
    LunDataAccess1.Tst_ByteAccess.usRegData1 = (uint16)LddData;
    #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
    /* Data width is maximum 16-bit. Hence, load Tx data portion of the local */
    /* variable with the 16-bit data to be transmitted */
    LunDataAccess1.Tst_ByteAccess.usRegData1 = LddData;
    #else
    /* Data width is maximum 32-bit. Tx data needs to be split to */
    /* LS Byte and MS Byte. Hence, load the Tx data to local union variable */
      #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
      if((LpPBChannelConfig->blEDLEnabled == SPI_TRUE)&& \
                        (LpPBChannelConfig->blDirection != SPI_TRUE) && \
                      (LpPBChannelConfig->ucDLSSetting <= SPI_FIFTEEN) && \
                       (LpPBChannelConfig->ucDLSSetting >= SPI_ONE))
      {
        LddData = LddData << (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
      #endif
      LunDataAccess2.ulRegData = LddData;
      #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
      LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[0] \
       >>(SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
      LunDataAccess2.usRegData5[0] = LunDataAccess1.Tst_ByteAccess.usRegData1;
      }
      else
      {
        LunDataAccess2.ulRegData = LddData;
      }
      #endif

    /* Since data width is maximum 32-bit, check if the the data width of */
    /* requested channel is more than 16 bits */
    if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE)
    {
      /* Data width is maximum 16-bit. Hence, load LSB portion of the */
      /* local variable with the 16-bit data to be transmitted */
      LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[0];
    }
    else
    {
      /* Check if the configured data direction is LSB first */
      if(LpPBChannelConfig->blDirection == SPI_TRUE)
      {
        /* Load Tx data portion of the local variable with LSB first */
        LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[0];
      }
      else
      {
        /* Load Tx data portion of the local variable with MSB first */
        LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[1];
      }
      /* Set the EDL bit in the local union variable */
      LunDataAccess1.Tst_ByteAccess.ucRegData3 |= SPI_SET_EDL;
      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Disable relevant interrupts */
      SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
      #endif
      /* Load the value of the local union variable to TX0W register */
      LpCsihUserBaseAddr->ulCSIHTX0W = LunDataAccess1.ulRegData;
      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Enable relevant interrupts */
      SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
      #endif
      /* Decrement the counter for number of buffers */
      LddNoOfBuffers--;
      /* Reset the EDL bit in the local union variable */
      LunDataAccess1.Tst_ByteAccess.ucRegData3 &= SPI_RESET_EDL;

      /* Check the configured data direction again to load other part of data */
      if(LpPBChannelConfig->blDirection == SPI_TRUE)
      {
        /* Load Tx data portion of the local variable with MSB */
        LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[1];
      }
      else
      {
        /* Load Tx data portion of the local variable with LSB */
        LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[0];
      }
    } /* End of !if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE)*/
    #endif /* End of (SPI_8BIT_DATA_WIDTH == STD_ON) */
    /* Check if the buffer is last buffer of the channel */
    if(LddNoOfBuffers == SPI_ONE)
    {
      /* Check if it is last channel in the job */
      if(LpPBChannelConfig->ucChannelInfo == SPI_ONE)
      {
        /* Since buffer is last buffer of the job and not last buffer of */
        /* the sequence, set only EOJ */
        LunDataAccess1.Tst_ByteAccess.ucRegData3 |= SPI_SET_EOJ;
      }
      else if(LpPBChannelConfig->ucChannelInfo == SPI_TWO)
      {
        /* Since last buffer of sequence, set both CIRE and EOJ */
        LunDataAccess1.Tst_ByteAccess.ucRegData3 |= SPI_SET_CIREEOJ;
      }
      /* To avoid MISRA warning */
      else
      {

      }
    } /* End of if(LddNoOfBuffers == SPI_ONE) */
      /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
    #endif
    /* Load the value of the local union variable to TX0W register */
    LpCsihUserBaseAddr->ulCSIHTX0W = LunDataAccess1.ulRegData;

    /* Decrement the counter for number of buffers */
    LddNoOfBuffers--;
    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
    #endif
  }while(LddNoOfBuffers > SPI_ZERO);
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of ((SPI_DUAL_BUFFER_MODE == STD_ON) ||
                  (SPI_TX_ONLY_MODE == STD_ON)) */

/*******************************************************************************
** Function Name      : Spi_HWReadIB
**
** Service ID         : Not Applicable
**
** Description        : This service reads the data from HW Buffer
**
** Sync/Async         : Synchronous
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Channel - Channel ID
**                      DataBufferPtr - Pointer to destination data buffer
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      None
**
**                      Function Invoked:
**                      SchM_Enter_Spi
**                      SchM_Exit_Spi
**
*******************************************************************************/
#if (SPI_DUAL_BUFFER_MODE == STD_ON)

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_HWReadIB(Spi_ChannelType Channel,
           P2VAR(Spi_DataType, AUTOMATIC, SPI_APPL_CONST) LpDataBufferPtr)
{
  P2CONST(Tdd_Spi_ChannelLTConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpLTChannelConfig;
  P2CONST(Tdd_Spi_ChannelPBConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpPBChannelConfig;
  P2CONST(Tdd_Spi_HWUnitInfo, AUTOMATIC, SPI_PRIVATE_CONST)LpHWUnitInfo;
  P2VAR(Tdd_Spi_CsihUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsihUserBaseAddr;

  P2VAR(Spi_DataType, AUTOMATIC, SPI_CONFIG_DATA) LpDesPtr;
  Spi_NumberOfDataType LddNoOfBuffers;

  /*  MISRA Rule         : 18.4                                           */
  /*  Message            : Unions shall not be used.                      */
  /*  Reason             : To access the converted values                 */
  /*  Verification       : However, part of the code is verified manually */
  /*                       and it is not having any impact.               */
  Tun_Spi_DataAccess LunDataAccess1;
  Tun_Spi_DataAccess LunDataAccess2;

  Spi_HWUnitType LddHWUnit;
  Spi_DataType LddData;

  /* Get the pointer to the link-time structure of the channel*/
  LpLTChannelConfig = &Spi_GstChannelLTConfig[Channel];

  /* Get the pointer to the post-build structure of the channel */
  /* MISRA Rule         : 17.4                           */
  /* Message            : Performing pointer arithmetic  */
  /* Reason             : It is used to achieve          */
  /*                      better throughput.             */
  /* Verification       : However, part of the code      */
  /*                      is verified manually and       */
  /*                      it is not having any impact.   */
  LpPBChannelConfig = Spi_GpFirstChannel + Channel;

  /* Get the index of the HW Unit */
  LddHWUnit = (Spi_HWUnitType)
                         ((LpPBChannelConfig->ucChannelBufferType) + SPI_FOUR);

  /* Get the pointer to the structure of HW Unit information */
  LpHWUnitInfo = &Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[LddHWUnit]];

  /* Get the base address of the HW Unit */
  /* MISRA Rule         : 11.3                                            */
  /* Message            : A cast should not be performed between a        */
  /*                      pointer type and an integral type.              */
  /* Reason             : This is to access the hardware registers.       */
  /* Verification       : However, this part of the code is verified      */
  /*                      manually and it is not having any impact.       */
  LpCsihUserBaseAddr =
  (Tdd_Spi_CsihUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;

  /* Read the existing value from the Read-Write pointer register to */
  /* a local variable */
  LunDataAccess1.ulRegData = LpCsihUserBaseAddr->ulCSIHMRWP0;

  /* Load the local variable with the value of Tx-Buffer Read/Write Pointer */
  LunDataAccess1.usRegData5[1] = LpLTChannelConfig->ddBufferIndex;

  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Disable relevant interrupts */
  SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
  #endif

  /* Load back the value of the local variable to Read-Write pointer register */
  LpCsihUserBaseAddr->ulCSIHMRWP0 = LunDataAccess1.ulRegData;

  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Enable relevant interrupts */
  SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
  #endif

  /* Copy the destination pointer value to a local pointer variable */
  LpDesPtr = LpDataBufferPtr;

  /* Get the number of buffers configured for the requested channel */
  LddNoOfBuffers = LpLTChannelConfig->ddNoOfBuffers;

  /* Take a local union variable to construct the value for TX0W register */
  LunDataAccess1.ulRegData = SPI_ZERO;

  do
  {
    #if(SPI_8BIT_DATA_WIDTH == STD_ON)
    /* Data width is maximum 8-bit. Hence, Receive the data from the */
    /* Rx register to local union variable */
    LunDataAccess2.ulRegData = LpCsihUserBaseAddr->ulCSIHRX0W;
    LddData = LunDataAccess2.ucRegData4[0];
    /* Data width is maximum 16-bit. Hence, Receive the data from the */
    /* Rx register to local union variable */
    #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
    LunDataAccess2.ulRegData = LpCsihUserBaseAddr->ulCSIHRX0W;
    /* Load data from union variable to local variable */
    LddData = LunDataAccess2.usRegData5[0];

    #else
    /* Data width is maximum 32-bit, Check if the the data width of */
    /* requested channel is more than 16 bits */
    /* Data width is maximum 32-bit. Hence, Receive the data from the */
    /* Rx register to local union variable */
    LunDataAccess2.ulRegData = LpCsihUserBaseAddr->ulCSIHRX0W;
    /* Load data from union variable to local variable */
    LddData = LunDataAccess2.usRegData5[0];

    if(LpPBChannelConfig->blEDLEnabled == SPI_TRUE)
    {
      LddNoOfBuffers--;
      /* Check if the configured data direction is LSB first */
      if(LpPBChannelConfig->blDirection == SPI_TRUE)
      {
        /* Take a local union variable to construct the value from RX0W */
        /* register */
        LunDataAccess2.usRegData5[0] = (uint16)LddData;
        LunDataAccess2.usRegData5[1] = LpCsihUserBaseAddr->usCSIHRX0H;
      }
      else
      {
        /* Take a local union variable to construct the value from RX0W */
        /* register */

        LunDataAccess2.usRegData5[1] = (uint16)LddData;

        LunDataAccess2.usRegData5[0] = (uint16)
            (LpCsihUserBaseAddr->usCSIHRX0H << LpPBChannelConfig->ucDLSSetting);

        LunDataAccess2.ulRegData = LunDataAccess2.ulRegData >> \
                                                LpPBChannelConfig->ucDLSSetting;

      }
      /* Load data from union variable to local variable */
      LddData = LunDataAccess2.ulRegData;
    }
    #endif /* End of (SPI_8BIT_DATA_WIDTH == STD_ON)*/

    *LpDesPtr = LddData;

    /* Increment the destination pointer */
    /* MISRA Rule         : 17.4                                          */
    /* Message            : Increment or decrement operation performed on */
    /*                    : pointer                                       */
    /* Reason             : To access the pointer in optimized            */
    /*                      way in this function                          */
    /* Verification       : However, part of the code is verified         */
    /*                      manually and it is not having any impact      */
    LpDesPtr++;

    /* Decrement the counter for number of buffers */
    LddNoOfBuffers--;
  }while(LddNoOfBuffers > SPI_ZERO);
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of SPI_DUAL_BUFFER_MODE == STD_ON */

/*******************************************************************************
** Function Name      : Spi_HWActivateCS
**
** Service ID         : Not Applicable
**
** Description        : This service performs activation of the Chip Select pin
**
** Sync/Async         : Synchronous.
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Tdd_Spi_JobConfigType *LpJobConfiguration
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      None
**
**                      Function Invoked:
**                      SchM_Enter_Spi
**                      SchM_Exit_Spi
**
*******************************************************************************/
#if (SPI_CSIG_CONFIGURED == STD_ON)

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_HWActivateCS
(P2CONST(Tdd_Spi_JobConfigType, AUTOMATIC, SPI_PRIVATE_CONST)
                             LpJobConfiguration, uint8 LucLoopCount)
{
  P2VAR(uint32,AUTOMATIC,SPI_CONFIG_DATA) LpPortGpAdd;
  uint32 LulPinMskVal;

  /* Get the port group address */
  LpPortGpAdd = LpJobConfiguration->pPortGrpRegAddress;
  /* Get the pin mask value */
  LulPinMskVal = LpJobConfiguration->ulPortPinMask;

  /* Wait till counter reaches zero */
  while(LucLoopCount > SPI_ZERO)
  {
    LucLoopCount--;
  }

  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Disable relevant interrupts to protect this critical section */
  SchM_Enter_Spi(SPI_CHIP_SELECT_PROTECTION);
  #endif

  /* If Chip select polarity is configured as active High */
  if(LpJobConfiguration->blCsPolarity == SPI_ONE)
  {
    /* Load the port register */
    *LpPortGpAdd = LulPinMskVal;
  }
  else /* If Chip select polarity is configured as active Low */
  {
    /* Load the port register */
    *LpPortGpAdd = LulPinMskVal & SPI_PORT_REG_MASK;
  }

  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Enable relevant interrupts */
  SchM_Exit_Spi(SPI_CHIP_SELECT_PROTECTION);
  #endif
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of (SPI_CSIG_CONFIGURED == STD_ON) */

/*******************************************************************************
** Function Name      : Spi_HWDeActivateCS
**
** Service ID         : Not Applicable
**
** Description        : This service performs de-activation of the Chip Select
**                      pin.
**
** Sync/Async         : Synchronous
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Tdd_Spi_JobConfigType *LpJobConfiguration
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      None
**
**                      Function Invoked:
**                      SchM_Enter_Spi
**                      SchM_Exit_Spi
**
*******************************************************************************/
#if (SPI_CSIG_CONFIGURED == STD_ON)

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_HWDeActivateCS
(P2CONST(Tdd_Spi_JobConfigType, AUTOMATIC, SPI_PRIVATE_CONST)
                                                  LpJobConfiguration)
{
  P2VAR(uint32,AUTOMATIC,SPI_CONFIG_DATA) LpPortGpAdd;
  uint32 LulPinMskVal;

  /* Get the port group address */
  LpPortGpAdd = LpJobConfiguration->pPortGrpRegAddress;
  /* Get the pin mask value */
  LulPinMskVal = LpJobConfiguration->ulPortPinMask;

  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Disable relevant interrupts to protect this critical section */
  SchM_Enter_Spi(SPI_CHIP_SELECT_PROTECTION);
  #endif

  /* If Chip select polarity is configured as active High */
  if(LpJobConfiguration->blCsPolarity == SPI_ONE)
  {
    /* Load the port register */
    *LpPortGpAdd = LulPinMskVal & SPI_PORT_REG_MASK;
  }
  else /* If Chip select polarity is configured as active Low */
  {
    /* Load the port register */
    *LpPortGpAdd = LulPinMskVal;
  }

  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Enable relevant interrupts */
  SchM_Exit_Spi(SPI_CHIP_SELECT_PROTECTION);
  #endif
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of (SPI_CSIG_CONFIGURED == STD_ON) */

/*******************************************************************************
** Function Name      : Spi_HWUnitStatus
**
** Service ID         : Not Applicable
**
** Description        : This service gets the Hardware unit status
**
** Sync/Async         : Synchronous.
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Spi_HWUnitType HWUnit
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : Spi_StatusType (SPI_IDLE/SPI_BUSY)
**
** Preconditions      : Spi_Init should have been invoked.
**
** Remarks            : Global Variable:
**                      Spi_GstHWUnitInfo
**                      Spi_GaaHWIndexMap
**
**                      Function invoked:
**                      None
**
*******************************************************************************/

#if (SPI_HW_STATUS_API == STD_ON)

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(Spi_StatusType, SPI_PRIVATE_CODE) Spi_HWUnitStatus
                                                     (Spi_HWUnitType HWUnit)
{
  P2VAR(Tdd_Spi_MainUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpMainUserBaseAddr;

  Spi_StatusType LddHWUnitSts = SPI_IDLE;
  /* Check if the HW unit is valid */
  if(HWUnit < SPI_MAX_HW_UNIT)
  {
    LpMainUserBaseAddr =
       (Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[HWUnit]]).pHwMainUserBaseAddress;

    /* Is bit TSF = 1 */
    if(((LpMainUserBaseAddr->ulMainSTR0) & SPI_CSIG_CSIH_BUSY) ==
                                                     SPI_CSIG_CSIH_BUSY)
    {
      LddHWUnitSts = SPI_BUSY;
    }
    else /* Is bit TSF = 0 */
    {
      LddHWUnitSts = SPI_IDLE;
    }
  }
  return(LddHWUnitSts);
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of (SPI_HW_STATUS_API == STD_ON) */

/*******************************************************************************
** Function Name      : Spi_HWTransmitSyncJob
**
** Service ID         : Not Applicable
**
** Description        : This service is used for transmitting the sequences
**                      synchronously
**
** Sync/Async         : Synchronous
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Tdd_Spi_JobConfigType *LpJobConfiguration
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : Std_ReturnType (E_OK/E_NOT_OK)
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      Spi_GstHWUnitInfo
**                      Spi_GaaHWIndexMap
**                      Spi_GstChannelLTConfig
**                      Spi_GaaChannelIBWrite
**                      Spi_GaaChannelIBRead
**                      Spi_GaaChannelEBData
**
**                      Function Invoked:
**                      Spi_HWActivateCS
**                      Spi_HWDeActivateCS
**
*******************************************************************************/
#if ((SPI_LEVEL_DELIVERED == SPI_ZERO)||(SPI_LEVEL_DELIVERED == SPI_TWO))

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(Std_ReturnType, SPI_PRIVATE_CODE) Spi_HWTransmitSyncJob
(P2CONST(Tdd_Spi_JobConfigType, AUTOMATIC, SPI_PRIVATE_CONST) LpJobConfig)
{
  P2CONST(Tdd_Spi_ChannelLTConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                         LpLTChannelConfig;
  P2VAR(Tdd_Spi_MainUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpMainUserBaseAddr;
  P2VAR(Tdd_Spi_MainOsRegs,AUTOMATIC,SPI_CONFIG_DATA) LpMainOsBaseAddr;

  #if(SPI_CSIG_CONFIGURED == STD_ON)
  P2VAR(Tdd_Spi_CsigUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsigUserBaseAddr;
  P2VAR(Tdd_Spi_CsigOsRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsigOsBaseAddr;
  #endif

  #if(SPI_CSIH_CONFIGURED == STD_ON)
  P2VAR(Tdd_Spi_CsihUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsihUserBaseAddr;
  P2VAR(Tdd_Spi_CsihOsRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsihOsBaseAddr;
  P2CONST(uint8, AUTOMATIC, SPI_CONFIG_DATA) LpChipSelect;
  #endif

  P2CONST(Tdd_Spi_ChannelPBConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpPBChannelConfig;

  P2CONST(Spi_ChannelType, AUTOMATIC, SPI_PRIVATE_CONST) LpChannelList;
  P2CONST(Tdd_Spi_HWUnitInfo, AUTOMATIC, SPI_PRIVATE_CONST)LpHWUnitInfo;
  P2CONST(Spi_DataType,AUTOMATIC,SPI_CONFIG_DATA) LpNextTxData;
  P2VAR(Spi_DataType,AUTOMATIC,SPI_CONFIG_DATA) LpCurrentRxData;
  Spi_NumberOfDataType LddNoOfBuffers;
  Spi_ChannelType LddNoOfChannels;
  Std_ReturnType LenReturnValue;
  Spi_DataType LddData;
  Spi_HWUnitType LddHWUnit;
  Spi_NumberOfDataType LddBufferIndex;

  #if(SPI_CSIH_CONFIGURED == STD_ON)
  uint8 LucVar;
  #endif

  /*  MISRA Rule         : 18.4                                           */
  /*  Message            : Unions shall not be used.                      */
  /*  Reason             : To access the converted values                 */
  /*  Verification       : However, part of the code is verified manually */
  /*                       and it is not having any impact.               */
  Tun_Spi_DataAccess LunDataAccess1;

  #if ((SPI_8BIT_DATA_WIDTH == STD_OFF) && (SPI_16BIT_DATA_WIDTH == STD_OFF))

  /*  MISRA Rule         : 18.4                                           */
  /*  Message            : Unions shall not be used.                      */
  /*  Reason             : To access the converted values                 */
  /*  Verification       : However, part of the code is verified manually */
  /*                       and it is not having any impact.               */
  Tun_Spi_DataAccess LunDataAccess2;
  #endif

  #if((SPI_8BIT_DATA_WIDTH == STD_OFF) && (SPI_16BIT_DATA_WIDTH == STD_OFF))

  /*  MISRA Rule         : 18.4                                           */
  /*  Message            : Unions shall not be used.                      */
  /*  Reason             : To access the converted values                 */
  /*  Verification       : However, part of the code is verified manually */
  /*                       and it is not having any impact.               */
  Tun_Spi_DataAccess LunDataAccess3;
  #endif

  LenReturnValue = E_OK;

  /* Get the HW Unit index */
  LddHWUnit = LpJobConfig->ddHWUnitIndex;

  /* Get the number of channels */
  LddNoOfChannels = LpJobConfig->ddNoOfChannels;
  LpChannelList = LpJobConfig->pChannelList;

  /* Get the base address of the HW Unit */
  LpHWUnitInfo = &Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[LddHWUnit]];
  /* Get the main user base address */
  LpMainUserBaseAddr = LpHWUnitInfo->pHwMainUserBaseAddress;
  /* Get the main os base address */
  LpMainOsBaseAddr = LpHWUnitInfo->pHwMainOsBaseAddress;

  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Disable relevant interrupts */
  SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
  #endif
  /* Reset the PWR bit since other registers need to be written */
  LpMainUserBaseAddr->ucMainCTL0 &= SPI_RESET_PWR;

  #if(SPI_CSIH_CONFIGURED == STD_ON)
  if(LddHWUnit >= SPI_MAX_NUM_OF_CSIG)
  {
    /* Load the local union variable with Ctl1 register value of current Job */
    LunDataAccess1.ulRegData = LpJobConfig->ulMainCtl1Value;
    /* Write the value with configured polarity of all chip selects */
    LunDataAccess1.ucRegData4[1] =
      Spi_GpConfigPtr->aaChipSelect[LddHWUnit - SPI_MAX_NUM_OF_CSIG];
    /* Load the control register 1 with the value of local union variable */
    LpMainOsBaseAddr->ulMainCTL1 = ((LunDataAccess1.ulRegData)
                                                        & SPI_CSRI_AND_MASK);
  }
  else
  #endif
  {
    /*Load the control register 1 with Ctl1 register value of current Job */
    LpMainOsBaseAddr->ulMainCTL1 =
          ((LpJobConfig->ulMainCtl1Value) & SPI_CSRI_AND_MASK);
  }

  /* Update the Baudrate and Prescaler values from control register2 */
  LpMainOsBaseAddr->usMainCTL2 = LpJobConfig->usCtl2Value;

  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Enable relevant interrupts */
  SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
  #endif

  /* Clear the status register */
  LpMainUserBaseAddr->usMainSTCR0 = SPI_CLR_STS_FLAGS;

  #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
  if(LddHWUnit < SPI_MAX_NUM_OF_CSIG)
  #endif
  {
    #if(SPI_CSIG_CONFIGURED == STD_ON)
    LpCsigOsBaseAddr = (Tdd_Spi_CsigOsRegs *)LpHWUnitInfo->pHwOsBufferAddress;

    /* Get the user base address of the HW Unit */
    LpCsigUserBaseAddr =
                     (Tdd_Spi_CsigUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;
    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
    #endif

    /* Load the values for configuring chip select */
    LpCsigOsBaseAddr->ulCSIGCFG0 = LpJobConfig->ulConfigRegValue;

    /* Load the control register 0 value, setting PWR bit at the same time */
    LpMainUserBaseAddr->ucMainCTL0 = SPI_SET_DIRECT_ACCESS;

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Enable relevant interrupts */
    SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
    #endif

    /* Activate the chip select */
    Spi_HWActivateCS(LpJobConfig, (LpJobConfig->ucClk2CsLoopCnt));
    #endif
  }
  #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
  else
  #endif
  {
    #if(SPI_CSIH_CONFIGURED == STD_ON)
    /* Get the user base address of the HW Unit */
    LpCsihUserBaseAddr =
                 (Tdd_Spi_CsihUserRegs *)(LpHWUnitInfo->pHwUserBufferAddress);

    /* Get the CSIH base address */
    LpCsihOsBaseAddr = (Tdd_Spi_CsihOsRegs *)(LpHWUnitInfo->pHwOsBufferAddress);

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
    #endif

   /* Initialize CSIH memory register */
    LpCsihOsBaseAddr->usCSIHMCTL0 = LpJobConfig->usMCtl0Value;

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Enable relevant interrupts */
    SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
    #endif


   /* Get the value for multiple chip selects configured  */

    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */
    LpChipSelect = (Spi_GpConfigPtr->pCSArray)+(LpJobConfig->ucCSArrayIndex);

   /* Load the number of chip selects */
    LucVar = LpJobConfig->ucNoOfCS;

    do
    {
      /* Initialize CSIH configuration register */
      LpCsihOsBaseAddr->ulCSIHCFG[*LpChipSelect] =
                                             LpJobConfig->ulConfigRegValue;
      /* Decrement the number of chip selects */
      LucVar--;

      /* MISRA Rule         : 17.4                                          */
      /* Message            : Increment or decrement operation performed on */
      /*                    : pointer                                       */
      /* Reason             : To access the pointer in optimized            */
      /*                      way in this function                          */
      /* Verification       : However, part of the code is verified         */
      /*                      manually and it is not having any impact      */
      LpChipSelect++;
    }while(LucVar > SPI_ZERO);

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
    #endif

    /* Load Main CTL0 register, setting PWR bit at the same time */
    LpMainUserBaseAddr->ucMainCTL0 = SPI_SET_DIRECT_ACCESS;

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Enable relevant interrupts */
    SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
    #endif

    #endif
  }

  do
  {
    /* Get the pointer to the channel linked to the job */
    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact    */
    LpPBChannelConfig = Spi_GpFirstChannel + (*LpChannelList);

    /* Get the link time pointer to the channel linked to the job */
    LpLTChannelConfig = Spi_GstChannelLTConfig + (*LpChannelList);
    /* Get the buffer index value */
    LddBufferIndex = LpLTChannelConfig->ddBufferIndex;

    #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
    /* Check if the buffer type is internal buffer */
    if((LpPBChannelConfig->ucChannelBufferType) == SPI_ZERO)
    #endif
    {
      #if(SPI_INTERNAL_RW_BUFFERS == STD_ON)
      /* Update the RAM variable for Tx pointer with channel IB index */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact    */
      LpNextTxData = &Spi_GaaChannelIBWrite[LddBufferIndex];

      /* Update the RAM variable for Rx pointer with channel IB index */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact    */
      LpCurrentRxData = &Spi_GaaChannelIBRead[LddBufferIndex];

      /* Update the RAM variable for number of buffers of the channel */
      LddNoOfBuffers = LpLTChannelConfig->ddNoOfBuffers;
      #endif
    }

    #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
    else if((LpPBChannelConfig->ucChannelBufferType) == SPI_ONE)
    #endif

    {
      #if(SPI_EB_CONFIGURED == STD_ON)
      /* Update the RAM variable for Tx pointer with channel EB
         source pointer */
      LpNextTxData = Spi_GaaChannelEBData[LddBufferIndex].pSrcPtr;

      /* Update the RAM variable for Rx pointer */
      LpCurrentRxData = Spi_GaaChannelEBData[LddBufferIndex].pDestPtr;

      /* Update the local counter with number of buffers of the channel */
      LddNoOfBuffers = Spi_GaaChannelEBData[LddBufferIndex].ddEBLength;
      #endif
    }

    #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
    else
    {
       /* To avoid MISRA warning */
    }
    #endif

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
    #endif
    #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
    if(LddHWUnit < SPI_MAX_NUM_OF_CSIG)
    #endif
    {
      #if(SPI_CSIG_CONFIGURED == STD_ON)
      /* Get the base address of the HW Unit */
      /* MISRA Rule         : 11.3                                       */
      /* Message            : A cast should not be performed between a   */
      /*                      pointer type and an integral type.         */
      /* Reason             : This is to access the hardware registers.  */
      /* Verification       : However, this part of the code is verified */
      /*                      manually and it is not having any impact.  */
      LpCsigOsBaseAddr = (Tdd_Spi_CsigOsRegs *)LpHWUnitInfo->pHwOsBufferAddress;


      /* To load configuration register, read the existing value to  */
      /* local union variable */
      LunDataAccess1.ulRegData = LpCsigOsBaseAddr->ulCSIGCFG0;

      /* Load the local variable with data width */
      LunDataAccess1.ucRegData4[3] = \
                LunDataAccess1.ucRegData4[3] & SPI_DLS_MASK;
      LunDataAccess1.ucRegData4[3] = \
                LunDataAccess1.ucRegData4[3] | LpPBChannelConfig->ucDLSSetting;
      /* Load the local variable with configured direction */
      if(LpPBChannelConfig->blDirection == SPI_TRUE)
      {
        /* Direction is LSB, set DIR bit */
        LunDataAccess1.ucRegData4[2] |= SPI_SET_DIR_LSB;
      }
      else
      {
        /* Direction is MSB, reset DIR bit */
        LunDataAccess1.ucRegData4[2] &= SPI_RESET_DIR_LSB;
      }

      /* Reset the PWR bit since CFG register needs to be written */
      LpMainUserBaseAddr->ucMainCTL0 &= SPI_RESET_PWR;

      /* Load back the value to configuration register */
      LpCsigOsBaseAddr->ulCSIGCFG0 = LunDataAccess1.ulRegData;

      /* Set the HW unit */
      LpMainUserBaseAddr->ucMainCTL0 |= SPI_SET_PWR;
      #endif
    }

    #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
    else
    #endif
    {
      #if(SPI_CSIH_CONFIGURED == STD_ON)
      /* Get the user base address of the HW Unit */
      LpCsihUserBaseAddr =
      (Tdd_Spi_CsihUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;
      /* Get the os base address of the HW Unit */
      LpCsihOsBaseAddr = (Tdd_Spi_CsihOsRegs *)LpHWUnitInfo->pHwOsBufferAddress;

      /* Get the value for multiple chip selects configured  */
      /* MISRA Rule         : 11.3                                          */
      /* Message            : A cast should not be performed between a      */
      /*                      pointer type and an integral type.            */
      /* Reason             : This is to access the hardware registers.     */
      /* Verification       : However, this part of the code is verified    */
      /*                      manually and it is not having any impact.     */
      LpChipSelect = (Spi_GpConfigPtr->pCSArray)+(LpJobConfig->ucCSArrayIndex);

      /* To load configuration register, read the existing value to  */
      /* local union variable */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact    */
      LunDataAccess1.ulRegData =
                  LpCsihOsBaseAddr->ulCSIHCFG[*LpChipSelect];
      /* Load the local variable with data width */
      LunDataAccess1.ucRegData4[3] = \
                LunDataAccess1.ucRegData4[3] & SPI_DLS_MASK;
      LunDataAccess1.ucRegData4[3] = \
                LunDataAccess1.ucRegData4[3] | LpPBChannelConfig->ucDLSSetting;
      /* Load the local variable with configured direction */
      if(LpPBChannelConfig->blDirection == SPI_TRUE)
      {
        /* Direction is LSB, set DIR bit */
        LunDataAccess1.ucRegData4[2] |= SPI_SET_DIR_LSB;
      }
      else
      {
        /* Direction is MSB, reset DIR bit */
        LunDataAccess1.ucRegData4[2] &= SPI_RESET_DIR_LSB;
      }

      /* Load the number of chip selects */
      LucVar = LpJobConfig->ucNoOfCS;

      /* Reset the PWR bit since CFG register needs to be written */
      LpMainUserBaseAddr->ucMainCTL0 &= SPI_RESET_PWR;

      do
      {
        /* Initialize CSIH configuration register */
        LpCsihOsBaseAddr->ulCSIHCFG[*LpChipSelect] = LunDataAccess1.ulRegData;
        /* Decrement number of chip selects */
        LucVar--;

        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        LpChipSelect++;
      }while(LucVar > SPI_ZERO);

      LpMainUserBaseAddr->ucMainCTL0 |= SPI_SET_PWR;

      #endif
    }
    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Enable relevant interrupts */
    SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
    #endif

    /* MISRA Rule         : 9.1                                               */
    /* Message            : The variable '-identifier-' is apparently         */
    /*                      unset at this point.                              */
    /* Reason             : This variable is initialized at two places under  */
    /*                      different pre-compile options                     */
    /* Verification       : However, it is manually verified that at least one*/
    /*                      of the pre-compile options will be ON and         */
    /*                      hence this variable will be always initialzed.    */
    /*                      Hence,this is not having any impact.              */
    do
    {
      /* Get the data to be transmitted */
      if(LpNextTxData == NULL_PTR)
      {
        LddData = LpPBChannelConfig->ddDefaultData;
      }
      else
      {
        LddData = *LpNextTxData;
        /* Increment the pointer to the buffer */
        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        LpNextTxData++;
      }

      /* Increment the source pointer */

      /* Message(3:3198)    : Assignment is redundant.                    */
      /*                      The value of LunDataAccess1.ulRegData       */
      /*                      is never used before being                  */
      /*                      modified.                                   */
      /* Reason             : This is done for proper initialisation of   */
      /*                      the union variable.                         */
      /* Verification       : However, part of the code is                */
      /*                      verified manually and it is not             */
      /*                      having any impact.                          */

      /* Take a local union variable to construct the value for TX0W */
      /* register */
      LunDataAccess1.ulRegData = SPI_ZERO;

    /* MISRA Rule         : 9.1                                               */
    /* Message            : The variable '-identifier-' is apparently         */
    /*                      unset at this point.                              */
    /* Reason             : This variable is initialized at two places under  */
    /*                      different pre-compile options                     */
    /* Verification       : However, it is manually verified that at least one*/
    /*                      of the pre-compile options will be ON and         */
    /*                      hence this variable will be always initialzed.    */
    /*                      Hence,this is not having any impact.              */
      if ((SPI_ONE == LddNoOfBuffers) && (SPI_ONE == LddNoOfChannels))
      {
      if ((LpJobConfig->ulMainCtl1Value & SPI_CSRI_MASK) ==
                                                            SPI_CSRI_MASK)
        {
          LpMainOsBaseAddr->ulMainCTL1 = LpMainOsBaseAddr->ulMainCTL1 |
                                                           (~SPI_CSRI_AND_MASK);
        }

      }

      #if(SPI_8BIT_DATA_WIDTH == STD_ON)
      /* Data width is maximum 8-bit. Hence, load Tx data portion of the
         local variable with the 8-bit data to be transmitted */
      LunDataAccess1.Tst_ByteAccess.usRegData1 = (uint16)LddData;
      #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
      /* Data width is maximum 16-bit. Hence, load Tx data portion of the
         local variable with the 16-bit data to be transmitted */
      LunDataAccess1.Tst_ByteAccess.usRegData1 = LddData;
      #else
      /* Data width is maximum 32-bit. Tx data needs to be split to
         LS Byte and MS Byte. Hence, load the Tx data to local union variable */
      #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
      if((LpPBChannelConfig->blEDLEnabled == SPI_TRUE)&& \
                        (LpPBChannelConfig->blDirection != SPI_TRUE) && \
                      (LpPBChannelConfig->ucDLSSetting <= SPI_FIFTEEN) && \
                       (LpPBChannelConfig->ucDLSSetting >= SPI_ONE))
      {
        LddData = LddData << (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
      #endif
      LunDataAccess2.ulRegData = LddData;
      #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
      LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[0]  \
      >> (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
      LunDataAccess2.usRegData5[0] = LunDataAccess1.Tst_ByteAccess.usRegData1;
      }
      else
      {
        LunDataAccess2.ulRegData = LddData;
      }
      #endif

      /* Since data width is maximum 32-bit, check if the the data width of
         requested channel is more than 16 bits */
      if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE)
      {
        /* Data width is maximum 16-bit. Hence, load LSB portion of the
           local variable with the 16-bit data to be transmitted */
        LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[0];
      }
      else
      {
        /* Check if the configured data direction is LSB first */
        if(LpPBChannelConfig->blDirection == SPI_TRUE)
        {
          /* Load Tx data portion of the local variable with LSB first */
          LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                                   LunDataAccess2.usRegData5[0];
        }
        else
        {
          /* Load Tx data portion of the local variable with MSB first */
          LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                                   LunDataAccess2.usRegData5[1];
        }

        /* Set the EDL bit in the local union variable */
        LunDataAccess1.Tst_ByteAccess.ucRegData3 = SPI_SET_EDL;

        LunDataAccess3.ulRegData = LunDataAccess1.ulRegData;

        /* Check the configured data direction again to load other
           part of data */
        if(LpPBChannelConfig->blDirection == SPI_TRUE)
        {
          /* Load Tx data portion of the local variable with MSB */
          LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                                  LunDataAccess2.usRegData5[1];
        }
        else
        {
          /* Load Tx data portion of the local variable with LSB */
          LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                                  LunDataAccess2.usRegData5[0];
        }

        /* Reset the EDL bit in the local union variable */
        LunDataAccess1.Tst_ByteAccess.ucRegData3 &= SPI_RESET_EDL;

      } /* End of !if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE) */
      #endif /* End of (SPI_8BIT_DATA_WIDTH == STD_ON) */

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Disable relevant interrupts */
      SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
      #endif
      #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
      if(LddHWUnit < SPI_MAX_NUM_OF_CSIG)
      #endif
      {
        #if(SPI_CSIG_CONFIGURED == STD_ON)
        #if((SPI_8BIT_DATA_WIDTH == STD_OFF)\
                                          && (SPI_16BIT_DATA_WIDTH == STD_OFF))
        /* Since data width is maximum 32-bit, check if the the data width of */
        /* requested channel is more than 16 bits */
        if(LpPBChannelConfig->blEDLEnabled == SPI_TRUE)
        {
          /* Load the value of the local union variable to TX0W register */
          /* MISRA Rule      : 9.1                                            */
          /* Message         : The variable '-identifier-' is apparently      */
          /*                   unset at this point.                           */
          /* Reason          : This variable is initialized at two places     */
          /*                   under                                          */
          /*                   different pre-compile options                  */
          /* Verification    : However, it is manually verified that at least */
          /*                   one of the pre-compile options will be ON and  */
          /*                   hence this variable will be always initialized */
          /*                   Hence,this is not having any impact.           */
          LpCsigUserBaseAddr->ulCSIGTX0W = LunDataAccess3.ulRegData;

          /* Wait in the loop until the communication is stopped */
          while(((LpMainUserBaseAddr->ulMainSTR0) & SPI_HW_BUSY) == SPI_HW_BUSY)
          {
            ;
          }

          /* Save the received data to a variable */
          Spi_GusDataAccess = LpCsigUserBaseAddr->usCSIGRX0;
        }
        #endif
        /* Load the value of the local union variable to TX0W register */
        /* MISRA Rule        : 9.1                                            */
        /* Message           : The variable '-identifier-' is apparently      */
        /*                     unset at this point.                           */
        /* Reason            : This variable is initialized at two places     */
        /*                     under                                          */
        /*                     different pre-compile options                  */
        /* Verification      : However, it is manually verified that atnleast */
        /*                     one of the pre-compile options will be ON and  */
        /*                     hence this variable will be always initialized */
        /*                     Hence,this is not having any impact.           */
        LpCsigUserBaseAddr->ulCSIGTX0W = LunDataAccess1.ulRegData;
        #endif
      }

      #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
      else
      #endif
      {
        #if(SPI_CSIH_CONFIGURED == STD_ON)
        /* Set chip select in the local union variable */
        LunDataAccess1.Tst_ByteAccess.ucRegData2 = LpJobConfig->ucChipSelect;

        #if((SPI_8BIT_DATA_WIDTH == STD_OFF) \
                                          && (SPI_16BIT_DATA_WIDTH == STD_OFF))
        /* Since data width is maximum 32-bit, check if the the data width of */
        /* requested channel is more than 16 bits */
        if(LpPBChannelConfig->blEDLEnabled == SPI_TRUE)
        {
          /* Set chip select in the local union variable */
          LunDataAccess3.Tst_ByteAccess.ucRegData2 = LpJobConfig->ucChipSelect;

          /* Load the value of the local union variable to TX0W register */
          /* MISRA Rule      : 9.1                                            */
          /* Message         : The variable '-identifier-' is apparently      */
          /*                   unset at this point.                           */
          /* Reason          : This variable is initialized at two places     */
          /*                   under                                          */
          /*                   different pre-compile options                  */
          /* Verification    : However, it is manually verified that at least */
          /*                   one of the pre-compile options will be ON and  */
          /*                   hence this variable will be always initialized */
          /*                   Hence,this is not having any impact.           */
          LpCsihUserBaseAddr->ulCSIHTX0W = LunDataAccess3.ulRegData;

          /* Wait in the loop until the communication is stopped */
          while(((LpMainUserBaseAddr->ulMainSTR0) & SPI_HW_BUSY) == SPI_HW_BUSY)
          {
            ;
          }

          /* Save the received data to a variable */
          Spi_GusDataAccess = LpCsihUserBaseAddr->usCSIHRX0H;
        }
        #endif
        /* Load the value of the local union variable to TX0W register */
        /* MISRA Rule        : 9.1                                            */
        /* Message           : The variable '-identifier-' is apparently      */
        /*                     unset at this point.                           */
        /* Reason            : This variable is initialized at two places     */
        /*                     under                                          */
        /*                     different pre-compile options                  */
        /* Verification      : However, it is manually verified that at least */
        /*                     one of the pre-compile options will be ON and  */
        /*                     hence this variable will be always initialized */
        /*                     Hence,this is not having any impact.           */
        LpCsihUserBaseAddr->ulCSIHTX0W = LunDataAccess1.ulRegData;
         #endif
      }
      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Enable relevant interrupts */
      SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
      #endif

      /* Wait in the loop until the communication is stopped */
      while(((LpMainUserBaseAddr->ulMainSTR0) & SPI_HW_BUSY)
                                                  == SPI_HW_BUSY)
      {
         ;
      }

      if(((LpMainUserBaseAddr->ulMainSTR0) & SPI_ERROR_MASK) != SPI_ZERO)
      {
        LenReturnValue = E_NOT_OK;
        /* To break the loop */
        LddNoOfBuffers = SPI_ONE;
      }

      #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
      if(LddHWUnit < SPI_MAX_NUM_OF_CSIG)
      #endif
      {
        #if(SPI_CSIG_CONFIGURED == STD_ON)

        #if(SPI_8BIT_DATA_WIDTH == STD_ON)
        /* Data width is maximum 8-bit. Hence, Receive the data from the */
        /* Rx register to local union variable */
        LddData = (uint8)LpCsigUserBaseAddr->usCSIGRX0;

        #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
        LddData = LpCsigUserBaseAddr->usCSIGRX0;

        #else
        /* Data width is maximum 32-bit, Check if the the data width of */
        /* requested channel is more than 16 bits */
        if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE)
        {
          /* Data width is maximum 8-bit. Hence, Receive the data from the */
          /* Rx register to local union variable */
          LddData = LpCsigUserBaseAddr->usCSIGRX0;
        }
        else
        {
          /* Check if the configured data direction is LSB first */
          if(LpPBChannelConfig->blDirection == SPI_TRUE)
          {
            /* Take a local union variable to construct the value from RX0W */
            /* register */
            LunDataAccess2.usRegData5[1] = LpCsigUserBaseAddr->usCSIGRX0;
            LunDataAccess2.usRegData5[0] = Spi_GusDataAccess;
          }
          else
          {
            /* Take a local union variable to construct the value from RX0W */
            /* register */
            LunDataAccess2.usRegData5[0] = (uint16)
             (LpCsigUserBaseAddr->usCSIGRX0 << LpPBChannelConfig->ucDLSSetting);
            LunDataAccess2.usRegData5[1] = Spi_GusDataAccess;
            LunDataAccess2.ulRegData = LunDataAccess2.ulRegData >> \
                                                LpPBChannelConfig->ucDLSSetting;
          } /* End of if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE) */
          LddData = LunDataAccess2.ulRegData;
        }
        #endif
        #endif
      }
      #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
      else
      #endif
      {
        #if(SPI_CSIH_CONFIGURED == STD_ON)

        #if(SPI_8BIT_DATA_WIDTH == STD_ON)
        /* Data width is maximum 8-bit. Hence, Receive the data from the */
        /* Rx register to local union variable */
        LddData = (uint8)LpCsihUserBaseAddr->usCSIHRX0H;

        #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
        LddData = LpCsihUserBaseAddr->usCSIHRX0H;

        #else
        /* Data width is maximum 32-bit, Check if the the data width of */
        /* requested channel is more than 16 bits */
        if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE)
        {
          /* Data width is maximum 8-bit. Hence, Receive the data from the */
          /* Rx register to local union variable */
          LddData = LpCsihUserBaseAddr->usCSIHRX0H;
        }
        else
        {
          /* Check if the configured data direction is LSB first */
          if(LpPBChannelConfig->blDirection == SPI_TRUE)
          {
            /* Take a local union variable to construct the value from RX0W */
            /* register */
            LunDataAccess2.usRegData5[1] =
            (uint16)LpCsihUserBaseAddr->ulCSIHRX0W;
            LunDataAccess2.usRegData5[0] = Spi_GusDataAccess;
          }
          else
          {
            /* Take a local union variable to construct the value from RX0W */
            /* register */
            LunDataAccess2.usRegData5[0] = (uint16)
            (LpCsihUserBaseAddr->ulCSIHRX0W << LpPBChannelConfig->ucDLSSetting);
            LunDataAccess2.usRegData5[1] = Spi_GusDataAccess;
            LunDataAccess2.ulRegData = LunDataAccess2.ulRegData >> \
                                                LpPBChannelConfig->ucDLSSetting;
          } /* End of if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE) */

          LddData = LunDataAccess2.ulRegData;
        }
        #endif
        #endif
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
      if(LpCurrentRxData != NULL_PTR)
      {
        *LpCurrentRxData = LddData;
        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        LpCurrentRxData++;
      }

      /* Decrement the counter for number of buffers */
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
      LddNoOfBuffers--;

    }while(LddNoOfBuffers > SPI_ZERO);

    /* Increment the pointer to the channel */
    /* MISRA Rule         : 17.4                                          */
    /* Message            : Increment or decrement operation performed on */
    /*                    : pointer                                       */
    /* Reason             : To access the pointer in optimized            */
    /*                      way in this function                          */
    /* Verification       : However, part of the code is verified         */
    /*                      manually and it is not having any impact      */
    LpChannelList++;

    /* Decrement the counter for number of channels */
    LddNoOfChannels--;
  }while(LddNoOfChannels > SPI_ZERO);

  #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
  if(LddHWUnit < SPI_MAX_NUM_OF_CSIG)
  #endif
  {
    #if(SPI_CSIG_CONFIGURED == STD_ON)
    if((LpJobConfig->ulMainCtl1Value & SPI_CSRI_MASK) != SPI_ZERO)
    {
      /* Deactivate the chip select */
      Spi_HWDeActivateCS(LpJobConfig);
    }
    #endif
  }

  return(LenReturnValue);
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif

/*******************************************************************************
** Function Name      : Spi_HWInitiateTx
**
** Service ID         : Not Applicable
**
** Description        : This service initiates the SPI transmission. This is
**                      common for all modes of asynchronous transmission
**
** Sync/Async         : Synchronous.
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Spi_JobType LddJobListIndex
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      Spi_GpFirstSeq
**                      Spi_GpFirstJob
**                      Spi_GaaJobResult
**                      Spi_GstHWUnitInfo
**                      Spi_GaaHWIndexMap
**                      Spi_GstChannelLTConfig
**                      Spi_GaaChannelIBWrite
**                      Spi_GaaChannelIBRead
**                      Spi_GaaChannelEBData
**                      Spi_GddAsyncMode
**
**                      Function Invoked:
**                      Spi_HWActivateCS
**                      Spi_ProcessChannel
**                      SchM_Enter_Spi
**                      SchM_Exit_Spi
**
*******************************************************************************/
#if ((SPI_LEVEL_DELIVERED == SPI_ONE) || (SPI_LEVEL_DELIVERED == SPI_TWO))

/* MISRA Rule         : 1.1                                                   */
/* Message            : Number of macros defined within the                   */
/*                      translation exceeds 1024.                             */
/* Reason             : This macro is required for memory section definition. */
/* Verification       : Not necessarily an indication of incorrect code.      */
/*                      However, part of the code is verified manually and    */
/*                      it is not having any impact.                          */

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_HWInitiateTx(Spi_JobType LddJobListIndex)
{
  P2CONST(Tdd_Spi_JobConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpJobConfig;
  #if ((SPI_TX_ONLY_MODE == STD_ON) || (SPI_DUAL_BUFFER_MODE == STD_ON))
  P2CONST(Tdd_Spi_SequenceConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpSeqConfig;
  #endif
  P2CONST(Tdd_Spi_JobListType,AUTOMATIC,SPI_CONFIG_CONST) LpJobList;
  P2VAR(Tdd_Spi_MainUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpMainUserBaseAddr;
  P2VAR(Tdd_Spi_MainOsRegs,AUTOMATIC,SPI_CONFIG_DATA) LpMainOsBaseAddr;

  #if (SPI_TX_ONLY_MODE == STD_ON)
  P2CONST(Tdd_Spi_ChannelLTConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                         LpLTChannelConfig;
  Spi_NumberOfDataType LddBufferIndex;
  #endif

  #if(SPI_CSIG_CONFIGURED == STD_ON)
  P2VAR(Tdd_Spi_CsigOsRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsigOsBaseAddr;
  #endif
  #if(SPI_CSIH_CONFIGURED == STD_ON)
  P2VAR(Tdd_Spi_CsihOsRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsihOsBaseAddr;
  #endif

  P2CONST(Tdd_Spi_HWUnitInfo, AUTOMATIC, SPI_CONFIG_DATA)LpHWUnitInfo;

  Spi_JobType LddJobIndex;
  Spi_SequenceType LddSeqIndex;
  Spi_HWUnitType LddHWUnit;
  uint8 LucHWMemoryMode;
  #if ((SPI_TX_ONLY_MODE == STD_ON) || (SPI_DUAL_BUFFER_MODE == STD_ON))
  Spi_NumberOfDataType LddNxtNotifyIndex;
  #endif

  #if(SPI_CSIH_CONFIGURED == STD_ON)
  P2CONST(uint8, AUTOMATIC, SPI_CONFIG_DATA) LpChipSelect;
  Spi_JobType LddNoOfJobs;
  uint8 LucVar;
  /*  MISRA Rule         : 18.4                                           */
  /*  Message            : Unions shall not be used.                      */
  /*  Reason             : To access the converted values                 */
  /*  Verification       : However, part of the code is verified manually */
  /*                       and it is not having any impact.               */
  Tun_Spi_DataAccess LunDataAccess1;
  #endif

  /* Get the pointer to the respective job list index */
  /* MISRA Rule         : 17.4                           */
  /* Message            : Performing pointer arithmetic  */
  /* Reason             : It is used to achieve          */
  /*                      better throughput.             */
  /* Verification       : However, part of the code      */
  /*                      is verified manually and       */
  /*                      it is not having any impact    */
  LpJobList = Spi_GpFirstJobList + LddJobListIndex;

  /* Get the index of the job */
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
  LddJobIndex = LpJobList->ddJobIndex;

  /* Get the index of the sequence */
  LddSeqIndex = LpJobList->ddSequenceIndex;

  #if ((SPI_TX_ONLY_MODE == STD_ON) || (SPI_DUAL_BUFFER_MODE == STD_ON))
  /* Get the pointer to the sequence structure */
  /* MISRA Rule         : 17.4                           */
  /* Message            : Performing pointer arithmetic  */
  /* Reason             : It is used to achieve          */
  /*                      better throughput.             */
  /* Verification       : However, part of the code      */
  /*                      is verified manually and       */
  /*                      it is not having any impact    */
  LpSeqConfig = Spi_GpFirstSeq + LddSeqIndex;
  #endif

  /* Take up the requested sequence for transmission */
  Spi_GaaSeqResult[LddSeqIndex] = SPI_SEQ_PENDING;

  /* Get the pointer to the job structure */
  /* MISRA Rule         : 17.4                           */
  /* Message            : Performing pointer arithmetic  */
  /* Reason             : It is used to achieve          */
  /*                      better throughput.             */
  /* Verification       : However, part of the code      */
  /*                      is verified manually and       */
  /*                      it is not having any impact    */
  LpJobConfig = Spi_GpFirstJob + LddJobIndex;

  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Disable relevant interrupts to protect this critical section */
  SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
  #endif

  /* Update driver status as busy */
  Spi_GddDriverStatus = SPI_BUSY;

  #if (SPI_DIRECT_ACCESS_MODE == STD_ON)
  Spi_GblTxEDL = SPI_FALSE;
  Spi_GblRxEDL = SPI_FALSE;
  #endif

  /* Get the HW Unit index of the any of the job in the sequence */
  LddHWUnit = LpJobConfig->ddHWUnitIndex;

  #if (SPI_CSIH_CONFIGURED == STD_ON)
  /* Check if the HW Unit is CSIH */
  if(LddHWUnit >= SPI_MAX_NUM_OF_CSIG)
  {
  /* Get the configured memory mode for this HW Unit */
    LucHWMemoryMode =
         Spi_GpConfigPtr->aaHWMemoryMode[LddHWUnit - SPI_MAX_NUM_OF_CSIG];
  }
  else
  #endif
  /* MISRA Rule         : 13.7                                           */
  /* Message            : The result of this logical operation is        */
  /*                      always true .                                  */
  /* Reason             : Logical operation performed to check           */
  /*                      configured mode as Direct access mode          */
  /* Verification       : However, part of the code is verified manually */
  /*                      and it is not having any impact.               */
  {
  /* Since HW Unit is CSIG, memory mode is DIRECT ACCESS by default */
    LucHWMemoryMode = SPI_DIRECT_ACCESS_MODE_CONFIGURED;
  }

  #if (SPI_DIRECT_ACCESS_MODE == STD_ON)
  if(LucHWMemoryMode == SPI_DIRECT_ACCESS_MODE_CONFIGURED)
  {
    /* Save the current channel list to be transmitted */
    Spi_GstCurrentCommData.pCurrentTxChannelList = (LpJobConfig->pChannelList);

    /* Save the current channel list to be received */
    Spi_GstCurrentCommData.pCurrentRxChannelList = (LpJobConfig->pChannelList);

    /* Save the number of channels yet to be transmitted */
    Spi_GstCurrentCommData.ddNoOfTxChannels = (LpJobConfig->ddNoOfChannels);

    /* Save the number of channels yet to be received */
    Spi_GstCurrentCommData.ddNoOfRxChannels = (LpJobConfig->ddNoOfChannels);
  }
  #endif

  #if (SPI_TX_ONLY_MODE == STD_ON)
  if(LucHWMemoryMode == SPI_TX_ONLY_MODE_CONFIGURED)
  {
    LpLTChannelConfig = &Spi_GstChannelLTConfig[*(LpJobConfig->pChannelList)];
    /* Get the buffer index value */
    LddBufferIndex = LpLTChannelConfig->ddBufferIndex;

    Spi_GstTxOnlyCurrentCommData.pCurrentRxData =
                 &Spi_GaaTxOnlyRead[LddBufferIndex];
    /* Get the pointer to the post-build structure of the channel */
    Spi_GstTxOnlyCurrentCommData.ddChannelIndex = *(LpJobConfig->pChannelList);

    /* Save the number of buffers to be received */
    Spi_GstTxOnlyCurrentCommData.ddNoOfBuffers = LpSeqConfig->ddNoOfBuffers;

    /* Get the notify index value */
    LddNxtNotifyIndex = LpSeqConfig->ddNxtNotifyIndex;

    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact    */
    Spi_GpCurrentNotifyIdx =
           (Spi_GpConfigPtr->pJobNotifyIndex) + LddNxtNotifyIndex;

    Spi_GddNoOfBuffers = SPI_ZERO;
  }
  #endif

  #if (SPI_DUAL_BUFFER_MODE == STD_ON)
  if(LucHWMemoryMode == SPI_DUAL_BUFFER_MODE_CONFIGURED)
  {
    /* Get the notify index value */
    LddNxtNotifyIndex = LpSeqConfig->ddNxtNotifyIndex;

    /* MISRA Rule       : 17.4                           */
    /* Message          : Performing pointer arithmetic  */
    /* Reason           : It is used to achieve          */
    /*                    better throughput.             */
    /* Verification     : However, part of the code      */
    /*                    is verified manually and       */
    /*                    it is not having any impact    */
    Spi_GpCurrentNotifyIdx =
           (Spi_GpConfigPtr->pJobNotifyIndex) + LddNxtNotifyIndex;

    Spi_GddNoOfBuffers = *Spi_GpCurrentNotifyIdx;
  }
  #endif

  #if(SPI_CANCEL_API == STD_ON)
  Spi_GaaSeqCurrentHWUnit[LddSeqIndex] = LddHWUnit;
  #endif

  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Enable relevant interrupts */
  SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
  #endif

  /* Get the pointer to the structure of HW Unit information */
  LpHWUnitInfo = &Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[LddHWUnit]];

  /* Get the main user base address */
  LpMainUserBaseAddr = LpHWUnitInfo->pHwMainUserBaseAddress;
  /* Get the main os base address */
  LpMainOsBaseAddr = LpHWUnitInfo->pHwMainOsBaseAddress;
  /* Check if critical section protection is required */

 #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Disable relevant interrupts */
  SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
  #endif

  /* Reset the PWR bit since other registers need to be written */
  LpMainUserBaseAddr->ucMainCTL0 &= SPI_RESET_PWR;

  #if(SPI_CSIH_CONFIGURED == STD_ON)
  if(LddHWUnit >= SPI_MAX_NUM_OF_CSIG)
  {
    /* Load the local union variable with Ctl1 register value of current Job */
    LunDataAccess1.ulRegData = LpJobConfig->ulMainCtl1Value;
    /* Write the value with configured polarity of all chip selects */
    LunDataAccess1.ucRegData4[1] =
      Spi_GpConfigPtr->aaChipSelect[LddHWUnit - SPI_MAX_NUM_OF_CSIG];
    /* Load the control register 1 with the value of local union variable */
    LpMainOsBaseAddr->ulMainCTL1 = LunDataAccess1.ulRegData;
  }
  else
  #endif
  {
    /* Load the control register 1 with Ctl1 register value of current Job */
    LpMainOsBaseAddr->ulMainCTL1 =
          (LpJobConfig->ulMainCtl1Value) & SPI_CSRI_AND_MASK;
  }

  /* Update the Prescalar and Baudrate values  */
  LpMainOsBaseAddr->usMainCTL2 = LpJobConfig->usCtl2Value;

  LpMainUserBaseAddr->usMainSTCR0 = SPI_CLR_STS_FLAGS;

  /* Check if critical section protection is required */
  #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
  /* Enable relevant interrupts */
  SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
  #endif

  #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
  /* Check if the HW Unit is CSIG */
  if(LddHWUnit < SPI_MAX_NUM_OF_CSIG)
  #endif
  {
    #if(SPI_CSIG_CONFIGURED == STD_ON)
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
    #endif

    /* Update the job result variable for current job */
    Spi_GaaJobResult[LddJobIndex] = SPI_JOB_PENDING;

    LpCsigOsBaseAddr = (Tdd_Spi_CsigOsRegs *)LpHWUnitInfo->pHwOsBufferAddress;

    /* Load CSIG configuration register with communication type values */
    LpCsigOsBaseAddr->ulCSIGCFG0 = LpJobConfig->ulConfigRegValue;

    /* Load Main CTL0 register, setting PWR bit at the same time */
    LpMainUserBaseAddr->ucMainCTL0 = SPI_SET_DIRECT_ACCESS;

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Enable relevant interrupts */
    SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
    #endif

    if(Spi_GddAsyncMode == SPI_POLLING_MODE)
    {
      /* Activate the chip select with delay loop */
      Spi_HWActivateCS(LpJobConfig, (LpJobConfig->ucClk2CsLoopCnt));
    }
    else
    {
      /* Activate the chip select */
      Spi_HWActivateCS(LpJobConfig, SPI_ZERO);
    }
    #endif /* End of (SPI_CSIG_CONFIGURED == STD_ON) */
  } /* End of if(LddHWUnit <= SPI_MAX_NUM_OF_CSIG) */

  #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
  else
  #endif
  {
    #if(SPI_CSIH_CONFIGURED == STD_ON)

    /* Get the CSIH base address */
    LpCsihOsBaseAddr = (Tdd_Spi_CsihOsRegs *)LpHWUnitInfo->pHwOsBufferAddress;

    /* Initialize CSIH memory register */
    /* MISRA Rule         : 11.3                                       */
    /* Message            : A cast should not be performed between a   */
    /*                      pointer type and an integral type.         */
    /* Reason             : This is to access the hardware registers.  */
    /* Verification       : However, this part of the code is verified */
    /*                      manually and it is not having any impact   */

    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
    #endif

     LpCsihOsBaseAddr->usCSIHMCTL0 = LpJobConfig->usMCtl0Value;

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Enable relevant interrupts */
    SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
    #endif


    #if ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON))
    if((LucHWMemoryMode == SPI_DUAL_BUFFER_MODE_CONFIGURED) ||
                    (LucHWMemoryMode == SPI_TX_ONLY_MODE_CONFIGURED))
    {
      /* Get the number of jobs */
      LddNoOfJobs = LpSeqConfig->ddNoOfJobs;
    }
    else
    #endif
    {
      LddNoOfJobs = SPI_ONE;
    }

    /* Get the pointer to the respective job list index */
    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact    */
    LpJobList = Spi_GpFirstJobList + LddJobListIndex;

    do
    {
      /* Get the index of the job */
      LddJobIndex = LpJobList->ddJobIndex;
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Disable relevant interrupts */
      SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
      #endif

      /* Update the job result variable */
      Spi_GaaJobResult[LddJobIndex] = SPI_JOB_PENDING;

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Enable relevant interrupts */
      SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
      #endif


      /* Get the pointer to the job structure */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact    */
      LpJobConfig = Spi_GpFirstJob + LddJobIndex;
      /* Get the value for multiple chip selects configured */
      LpChipSelect = (Spi_GpConfigPtr->pCSArray)+(LpJobConfig->ucCSArrayIndex);

      LucVar = LpJobConfig->ucNoOfCS;

      do
      {
        /* Initialize CSIH configuration register */
        LpCsihOsBaseAddr->ulCSIHCFG[*LpChipSelect] =
                                           LpJobConfig->ulConfigRegValue;
        LucVar--;
        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        LpChipSelect++;
      }while(LucVar > SPI_ZERO);

      /* MISRA Rule         : 17.4                                          */
      /* Message            : Increment or decrement operation performed on */
      /*                    : pointer                                       */
      /* Reason             : To access the pointer in optimized            */
      /*                      way in this function                          */
      /* Verification       : However, part of the code is verified         */
      /*                      manually and it is not having any impact      */
      LpJobList--;
      LddNoOfJobs--;

      /* MISRA Rule         : 13.7                             */
      /* Message            : The result of this logical       */
      /*                      operation or control expression  */
      /*                      is always 'false'.               */
      /* Reason             : The loop is used for total       */
      /*                      the number jobs configured       */
      /* Verification       : However, part of the code is     */
      /*                      verified manually and it is not  */
      /*                      having any impact.               */
    }while(LddNoOfJobs > SPI_ZERO);

      /* MISRA Rule         : 13.7                                           */
      /* Message            : Boolean operations whose results are invariant */
      /*                      shall not be permitted.                        */
      /* Reason             : It is used to achieve better performance       */
      /* Verification       : However, part of the code                      */
      /*                      is verified manually and                       */
      /*                      it is not having any impact                    */

      /* MISRA Rule         : 17.4                                          */
      /* Message            : Increment or decrement operation performed on */
      /*                    : pointer                                       */
      /* Reason             : To access the pointer in optimized            */
      /*                      way in this function                          */
      /* Verification       : However, part of the code is verified         */
      /*                      manually and it is not having any impact      */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
    #endif

    if(LucHWMemoryMode > SPI_ZERO)
    {
      /* Load Main CTL0 register, without setting PWR bit */
      LpMainUserBaseAddr->ucMainCTL0 = SPI_SET_MEMORY_ACCESS;
    }
    else
    {
      /* Load Main CTL0 register, setting PWR bit at the same time */
      LpMainUserBaseAddr->ucMainCTL0 = SPI_SET_DIRECT_ACCESS;
    }
    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Enable relevant interrupts */
    SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
    #endif

    #endif
  }

  #if(SPI_CSIH_CONFIGURED == STD_ON)
  /* Global boolean variable to avoid copy of properties repeatedly if
  properties of all channels are same. This boolean allows copying channel
  properties once. This boolean will be reset immediately after its use */
  Spi_GblChannelSameFlag = SPI_TRUE;
  #endif

  Spi_ProcessChannel(LddHWUnit, LucHWMemoryMode);
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of ((SPI_LEVEL_DELIVERED == SPI_ONE) ||
                  (SPI_LEVEL_DELIVERED == SPI_TWO))*/

/*******************************************************************************
** Function Name      : Spi_ProcessChannel
**
** Service ID         : Not Applicable
**
** Description        : This function is to process remaining channels
**                      to be transmitted
**
** Sync/Async         : Synchronous
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Spi_HWUnitType LddHWUnit
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      Spi_GaaHWIndexMap
**                      Spi_GstHWUnitInfo
**                      Spi_GaaHWIndexMap
**                      Spi_GpFirstChannel
**                      Spi_GstChannelLTConfig
**                      Spi_GaaChannelIBWrite
**                      Spi_GaaChannelIBRead
**                      Spi_GaaChannelEBData
**                      Spi_GddAsyncMode
**
**                      Function Invoked:
**                      Spi_TxDmaConfig
**                      Spi_RxDmaConfig
**                      SchM_Enter_Spi
**                      SchM_Exit_Spi
**
*******************************************************************************/
#if ((SPI_LEVEL_DELIVERED == SPI_ONE) || (SPI_LEVEL_DELIVERED == SPI_TWO))

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_ProcessChannel
                 (Spi_HWUnitType LddHWUnit, uint8 LucHWMemoryMode)
{
  #if ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON))
  P2CONST(Tdd_Spi_SequenceConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpSeqConfig;
  Spi_SequenceType LddSeqIndex;
  #endif

  #if((SPI_DMA_MODE_ENABLE == STD_ON) || (SPI_CSIH_CONFIGURED == STD_ON))
  P2CONST(Tdd_Spi_JobConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpJobConfig;
  P2CONST(Tdd_Spi_JobListType,AUTOMATIC,SPI_CONFIG_CONST) LpJobList;
  Spi_JobType LddJobIndex;
  Spi_JobType LddJobListIndex;
  #endif

  #if ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON))
  P2CONST(Spi_DataType,AUTOMATIC,SPI_CONFIG_DATA) LpNextTxData;
  #endif

  P2CONST(Spi_ChannelType,AUTOMATIC,SPI_CONFIG_CONST) LpCurrentTxChannelList;
  P2CONST(Tdd_Spi_ChannelLTConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpLTChannelConfig;
  P2CONST(Tdd_Spi_ChannelPBConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpPBChannelConfig;

  P2VAR(Tdd_Spi_MainUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpMainUserBaseAddr;
  #if (SPI_DIRECT_ACCESS_MODE == STD_ON)
  P2VAR(Tdd_Spi_MainOsRegs,AUTOMATIC,SPI_CONFIG_DATA) LpMainOsBaseAddr;
  #endif

  #if(SPI_DIRECT_ACCESS_MODE == STD_ON)
  P2VAR(Spi_DataType,AUTOMATIC,SPI_CONFIG_DATA) LpCurrentRxData;
  #endif

  #if(SPI_CSIG_CONFIGURED == STD_ON)
  P2VAR(Tdd_Spi_CsigUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsigUserBaseAddr;
  P2VAR(Tdd_Spi_CsigOsRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsigOsBaseAddr;
  #endif
  #if(SPI_CSIH_CONFIGURED == STD_ON)
  P2VAR(Tdd_Spi_CsihUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsihUserBaseAddr;
  P2VAR(Tdd_Spi_CsihOsRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsihOsBaseAddr;
  #endif

  P2CONST(Tdd_Spi_HWUnitInfo, AUTOMATIC, SPI_PRIVATE_CONST)LpHWUnitInfo;

  Spi_NumberOfDataType LddBufferIndex;

  /*  MISRA Rule         : 18.4                                           */
  /*  Message            : Unions shall not be used.                      */
  /*  Reason             : To access the converted values                 */
  /*  Verification       : However, part of the code is verified manually */
  /*                       and it is not having any impact.               */
  Tun_Spi_DataAccess LunDataAccess1;

  Spi_DataType LddData;
  #if ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON))
  Spi_NumberOfDataType LddNoOfBuffers;
  #endif

  #if ((SPI_8BIT_DATA_WIDTH == STD_OFF) && (SPI_16BIT_DATA_WIDTH == STD_OFF))
  #if ((SPI_CSIG_CONFIGURED == STD_ON) || (SPI_DIRECT_ACCESS_MODE == STD_ON) \
        || (SPI_FIFO_MODE == STD_ON))

  /*  MISRA Rule         : 18.4                                           */
  /*  Message            : Unions shall not be used.                      */
  /*  Reason             : To access the converted values                 */
  /*  Verification       : However, part of the code is verified manually */
  /*                       and it is not having any impact.               */
  Tun_Spi_DataAccess LunDataAccess2;
  #endif
  #endif

  #if (SPI_FIFO_MODE == STD_ON)
  Spi_ChannelType LddNoOfChannels;
  #endif

  #if(SPI_CSIH_CONFIGURED == STD_ON)
  P2CONST(uint8, AUTOMATIC, SPI_CONFIG_DATA) LpChipSelect;
  uint8 LucVar;
  #endif

  /* Get the pointer to the structure of HW Unit information */
  LpHWUnitInfo = &Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[LddHWUnit]];

  LpMainUserBaseAddr = LpHWUnitInfo->pHwMainUserBaseAddress;

  LpCurrentTxChannelList = Spi_GstCurrentCommData.pCurrentTxChannelList;
  /* Get the pointer to the post-build structure of the channel */
  /* MISRA Rule         : 17.4                           */
  /* Message            : Performing pointer arithmetic  */
  /* Reason             : It is used to achieve          */
  /*                      better throughput.             */
  /* Verification       : However, part of the code      */
  /*                      is verified manually and       */
  /*                      it is not having any impact.   */
  LpPBChannelConfig = Spi_GpFirstChannel + (*LpCurrentTxChannelList);
  LddData = LpPBChannelConfig->ddDefaultData;
  
  #if (SPI_DIRECT_ACCESS_MODE == STD_ON)
  LpMainOsBaseAddr = LpHWUnitInfo->pHwMainOsBaseAddress;
  #endif

  #if(SPI_DIRECT_ACCESS_MODE == STD_ON)
  if(LucHWMemoryMode == SPI_DIRECT_ACCESS_MODE_CONFIGURED)
  {
    LpLTChannelConfig = &Spi_GstChannelLTConfig[(*LpCurrentTxChannelList)];

    LddBufferIndex = LpLTChannelConfig->ddBufferIndex;

    #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
    /* Check if the buffer type is internal buffer */
    if((LpPBChannelConfig->ucChannelBufferType) == SPI_ZERO)
    #endif
    {
      #if(SPI_INTERNAL_RW_BUFFERS == STD_ON)
      /* Update the RAM variable for Tx pointer with channel IB index */
      LpNextTxData = &Spi_GaaChannelIBWrite[LddBufferIndex];

      /* Update the RAM variable for Rx pointer with channel IB index */
      LpCurrentRxData = &Spi_GaaChannelIBRead[LddBufferIndex];

      /* Update the RAM variable for number of buffers of the channel */
      LddNoOfBuffers = LpLTChannelConfig->ddNoOfBuffers;
      #endif
    }

    #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
    else if((LpPBChannelConfig->ucChannelBufferType) == SPI_ONE)
    #endif

    {
      #if(SPI_EB_CONFIGURED == STD_ON)
      /* Update the RAM variable for Tx pointer with channel EB
         source pointer */
      LpNextTxData = Spi_GaaChannelEBData[LddBufferIndex].pSrcPtr;
      /* Update the RAM variable for Rx pointer */
      LpCurrentRxData = Spi_GaaChannelEBData[LddBufferIndex].pDestPtr;
      /* Update the local counter with number of buffers of the channel */
      LddNoOfBuffers =
       Spi_GaaChannelEBData[LddBufferIndex].ddEBLength;
      #endif
    }

    #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
    else
    {
       /* To avoid MISRA warning */
    }
    #endif

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts to protect this critical section */
    SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
    #endif

    /* Increment the source pointer */
    /* MISRA Rule         : 17.4                                          */
    /* Message            : Increment or decrement operation performed on */
    /*                    : pointer                                       */
    /* Reason             : To access the pointer in optimized            */
    /*                      way in this function                          */
    /* Verification       : However, part of the code is verified         */
    /*                      manually and it is not having any impact      */

    /* Decrement the number of channels yet to be transmitted  */
    (Spi_GstCurrentCommData.pCurrentTxChannelList)++;
    (Spi_GstCurrentCommData.pCurrentRxChannelList)++;

    (Spi_GstCurrentCommData.ddNoOfTxChannels)--;
    (Spi_GstCurrentCommData.ddNoOfRxChannels)--;

    /* Save the pointer to data to be received */
    /* MISRA Rule         : 9.1                                               */
    /* Message            : The variable '-identifier-' is apparently         */
    /*                      unset at this point.                              */
    /* Reason             : This variable is initialized at two places under  */
    /*                      different pre-compile options                     */
    /* Verification       : However, it is manually verified that atleast one */
    /*                      of the pre-compile options will be ON and         */
    /*                      hence this variable will be always initialized.   */
    /*                      Hence,this is not having any impact.              */
    Spi_GstCurrentCommData.pCurrentRxData = LpCurrentRxData;

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
    /* Get the data to be transmitted */
    if(LpNextTxData == NULL_PTR)
    {
      LddData = LpPBChannelConfig->ddDefaultData;
    }
    else
    {
      LddData = *LpNextTxData;
    }

    /* Save the pointer to next data to be transmitted */
    Spi_GstCurrentCommData.pNextTxData = LpNextTxData;

    /* Save and decrement the number of buffers */

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

    Spi_GstCurrentCommData.ddNoOfTxBuffers = LddNoOfBuffers;
    Spi_GstCurrentCommData.ddNoOfRxBuffers = LddNoOfBuffers;
    (Spi_GstCurrentCommData.ddNoOfTxBuffers)--;
    (Spi_GstCurrentCommData.ddNoOfRxBuffers)--;

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Enable relevant interrupts */
    SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
    #endif

  } /*
     * End of if ((LucHWMemoryMode == SPI_DIRECT_ACCESS_CONFIGURED) ||
     *           (LucHWMemoryMode == SPI_FIFO_CONFIGURED))
     */
  #endif /*
          * End of (SPI_DIRECT_ACCESS_MODE == STD_ON)
          */


  #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))

  if(LddHWUnit < SPI_MAX_NUM_OF_CSIG)
  #endif
  {
    #if(SPI_CSIG_CONFIGURED == STD_ON)
    /* Reset the PWR bit */
    LpMainUserBaseAddr->ucMainCTL0 &= SPI_RESET_PWR;

    /* Get the user base address of the HW Unit */
    LpCsigUserBaseAddr =
                     (Tdd_Spi_CsigUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;

    /* Get the user base address of the HW Unit */
    LpCsigOsBaseAddr = (Tdd_Spi_CsigOsRegs *)LpHWUnitInfo->pHwOsBufferAddress;

    /* To load configuration register, read the existing value to  */
    /* local union variable */
    /* MISRA Rule         : 11.3                                          */
    /* Message            : A cast should not be performed between a      */
    /*                      pointer type and an integral type.            */
    /* Reason             : This is to access the hardware registers.     */
    /* Verification       : However, this part of the code is verified    */
    /*                      manually and it is not having any impact.     */
    LunDataAccess1.ulRegData = LpCsigOsBaseAddr->ulCSIGCFG0;

    /* Load the local variable with data width */
    /* MISRA Rule         : 9.1                                            */
    /* Message            : The variable '-identifier-' is apparently      */
    /*                      unset at this point.                           */
    /* Reason             : This variable is initialized at two places     */
    /*                      under                                          */
    /*                      different pre-compile options                  */
    /* Verification       : However, it is manually verified that atleast  */
    /*                      one of the pre-compile options will be ON and  */
    /*                      hence this variable will be always initialzed  */
    /*                      Hence,this is not having any impact.           */
    /* Load the local variable with data width */
      LunDataAccess1.ucRegData4[3] = \
                LunDataAccess1.ucRegData4[3] & SPI_DLS_MASK;
      LunDataAccess1.ucRegData4[3] = \
                LunDataAccess1.ucRegData4[3] | LpPBChannelConfig->ucDLSSetting;

    /* Load the local variable with configured direction */
    if(LpPBChannelConfig->blDirection == SPI_TRUE)
    {
      /* Direction is LSB, set DIR bit */
      LunDataAccess1.ucRegData4[2] |= SPI_SET_DIR_LSB;
    }
    else
    {
      /* Direction is MSB, reset DIR bit */
      LunDataAccess1.ucRegData4[2] &= SPI_RESET_DIR_LSB;
    }

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
    #endif

    /* Load back the value to configuration register */
    LpCsigOsBaseAddr->ulCSIGCFG0 = LunDataAccess1.ulRegData;

    /* Set the SLIT bit */
    LpMainOsBaseAddr->ulMainCTL1 |= SPI_SET_SLIT;

    /* Set the HW unit */
    LpMainUserBaseAddr->ucMainCTL0 |= SPI_SET_PWR;

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Enable relevant interrupts */
    SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
    #endif

    #if(SPI_DMA_MODE_ENABLE == STD_ON)

    /* MISRA Rule         : 21.1                                */
    /* Message            : Indexing array with value that will */
    /*                      apparently be out of bounds.        */
    /* Reason             : It is used for array indexing       */
    /* Verification       : However, part of the code           */
    /*                      is verified manually and            */
    /*                      it is not having any impact         */

    /* Get the index of job list for the current job from the job queue */
    LddJobListIndex =  Spi_GaaJobQueue[Spi_GddQueueIndex[LucHWMemoryMode]];

    /* Get the pointer to the respective job list index      */
    /* MISRA Rule         : 17.4                             */
    /* Message            : Performing pointer arithmetic    */
    /* Reason             : It is used to achieve            */
    /*                      better throughput.               */
    /* Verification       : However, part of the code        */
    /*                      is verified manually and         */
    /*                      it is not having any impact.     */
    LpJobList = Spi_GpFirstJobList + LddJobListIndex;

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
    /* Get the index of the job */
    LddJobIndex = LpJobList->ddJobIndex;

    /* Get the pointer to the job structure */
    /* MISRA Rule         : 17.4                              */
    /* Message            : Performing pointer arithmetic     */
    /* Reason             : It is used to achieve             */
    /*                      better throughput.                */
    /* Verification       : However, part of the code         */
    /*                      is verified manually and          */
    /*                      it is not having any impact.      */
    LpJobConfig = Spi_GpFirstJob + LddJobIndex;


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
    if(Spi_GddAsyncMode == SPI_INTERRUPT_MODE)
    {
        /* If the selected asynchronous mode is DMA mode */

        /* MISRA Rule         : 9.1                                        */
        /* Message            : The variable '-identifier-' is apparently  */
        /*                      unset at this point.                       */
        /* Reason             : This variable is initialized at two places */
        /*                      under different pre-compile options        */
        /* Verification       : However, it is manually verified that      */
        /*                      at least one of the pre-compile options    */
        /*                      will be ON and  hence this variable will   */
        /*                      be always initialzed Hence,this is not     */
        /*                      having any impact.                         */

        /* Function to configure the DMA reception */
        Spi_RxDmaConfig(LpJobConfig, Spi_GstCurrentCommData.pCurrentRxData,
                                                               LddNoOfBuffers);
        Spi_GddDmaTxData = LpPBChannelConfig->ddDefaultData;
        /* Function to configure the DMA transmission */
        Spi_TxDmaConfig(LpJobConfig, LpNextTxData, LddNoOfBuffers);
    }
    #endif

      /* Message(3:3198)    : Assignment is redundant.                    */
      /*                      The value of LunDataAccess1.ulRegData       */
      /*                      is never used before being                  */
      /*                      modified.                                   */
      /* Reason             : This is done for proper initialisation of   */
      /*                      the union variable.                         */
      /* Verification       : However, part of the code is                */
      /*                      verified manually and it is not             */
      /*                      having any impact.                          */

    /* Take a local union variable to construct the value for TX0W register */
    LunDataAccess1.ulRegData = SPI_ZERO;

    #if(SPI_8BIT_DATA_WIDTH == STD_ON)
    /* Data width is maximum 8-bit. Hence, load Tx data portion of the local */
    /* variable with the 8-bit data to be transmitted */
    LunDataAccess1.ucRegData4[0] = LddData;
    #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
    /* Data width is maximum 16-bit. Hence, load Tx data portion of the local */
    /* variable with the 16-bit data to be transmitted */
      /* Message(4:3353)    :The variable '-identifier-' is possibly unset  */
      /*                     at this point.                                 */
      /* Reason             : This variable is initialized at two places    */
      /*                      under                                         */
      /*                      different pre-compile options                 */
      /* Verification       : However, it is manually verified that at least*/
      /*                      one of the pre-compile options will be ON and */
      /*                      hence this variable will be always initialized*/
      /*                      Hence,this is not having any impact.          */
    LunDataAccess1.Tst_ByteAccess.usRegData1 = LddData;
    #else
    /* Data width is maximum 32-bit. Tx data needs to be split to */
    /* LS Byte and MS Byte. Hence, load the Tx data to local union variable */
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

      #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
      if((LpPBChannelConfig->blEDLEnabled == SPI_TRUE)&& \
                        (LpPBChannelConfig->blDirection != SPI_TRUE) && \
                      (LpPBChannelConfig->ucDLSSetting <= SPI_FIFTEEN) && \
                       (LpPBChannelConfig->ucDLSSetting >= SPI_ONE))
      {
        LddData = LddData << (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
      #endif
      LunDataAccess2.ulRegData = LddData;
      #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
      LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[0]  \
      >> (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
      LunDataAccess2.usRegData5[0] = LunDataAccess1.Tst_ByteAccess.usRegData1;
      }
      else
      {
        LunDataAccess2.ulRegData = LddData;
      }
      #endif

    /* Since data width is maximum 32-bit, check if the the data width of */
    /* requested channel is more than 16 bits */
    if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE)
    {
      /* Data width is maximum 16-bit. Hence, load LSB portion of the */
      /* local variable with the 16-bit data to be transmitted */
      LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[0];
    }
    else
    {
      /* Check if the configured data direction is LSB first */
      if(LpPBChannelConfig->blDirection == SPI_TRUE)
      {
        /* Load Tx data portion of the local variable with LSB first */
        LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[0];
      }
      else
      {
        /* Load Tx data portion of the local variable with MSB first */
        LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[1];
      }

      /* Set the EDL bit in the local union variable */
      LunDataAccess1.Tst_ByteAccess.ucRegData3 = SPI_SET_EDL;

      /* Set the flag for indicating EDL */
      Spi_GblTxEDL = SPI_TRUE;
      Spi_GblRxEDL = SPI_TRUE;
    } /* End of !if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE) */
    #endif /* End of (SPI_8BIT_DATA_WIDTH == STD_ON) */

    #if(SPI_DMA_MODE_ENABLE == STD_OFF)

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
    #endif

    if(Spi_GddAsyncMode == SPI_INTERRUPT_MODE)
    {
      *LpHWUnitInfo->pTxImrAddress &= LpHWUnitInfo->ucTxImrMask;
      *LpHWUnitInfo->pRxImrAddress &= LpHWUnitInfo->ucRxImrMask;
    }

    /* Check if critical section protection is required */
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Enable relevant interrupts */
    SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
    #endif

    #endif

    #if(SPI_DMA_MODE_ENABLE == STD_ON)
    if(LddNoOfBuffers > SPI_ONE)
    #endif
    {
      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Disable relevant interrupts */
      SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
      #endif

      /* Load the value of the local union variable to TX0W register */
      LpCsigUserBaseAddr->ulCSIGTX0W = LunDataAccess1.ulRegData;

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Enable relevant interrupts */
      SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
      #endif

    }

    #endif /* End of (SPI_CSIG_CONFIGURED == STD_ON) */
  } /* End of if(LddHWUnit <= SPI_MAX_NUM_OF_CSIG) */
  #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
  else
  #endif
  {
    #if(SPI_CSIH_CONFIGURED == STD_ON)
    /* Get the user base address of the HW Unit */
    LpCsihUserBaseAddr =
                  (Tdd_Spi_CsihUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;
    /* Get the os base address of the HW Unit */
    LpCsihOsBaseAddr = (Tdd_Spi_CsihOsRegs *)LpHWUnitInfo->pHwOsBufferAddress;

    /* MISRA Rule         : 11.3                                          */
    /* Message            : A cast should not be performed between a      */
    /*                      pointer type and an integral type.            */
    /* Reason             : This is to access the hardware registers.     */
    /* Verification       : However, this part of the code is verified    */
    /*                      manually and it is not having any impact.     */

    #if ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON))
    if((LucHWMemoryMode == SPI_DIRECT_ACCESS_MODE_CONFIGURED) ||
       (LucHWMemoryMode == SPI_FIFO_MODE_CONFIGURED))
    {
      /* MISRA Rule         : 21.1                                */
      /* Message            : Indexing array with value that will */
      /*                      apparently be out of bounds.        */
      /* Reason             : It is used for array indexing       */
      /* Verification       : However, part of the code           */
      /*                      is verified manually and            */
      /*                      it is not having any impact         */

      /* Get the index of job list for the current job from the job queue */
      LddJobListIndex =  Spi_GaaJobQueue[Spi_GddQueueIndex[LucHWMemoryMode]];
    }
    #endif

    #if ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON))
    if((LucHWMemoryMode == SPI_DUAL_BUFFER_MODE_CONFIGURED) ||
       (LucHWMemoryMode == SPI_TX_ONLY_MODE_CONFIGURED))
    {
      LddSeqIndex = Spi_GaaSeqQueue[Spi_GddQueueIndex[LucHWMemoryMode]];

      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact    */
      LpSeqConfig = Spi_GpFirstSeq + LddSeqIndex;

      /* Get the job list index of the last job of the sequence */
      LddJobListIndex = LpSeqConfig->ddJobListIndex;

      /* Get the index of the job list for the first job of the sequence */
      LddJobListIndex += ((LpSeqConfig->ddNoOfJobs) - SPI_ONE);
    }
    #endif

    /* Get the pointer to the respective job list index  */
    /* MISRA Rule         : 17.4                             */
    /* Message            : Performing pointer arithmetic    */
    /* Reason             : It is used to achieve            */
    /*                      better throughput.               */
    /* Verification       : However, part of the code        */
    /*                      is verified manually and         */
    /*                      it is not having any impact.     */
    LpJobList = Spi_GpFirstJobList + LddJobListIndex;

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
    /* Get the index of the job */
    LddJobIndex = LpJobList->ddJobIndex;

    /* Get the pointer to the job structure */
    /* MISRA Rule         : 17.4                              */
    /* Message            : Performing pointer arithmetic     */
    /* Reason             : It is used to achieve             */
    /*                      better throughput.                */
    /* Verification       : However, part of the code         */
    /*                      is verified manually and          */
    /*                      it is not having any impact.      */
    LpJobConfig = Spi_GpFirstJob + LddJobIndex;

    if(LucHWMemoryMode != SPI_DIRECT_ACCESS_MODE_CONFIGURED)
    {
      /* Get the pointer to the post-build structure of the channel */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact.   */
      LpPBChannelConfig = Spi_GpFirstChannel + (*LpJobConfig->pChannelList);
      LpLTChannelConfig = &Spi_GstChannelLTConfig[*LpJobConfig->pChannelList];
    }

    LpChipSelect = (Spi_GpConfigPtr->pCSArray)+(LpJobConfig->ucCSArrayIndex);

    /* To load configuration register, read the existing value to  */
    /* local union variable */
    LunDataAccess1.ulRegData =
                LpCsihOsBaseAddr->ulCSIHCFG[*LpChipSelect];

    /* Check if all channel properties are same or different in the sequence */
    if((LpJobConfig->blIsChannelPropSame == SPI_FALSE) ||
       (Spi_GblChannelSameFlag == SPI_TRUE))
    {
      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Disable relevant interrupts */
      SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
      #endif

      /* Clear the flag after its use */
      Spi_GblChannelSameFlag = SPI_FALSE;

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Enable relevant interrupts */
      SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
      #endif

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Disable relevant interrupts */
      SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
      #endif

      /* Reset the PWR bit since other registers need to be written */
      LpMainUserBaseAddr->ucMainCTL0 &= SPI_RESET_PWR;

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Enable relevant interrupts */
      SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
      #endif

      /* Load the local variable with data width */

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
      /* Load the local variable with data width */
      LunDataAccess1.ucRegData4[3] = \
                LunDataAccess1.ucRegData4[3] & SPI_DLS_MASK;
      LunDataAccess1.ucRegData4[3] = \
                LunDataAccess1.ucRegData4[3] | LpPBChannelConfig->ucDLSSetting;

      /* Load the local variable with configured direction */
      if(LpPBChannelConfig->blDirection == SPI_TRUE)
      {
        /* Direction is LSB, set DIR bit */
        LunDataAccess1.ucRegData4[2] |= SPI_SET_DIR_LSB;
      }
      else
      {
        /* Direction is MSB, reset DIR bit */
        LunDataAccess1.ucRegData4[2] &= SPI_RESET_DIR_LSB;
      }
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
      /* Get the number of chip selects configured */
      LucVar = LpJobConfig->ucNoOfCS;

      do
      {
        /* Check if critical section protection is required */
        #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
        /* Disable relevant interrupts */
        SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
        #endif

        /* Initialize CSIH configuration register */
        LpCsihOsBaseAddr->ulCSIHCFG[*LpChipSelect] = LunDataAccess1.ulRegData;

        /* Check if critical section protection is required */
        #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
        /* Enable relevant interrupts */
        SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
        #endif

        LucVar--;

        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        LpChipSelect++;
      }while(LucVar > SPI_ZERO);

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Disable relevant interrupts */
      SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
      #endif

      #if (SPI_DIRECT_ACCESS_MODE == STD_ON)
      if(LucHWMemoryMode == SPI_DIRECT_ACCESS_MODE_CONFIGURED)
      {
        /* Set the SLIT bit */
        LpMainOsBaseAddr->ulMainCTL1 |= SPI_SET_SLIT;
      }
      #endif

      LpMainUserBaseAddr->ucMainCTL0 |= SPI_SET_PWR;

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Enable relevant interrupts */
      SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
      #endif
    }

    #if (SPI_DIRECT_ACCESS_MODE == STD_ON)
    if(LucHWMemoryMode == SPI_DIRECT_ACCESS_MODE_CONFIGURED)
    {
      #if(SPI_DMA_MODE_ENABLE == STD_ON)
      if(Spi_GddAsyncMode == SPI_INTERRUPT_MODE)
      {

          /* Function call to configure the DMA reception */
          Spi_RxDmaConfig(LpJobConfig, Spi_GstCurrentCommData.pCurrentRxData,
                                                             LddNoOfBuffers);
          Spi_GddDmaTxData = LpPBChannelConfig->ddDefaultData;
          /* Function call to configure the DMA transmission */
          Spi_TxDmaConfig(LpJobConfig, LpNextTxData, LddNoOfBuffers);
      }
      #endif

      /* Take a local union variable to construct the value for TX0W register */
      LunDataAccess1.ulRegData = SPI_ZERO;

      /* Set chip select in the local union variable */
      LunDataAccess1.Tst_ByteAccess.ucRegData2 = LpJobConfig->ucChipSelect;

      #if(SPI_8BIT_DATA_WIDTH == STD_ON)
      /* Data width is maximum 8-bit. Hence, load Tx data portion of the */
      /* local variable with the 8-bit data to be transmitted */
      LunDataAccess1.Tst_ByteAccess.usRegData1 = (uint16)LddData;
      #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
      /* Data width is maximum 16-bit. Hence, load Tx data portion of the */
      /* local variable with the 16-bit data to be transmitted */
      LunDataAccess1.Tst_ByteAccess.usRegData1 = LddData;
      #else
      /* Data width is maximum 32-bit. Tx data needs to be split to */
      /* LS Byte and MS Byte. Hence, load the Tx data to local union variable */
      #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
      if((LpPBChannelConfig->blEDLEnabled == SPI_TRUE)&& \
                        (LpPBChannelConfig->blDirection != SPI_TRUE) && \
                      (LpPBChannelConfig->ucDLSSetting <= SPI_FIFTEEN) && \
                       (LpPBChannelConfig->ucDLSSetting >= SPI_ONE))
      {
        LddData = LddData << (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
      #endif
      LunDataAccess2.ulRegData = LddData;
      #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
      LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[0]  \
      >> (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
      LunDataAccess2.usRegData5[0] = LunDataAccess1.Tst_ByteAccess.usRegData1;
      }
      else
      {
        LunDataAccess2.ulRegData = LddData;
      }
      #endif

      /* Since data width is maximum 32-bit, check if the the data width of */
      /* requested channel is more than 16 bits */
      if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE)
      {
        /* Data width is maximum 16-bit. Hence, load LSB portion of the */
        /* local variable with the 16-bit data to be transmitted */
        LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[0];
      }
      else
      {
        /* Check if the configured data direction is LSB first */
        if(LpPBChannelConfig->blDirection == SPI_TRUE)
        {
          /* Load Tx data portion of the local variable with LSB first */
        LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[0];
        }
        else
        {
          /* Load Tx data portion of the local variable with MSB first */
          LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                               LunDataAccess2.usRegData5[1];
        }

        /* Set the EDL bit in the local union variable */
        LunDataAccess1.Tst_ByteAccess.ucRegData3 |= SPI_SET_EDL;

        /* Set the flag for indicating EDL */
        Spi_GblTxEDL = SPI_TRUE;
        Spi_GblRxEDL = SPI_TRUE;
      } /* End of !if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE) */
      #endif /* End of (SPI_8BIT_DATA_WIDTH == STD_ON) */

      /* Check if the buffer is last buffer of the channel */
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
      if(LddNoOfBuffers == SPI_ONE)
      {
        /* Check if it is last channel in the job */
        if((LpPBChannelConfig->ucChannelInfo == SPI_ONE) ||
           (LpPBChannelConfig->ucChannelInfo == SPI_TWO))
        {
          /* Since buffer is last buffer of the job or */
          /* the sequence, set EOJ */
          LunDataAccess1.Tst_ByteAccess.ucRegData3 |= SPI_SET_EOJ;
        }
      } /* End of if(LddNoOfBuffers == SPI_ONE) */

      #if(SPI_DMA_MODE_ENABLE == STD_OFF)

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Disable relevant interrupts */
      SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
      #endif

      if(Spi_GddAsyncMode == SPI_INTERRUPT_MODE)
      {
        *LpHWUnitInfo->pTxImrAddress &= LpHWUnitInfo->ucTxImrMask;
        *LpHWUnitInfo->pRxImrAddress &= LpHWUnitInfo->ucRxImrMask;
      }

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Enable relevant interrupts */
      SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
      #endif

      #endif

      #if(SPI_DMA_MODE_ENABLE == STD_ON)
      if(LddNoOfBuffers > SPI_ONE)
      #endif
      {
        /* Check if critical section protection is required */
        #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
        /* Disable relevant interrupts */
        SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
        #endif

        /* Load the value of the local union variable to TX0W register */
        LpCsihUserBaseAddr->ulCSIHTX0W = LunDataAccess1.ulRegData;

        /* Check if critical section protection is required */
        #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
        /* Enable relevant interrupts */
        SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
        #endif

      }
    } /* End of if(LucHWMemoryMode == SPI_DIRECT_ACCESS_CONFIGURED) */
    #endif /* End of (SPI_DIRECT_ACCESS_MODE == STD_ON) */

    /* MISRA Rule         : 10.1                           */
    /* Message            : Implicit conversion            */
    /* Reason             : Both sides data type is same.  */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */
    #if ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON))
    if((LucHWMemoryMode == SPI_DUAL_BUFFER_MODE_CONFIGURED) ||
       (LucHWMemoryMode == SPI_TX_ONLY_MODE_CONFIGURED))
    {
      LpLTChannelConfig =  &Spi_GstChannelLTConfig[*LpJobConfig->pChannelList];

      LddBufferIndex = LpLTChannelConfig->ddBufferIndex;

      /* Take a local union variable to construct the value for MCTL2 */
      /* register and load the local union variable with SOP value */
      LunDataAccess1.Tst_ByteAccess.usRegData1 = LddBufferIndex;

      if (LucHWMemoryMode == SPI_DUAL_BUFFER_MODE_CONFIGURED)
      {
        LunDataAccess1.Tst_ByteAccess.ucRegData2 =
                                 (uint8)(*Spi_GpCurrentNotifyIdx);
        if(Spi_GddAsyncMode == SPI_INTERRUPT_MODE)
        {
          *LpHWUnitInfo->pTxImrAddress &= LpHWUnitInfo->ucTxImrMask;
        }
      }
      #if (SPI_TX_ONLY_MODE == STD_ON)
      else
      {
        LunDataAccess1.Tst_ByteAccess.ucRegData2 =
                            (uint8)Spi_GstTxOnlyCurrentCommData.ddNoOfBuffers;

        if(Spi_GddAsyncMode == SPI_INTERRUPT_MODE)
        {
          #if(SPI_DMA_MODE_ENABLE == STD_ON)
          Spi_RxDmaConfig(LpJobConfig,
                          Spi_GstTxOnlyCurrentCommData.pCurrentRxData,
                          Spi_GstTxOnlyCurrentCommData.ddNoOfBuffers);
          #else
          *LpHWUnitInfo->pRxImrAddress &= LpHWUnitInfo->ucRxImrMask;
          #endif
        }
      }
      #endif

      LunDataAccess1.Tst_ByteAccess.ucRegData3 = SPI_SET_BTST;
     /* Check if critical section protection is required */
     #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
     /* Disable relevant interrupts */
     SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
     #endif
      /* Load the MCTL2 register to start the communication */
      LpCsihUserBaseAddr->ulCSIHMCTL2 = LunDataAccess1.ulRegData;
     /* Check if critical section protection is required */
     #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
     /* Disable relevant interrupts */
     SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
     #endif
    }
    #endif /* End of ((SPI_DUAL_BUFFER_MODE == STD_ON) ||
                      (SPI_TX_ONLY_MODE == STD_ON)) */

    #if (SPI_FIFO_MODE == STD_ON)
    if(LucHWMemoryMode == SPI_FIFO_MODE_CONFIGURED)
    {
     /* Check if critical section protection is required */
     #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
     /* Disable relevant interrupts */
     SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
     #endif
      /* Enable transmit interrupt */
      *LpHWUnitInfo->pTxImrAddress &= LpHWUnitInfo->ucTxImrMask;

      /* Load FES value as zero */
      LpCsihUserBaseAddr->ulCSIHMCTL1 = SPI_ZERO;

      LpCurrentTxChannelList = LpJobConfig->pChannelList;
      LddNoOfChannels = LpJobConfig->ddNoOfChannels;
     /* Check if critical section protection is required */
     #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
     /* Disable relevant interrupts */
     SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
     #endif
      do
      {
        /* Get the pointer to the post-build structure of the channel */
        /* MISRA Rule         : 17.4                           */
        /* Message            : Performing pointer arithmetic  */
        /* Reason             : It is used to achieve          */
        /*                      better throughput.             */
        /* Verification       : However, part of the code      */
        /*                      is verified manually and       */
        /*                      it is not having any impact.   */
        LpPBChannelConfig = Spi_GpFirstChannel + (*LpCurrentTxChannelList);
        LpLTChannelConfig = &Spi_GstChannelLTConfig[(*LpCurrentTxChannelList)];

        LddBufferIndex = LpLTChannelConfig->ddBufferIndex;

        #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
        /* Check if the buffer type is internal buffer */
        if((LpPBChannelConfig->ucChannelBufferType) == SPI_ZERO)
        #endif
        {
          #if(SPI_INTERNAL_RW_BUFFERS == STD_ON)
          /* Update the RAM variable for Tx pointer with channel IB index */
          /* MISRA Rule         : 17.4                           */
          /* Message            : Performing pointer arithmetic  */
          /* Reason             : It is used to achieve          */
          /*                      better throughput.             */
          /* Verification       : However, part of the code      */
          /*                      is verified manually and       */
          /*                      it is not having any impact    */
          LpNextTxData = &Spi_GaaChannelIBWrite[LddBufferIndex];

          /* Update the RAM variable for number of buffers of the channel */
          LddNoOfBuffers = LpLTChannelConfig->ddNoOfBuffers;
          #endif
        }

        #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
        else if((LpPBChannelConfig->ucChannelBufferType) == SPI_ONE)
        #endif

        {
          #if(SPI_EB_CONFIGURED == STD_ON)
          /* Update the RAM variable for Tx pointer with channel EB
             source pointer */
          LpNextTxData = Spi_GaaChannelEBData[LddBufferIndex].pSrcPtr;

          /* Update the local counter with number of buffers of the channel */
          LddNoOfBuffers = Spi_GaaChannelEBData[LddBufferIndex].ddEBLength;
          #endif
        }

        #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
        else
        {
          /* To avoid MISRA warning */
        }
        #endif

        /* If the selected asynchronous mode is DMA mode */
        #if (SPI_DMA_MODE_ENABLE == STD_ON)
        if((Spi_GddAsyncMode == SPI_INTERRUPT_MODE) &&
           ((LpJobConfig->ucTxDmaDeviceIndex) == SPI_INVALID_DMAUNIT))
        #endif
        {
          do
          {
           /* Get the data to be transmitted */
           /* MISRA Rule   : 9.1                                            */
           /* Message      : The variable '-identifier-' is apparently      */
           /*                unset at this point.                           */
           /* Reason       : This variable is initialized at two places     */
           /*                under                                          */
           /*                different pre-compile options                  */
           /* Verification : However, it is manually verified that at least */
           /*                one of the pre-compile options will be ON and  */
           /*                hence this variable will be always initialized */
           /*                Hence,this is not having any impact.           */
            if(LpNextTxData == NULL_PTR)
            {
              LddData = LpPBChannelConfig->ddDefaultData;
            }
            else
            {
              LddData = *LpNextTxData;

              /* Increment the pointer to the buffer */
              /* MISRA Rule   : 17.4                                          */
              /* Message      : Increment or decrement operation performed on */
              /*              : pointer                                       */
              /* Reason       : To access the pointer in optimized            */
              /*                way in this function                          */
              /* Verification : However, part of the code is verified         */
              /*                manually and it is not having any impact      */
              LpNextTxData++;
            }

           /* Take a local union variable to construct the value for TX0W */
           /* register */
           LunDataAccess1.ulRegData = SPI_ZERO;

           /* Set chip select in the local union variable */
           LunDataAccess1.Tst_ByteAccess.ucRegData2 = LpJobConfig->ucChipSelect;

            #if(SPI_8BIT_DATA_WIDTH == STD_ON)
            /* Data width is maximum 8-bit. Hence, load Tx data portion of */
            /* the local variable with the 8-bit data to be transmitted */
            LunDataAccess1.Tst_ByteAccess.usRegData1 = (uint16)LddData;
            #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
            /* Data width is maximum 16-bit. Hence, load Tx data portion of */
            /* the local variable with the 16-bit data to be transmitted */
            LunDataAccess1.Tst_ByteAccess.usRegData1 = LddData;
            #else
            /* Data width is maximum 32-bit. Tx data needs to be split to */
            /* LS Byte and MS Byte. Hence, load the Tx data to local union */
            /* variable */
            #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
            if((LpPBChannelConfig->blEDLEnabled == SPI_TRUE)&& \
                           (LpPBChannelConfig->blDirection != SPI_TRUE) && \
                           (LpPBChannelConfig->ucDLSSetting <= SPI_FIFTEEN) && \
                            (LpPBChannelConfig->ucDLSSetting >= SPI_ONE))
            {
              LddData = LddData << \
                              (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
              #endif
              LunDataAccess2.ulRegData = LddData;
              #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
              LunDataAccess1.Tst_ByteAccess.usRegData1 = \
                                LunDataAccess2.usRegData5[0] >> (SPI_SIXTEEN - \
                                (LpPBChannelConfig->ucDLSSetting));
              LunDataAccess2.usRegData5[0] = \
                                      LunDataAccess1.Tst_ByteAccess.usRegData1;
            }
            else
            {
              LunDataAccess2.ulRegData = LddData;
            }
            #endif

            /* Since data width is maximum 32-bit, check if the the data */
            /* of requested channel is more than 16 bits */
            if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE)
            {
              /* Data width is maximum 16-bit. Hence, load LSB portion of the */
              /* width local variable with the 16-bit data to be transmitted */
              LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                                   LunDataAccess2.usRegData5[0];
            }
            else
            {
              /* Check if the configured data direction is LSB first */
              if(LpPBChannelConfig->blDirection == SPI_TRUE)
              {
                /* Load Tx data portion of the local variable with LSB first */
                LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                                  LunDataAccess2.usRegData5[0];
              }
              else
              {
                /* Load Tx data portion of the local variable with MSB first */
                LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                                   LunDataAccess2.usRegData5[1];
              }

              /* Set the EDL bit in the local union variable */
              LunDataAccess1.Tst_ByteAccess.ucRegData3 |= SPI_SET_EDL;

              /* Load the value of the local union variable to TX0W register */
              LpCsihUserBaseAddr->ulCSIHTX0W = LunDataAccess1.ulRegData;

              /* Reset the EDL bit in the local union variable */
              LunDataAccess1.Tst_ByteAccess.ucRegData3 &= SPI_RESET_EDL;

              /* Check the configured data direction again to load other part */
              /* of data */
              if(LpPBChannelConfig->blDirection == SPI_TRUE)
              {
                /* Load Tx data portion of the local variable with MSB */
                LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                         LunDataAccess2.usRegData5[1];
              }
              else
              {
                /* Load Tx data portion of the local variable with LSB */
                LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                         LunDataAccess2.usRegData5[0];
              }
            } /* End of !if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE) */
            #endif /* End of (SPI_8BIT_DATA_WIDTH == STD_ON) */

            /* Check if the buffer is last buffer of the channel */
            /* MISRA Rule    : 9.1                                           */
            /* Message       : The variable '-identifier-' is apparently     */
            /*                 unset at this point.                          */
            /* Reason        : This variable is initialized at two places    */
            /*                 under different pre-compile options.          */
            /* Verification  : However, it is manually verified that at least*/
            /*                 one of the pre-compile options will be ON and */
            /*                 hence this variable will be always initialized*/
            /*                 Hence,this is not having any impact.          */
            if((LddNoOfBuffers == SPI_ONE) && (LddNoOfChannels == SPI_ONE))
            {
              /* Check if it is last channel in the job */
              if(LpPBChannelConfig->ucChannelInfo == SPI_ONE)
              {
                /* Since buffer is last buffer of the job and not last buffer */
                /* of the sequence, set only EOJ */
                LunDataAccess1.Tst_ByteAccess.ucRegData3 |= SPI_SET_EOJ;
              }
              else if(LpPBChannelConfig->ucChannelInfo == SPI_TWO)
              {
                /* Since last buffer of sequence, set both CIRE and EOJ */
                LunDataAccess1.Tst_ByteAccess.ucRegData3 |= SPI_SET_CIREEOJ;
              }
              /* To avoid MISRA warning */
              else
              {

              }
            } /* End of if((LddNoOfBuffers == SPI_ONE) &&
                           (LddNoOfChannels == SPI_ONE)) */

            /* Load the value of the local union variable to TX0W register */
            LpCsihUserBaseAddr->ulCSIHTX0W = LunDataAccess1.ulRegData;

            /* Decrement the counter for number of buffers */
            LddNoOfBuffers--;

          }while(LddNoOfBuffers > SPI_ZERO);
        }
        #if (SPI_DMA_MODE_ENABLE == STD_ON)
        else
        {
          /* Check if critical section protection is required */
          #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
          /* Disable relevant interrupts to protect this critical section */
          SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
          #endif
          Spi_GddDmaTxData = LpPBChannelConfig->ddDefaultData;
          Spi_GstFifoCurrentCommData.pCurrentChannelList =
                                                       LpCurrentTxChannelList;
          Spi_GstFifoCurrentCommData.ddNoOfChannels = LddNoOfChannels;
          Spi_GstFifoCurrentCommData.ddJobIndex = LddJobIndex;
          Spi_GstFifoCurrentCommData.ucDmaDeviceIndex =
                                               LpJobConfig->ucTxDmaDeviceIndex;

          /* Check if critical section protection is required */
          #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
          /* Enable relevant interrupts to protect this critical section */
          SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
          #endif

          /* MISRA Rule     : 9.1                                            */
          /* Message        : The variable '-identifier-' is apparently      */
          /*                  unset at this point.                           */
          /* Reason         : This variable is initialized at two places     */
          /*                  under                                          */
          /*                  different pre-compile options                  */
          /* Verification   : However, it is manually verified that at least */
          /*                  one of the pre-compile options will be ON and  */
          /*                  hence this variable will be always initialized */
          /*                  Hence,this is not having any impact.           */
          Spi_TxDmaConfig(LpJobConfig, LpNextTxData, LddNoOfBuffers);
          /* Break the loop */
          LddNoOfChannels = SPI_ONE;
        }
        #endif
        /* Increment the pointer to the channel */
        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        LpCurrentTxChannelList++;

        /* Decrement the counter for number of channels */
        LddNoOfChannels--;
      }while(LddNoOfChannels > SPI_ZERO);
    } /* if(LucHWMemoryMode == SPI_FIFO_CONFIGURED) */
    #endif /* End of (SPI_FIFO_MODE == STD_ON) */
    #endif /* End of (SPI_CSIH_CONFIGURED == STD_ON) */
  } /* End of !if(LddHWUnit <= SPI_MAX_NUM_OF_CSIG) */
}
#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of ((SPI_LEVEL_DELIVERED == SPI_ONE) ||
                  (SPI_LEVEL_DELIVERED == SPI_TWO)) */

/*******************************************************************************
** Function Name      : Spi_TransmitISR
**
** Service ID         : Not Applicable
**
** Description        : This is the interrupt service routine for transmission
**
** Sync/Async         : Synchronous
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Spi_HWUnitType LddHWUnit
**                      uint8 LucHWMemoryMode
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      Spi_GaaHWIndexMap
**                      Spi_GstHWUnitInfo
**
**                      Function Invoked:
**                      Spi_ProcessSequence
**                      Spi_RxDmaConfig
**
*******************************************************************************/
#if ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON) \
        || (SPI_DUAL_BUFFER_MODE == STD_ON))

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_TransmitISR
                 (Spi_HWUnitType LddHWUnit, uint8 LucHWMemoryMode)
{
  #if (SPI_DIRECT_ACCESS_MODE == STD_ON)
  P2CONST(Spi_DataType,AUTOMATIC,SPI_CONFIG_DATA) LpNextTxData;
  #endif

  #if (((SPI_DIRECT_ACCESS_MODE == STD_ON) && \
               (SPI_CSIH_CONFIGURED == STD_ON)) || (SPI_FIFO_MODE == STD_ON))
  P2CONST(Tdd_Spi_ChannelLTConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpLTChannelConfig;
  #endif
  P2CONST(Tdd_Spi_HWUnitInfo, AUTOMATIC, SPI_PRIVATE_CONST)LpHWUnitInfo;

  #if (SPI_DIRECT_ACCESS_MODE == STD_ON)
  P2CONST(Spi_ChannelType,AUTOMATIC,SPI_CONFIG_CONST) LpTxCurrentChannelList;
  #endif

  #if(SPI_CSIG_CONFIGURED == STD_ON)
  P2VAR(Tdd_Spi_CsigUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsigUserBaseAddr;
  #endif

  #if(SPI_CSIH_CONFIGURED == STD_ON)
  P2VAR(Tdd_Spi_CsihUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsihUserBaseAddr;
  #endif

  #if (SPI_FIFO_MODE == STD_ON)
  P2CONST(Tdd_Spi_JobListType,AUTOMATIC,SPI_CONFIG_CONST) LpJobList;

  P2CONST(Spi_ChannelType,AUTOMATIC,SPI_CONFIG_CONST) LpCurrentTxChannelList;
  P2CONST(Tdd_Spi_JobConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpJobConfig;
  #endif

  #if ((SPI_FIFO_MODE == STD_ON) || (SPI_DUAL_BUFFER_MODE == STD_ON))
  Spi_JobType LddJobIndex;
  #endif

  #if((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_DUAL_BUFFER_MODE == STD_ON))
  /*  MISRA Rule         : 18.4                                           */
  /*  Message            : Unions shall not be used.                      */
  /*  Reason             : To access the converted values                 */
  /*  Verification       : However, part of the code is verified manually */
  /*                       and it is not having any impact.               */
  Tun_Spi_DataAccess LunDataAccess1;
  #endif

  #if ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON))
  P2CONST(Tdd_Spi_ChannelPBConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpPBChannelConfig;
  /*  MISRA Rule         : 18.4                                           */
  /*  Message            : Unions shall not be used.                      */
  /*  Reason             : To access the converted values                 */
  /*  Verification       : However, part of the code is verified manually */
  /*                       and it is not having any impact.               */
  Tun_Spi_DataAccess LunDataAccess2;
  Spi_DataType LddData;
  #endif

  #if ((SPI_FIFO_MODE == STD_ON) || (SPI_DUAL_BUFFER_MODE == STD_ON))
  Spi_NumberOfDataType LddNoOfBuffers;
  Spi_NumberOfDataType LddBufferIndex;
  #endif

  #if (SPI_FIFO_MODE == STD_ON)
  Spi_ChannelType LddNoOfChannels;
  P2VAR(Spi_DataType,AUTOMATIC,SPI_CONFIG_DATA) LpCurrentRxData;
  #endif

  #if (SPI_DUAL_BUFFER_MODE == STD_ON)
  Spi_NumberOfDataType LddNotifyFuncIndex;
  Spi_JobType LddNoOfJobs;
  Spi_SequenceType LddSeqIndex;
  P2CONST(Tdd_Spi_SequenceConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpSeqConfig;
  P2CONST(Tdd_Spi_JobListType,AUTOMATIC,SPI_CONFIG_CONST) LpJobListTmptr;
  #endif

  /* Get the pointer to the structure of HW Unit information */
  LpHWUnitInfo = &Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[LddHWUnit]];

  #if(SPI_CSIH_CONFIGURED == STD_ON)
  /* Get the base address of the HW Unit */
  LpCsihUserBaseAddr =
               (Tdd_Spi_CsihUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;
  #endif

  #if (((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON)) \
        &&(SPI_DUAL_BUFFER_MODE == STD_ON))
  if(LucHWMemoryMode < SPI_TWO)
  #endif
  {
    if(LucHWMemoryMode == SPI_DIRECT_ACCESS_MODE_CONFIGURED)
    {
      #if (SPI_DIRECT_ACCESS_MODE == STD_ON)
      LpTxCurrentChannelList =
           Spi_GstCurrentCommData.pCurrentTxChannelList;

      /* Increment the source pointer */
      /* MISRA Rule         : 17.4                                          */
      /* Message            : Increment or decrement operation performed on */
      /*                    : pointer                                       */
      /* Reason             : To access the pointer in optimized            */
      /*                      way in this function                          */
      /* Verification       : However, part of the code is verified         */
      /*                      manually and it is not having any impact      */
      LpTxCurrentChannelList--;

      /* Get the pointer to the post-build structure of the channel */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact.   */
      LpPBChannelConfig = Spi_GpFirstChannel + (*LpTxCurrentChannelList);
      #if(SPI_CSIH_CONFIGURED == STD_ON)
      LpLTChannelConfig = &Spi_GstChannelLTConfig[(*LpTxCurrentChannelList)];
      #endif

      /* Get the pointer to the next data to be transmitted */
      LpNextTxData = Spi_GstCurrentCommData.pNextTxData;

      if(Spi_GblTxEDL == SPI_TRUE)
      {
        if(LpNextTxData == NULL_PTR)
        {
          LddData = LpPBChannelConfig->ddDefaultData;
        }
        else
        {
          LddData = *LpNextTxData;
        }

        /* Message(3:3198)    : Assignment is redundant.                    */
        /*                      The value of LunDataAccess1.ulRegData       */
        /*                      is never used before being                  */
        /*                      modified.                                   */
        /* Reason             : This is done for proper initialisation of   */
        /*                      the union variable.                         */
        /* Verification       : However, part of the code is                */
        /*                      verified manually and it is not             */
        /*                      having any impact.                          */

        /* Take a local union variable to construct the value for TX0W */
        /* register */
        LunDataAccess1.ulRegData = SPI_ZERO;

        /* Data width is maximum 32-bit. Tx data needs to be split to */
        /* LS Byte and MS Byte. Hence, load the Tx data to local variable */
        #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
        if((LpPBChannelConfig->blEDLEnabled == SPI_TRUE)&& \
                          (LpPBChannelConfig->blDirection != SPI_TRUE) && \
                        (LpPBChannelConfig->ucDLSSetting <= SPI_FIFTEEN) && \
                         (LpPBChannelConfig->ucDLSSetting >= SPI_ONE))
        {
         LddData = LddData << (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
        #endif
        LunDataAccess2.ulRegData = LddData;
        #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
        LunDataAccess1.Tst_ByteAccess.usRegData1 = LunDataAccess2.usRegData5[0]\
                          >> (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
        LunDataAccess2.usRegData5[0] = LunDataAccess1.Tst_ByteAccess.usRegData1;
        }
        else
        {
          LunDataAccess2.ulRegData = LddData;
        }
        #endif

        /* Check if the configured data direction is LSB first */
        if(LpPBChannelConfig->blDirection == SPI_TRUE)
        {
          /* Load Tx data portion of the local variable with LSB first */
          LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                         LunDataAccess2.usRegData5[1];
        }
        else
        {
          /* Load Tx data portion of the local variable with MSB first */
          LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                        LunDataAccess2.usRegData5[0];
        }

        /* Reset the flag for indicating EDL */
        Spi_GblTxEDL = SPI_FALSE;

        #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
        if(LddHWUnit < SPI_MAX_NUM_OF_CSIG)
        #endif
        {
          #if(SPI_CSIG_CONFIGURED == STD_ON)
          /* Get the base address of the HW Unit */
          LpCsigUserBaseAddr =
                     (Tdd_Spi_CsigUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;
          /* Check if critical section protection is required */
          #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
          /* Disable relevant interrupts */
          SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
          #endif
          /* Load the value of the local union variable to TX0W register */
          LpCsigUserBaseAddr->ulCSIGTX0W = LunDataAccess1.ulRegData;
          #endif
          /* Check if critical section protection is required */
          #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
          /* Enable relevant interrupts */
          SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
          #endif
        }
        #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
        else
        #endif
        {
          #if(SPI_CSIH_CONFIGURED == STD_ON)
          /* Check if the buffer is last buffer of the channel */
          if(LpLTChannelConfig->ddNoOfBuffers == SPI_ONE)
          {
            /* Check if it is last channel in the job */
            if(LpPBChannelConfig->ucChannelInfo == SPI_ONE)
            {
              /* Since buffer is last buffer of the job and not last buffer */
              /* of the sequence, set only EOJ */
              LunDataAccess1.Tst_ByteAccess.ucRegData3 |= SPI_SET_EOJ;
            }
            else if(LpPBChannelConfig->ucChannelInfo == SPI_TWO)
            {
              /* Since last buffer of sequence, set both CIRE and EOJ */
              LunDataAccess1.Tst_ByteAccess.ucRegData3 |= SPI_SET_CIREEOJ;
            }
            /* To avoid MISRA warning */
            else
            {

            }
          }
          /* Check if critical section protection is required */
          #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
          /* Disable relevant interrupts */
          SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
          #endif
          /* Load the value of the local union variable to TX0W register */
          LpCsihUserBaseAddr->ulCSIHTX0W =
                       (LpCsihUserBaseAddr->ulCSIHTX0W & SPI_CSIHTX0W_MASK) |
                        LunDataAccess1.ulRegData;
          /* Check if critical section protection is required */
          #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
          /* Enable relevant interrupts */
          SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
          #endif
          #endif
        }
      }
      else
      {
        /* Check if all buffers in the channel are transmitted */
        if((Spi_GstCurrentCommData.ddNoOfTxBuffers) > SPI_ZERO)
        {
          if(LpNextTxData == NULL_PTR)
          {
            LddData = LpPBChannelConfig->ddDefaultData;
          }
          else
          {
            /* MISRA Rule    : 17.4                                          */
            /* Message       : Increment or decrement operation performed on */
            /*               : pointer                                       */
            /* Reason        : To access the pointer in optimized            */
            /*                 way in this function                          */
            /* Verification  : However, part of the code is verified         */
            /*                 manually and it is not having any impact      */
            LpNextTxData++;
            LddData = *LpNextTxData;
            Spi_GstCurrentCommData.pNextTxData = LpNextTxData;
          }
          /* Decrement the number of buffers to be transmitted */
          /* MISRA Rule    : 17.4                                          */
          /* Message       : Increment or decrement operation performed on */
          /*               : pointer                                       */
          /* Reason        : To access the pointer in optimized            */
          /*                 way in this function                          */
          /* Verification  : However, part of the code is verified         */
          /*                 manually and it is not having any impact      */
          (Spi_GstCurrentCommData.ddNoOfTxBuffers)--;

         #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
         if(LddHWUnit < SPI_MAX_NUM_OF_CSIG)
          #endif
          {
            #if (SPI_CSIG_CONFIGURED == STD_ON)
            /* Get the base address of the HW Unit */
            LpCsigUserBaseAddr =
                  (Tdd_Spi_CsigUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;

            /* Check if critical section protection is required */
            #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
            /* Disable relevant interrupts */
            SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
            #endif
            #if(SPI_8BIT_DATA_WIDTH == STD_ON)
            /* Data width is maximum 8-bit. Hence, load Tx data portion of */
            /* the local variable to TXOH */
            LpCsigUserBaseAddr->usCSIGTX0H = (uint16)LddData;

            #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
            /* Data width is maximum 16-bit. Hence, load Tx data portion of the
            local */
            /* variable with the 16-bit data to be transmitted */
            LpCsigUserBaseAddr->usCSIGTX0H = LddData;
            #else
            /* Take a local union variable to construct the value for TX0W */
            /* register */
            LunDataAccess1.ulRegData = SPI_ZERO;

            /* Data width is maximum 32-bit. Tx data needs to be split to */
            /* LS Byte and MS Byte. Hence, load the Tx data to local variable */
            #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
            if((LpPBChannelConfig->blEDLEnabled == SPI_TRUE)&& \
                              (LpPBChannelConfig->blDirection != SPI_TRUE) && \
                           (LpPBChannelConfig->ucDLSSetting <= SPI_FIFTEEN) && \
                            (LpPBChannelConfig->ucDLSSetting >= SPI_ONE))
            {
              LddData = LddData << \
                           (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
              #endif
              LunDataAccess2.ulRegData = LddData;
              #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
              LunDataAccess1.Tst_ByteAccess.usRegData1 = \
                               LunDataAccess2.usRegData5[0] >> \
                              (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
               LunDataAccess2.usRegData5[0] = \
                                   LunDataAccess1.Tst_ByteAccess.usRegData1;
            }
            else
            {
              LunDataAccess2.ulRegData = LddData;
            }
            #endif

            /* Data width is maximum 32-bit, check if the the data width of */
            /* requested channel is more than 16 bits */
            if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE)
            {
              /* Data width is maximum 16-bit. Hence, load LSB portion of the */
              /* local variable with the 16-bit data to be transmitted */
              LpCsigUserBaseAddr->usCSIGTX0H = LunDataAccess2.usRegData5[0];
            }
            else
            {
              /* Check if the configured data direction is LSB first */
              if(LpPBChannelConfig->blDirection == SPI_TRUE)
              {
                /* Load Tx data portion of the local variable with LSB first */
                LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                               LunDataAccess2.usRegData5[0];
              }
              else
              {
                /* Load Tx data portion of the local variable with MSB first */
                LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                              LunDataAccess2.usRegData5[1];
              }

              /* Set the EDL bit in the local union variable */
              LunDataAccess1.Tst_ByteAccess.ucRegData3 = SPI_SET_EDL;

              /* Set the flag for indicating EDL */
              Spi_GblTxEDL = SPI_TRUE;

              /* Load the value of the local union variable to TX0W register */
              LpCsigUserBaseAddr->ulCSIGTX0W = LunDataAccess1.ulRegData;
            }
            #endif
            /* Check if critical section protection is required */
            #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
            /* Disable relevant interrupts */
            SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
            #endif
            #endif
          }
       #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
         else
          #endif
          {
            #if (SPI_CSIH_CONFIGURED == STD_ON)
            /* Get the base address of the HW Unit */
            LpCsihUserBaseAddr =
                     (Tdd_Spi_CsihUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;
            /* Check if critical section protection is required */
            #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
            /* Disable relevant interrupts */
            SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
            #endif
            #if(SPI_8BIT_DATA_WIDTH == STD_ON)
            /* Data width is maximum 8-bit. Hence, load Tx data portion of */
            /* the local variable to TXOH */
            LpCsihUserBaseAddr->usCSIHTX0H = (uint16)LddData;
            #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
            /* Data width is maximum 16-bit. Hence, load Tx data portion of the
            local */
            /* variable with the 16-bit data to be transmitted */
            LpCsihUserBaseAddr->usCSIHTX0H = LddData;
            #else
            /* Take a local union variable to construct the value for TX0W */
            /* register */
            LunDataAccess1.ulRegData = SPI_ZERO;

            /* Data width is maximum 32-bit. Tx data needs to be split to */
            /* LS Byte and MS Byte. Hence, load the Tx data to local variable */
            #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
            if((LpPBChannelConfig->blEDLEnabled == SPI_TRUE)&& \
                    (LpPBChannelConfig->blDirection != SPI_TRUE) && \
                 (LpPBChannelConfig->ucDLSSetting <= SPI_FIFTEEN) && \
                        (LpPBChannelConfig->ucDLSSetting >= SPI_ONE))
            {
              LddData = LddData << \
                              (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
              #endif
              LunDataAccess2.ulRegData = LddData;
              #if (SPI_EXTENDED_DATA_LENGTH == STD_ON)
              LunDataAccess1.Tst_ByteAccess.usRegData1 = \
                              LunDataAccess2.usRegData5[0] >> \
                             (SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting));
              LunDataAccess2.usRegData5[0] = \
                                       LunDataAccess1.Tst_ByteAccess.usRegData1;
            }
            else
            {
              LunDataAccess2.ulRegData = LddData;
            }
            #endif
            /* Data width is maximum 32-bit, check if the the data width of */
            /* requested channel is more than 16 bits */
            if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE)
            {
              /* Data width is maximum 16-bit. Hence, load LSB portion of the */
              /* local variable with the 16-bit data to be transmitted */
              LpCsihUserBaseAddr->usCSIHTX0H = LunDataAccess2.usRegData5[0];
            }
            else
            {
              /* Check if the configured data direction is LSB first */
              if(LpPBChannelConfig->blDirection == SPI_TRUE)
              {
                /* Load Tx data portion of the local variable with LSB first */
                LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                               LunDataAccess2.usRegData5[0];
              }
              else
              {
                /* Load Tx data portion of the local variable with MSB first */
                LunDataAccess1.Tst_ByteAccess.usRegData1 =
                                              LunDataAccess2.usRegData5[1];
              }

              /* Set the EDL bit in the local union variable */
              LunDataAccess1.Tst_ByteAccess.ucRegData3 = SPI_SET_EDL;

              /* Set the flag for indicating EDL */
              Spi_GblTxEDL = SPI_TRUE;

              /* Check if the buffer is last buffer of the channel */
              if(LpLTChannelConfig->ddNoOfBuffers == SPI_ONE)
              {
                /* Check if it is last channel in the job */
                /* MISRA Rule    : 1.1                                      */
                /* Message       : Nesting of control statements exceeds 15 */
                /* Reason        : It is used to achieve                    */
                /*                 better throughput instead of invoking    */
                /*                 a function call                          */
                /* Verification  : However, part of the code                */
                /*                 is verified with compiler and            */
                /*                 it is not having any impact.             */
                if(LpPBChannelConfig->ucChannelInfo == SPI_ONE)
                {
                /* Since buffer is last buffer of the job and not last buffer */
                /* of the sequence, set only EOJ */
                  LunDataAccess1.Tst_ByteAccess.ucRegData3 |= SPI_SET_EOJ;
                }

                /* MISRA Rule    : 1.1                                      */
                /* Message       : Nesting of control statements exceeds 15 */
                /* Reason        : It is used to achieve                    */
                /*                 better throughput instead of invoking    */
                /*                 a function call                          */
                /* Verification  : However, part of the code                */
                /*                 is verified with compiler and            */
                /*                 it is not having any impact.             */
                else if(LpPBChannelConfig->ucChannelInfo == SPI_TWO)
                {
                  /* Since last buffer of sequence, set both CIRE and EOJ */
                  LunDataAccess1.Tst_ByteAccess.ucRegData3 |= SPI_SET_CIREEOJ;
                }
                /* To avoid MISRA warning */
                else
                {

                }
              }
              /* Load the value of the local union variable to TX0W register */
              LpCsihUserBaseAddr->ulCSIHTX0W =
                       (LpCsihUserBaseAddr->ulCSIHTX0W & SPI_CSIHTX0W_MASK) |
                        LunDataAccess1.ulRegData;
            }
            #endif
            /* Check if critical section protection is required */
            #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
            /* Disable relevant interrupts */
            SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
            #endif
            #endif
          }
        }
      }
      #endif /* End of (SPI_DIRECT_ACCESS_MODE == STD_ON) */
    } /* End of if(LucHWMemoryMode == SPI_DIRECT_ACCESS_CONFIGURED) */
    else
    {
      #if (SPI_FIFO_MODE == STD_ON)

      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact   */

      LpJobList = Spi_GpFirstJobList +
                   Spi_GaaJobQueue[Spi_GddQueueIndex[LucHWMemoryMode]];


      /* Get the index of the first job linked to this sequence */
      LddJobIndex = LpJobList->ddJobIndex;

      /* Get the pointer to the job structure */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact    */
      LpJobConfig = Spi_GpFirstJob + LddJobIndex;

      LpCurrentTxChannelList = LpJobConfig->pChannelList;
      LddNoOfChannels = LpJobConfig->ddNoOfChannels;

      do
      {
        /* Get the pointer to the post-build structure of the channel */
        /* MISRA Rule         : 17.4                           */
        /* Message            : Performing pointer arithmetic  */
        /* Reason             : It is used to achieve          */
        /*                      better throughput.             */
        /* Verification       : However, part of the code      */
        /*                      is verified manually and       */
        /*                      it is not having any impact.   */
        LpPBChannelConfig = Spi_GpFirstChannel + (*LpCurrentTxChannelList);

        /* Get the pointer to the link-time structure of the channel */
        LpLTChannelConfig = &Spi_GstChannelLTConfig[(*LpCurrentTxChannelList)];

        LddBufferIndex = LpLTChannelConfig->ddBufferIndex;

        #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
        /* Check if the buffer type is internal buffer */
        if((LpPBChannelConfig->ucChannelBufferType) == SPI_ZERO)
        #endif
        {
          #if(SPI_INTERNAL_RW_BUFFERS == STD_ON)
          /* Update the RAM variable for Rx pointer with channel IB index */
          LpCurrentRxData = &Spi_GaaChannelIBRead[LddBufferIndex];

          /* Update the RAM variable for number of buffers of the channel */
          LddNoOfBuffers = LpLTChannelConfig->ddNoOfBuffers;
          #endif
        }

        #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
        else if((LpPBChannelConfig->ucChannelBufferType) == SPI_ONE)
        #endif

        {
          #if(SPI_EB_CONFIGURED == STD_ON)
          /* Update the RAM variable for Rx pointer */
          LpCurrentRxData = Spi_GaaChannelEBData[LddBufferIndex].pDestPtr;
          /* Update the local counter with number of buffers of the channel */
          LddNoOfBuffers = Spi_GaaChannelEBData[LddBufferIndex].ddEBLength;
          #endif
        }

        #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
        else
        {
          /* To avoid MISRA warning */
        }
        #endif


        /* Get the base address of the HW Unit */
        LpCsihUserBaseAddr =
               (Tdd_Spi_CsihUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;

        /* If the selected asynchronous mode is DMA mode */
        #if (SPI_DMA_MODE_ENABLE == STD_ON)
        if((Spi_GddAsyncMode == SPI_INTERRUPT_MODE) &&
           ((LpJobConfig->ucRxDmaDeviceIndex) == SPI_INVALID_DMAUNIT))
        #endif
        {
          do
          {
            #if(SPI_8BIT_DATA_WIDTH == STD_ON)
            /* Data width is maximum 8-bit. Hence, Receive the data from the */
            /* Rx register to local union variable */
            LunDataAccess2.ulRegData = LpCsihUserBaseAddr->ulCSIHRX0W;
            LddData = LunDataAccess2.ucRegData4[0];

            #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
            LunDataAccess2.ulRegData = LpCsihUserBaseAddr->ulCSIHRX0W;
            /* Load the value from union variable to local variable */
            LddData = LunDataAccess2.usRegData5[0];

            #else
            /* Data width is maximum 32-bit, Check if the the data width of */
            /* requested channel is more than 16 bits */
            if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE)
            {
              LunDataAccess2.ulRegData = LpCsihUserBaseAddr->ulCSIHRX0W;

              LddData = LunDataAccess2.usRegData5[0];
            }
            else
            {
              /* Check if the configured data direction is LSB first */
              if(LpPBChannelConfig->blDirection == SPI_TRUE)
              {
              /* Take a local union variable to construct the value from RX0W */
              /* register */
                LunDataAccess2.ulRegData = LpCsihUserBaseAddr->ulCSIHRX0W;
                LunDataAccess2.usRegData5[1] = LpCsihUserBaseAddr->usCSIHRX0H;
              }
              else
              {
              /* Take a local union variable to construct the value from RX0W */
              /* register */
                LunDataAccess2.ulRegData = LpCsihUserBaseAddr->ulCSIHRX0W;
                LunDataAccess2.usRegData5[1] = LunDataAccess2.usRegData5[0];
                LunDataAccess2.usRegData5[0] = (uint16)
                  ( LpCsihUserBaseAddr->usCSIHRX0H << \
                          ( SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting)));
                LunDataAccess2.ulRegData = (uint32)
                  ( LunDataAccess2.ulRegData >> \
                          ( SPI_SIXTEEN - (LpPBChannelConfig->ucDLSSetting)));
              }

              LddData = LunDataAccess2.ulRegData;
            }
            #endif /* End of (SPI_8BIT_DATA_WIDTH == STD_ON)*/

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
            if(LpCurrentRxData != NULL_PTR)
            {
              *LpCurrentRxData = LddData;

              /* MISRA Rule   : 17.4                                          */
              /* Message      : Increment or decrement operation performed on */
              /*              : pointer                                       */
              /* Reason       : To access the pointer in optimized            */
              /*                way in this function                          */
              /* Verification : However, part of the code is verified         */
              /*                manually and it is not having any impact      */
              LpCurrentRxData++;
            }

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
            LddNoOfBuffers--;
          }while(LddNoOfBuffers > SPI_ZERO);
        }

        /* MISRA Rule        : 9.1                                            */
        /* Message           : The variable '-identifier-' is apparently      */
        /*                     unset at this point.                           */
        /* Reason            : This variable is initialized at two places     */
        /*                     under                                          */
        /*                     different pre-compile options                  */
        /* Verification      : However, it is manually verified that at least */
        /*                     one of the pre-compile options will be ON and  */
        /*                     hence this variable will be always initialized */
        /*                     Hence,this is not having any impact.           */
        #if (SPI_DMA_MODE_ENABLE == STD_ON)
        else
        {
          /* Check if critical section protection is required */
          #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
          /* Disable relevant interrupts to protect this critical section */
          SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
          #endif
          Spi_GstFifoCurrentCommData.pCurrentChannelList =
                                                       LpCurrentTxChannelList;
          Spi_GstFifoCurrentCommData.ddNoOfChannels = LddNoOfChannels;
          Spi_GstFifoCurrentCommData.ddJobIndex = LddJobIndex;
          Spi_GstFifoCurrentCommData.ucDmaDeviceIndex =
                                               LpJobConfig->ucRxDmaDeviceIndex;
          Spi_GddDmaRxData = LpPBChannelConfig->ddDefaultData;

          /* Check if critical section protection is required */
          #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
          /* Enable relevant interrupts to protect this critical section */
          SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
          #endif
          Spi_RxDmaConfig(LpJobConfig, LpCurrentRxData, LddNoOfBuffers);
          LddNoOfChannels = SPI_ONE;
        }
        #endif

        /* Increment the pointer to the channel */
        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        LpCurrentTxChannelList++;

        /* Decrement the counter for number of channels */
        LddNoOfChannels--;
      }while(LddNoOfChannels > SPI_ZERO);

        /* If the selected asynchronous mode is DMA mode */
      #if (SPI_DMA_MODE_ENABLE == STD_ON)
      if((Spi_GddAsyncMode == SPI_INTERRUPT_MODE) &&
         ((LpJobConfig->ucRxDmaDeviceIndex) == SPI_INVALID_DMAUNIT))
      #endif
      {
        /* All channels are transmitted. Hence the job is transmitted */
        Spi_ProcessSequence(LddHWUnit, LucHWMemoryMode);
      }
      #endif
    }
  }

  #if (((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON)) \
      && (SPI_DUAL_BUFFER_MODE == STD_ON))
  else
  #endif
  {
    #if (SPI_DUAL_BUFFER_MODE == STD_ON)
    if(LucHWMemoryMode == SPI_DUAL_BUFFER_MODE_CONFIGURED)
    {
      LddSeqIndex = Spi_GaaSeqQueue[Spi_GddQueueIndex[LucHWMemoryMode]];

      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact.   */
      LpSeqConfig = Spi_GpFirstSeq + LddSeqIndex;

      LddBufferIndex = *Spi_GpCurrentNotifyIdx;

      /* Increment the notify index value */
      /* MISRA Rule         : 17.4                                          */
      /* Message            : Increment or decrement operation performed on */
      /*                    : pointer                                       */
      /* Reason             : To access the pointer in optimized            */
      /*                      way in this function                          */
      /* Verification       : However, part of the code is verified         */
      /*                      manually and it is not having any impact      */
      Spi_GpCurrentNotifyIdx++;

      LddJobIndex = (Spi_JobType)(*Spi_GpCurrentNotifyIdx);

      /* Get the notification function index */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact.   */
      LddNotifyFuncIndex = (Spi_GpFirstJob + LddJobIndex)->ddNotifyFuncIndex;

      /* Check if notification is configured */
      if (LddNotifyFuncIndex != SPI_NO_NOTIFICATION)
      {
        Spi_GstJobFunc[LddNotifyFuncIndex].pSpiJobNotification();
      }

      if(Spi_GddNoOfBuffers < LpSeqConfig->ddNoOfBuffers)
      {
        /* Increment the notify index value */
        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        Spi_GpCurrentNotifyIdx++;

        LddNoOfBuffers = *Spi_GpCurrentNotifyIdx - LddBufferIndex;

        Spi_GddNoOfBuffers += LddNoOfBuffers;

        /* Take a local union variable to construct the value for MCTL2 */
        /* register and load the local union variable with SOP value */
        LunDataAccess1.Tst_ByteAccess.usRegData1 = LddBufferIndex;

        /* Load the local union variable with number of data */
        /* MISRA Rule         : 10.1                                     */
        /* Message            : Implicit conversion                      */
        /* Reason             : Both sides data type is same.            */
        /* Verification       : However, part of the code is verified    */
        /*                      manually and it is not having any impact */
        LunDataAccess1.Tst_ByteAccess.ucRegData2 = (uint8)LddNoOfBuffers;


       /* Set BTST bit */
        LunDataAccess1.Tst_ByteAccess.ucRegData3 = SPI_SET_BTST;
      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Disable relevant interrupts */
      SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
      #endif
        /* Load the MCTL2 register to start the communication */
        LpCsihUserBaseAddr->ulCSIHMCTL2 = LunDataAccess1.ulRegData;

      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Enable relevant interrupts */
      SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
      #endif
      }
      else
      {
        Spi_GddNoOfBuffers = SPI_ZERO;

        /* Update sequence result as OK */
        Spi_GaaSeqResult[LddSeqIndex] = SPI_SEQ_OK;
        /* All channels are transmitted and received. Hence the sequence is */
        /* completed */

        /* Load Job List pointer to temporary variable */
        /* MISRA Rule         : 17.4                           */
        /* Message            : Performing pointer arithmetic  */
        /* Reason             : It is used to achieve          */
        /*                      better throughput.             */
        /* Verification       : However, part of the code      */
        /*                      is verified manually and       */
        /*                      it is not having any impact.   */
        LpJobListTmptr = Spi_GpFirstJobList + (LpSeqConfig->ddJobListIndex);

        /* Load total number of Jobs in current sequence to local variable */
        LddNoOfJobs = LpSeqConfig->ddNoOfJobs;

        do
        {
          /* Update the job result variable */
          Spi_GaaJobResult[LpJobListTmptr->ddJobIndex] = SPI_JOB_OK;

          /* Increment local pointer to the job list */
          /* MISRA Rule       : 17.4                                          */
          /* Message          : Increment or decrement operation performed on */
          /*                    pointer                                       */
          /* Reason           : To access the pointer in optimized            */
          /*                    way in this function                          */
          /* Verification     : However, part of the code is verified         */
          /*                    manually and it is not having any impact      */
          LpJobListTmptr++;
          LddNoOfJobs--;
        }
        while(LddNoOfJobs > SPI_ZERO);

        Spi_ProcessSequence(LddHWUnit, LucHWMemoryMode);
      }
    }
    #endif /* End of (SPI_DUAL_BUFFER_MODE == STD_ON) */
  } /* End of else loop of LucHWMemoryMode < SPI_TWO */
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of ((SPI_LEVEL_DELIVERED == SPI_ONE) ||
                   (SPI_LEVEL_DELIVERED == SPI_TWO)) */

/*******************************************************************************
** Function Name      : Spi_TransmitCancelISR
**
** Service ID         : Not Applicable
**
** Description        : This is the interrupt service routine for transmission
**
** Sync/Async         : Synchronous
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Spi_HWUnitType LddHWUnit
**                      uint8 LucHWMemoryMode
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      Spi_GaaHWIndexMap
**                      Spi_GstHWUnitInfo
**
**                      Function Invoked:
**                      Spi_ProcessSequence
**
*******************************************************************************/
#if((SPI_CANCEL_API == STD_ON) && \
    ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON)))

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_TransmitCancelISR
                 (Spi_HWUnitType LddHWUnit, uint8 LucHWMemoryMode)
{
  P2CONST(Tdd_Spi_HWUnitInfo, AUTOMATIC, SPI_PRIVATE_CONST) LpHWUnitInfo;

  Spi_SequenceType LddSeqIndex;

  LddSeqIndex = Spi_GaaSeqQueue[Spi_GddQueueIndex[LucHWMemoryMode]];

  /* Get the pointer to the structure of HW Unit information */
  LpHWUnitInfo = &Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[LddHWUnit]];

  /* MISRA Rule         : 12.7                                         */
  /* Message            : Bitwise operations on the signed data will   */
  /*                      give implementation defined results          */
  /* Reason             : Though the bitwise operation is performed on */
  /*                      unsigned data, this warning is generated by  */
  /*                      the QAC tool V6.2.1 as an indirect result of */
  /*                      integral promotion in complex bitwise        */
  /*                      operations.                                  */
  /* Verification       : However, this part of the code is verified   */
  /*                      manually and it is not having any impact.    */

  /* Disable transmit cancel interrupt control */
  *(LpHWUnitInfo->pTxCancelImrAddress) |= ~(LpHWUnitInfo->ucTxCancelImrMask);

  /* Update sequence result as CANCELLED */
  Spi_GaaSeqResult[LddSeqIndex] = SPI_SEQ_CANCELLED;

   /* Clear the 'pStatusArray' */
   Spi_ClearStatusArray(LddSeqIndex);

  Spi_ProcessSequence(LddHWUnit, LucHWMemoryMode);
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of (SPI_CANCEL_API == STD_OFF) */

/*******************************************************************************
** Function Name      : Spi_ReceiveISR
**
** Service ID         : Not Applicable
**
** Description        : This is the interrupt service routine for CSIG receive
**
** Sync/Async         : Synchronous
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Spi_HWUnitType LddHWUnit
**                      uint8 LucHWMemoryMode
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      Spi_GaaHWIndexMap
**                      Spi_GstHWUnitInfo
**
**                      Function Invoked:
**                      Spi_ProcessSequence
**
*******************************************************************************/
#if ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON))

/* MISRA Rule         : 1.1                                                   */
/* Message            : Number of macros defined within the                   */
/*                      translation exceeds 1024.                             */
/* Reason             : This macro is required for memory section definition. */
/* Verification       : Not necessarily an indication of incorrect code.      */
/*                      However, part of the code is verified manually and    */
/*                      it is not having any impact.                          */

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_ReceiveISR
                              (Spi_HWUnitType LddHWUnit, uint8 LucHWMemoryMode)
{
  P2CONST(Tdd_Spi_HWUnitInfo, AUTOMATIC, SPI_PRIVATE_CONST) LpHWUnitInfo;
  #if ((SPI_8BIT_DATA_WIDTH == STD_OFF) && (SPI_16BIT_DATA_WIDTH == STD_OFF))
  P2CONST(Tdd_Spi_ChannelPBConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                          LpPBChannelConfig;
  #endif
  P2CONST(Spi_ChannelType,AUTOMATIC,SPI_CONFIG_CONST) LpCurrentRxChannelList;

  #if(SPI_CSIG_CONFIGURED == STD_ON)
  P2VAR(Tdd_Spi_CsigUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsigUserBaseAddr;
  #endif

  #if(SPI_CSIH_CONFIGURED == STD_ON)
  P2VAR(Tdd_Spi_CsihUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpCsihUserBaseAddr;
  #endif
  #if (SPI_DIRECT_ACCESS_MODE == STD_ON)
  P2VAR(Spi_DataType,AUTOMATIC,SPI_CONFIG_DATA) LpCurrentRxData;
  #endif

  #if (SPI_TX_ONLY_MODE == STD_ON)
  P2VAR(uint16, AUTOMATIC, SPI_CONFIG_DATA) LpTxOnlyRxData;
  Spi_JobType LddJobIndex;
  Spi_JobType LddNotifyFuncIndex;
  Spi_JobType LddNoOfJobs;
  Spi_SequenceType LddSeqIndex;
  #endif
  Spi_DataType LddData;

  #if (SPI_TX_ONLY_MODE == STD_ON)
  P2CONST(Tdd_Spi_SequenceConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpSeqConfig;
  P2CONST(Tdd_Spi_JobListType,AUTOMATIC,SPI_CONFIG_CONST) LpJobListTmptr;
  #endif

  #if ((SPI_DIRECT_ACCESS_MODE == STD_ON) && \
      ((SPI_8BIT_DATA_WIDTH == STD_OFF) && (SPI_16BIT_DATA_WIDTH == STD_OFF)))
  /*  MISRA Rule         : 18.4                                           */
  /*  Message            : Unions shall not be used.                      */
  /*  Reason             : To access the converted values                 */
  /*  Verification       : However, part of the code is verified manually */
  /*                       and it is not having any impact.               */
  Tun_Spi_DataAccess LunDataAccess1;
  #endif


  /* Get the pointer to the structure of HW Unit information */
  LpHWUnitInfo = &Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[LddHWUnit]];

  /* MISRA Rule         : 17.4                           */
  /* Message            : Performing pointer arithmetic  */
  /* Reason             : It is used to achieve          */
  /*                      better throughput.             */
  /* Verification       : However, part of the code      */
  /*                      is verified manually and       */
  /*                      it is not having any impact.   */

  /* Get the pointer to the post-build structure of the channel */
  LpCurrentRxChannelList = Spi_GstCurrentCommData.pCurrentRxChannelList;

  /* Message(3:3199)    : Assignment is redundant.                   */
  /*                      The value of LpCurrentRxChannelList        */
  /*                      is never used before being                 */
  /*                      modified.                                  */
  /* Reason             : This is done for proper initialisation of  */
  /*                      the union variable.                        */
  /* Verification       : However, part of the code is               */
  /*                      verified manually and it is not            */
  /*                      having any impact.                         */

  /* MISRA Rule         : 17.4                                          */
  /* Message            : Increment or decrement operation performed on */
  /*                    : pointer                                       */
  /* Reason             : To access the pointer in optimized            */
  /*                      way in this function                          */
  /* Verification       : However, part of the code is verified         */
  /*                      manually and it is not having any impact      */
  LpCurrentRxChannelList--;
  #if ((SPI_8BIT_DATA_WIDTH == STD_OFF) && (SPI_16BIT_DATA_WIDTH == STD_OFF))

  /* MISRA Rule         : 17.4                           */
  /* Message            : Performing pointer arithmetic  */
  /* Reason             : It is used to achieve          */
  /*                      better throughput.             */
  /* Verification       : However, part of the code      */
  /*                      is verified manually and       */
  /*                      it is not having any impact.   */

  LpPBChannelConfig = Spi_GpFirstChannel + (*LpCurrentRxChannelList);
  #endif

  #if (SPI_DIRECT_ACCESS_MODE == STD_ON)

  if(LucHWMemoryMode == SPI_DIRECT_ACCESS_MODE_CONFIGURED)
  {
    if(Spi_GblRxEDL == SPI_TRUE)
    {
      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Disable relevant interrupts */
      SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
      #endif
      Spi_GblRxEDL = SPI_FALSE;

      #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
      if(LddHWUnit < SPI_MAX_NUM_OF_CSIG)
      #endif
      {
        #if(SPI_CSIG_CONFIGURED == STD_ON)
        /* Get the base address of the HW Unit */
        LpCsigUserBaseAddr =
                     (Tdd_Spi_CsigUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;

        /* Store the received data in a variable */
        Spi_GusDataAccess = LpCsigUserBaseAddr->usCSIGRX0;
        #endif
      }
      #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
      else
      #endif
      {
        #if(SPI_CSIH_CONFIGURED == STD_ON)
        /* Get the base address of the HW Unit */
        LpCsihUserBaseAddr =
                     (Tdd_Spi_CsihUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;

        /* Store the received data in a variable */
        Spi_GusDataAccess = LpCsihUserBaseAddr->usCSIHRX0H;
        #endif
      }
      /* Check if critical section protection is required */
      #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
      /* Enable relevant interrupts */
      SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
      #endif
    }
    else
    {
      #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
      if(LddHWUnit < SPI_MAX_NUM_OF_CSIG)
      #endif
      {
        #if(SPI_CSIG_CONFIGURED == STD_ON)
        /* Get the base address of the HW Unit */
        LpCsigUserBaseAddr =
            (Tdd_Spi_CsigUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;

        #if(SPI_8BIT_DATA_WIDTH == STD_ON)
        /* Data width is maximum 8-bit. Hence, Receive the data from the */
        /* Rx register to local union variable */
        LddData = (uint8) LpCsigUserBaseAddr->usCSIGRX0;

        #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
        LddData = LpCsigUserBaseAddr->usCSIGRX0;

        #else
        /* Data width is maximum 32-bit, check if the the data width of */
        /* requested channel is more than 16 bits */
        if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE)
        {
          LddData = LpCsigUserBaseAddr->usCSIGRX0;
        }
        else
        {
          /* Check if the configured data direction is LSB first */
          if(LpPBChannelConfig->blDirection == SPI_TRUE)
          {
            /* Take a local union variable to construct the value from RX0W */
            /* register */
            LunDataAccess1.usRegData5[1] = LpCsigUserBaseAddr->usCSIGRX0;
            LunDataAccess1.usRegData5[0] = Spi_GusDataAccess;
          }
          else
          {
            /* Take a local union variable to construct the value from RX0W */
            /* register */

             LunDataAccess1.usRegData5[0] = (uint16)
             (LpCsigUserBaseAddr->usCSIGRX0 << LpPBChannelConfig->ucDLSSetting);
            LunDataAccess1.usRegData5[1] = Spi_GusDataAccess;
            LunDataAccess1.ulRegData = LunDataAccess1.ulRegData >> \
                                                LpPBChannelConfig->ucDLSSetting;


          } /* End of if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE) */

          LddData = LunDataAccess1.ulRegData;

          Spi_GblRxEDL = SPI_TRUE;
        }
        #endif
        #endif
      }
      #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
      else
      #endif
      {
        #if(SPI_CSIH_CONFIGURED == STD_ON)
        /* Get the base address of the HW Unit */
        LpCsihUserBaseAddr =
                     (Tdd_Spi_CsihUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;

        #if(SPI_8BIT_DATA_WIDTH == STD_ON)
        /* Data width is maximum 8-bit. Hence, Receive the data from the */
        /* Rx register to local union variable */
        LddData = (uint8) LpCsihUserBaseAddr->usCSIHRX0H;

        #elif (SPI_16BIT_DATA_WIDTH == STD_ON)
        LddData = LpCsihUserBaseAddr->usCSIHRX0H;

        #else
        /* Data width is maximum 32-bit, check if the the data width of */
        /* requested channel is more than 16 bits */
        if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE)
        {
          LddData = LpCsihUserBaseAddr->usCSIHRX0H;
        }
        else
        {
          /* Check if the configured data direction is LSB first */
          if(LpPBChannelConfig->blDirection == SPI_TRUE)
          {
            /* Take a local union variable to construct the value from RX0W */
            /* register */
            LunDataAccess1.usRegData5[1] = LpCsihUserBaseAddr->usCSIHRX0H;
            LunDataAccess1.usRegData5[0] = Spi_GusDataAccess;
          }
          else
          {
            /* Take a local union variable to construct the value from RX0W */
            /* register */

            LunDataAccess1.usRegData5[0] = (uint16)
            (LpCsihUserBaseAddr->usCSIHRX0H << LpPBChannelConfig->ucDLSSetting);
            LunDataAccess1.usRegData5[1] = Spi_GusDataAccess;
            LunDataAccess1.ulRegData = LunDataAccess1.ulRegData >> \
                                                LpPBChannelConfig->ucDLSSetting;


          } /* End of if(LpPBChannelConfig->blEDLEnabled != SPI_TRUE) */
          LddData = LunDataAccess1.ulRegData;

          Spi_GblRxEDL = SPI_TRUE;
        }
        #endif
        #endif
      }

      LpCurrentRxData = Spi_GstCurrentCommData.pCurrentRxData;

      if(LpCurrentRxData != NULL_PTR)
      {
        *LpCurrentRxData = LddData;

        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        LpCurrentRxData++;

        /* Check if critical section protection is required */
        #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
        /* Disable relevant interrupts */
        SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
        #endif

        Spi_GstCurrentCommData.pCurrentRxData = LpCurrentRxData;

        /* Check if critical section protection is required */
        #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
        /* Enable relevant interrupts */
        SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
        #endif
      }

      /* Check if all buffers in the channel are transmitted */
      if((Spi_GstCurrentCommData.ddNoOfRxBuffers) > SPI_ZERO)
      {
        /* Decrement the number of buffers to be transmitted */
        (Spi_GstCurrentCommData.ddNoOfRxBuffers)--;
      }
      else
      {
        /* All buffers are transmitted. */
        Spi_ProcessSequence(LddHWUnit, LucHWMemoryMode);
      }
    }
  }
  #endif
  #if (SPI_TX_ONLY_MODE == STD_ON)

  if(LucHWMemoryMode == SPI_TX_ONLY_MODE_CONFIGURED)
  {
    #if (SPI_DMA_MODE_ENABLE == STD_ON)
    LddJobIndex =
               (Spi_JobType)(*(Spi_GstTxOnlyCurrentCommData.pCurrentNotifyIdx));

    /* MISRA Rule       : 17.4                           */
    /* Message          : Performing pointer arithmetic  */
    /* Reason           : It is used to achieve          */
    /*                    better throughput.             */
    /* Verification     : However, part of the code      */
    /*                    is verified manually and       */
    /*                    it is not having any impact   */
    if(((Spi_GpFirstJob + LddJobIndex)->ucRxDmaDeviceIndex) ==
                                               SPI_INVALID_DMAUNIT)
    #endif
    {
      /* Get the base address of the HW Unit */
      LpCsihUserBaseAddr =
                     (Tdd_Spi_CsihUserRegs *)LpHWUnitInfo->pHwUserBufferAddress;

      LddData = LpCsihUserBaseAddr->usCSIHRX0H;

      LpTxOnlyRxData = Spi_GstTxOnlyCurrentCommData.pCurrentRxData;

      if(LpTxOnlyRxData != NULL_PTR)
      {
        *LpTxOnlyRxData = (uint16)LddData;

        /* Increment Rx pointer */
        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        LpTxOnlyRxData++;

        Spi_GstTxOnlyCurrentCommData.pCurrentRxData = LpTxOnlyRxData;
      }
    } /* End of checking if DMA is configured */

    LddSeqIndex = Spi_GaaSeqQueue[Spi_GddQueueIndex[LucHWMemoryMode]];

    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact.   */

    LpSeqConfig = Spi_GpFirstSeq + LddSeqIndex;

    Spi_GddNoOfBuffers++;

    if(Spi_GddNoOfBuffers < LpSeqConfig->ddNoOfBuffers)
    {
      if(Spi_GddNoOfBuffers >= *Spi_GpCurrentNotifyIdx)
      {
        /* Increment the notify index value */
        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        Spi_GpCurrentNotifyIdx++;

        LddJobIndex = (Spi_JobType)(*Spi_GpCurrentNotifyIdx);

        /* Get the notification function index */
        /* MISRA Rule       : 17.4                           */
        /* Message          : Performing pointer arithmetic  */
        /* Reason           : It is used to achieve          */
        /*                    better throughput.             */
        /* Verification     : However, part of the code      */
        /*                    is verified manually and       */
        /*                    it is not having any impact    */
        LddNotifyFuncIndex = (Spi_GpFirstJob + LddJobIndex)->ddNotifyFuncIndex;

        /* Check if notification is configured */
        if (LddNotifyFuncIndex != SPI_NO_NOTIFICATION)
        {
          Spi_GstJobFunc[LddNotifyFuncIndex].pSpiJobNotification();
        }

        /* Increment the notify index value */
        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        Spi_GpCurrentNotifyIdx++;
      }
    }
    else
    {
      Spi_GddNoOfBuffers = SPI_ZERO;

      /* Update sequence result as OK */
      Spi_GaaSeqResult[LddSeqIndex] = SPI_SEQ_OK;
      /* All channels are transmitted and received. Hence the sequence is */
      /* completed */

      /* Load Job List pointer to temporary variable */
      /* MISRA Rule       : 17.4                           */
      /* Message          : Performing pointer arithmetic  */
      /* Reason           : It is used to achieve          */
      /*                    better throughput.             */
      /* Verification     : However, part of the code      */
      /*                    is verified manually and       */
      /*                    it is not having any impact    */
      LpJobListTmptr = Spi_GpFirstJobList + (LpSeqConfig->ddJobListIndex);

      /* Load total number of Jobs in current sequence to local variable */
      LddNoOfJobs = LpSeqConfig->ddNoOfJobs;

      do
      {
        /* Update the job result variable */
        Spi_GaaJobResult[LpJobListTmptr->ddJobIndex] = SPI_JOB_OK;

        /* Increment local pointer to the job list */
        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        LpJobListTmptr++;
        LddNoOfJobs--;
      }
      while(LddNoOfJobs > SPI_ZERO);

      Spi_ProcessSequence(LddHWUnit, LucHWMemoryMode);
    }
  }
  #endif
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of (SPI_TX_ONLY_MODE == STD_ON) */

/*******************************************************************************
** Function Name      : Spi_ProcessSequence
**
** Service ID         : Not Applicable
**
** Description        : This function is to process remaining jobs in
**                      the sequence to be transmitted
**
** Sync/Async         : Synchronous
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Spi_HWUnitType LddHWUnit
**                      uint8 LucHWMemoryMode
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      Spi_GaaHWIndexMap
**                      Spi_GpFirstSeq
**                      Spi_GstSeqProcess
**                      Spi_GstJobFunc
**                      Spi_GpFirstJob
**                      Spi_GaaJobResult
**                      Spi_GddAsyncMode
**                      Spi_GstHWUnitInfo
**                      Spi_GaaSeqResult
**                      Spi_GaaJobResult
**                      Spi_GstQueuedSeq
**                      Spi_GddDriverStatus
**
**                      Function Invoked:
**                      Spi_ProcessChannel
**                      Spi_HWDeActivateCS
**                      Spi_HWActivateCS
**                      Spi_PopFromQueue
**                      Spi_HWInitiateTx
**                      SchM_Enter_Spi
**                      SchM_Exit_Spi
**
*******************************************************************************/
#if ((SPI_LEVEL_DELIVERED == SPI_ONE) || (SPI_LEVEL_DELIVERED == SPI_TWO))

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_ProcessSequence
                   (Spi_HWUnitType LddHWUnit, uint8 LucHWMemoryMode)
{
  #if ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON))
  P2CONST(Tdd_Spi_JobListType, AUTOMATIC, SPI_CONFIG_CONST) LpJobList;
  P2VAR(Tdd_Spi_MainUserRegs, AUTOMATIC, SPI_CONFIG_DATA) LpMainUserBaseAddr;
  P2CONST(Tdd_Spi_JobConfigType, AUTOMATIC, SPI_CONFIG_CONST) LpJobConfig;
  #endif

  P2CONST(Tdd_Spi_HWUnitInfo, AUTOMATIC, SPI_CONFIG_CONST) LpHWUnitInfo;


  #if ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON))
  P2CONST(Tdd_Spi_SequenceConfigType, AUTOMATIC, SPI_CONFIG_CONST) LpSeqConfig;

  Spi_JobType LddJobListIndex;
  Spi_JobType LddNoOfJobs;
  Spi_JobType LddReqJobListIndex;
  Spi_SequenceType LddSeqIndex;
  Spi_JobType LddLowestQueueIndex;
  #endif

  Spi_NumberOfDataType LddNotifyFuncIndex;
  #if ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON))
  Spi_JobType LddJobIndex;
  #endif

  #if ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON))
  #if (SPI_CANCEL_API == STD_ON)
  uint8 LucStatusOffset;
  uint8 LucStatusMask;
  uint8 LucVar;
  uint8 LucMask;
  #endif
  #endif

  /* Get the pointer to the structure of HW Unit information */
  LpHWUnitInfo = &Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[LddHWUnit]];

  #if (((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON)) \
        &&((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON)))
  if(LucHWMemoryMode < SPI_TWO)
  #endif
  {
    #if ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON))

    #if (SPI_DIRECT_ACCESS_MODE == STD_ON)
    if((LucHWMemoryMode == SPI_DIRECT_ACCESS_MODE_CONFIGURED) &&
       ((Spi_GstCurrentCommData.ddNoOfRxChannels) > SPI_ZERO))
    {
      /* More channels to be transmitted. Hence transmit next channel */
      Spi_ProcessChannel(LddHWUnit, LucHWMemoryMode);
    }
    else
    #endif
    {
      LpMainUserBaseAddr = LpHWUnitInfo->pHwMainUserBaseAddress;

      /* Get the pointer to the joblist structure */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact    */

      /* MISRA Rule         : 21.1                                */
      /* Message            : Indexing array with value that will */
      /*                      apparently be out of bounds.        */
      /* Reason             : It is used for array indexing       */
      /* Verification       : However, part of the code           */
      /*                      is verified manually and            */
      /*                      it is not having any impact         */
      LpJobList = Spi_GpFirstJobList +
                   Spi_GaaJobQueue[Spi_GddQueueIndex[LucHWMemoryMode]];

      /* Get the index of the first job linked to this sequence */
      LddJobIndex = LpJobList->ddJobIndex;

      /* Get the pointer to the job structure */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact    */
      LpJobConfig = Spi_GpFirstJob + LddJobIndex;

      if((LpJobConfig->ulMainCtl1Value & SPI_CSRI_MASK) != SPI_ZERO)
      {
        LpMainUserBaseAddr->ucMainCTL0 = SPI_ZERO;
      }

      /* MISRA Rule         : 12.7                                         */
      /* Message            : Bitwise operations on the signed data will   */
      /*                      give implementation defined results          */
      /* Reason             : Though the bitwise operation is performed on */
      /*                      unsigned data, this warning is generated by  */
      /*                      the QAC tool V6.2.1 as an indirect result of */
      /*                      integral promotion in complex bitwise        */
      /*                      operations.                                  */
      /* Verification       : However, this part of the code is verified   */
      /*                      manually and it is not having any impact.    */

      /* Disable transmit interrupt control */
      *(LpHWUnitInfo->pTxImrAddress) |= ~(LpHWUnitInfo->ucTxImrMask);

      /* Disable receive interrupt control */
      *(LpHWUnitInfo->pRxImrAddress) |= ~(LpHWUnitInfo->ucRxImrMask);

      #if((SPI_CSIG_CONFIGURED == STD_ON) && (SPI_CSIH_CONFIGURED == STD_ON))
      if(LddHWUnit < SPI_MAX_NUM_OF_CSIG)
      #endif
      {
        #if(SPI_CSIG_CONFIGURED == STD_ON)
        /* Deactivate the chip select */
        /* MISRA Rule         : 17.4                           */
        /* Message            : Performing pointer arithmetic  */
        /* Reason             : It is used to achieve          */
        /*                      better throughput.             */
        /* Verification       : However, part of the code      */
        /*                      is verified manually and       */
        /*                      it is not having any impact    */
        if((LpJobConfig->ulMainCtl1Value & SPI_CSRI_MASK) != SPI_ZERO)
        {
          Spi_HWDeActivateCS(Spi_GpFirstJob + LddJobIndex);
        }
        #endif
      }

      if(Spi_GaaJobResult[LddJobIndex] != SPI_JOB_FAILED)
      {
        /* Update the RAM area for job sequence */
        Spi_GaaJobResult[LddJobIndex] = SPI_JOB_OK;
      }

      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact    */
      LddNotifyFuncIndex = (Spi_GpFirstJob + LddJobIndex)->ddNotifyFuncIndex;

      /* Check if the notification function is configured */
      if(LddNotifyFuncIndex != SPI_NO_NOTIFICATION)
      {
        /* Invoke the notification function */
        Spi_GstJobFunc[LddNotifyFuncIndex].pSpiJobNotification();
      }

      Spi_PopFromQueue(LucHWMemoryMode);

    } /* End of checking memory modes */
    #endif /* End of ((SPI_DIRECT_ACCESS_MODE == STD_ON) ||
                            (SPI_FIFO_MODE == STD_ON)) */
  }
  #if (((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON)) \
      && ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON)))
  else
  #endif

  {
    #if ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON))

    /* MISRA Rule         : 12.7                                         */
    /* Message            : Bitwise operations on the signed data will   */
    /*                      give implementation defined results          */
    /* Reason             : Though the bitwise operation is performed on */
    /*                      unsigned data, this warning is generated by  */
    /*                      the QAC tool V6.2.1 as an indirect result of */
    /*                      integral promotion in complex bitwise        */
    /*                      operations.                                  */
    /* Verification       : However, this part of the code is verified   */
    /*                      manually and it is not having any impact.    */

    /* Disable transmit interrupt control */
    *(LpHWUnitInfo->pTxImrAddress) |= ~(LpHWUnitInfo->ucTxImrMask);

    /* Disable receive interrupt control */
    *(LpHWUnitInfo->pRxImrAddress) |= ~(LpHWUnitInfo->ucRxImrMask);

    LddSeqIndex = Spi_GaaSeqQueue[Spi_GddQueueIndex[LucHWMemoryMode]];

    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact    */
    LddNotifyFuncIndex = (Spi_GpFirstSeq + LddSeqIndex)->ddNotifyFuncIndex;

    /* Check if the notification function is configured */
    if(LddNotifyFuncIndex != SPI_NO_NOTIFICATION)
    {
      /* Invoke the notification function */
      Spi_GstSeqFunc[LddNotifyFuncIndex].pSpiSeqNotification();
    }

    if(LucHWMemoryMode == SPI_DUAL_BUFFER_MODE_CONFIGURED)
    {
      /* For dual buffer mode, lowest queue index is zero */
      LddLowestQueueIndex = SPI_ZERO;
    }
    else
    {
      /* For Tx-Only mode, lowest queue index is size of dual buffer size */
      LddLowestQueueIndex = Spi_GpConfigPtr->ddDualBufferQueueSize;
    }

    if(Spi_GddQueueIndex[LucHWMemoryMode] == LddLowestQueueIndex)
    {
      Spi_GblQueueStatus[LucHWMemoryMode] = SPI_QUEUE_EMPTY;

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
      Spi_GucAllQueueSts &= ~(SPI_ONE << LucHWMemoryMode);


      if(Spi_GucAllQueueSts == SPI_ZERO)
      {
        /* Check if critical section protection is required */
        #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
        /* Disable relevant interrupts to protect this critical section */
        SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
        #endif
        /* Update driver status as idle */
        Spi_GddDriverStatus = SPI_IDLE;
        /* Check if critical section protection is required */
        #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
        /* Enable relevant interrupts to protect this critical section */
        SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
        #endif
      }
    }
    else
    {
      #if(SPI_CANCEL_API == STD_ON)
      do
      {
        Spi_GddQueueIndex[LucHWMemoryMode]--;

        LddSeqIndex = Spi_GaaSeqQueue[Spi_GddQueueIndex[LucHWMemoryMode]];

        if(Spi_GaaSeqResult[LddSeqIndex] != SPI_SEQ_CANCELLED)
        {
          /* Get the cancel byte offset for the requested sequence  */
          LucStatusOffset = Spi_GstSeqProcess[LddSeqIndex].ucCancelOffset;
          /* Get the cancel byte mask */
          LucStatusMask = Spi_GstSeqProcess[LddSeqIndex].ucCancelMask;
          /* Get the cancel status for this sequence */
          LucStatusMask &= Spi_GaaSeqCancel[LucStatusOffset];
          if(LucStatusMask != SPI_ZERO)
          {
            Spi_GaaSeqResult[LddSeqIndex] = SPI_SEQ_CANCELLED;

            /* Clear the 'pStatusArray' */
            Spi_ClearStatusArray(LddSeqIndex);
          }
        }
      }while((Spi_GaaSeqResult[LddSeqIndex] == SPI_SEQ_CANCELLED) &&
             (Spi_GddQueueIndex[LucHWMemoryMode] > LddLowestQueueIndex));

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
      if(LucStatusMask == SPI_ZERO)
      #endif
      {
        #if(SPI_CANCEL_API == STD_OFF)
        Spi_GddQueueIndex[LucHWMemoryMode]--;
        LddSeqIndex = Spi_GaaSeqQueue[Spi_GddQueueIndex[LucHWMemoryMode]];
        #endif

        /* Get the pointer to the post-build sequence structure */
        /* MISRA Rule         : 17.4                            */
        /* Message            : Performing pointer arithmetic   */
        /* Reason             : It is used to achieve           */
        /*                      better throughput.              */
        /* Verification       : However, part of the code       */
        /*                      is verified manually and        */
        /*                      it is not having any impact     */
        LpSeqConfig = Spi_GpFirstSeq + LddSeqIndex;

        /* Get the job list index of the last job of the sequence */
        LddJobListIndex = LpSeqConfig->ddJobListIndex;

        /* Get the number of jobs configured for the requested sequence */
        LddNoOfJobs = LpSeqConfig->ddNoOfJobs;

        /* Get the index of the job list for the first job of the sequence */
        LddReqJobListIndex = LddJobListIndex + (LddNoOfJobs - SPI_ONE);

        /* Check if critical section protection is required */
        #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
        /* Disable relevant interrupts to protect this critical section */
        SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
        #endif

        #if(SPI_CANCEL_API == STD_ON)
        /* Get the cancel byte offset for the requested sequence  */
        LucVar = Spi_GstSeqProcess[LddSeqIndex].ucCancelOffset;
        /* Get the cancel byte mask */
        LucMask = Spi_GstSeqProcess[LddSeqIndex].ucCancelMask;
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
        Spi_GaaSeqCancel[LucVar] &= (~LucMask);

        #endif /* End of (SPI_CANCEL_API == STD_ON) */

        /* Check if critical section protection is required */
        #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
        /* Enable relevant interrupts to protect this critical section */
        SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
        #endif

        /* Initiate the transmission for that sequence */
        Spi_HWInitiateTx(LddReqJobListIndex);
      } /* End of if(Spi_GddQueueIndex[LucHWMemoryMode] != SPI_ZERO) */
      #if(SPI_CANCEL_API == STD_ON)
      else
      {
        Spi_GblQueueStatus[LucHWMemoryMode] = SPI_QUEUE_EMPTY;

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
        Spi_GucAllQueueSts &= ~(SPI_ONE << LucHWMemoryMode);


        if(Spi_GucAllQueueSts == SPI_ZERO)
        {
          /* Check if critical section protection is required */
          #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
          /* Disable relevant interrupts to protect this critical section */
          SchM_Enter_Spi(SPI_RAM_DATA_PROTECTION);
          #endif
          /* Update driver status as idle */
          Spi_GddDriverStatus = SPI_IDLE;
          /* Check if critical section protection is required */
          #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
          /* Enable relevant interrupts to protect this critical section */
          SchM_Exit_Spi(SPI_RAM_DATA_PROTECTION);
          #endif
        }
      }
      #endif /* End of (SPI_CANCEL_API == STD_ON) */
    }
    #endif /* End of ((SPI_DUAL_BUFFER_MODE == STD_ON)
                   || (SPI_TX_ONLY_MODE == STD_ON))*/
  }
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of ((SPI_LEVEL_DELIVERED == SPI_ONE) ||
                  (SPI_LEVEL_DELIVERED == SPI_TWO)) */

/*******************************************************************************
** Function Name      : Spi_TxDmaConfig
**
** Service ID         : Not Applicable
**
** Description        : This function is to set the attributes for DMA transfer
**
** Sync/Async         : Synchronous
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : LpJobConfig
**                      LpTxData
**                      LddNoOfBuffers
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:Spi_GpConfigPtr
**
**                      Function Invoked:
**                      None
**
*******************************************************************************/
#if (SPI_DMA_MODE_ENABLE == STD_ON)

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_TxDmaConfig
(P2CONST(Tdd_Spi_JobConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpJobConfig,
 P2CONST(Spi_DataType,AUTOMATIC,SPI_CONFIG_DATA) LpTxData,
 Spi_NumberOfDataType LddNoOfBuffers)
{
  P2CONST(Tdd_Spi_DmaUnitConfig, AUTOMATIC, SPI_CONFIG_DATA) LpDmaConfig;
  P2VAR(Tdd_Spi_DmaAddrRegs, AUTOMATIC, SPI_CONFIG_DATA) LpDmaRegisters;
  P2VAR(uint8,AUTOMATIC,SPI_CONFIG_DATA) LpIntpCntrlReg;
  #if(SPI_CPU_CORE != SPI_E2M)
  P2VAR(uint16, AUTOMATIC, SPI_CONFIG_DATA) LpDmaTrigFactor;
  #endif
  #if (SPI_CSIH_CONFIGURED == STD_ON)
  Spi_HWUnitType LddHWUnit;
  #endif
  uint8 LucHWMemoryMode;
  uint8 LucIndex;

  /* Get the Tx DMA index for this HW Unit */
  LucIndex = LpJobConfig->ucTxDmaDeviceIndex;

  /* Initialize memory mode as DIRECT ACCESS */
  LucHWMemoryMode = SPI_DIRECT_ACCESS_MODE_CONFIGURED;

  #if (SPI_CSIH_CONFIGURED == STD_ON)
  /* Get the Hardware Unit index */
  LddHWUnit = LpJobConfig->ddHWUnitIndex;

  /* Check if the HW Unit is CSIH */
  if(LddHWUnit >= SPI_MAX_NUM_OF_CSIG)
  {
    /* Get the configured memory mode for this HW Unit */
    LucHWMemoryMode =
         Spi_GpConfigPtr->aaHWMemoryMode[LddHWUnit - SPI_MAX_NUM_OF_CSIG];
  }
  #endif

   /* Check if critical section protection is required */
   #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
   /* Disable relevant interrupts */
   SchM_Enter_Spi(SPI_DMA_PROTECTION);
   #endif
  if(LucIndex != SPI_INVALID_DMAUNIT)
  {
    /* Set the parameters for Tx DMA Unit */
    /* Set the DMA channel control register value */
    LpDmaConfig = &Spi_GpDmaUnitConfig[LucIndex];
    LpDmaRegisters = LpDmaConfig->pDmaCntlRegBase;

    /* Clear the DTS bit */
    LpDmaRegisters->ucDTSn &= SPI_DMA_DISABLE;

    if(LpTxData == NULL_PTR)
    {
      /* MISRA Rule         : 11.3                                          */
      /* Message            : A cast should not be performed between a      */
      /*                      pointer type and an integral type.            */
      /* Reason             : This is to access the hardware registers.     */
      /* Verification       : However, this part of the code is verified    */
      /*                      manually and it is not having any impact.     */
      LpDmaRegisters->ulDSAn = (uint32)(&Spi_GddDmaTxData);
      LpDmaRegisters->usDTCTn |= SPI_DMA_FIXED_TX_SETTINGS;
    }
    else
    {
      /* MISRA Rule         : 17.4                                          */
      /* Message            : Increment or decrement operation performed on */
      /*                    : pointer                                       */
      /* Reason             : To access the pointer in optimized            */
      /*                      way in this function                          */
      /* Verification       : However, part of the code is verified         */
      /*                      manually and it is not having any impact      */

      /* MISRA Rule         : 13.7                                           */
      /* Message            : The result of this logical operation is        */
      /*                      always FALSE .                                 */
      /* Reason             : Logical operation performed to check           */
      /*                      configured mode as FIFO MODE                   */
      /* Verification       : However, part of the code is verified manually */
      /*                      and it is not having any impact.               */
      if((LucHWMemoryMode == SPI_FIFO_MODE_CONFIGURED) || \
         (LddNoOfBuffers == SPI_ONE))
      {
        /* MISRA Rule         : 11.3                                          */
        /* Message            : A cast should not be performed between a      */
        /*                      pointer type and an integral type.            */
        /* Reason             : This is to access the hardware registers.     */
        /* Verification       : However, this part of the code is verified    */
        /*                      manually and it is not having any impact.     */

        LpDmaRegisters->ulDSAn = (uint32)LpTxData;
      }
      else
      {
        /* MISRA Rule         : 11.3                                          */
        /* Message            : A cast should not be performed between a      */
        /*                      pointer type and an integral type.            */
        /* Reason             : This is to access the hardware registers.     */
        /* Verification       : However, this part of the code is verified    */
        /*                      manually and it is not having any impact.     */

        /* MISRA Rule         : 17.4                                          */
        /* Message            : Increment or decrement operation performed on */
        /*                    : pointer                                       */
        /* Reason             : To access the pointer in optimized            */
        /*                      way in this function                          */
        /* Verification       : However, part of the code is verified         */
        /*                      manually and it is not having any impact      */
        LpDmaRegisters->ulDSAn = (uint32)(LpTxData + SPI_ONE);
      }

      LpDmaRegisters->usDTCTn &= SPI_DMA_INV_TX_SETTINGS;
    }
      /* MISRA Rule         : 13.7                                           */
      /* Message            : The result of this logical operation is        */
      /*                      always FALSE .                                 */
      /* Reason             : Logical operation performed to check           */
      /*                      configured mode as FIFO MODE                   */
      /* Verification       : However, part of the code is verified manually */
      /*                      and it is not having any impact.               */
    if(LucHWMemoryMode == SPI_FIFO_MODE_CONFIGURED)
    {
      /* MISRA Rule         : 14.1                                          */
      /* Message            : There shall be no unreachable code.           */
      /* Reason             : The condition will be satisfied in some other */
      /*                      configuration.                                */
      /* Verification       : However, part of the code is verified manually*/
      /*                      and it is not having any impact.              */
      #if(SPI_CPU_CORE == SPI_E2M)
      /* Load the transfer request select register for software transfer */
      LpDmaRegisters->usDTRSn = SPI_ZERO;
      #else
      /* Get the address of the trigger factor register */
      LpDmaTrigFactor = LpDmaConfig->pDmaDTFRRegAddr;
      /* Disable the triger factor configured */
      *LpDmaTrigFactor = SPI_ZERO;
      #endif
      /* Load the transfer count value to the DMA register */
      LpDmaRegisters->usDTCn = LddNoOfBuffers;

      /* Enable the DMA interrupt control register */
      LpIntpCntrlReg = LpDmaConfig->pDmaImrIntCntlReg;
      *LpIntpCntrlReg &= LpDmaConfig->ucDmaImrMask;
    }
    else if (LddNoOfBuffers == SPI_ONE)
    {
      LpDmaRegisters->usDTCn = LddNoOfBuffers;
      #if(SPI_CPU_CORE == SPI_E2M)
      /* Load the transfer request select register for software transfer */
      LpDmaRegisters->usDTRSn = SPI_ZERO;
      #else
      /* Get the address of the trigger factor register */
      LpDmaTrigFactor = LpDmaConfig->pDmaDTFRRegAddr;
      /* Disable the triger factor configured */
      *LpDmaTrigFactor = SPI_ZERO;
      #endif
    }
    else
    {
      #if(SPI_CPU_CORE == SPI_E2M)
      /* Load the transfer request select register for hardware transfer */
      LpDmaRegisters->usDTRSn = SPI_ONE;
      #else
      LpDmaTrigFactor = LpDmaConfig->pDmaDTFRRegAddr;
      /* Load the triger factor configured */
      *LpDmaTrigFactor = LpDmaConfig->usDmaDtfrRegValue;
      #endif
      /* Load the transfer count value to the DMA register */
      LpDmaRegisters->usDTCn = LddNoOfBuffers - SPI_ONE;
    }

    /* Clear the MLE bit for data transfer once */
    LpDmaRegisters->usDTCTn &= SPI_DMA_ONCE;

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

    /* MISRA Rule         : 11.3                                          */
    /* Message            : A cast should not be performed between a      */
    /*                      pointer type and an integral type.            */
    /* Reason             : This is to access the hardware registers.     */
    /* Verification       : However, this part of the code is verified    */
    /*                      manually and it is not having any impact.     */

    /* Clear interrupt flag for corresponding DMA channel */
    SPI_DMA_DRQCLR |= LpDmaConfig->usDmaChannelMask;
    /* Check DTSnTC (DMA transfer end status) */
    if ((LpDmaRegisters->ucDTSn & SPI_TC_SET) == SPI_TC_SET)
    {
        /* Set DTSn.DTSnTC to 0 (Clear TC bit) */
        LpDmaRegisters->ucDTSn &= SPI_TC_CLR;
    }
      /* MISRA Rule         : 13.7                                           */
      /* Message            : The result of this logical operation is        */
      /*                      always FALSE .                                 */
      /* Reason             : Logical operation performed to check           */
      /*                      configured mode as FIFO MODE                   */
      /* Verification       : However, part of the code is verified manually */
      /*                      and it is not having any impact.               */
    if((LucHWMemoryMode == SPI_FIFO_MODE_CONFIGURED) || \
       (LddNoOfBuffers == SPI_ONE))
    {
      /* DMA transfer enable along with software transfer request */
      LpDmaRegisters->ucDTSn = SPI_DMA_SR_ENABLE;
    }
    else
    {
      /* DMA transfer enable */
      LpDmaRegisters->ucDTSn = SPI_DMA_ENABLE;
    }
  }

   /* Check if critical section protection is required */
   #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
   /* Disable relevant interrupts */
   SchM_Exit_Spi(SPI_DMA_PROTECTION);
   #endif
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of (SPI_DMA_MODE_ENABLE == STD_ON) */

/*******************************************************************************
** Function Name      : Spi_RxDmaConfig
**
** Service ID         : Not Applicable
**
** Description        : This function is to set the attributes for DMA transfer
**
** Sync/Async         : Synchronous
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : LpJobConfig
**                      LpRxData
**                      LddNoOfBuffers
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:Spi_GpConfigPtr
**
**                      Function Invoked:
**                      Spi_ProcessSequence
**
*******************************************************************************/
#if (SPI_DMA_MODE_ENABLE == STD_ON)

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_RxDmaConfig
(P2CONST(Tdd_Spi_JobConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpJobConfig,
 P2CONST(Spi_DataType,AUTOMATIC,SPI_CONFIG_DATA) LpRxData,
 Spi_NumberOfDataType LddNoOfBuffers)
{
  P2CONST(Tdd_Spi_DmaUnitConfig, AUTOMATIC, SPI_CONFIG_DATA) LpDmaConfig;
  P2VAR(Tdd_Spi_DmaAddrRegs, AUTOMATIC, SPI_CONFIG_DATA) LpDmaRegisters;
  P2VAR(uint8,AUTOMATIC,SPI_CONFIG_DATA) LpIntpCntrlReg;
  #if(SPI_CPU_CORE != SPI_E2M)
  P2VAR(uint16, AUTOMATIC, SPI_CONFIG_DATA) LpDmaTrigFactor;
  #endif
  #if (SPI_CSIH_CONFIGURED == STD_ON)
  Spi_HWUnitType LddHWUnit;
  #endif
  uint8 LucHWMemoryMode;
  uint8 LucIndex;

  /* Get the Rx DMA index for this HW Unit */
  LucIndex = LpJobConfig->ucRxDmaDeviceIndex;

  /* Initialize memory mode as DIRECT ACCESS */
  LucHWMemoryMode = SPI_DIRECT_ACCESS_MODE_CONFIGURED;

  /* Get the configured memory mode for this HW Unit */
  #if (SPI_CSIH_CONFIGURED == STD_ON)
  /* Get the Hardware Unit index */
  LddHWUnit = LpJobConfig->ddHWUnitIndex;

  /* Check if the HW Unit is CSIH */
  if(LddHWUnit >= SPI_MAX_NUM_OF_CSIG)
  {
    /* Get the configured memory mode for this HW Unit */
    LucHWMemoryMode =
         Spi_GpConfigPtr->aaHWMemoryMode[LddHWUnit - SPI_MAX_NUM_OF_CSIG];
  }
  #endif
   /* Check if critical section protection is required */
   #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
   /* Disable relevant interrupts */
   SchM_Enter_Spi(SPI_DMA_PROTECTION);
   #endif
  if(LucIndex != SPI_INVALID_DMAUNIT)
  {
    /* Set the DMA channel control register value */
    LpDmaConfig = &Spi_GpDmaUnitConfig[LucIndex];
    LpDmaRegisters = LpDmaConfig->pDmaCntlRegBase;

    /* Clear the DTE bit */
    LpDmaRegisters->ucDTSn &= SPI_DMA_DISABLE;

    if(LpRxData == NULL_PTR)
    {
      /* MISRA Rule         : 11.3                                          */
      /* Message            : A cast should not be performed between a      */
      /*                      pointer type and an integral type.            */
      /* Reason             : This is to access the hardware registers.     */
      /* Verification       : However, this part of the code is verified    */
      /*                      manually and it is not having any impact.     */
      LpDmaRegisters->ulDDAn = (uint32)(&Spi_GddDmaRxData);
      LpDmaRegisters->usDTCTn |= SPI_DMA_FIXED_RX_SETTINGS;
    }
    else
    {
      /* MISRA Rule         : 11.3                                          */
      /* Message            : A cast should not be performed between a      */
      /*                      pointer type and an integral type.            */
      /* Reason             : This is to access the hardware registers.     */
      /* Verification       : However, this part of the code is verified    */
      /*                      manually and it is not having any impact.     */
      LpDmaRegisters->ulDDAn = (uint32)LpRxData;
      LpDmaRegisters->usDTCTn &= SPI_DMA_INV_RX_SETTINGS;
    }


    /* Load the transfer count value to the DMA register */
    LpDmaRegisters->usDTCn = LddNoOfBuffers;

    /* Clear the MLE bit for data transfer once */
    LpDmaRegisters->usDTCTn &= SPI_DMA_ONCE;

    /* Enable the DMA interrupt control register */
    LpIntpCntrlReg = LpDmaConfig->pDmaImrIntCntlReg;
    *LpIntpCntrlReg &= LpDmaConfig->ucDmaImrMask;
      /* MISRA Rule         : 13.7                                           */
      /* Message            : The result of this logical operation is        */
      /*                      always FALSE .                                 */
      /* Reason             : Logical operation performed to check           */
      /*                      configured mode as FIFO MODE                   */
      /* Verification       : However, part of the code is verified manually */
      /*                      and it is not having any impact.               */
    if(LucHWMemoryMode == SPI_FIFO_MODE_CONFIGURED)
    {
      /* MISRA Rule         : 14.1                                          */
      /* Message            : There shall be no unreachable code.           */
      /* Reason             : The condition will be satisfied in some other */
      /*                      configuration.                                */
      /* Verification       : However, part of the code is verified manually*/
      /*                      and it is not having any impact.              */
      #if(SPI_CPU_CORE == SPI_E2M)
      /* Load the transfer request select register for software transfer */
      LpDmaRegisters->usDTRSn = SPI_ZERO;
      #else
      /* Get the address of the trigger factor register */
      LpDmaTrigFactor = LpDmaConfig->pDmaDTFRRegAddr;
      /* Disable the triger factor configured */
      *LpDmaTrigFactor = SPI_ZERO;
      #endif
    }

    /* MISRA Rule         : 11.3                                          */
    /* Message            : A cast should not be performed between a      */
    /*                      pointer type and an integral type.            */
    /* Reason             : This is to access the hardware registers.     */
    /* Verification       : However, this part of the code is verified    */
    /*                      manually and it is not having any impact.     */

    /* Clear interrupt flag for corresponding DMA channel */
    SPI_DMA_DRQCLR |= LpDmaConfig->usDmaChannelMask;
    /* Check DTSnTC (DMA transfer end status) */
    if ((LpDmaRegisters->ucDTSn & SPI_TC_SET) == SPI_TC_SET)
    {
        /* Set DTSn.DTSnTC to 0 (Clear TC bit) */
        LpDmaRegisters->ucDTSn &= SPI_TC_CLR;
    }
      /* MISRA Rule         : 13.7                                           */
      /* Message            : The result of this logical operation is        */
      /*                      always FALSE .                                 */
      /* Reason             : Logical operation performed to check           */
      /*                      configured mode as FIFO MODE                   */
      /* Verification       : However, part of the code is verified manually */
      /*                      and it is not having any impact.               */
    if(LucHWMemoryMode == SPI_FIFO_MODE_CONFIGURED)
    {
      /* MISRA Rule         : 14.1                                          */
      /* Message            : There shall be no unreachable code.           */
      /* Reason             : The condition will be satisfied in some other */
      /*                      configuration.                                */
      /* Verification       : However, part of the code is verified manually*/
      /*                      and it is not having any impact.              */
      /* DMA transfer enable along with software transfer request */
      LpDmaRegisters->ucDTSn = SPI_DMA_SR_ENABLE;
    }
    else
    {
      /* DMA transfer enable */
      LpDmaRegisters->ucDTSn = SPI_DMA_ENABLE;
    }
  }

   /* Check if critical section protection is required */
   #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
   /* Disable relevant interrupts */
   SchM_Exit_Spi(SPI_DMA_PROTECTION);
   #endif
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of (SPI_DMA_MODE_ENABLE == STD_ON) */

/*******************************************************************************
** Function Name      : Spi_DmaISR
**
** Service ID         : Not Applicable
**
** Description        : This function is invoked in the DMA ISR for processing
**                      DMA request
**
** Sync/Async         : Synchronous
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : uint8 LddDmaUnit
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:Spi_GpConfigPtr
**
**                      Function Invoked:Spi_ProcessSequence
**
*******************************************************************************/
#if (SPI_DMA_MODE_ENABLE == STD_ON)

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_DmaISR (uint8 LucDmaUnit)
{
  Spi_HWUnitType LddHWUnit;
  uint8 LucHWMemoryMode;

  #if(SPI_FIFO_MODE == STD_ON)
  P2CONST(Tdd_Spi_ChannelLTConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpLTChannelConfig;
  P2CONST(Tdd_Spi_ChannelPBConfigType,AUTOMATIC,SPI_CONFIG_CONST)
                                                            LpPBChannelConfig;
  P2CONST(Spi_DataType,AUTOMATIC,SPI_CONFIG_DATA) LpNextTxData;
  P2CONST(Tdd_Spi_JobConfigType,AUTOMATIC,SPI_CONFIG_CONST) LpJobConfig;
  P2VAR(Spi_DataType,AUTOMATIC,SPI_CONFIG_DATA) LpCurrentRxData;

  P2CONST(Tdd_Spi_DmaUnitConfig,AUTOMATIC,SPI_CONFIG_DATA) LpDmaConfig;
  Spi_ChannelType LddChannelIndex;
  Spi_NumberOfDataType LddNoOfBuffers;
  Spi_NumberOfDataType LddBufferIndex;
  Spi_JobType LddJobIndex;
  #endif

  /* MISRA Rule         : 17.4                           */
  /* Message            : Performing pointer arithmetic  */
  /* Reason             : It is used to achieve          */
  /*                      better throughput.             */
  /* Verification       : However, part of the code      */
  /*                      is verified manually and       */
  /*                      it is not having any impact    */
  LddHWUnit = *((Spi_GpConfigPtr->pDmaSpiHWUnitMap) + LucDmaUnit);

  #if ((SPI_CSIH_CONFIGURED == STD_ON) && (SPI_LEVEL_DELIVERED != SPI_ZERO))
  /* Check if the HW Unit is CSIH */
  if(LddHWUnit >= SPI_MAX_NUM_OF_CSIG)
  {
    /* Get the configured memory mode for this HW Unit */
    LucHWMemoryMode =
         Spi_GpConfigPtr->aaHWMemoryMode[LddHWUnit - SPI_MAX_NUM_OF_CSIG];
  }
  else
  #endif
  {
    /* Since HW Unit is CSIG, memory mode is DIRECT ACCESS by default */
    LucHWMemoryMode = SPI_DIRECT_ACCESS_MODE_CONFIGURED;
  }

  #if(SPI_FIFO_MODE == STD_ON)
  if(LucHWMemoryMode == SPI_FIFO_MODE_CONFIGURED)
  {
    /* Increament the channel list to point to next channel */
    /* MISRA Rule         : 17.4                                          */
    /* Message            : Increment or decrement operation performed on */
    /*                    : pointer                                       */
    /* Reason             : To access the pointer in optimized            */
    /*                      way in this function                          */
    /* Verification       : However, part of the code is verified         */
    /*                      manually and it is not having any impact      */
    Spi_GstFifoCurrentCommData.pCurrentChannelList++;
    /* Decrement the number of channels */
    Spi_GstFifoCurrentCommData.ddNoOfChannels--;

    /* MISRA Rule       : 17.4                           */
    /* Message          : Performing pointer arithmetic  */
    /* Reason           : It is used to achieve          */
    /*                    better throughput.             */
    /* Verification     : However, part of the code      */
    /*                    is verified manually and       */
    /*                    it is not having any impact    */
    LpDmaConfig = Spi_GpDmaUnitConfig
                              + Spi_GstFifoCurrentCommData.ucDmaDeviceIndex;

    if(Spi_GstFifoCurrentCommData.ddNoOfChannels > SPI_ZERO)
    {
      LddChannelIndex = *Spi_GstFifoCurrentCommData.pCurrentChannelList;

      /* Get the pointer to the post-build structure of the channel */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact.   */
      LpPBChannelConfig = Spi_GpFirstChannel + LddChannelIndex;
      LpLTChannelConfig = &Spi_GstChannelLTConfig[LddChannelIndex];

      LddBufferIndex = LpLTChannelConfig->ddBufferIndex;

      #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
      /* Check if the buffer type is internal buffer */
      if((LpPBChannelConfig->ucChannelBufferType) == SPI_ZERO)
      #endif
      {
        #if(SPI_INTERNAL_RW_BUFFERS == STD_ON)
        /* Update the RAM variable for Tx pointer with channel IB index */
        /* MISRA Rule         : 17.4                           */
        /* Message            : Performing pointer arithmetic  */
        /* Reason             : It is used to achieve          */
        /*                      better throughput.             */
        /* Verification       : However, part of the code      */
        /*                      is verified manually and       */
        /*                      it is not having any impact    */
        LpNextTxData = &Spi_GaaChannelIBWrite[LddBufferIndex];
        /* Update the RAM variable for Rx pointer with channel IB index */
        LpCurrentRxData = &Spi_GaaChannelIBRead[LddBufferIndex];
        /* Update the RAM variable for number of buffers of the channel */
        LddNoOfBuffers = LpLTChannelConfig->ddNoOfBuffers;
        #endif
      }
      #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
      else if((LpPBChannelConfig->ucChannelBufferType) == SPI_ONE)
      #endif

      {
        #if(SPI_EB_CONFIGURED == STD_ON)
        /* Update the RAM variable for Tx pointer with channel EB
           source pointer */
        LpNextTxData = Spi_GaaChannelEBData[LddBufferIndex].pSrcPtr;
        /* Update the RAM variable for Rx pointer */
        LpCurrentRxData = Spi_GaaChannelEBData[LddBufferIndex].pDestPtr;
        /* Update the local counter with number of buffers of the channel */
        LddNoOfBuffers = Spi_GaaChannelEBData[LddBufferIndex].ddEBLength;
        #endif
      }

      #if(SPI_CHANNEL_BUFFERS_ALLOWED == SPI_TWO)
      else
      {
        /* To avoid MISRA warning */
      }
      #endif

      LddJobIndex = Spi_GstFifoCurrentCommData.ddJobIndex;
      /* Get the pointer to the job structure */
      /* MISRA Rule         : 17.4                           */
      /* Message            : Performing pointer arithmetic  */
      /* Reason             : It is used to achieve          */
      /*                      better throughput.             */
      /* Verification       : However, part of the code      */
      /*                      is verified manually and       */
      /*                      it is not having any impact    */
      LpJobConfig = Spi_GpFirstJob + LddJobIndex;

      /* Check if DMA channel is Rx or Tx */
      if((LpDmaConfig->blComDmaChannel) == SPI_ONE)
      {
        Spi_GddDmaRxData = LpPBChannelConfig->ddDefaultData;


        /* MISRA Rule     : 9.1                                               */
        /* Message        : The variable '-identifier-' is apparently         */
        /*                  unset at this point.                              */
        /* Reason         : This variable is initialized at two places under  */
        /*                  different pre-compile options                     */
        /* Verification   : However, it is manually verified that at least one*/
        /*                  of the pre-compile options will be ON and         */
        /*                  hence this variable will be always initialized.   */
        /*                  Hence,this is not having any impact.              */
        Spi_RxDmaConfig(LpJobConfig, LpCurrentRxData, LddNoOfBuffers);
      }
      else
      {
        Spi_GddDmaTxData = LpPBChannelConfig->ddDefaultData;
        Spi_TxDmaConfig(LpJobConfig, LpNextTxData, LddNoOfBuffers);
      }
    }
    else if((LpDmaConfig->blComDmaChannel) == SPI_ONE)
    {
      Spi_ProcessSequence(LddHWUnit, LucHWMemoryMode);
    }
    else
    {
      /* To avoid MISRA warning */
    }
  }
  else
  #endif
  {
    Spi_ProcessSequence(LddHWUnit, LucHWMemoryMode);
  }
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of (SPI_DMA_MODE_ENABLE == STD_ON) */

/*******************************************************************************
* Function Name          : Spi_HWCancel
*
* Service ID             : NA
*
* Description            : This service is for setting the JOBE bit of the
*                          HW Unit to cancel an on-going sequence
*
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
*                          None
*
*                          Function invoked:
*                          None
*
*******************************************************************************/

#if (((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON)) && \
        (SPI_CANCEL_API == STD_ON))

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PUBLIC_CODE) Spi_HWCancel(uint8 LucIndex)
{
  P2CONST(Tdd_Spi_HWUnitInfo, AUTOMATIC, SPI_PRIVATE_CONST) LpHWUnitInfo;
  P2VAR(Tdd_Spi_MainUserRegs, AUTOMATIC, SPI_CONFIG_DATA) LpMainUserBaseAddr;

  /* Get the base address of the HW Unit */
  LpHWUnitInfo = &Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[LucIndex]];

  /* Get the main base address */
  LpMainUserBaseAddr = LpHWUnitInfo->pHwMainUserBaseAddress;
   /* Check if critical section protection is required */
   #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
   /* Disable relevant interrupts */
   SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
   #endif

  if(((LpMainUserBaseAddr->ucMainCTL0) & SPI_CHECK_PWR) != SPI_ZERO)
  {
    /* Enable transmit cancel interrupt control */
    *(LpHWUnitInfo->pTxCancelImrAddress) &= LpHWUnitInfo->ucTxCancelImrMask;
    /* Set JOBE bit of the HW Unit */
    LpMainUserBaseAddr->ucMainCTL0 |= SPI_SET_JOBE;
  }
   #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
   /* Disable relevant interrupts */
   SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
   #endif
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif

/*******************************************************************************
** Function Name      : Spi_HWMainFunction_Driving
**
** Service ID         : Not Applicable
**
** Description        : This function is HW specific function for
**                      Spi_MainFunction_Driving API
**
** Sync/Async         : Synchronous
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : None
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      Spi_GpFirstJob
**                      Spi_GaaHWIndexMap
**                      Spi_GstHWUnitInfo
**
**                      Function Invoked:
**                      None
**
*******************************************************************************/
#if (SPI_LEVEL_DELIVERED == SPI_TWO)

#define SPI_START_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PUBLIC_CODE) Spi_HWMainFunction_Driving(void)
{
  P2CONST(Tdd_Spi_HWUnitInfo, AUTOMATIC, SPI_PRIVATE_CONST) LpHWUnitInfo;
  P2VAR(Tdd_Spi_MainUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpMainUserBaseAddr;

  Spi_HWUnitType LddHWUnit;
  uint8 LucHWMemoryMode;
  uint8 LucIndex;

  LddHWUnit = SPI_ZERO;

  do
  {
    /* Get the index of the hardware */
    LucIndex = Spi_GaaHWIndexMap[LddHWUnit];
    /* Check if the HW Unit configured */
    if((LucIndex != SPI_INVALID_HWUNIT)
       #if (SPI_LEVEL_DELIVERED == SPI_TWO)
       && (LddHWUnit != SPI_HW_UNIT_SYNCHRONOUS)
       #endif
       )
    {
      /* Get the pointer to the structure of HW Unit information */
      LpHWUnitInfo = &Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[LddHWUnit]];

      /* Get the main base address */
      LpMainUserBaseAddr = LpHWUnitInfo->pHwMainUserBaseAddress;

      #if(SPI_CSIH_CONFIGURED == STD_ON)
      /* Check if the HW Unit is CSIH */
      if(LddHWUnit >= SPI_MAX_NUM_OF_CSIG)
      {
        /* Get the configured memory mode for this HW Unit */
        /* MISRA Rule         : 21.1                                */
        /* Message            : Indexing array with value that will */
        /*                      apparently be out of bounds.        */
        /* Reason             : It is used for array indexing       */
        /* Verification       : However, part of the code           */
        /*                      is verified manually and            */
        /*                      it is not having any impact         */
        LucHWMemoryMode =
             Spi_GpConfigPtr->aaHWMemoryMode[LddHWUnit - SPI_MAX_NUM_OF_CSIG];
      }
      else
      #endif

      {
        /* Since HW Unit is CSIG, memory mode is DIRECT ACCESS by default */
        LucHWMemoryMode = SPI_DIRECT_ACCESS_MODE_CONFIGURED;
      }
      /* Check if the HWUnit is ON */
      if(((LpMainUserBaseAddr->ucMainCTL0) & SPI_SET_PWR) == SPI_SET_PWR)
      {
        /* Check if the HWUnit is busy */
        /* MISRA Rule         : 15.4                                      */
        /* Message            : A switch expression shall not represent a */
        /*                      value that is effectively Boolean.        */
        /* Reason             : It is used to achieve better throughput   */
        /*                      when more number of swithes are on.       */
        /* Verification       : However, part of the code                 */
        /*                      is verified manually and                  */
        /*                      it is not having any impact               */

        switch(LucHWMemoryMode)
        {
          #if (SPI_DIRECT_ACCESS_MODE == STD_ON)
          case SPI_DIRECT_ACCESS_MODE_CONFIGURED:
               if(((*LpHWUnitInfo->pTxIntCntlAddress) & SPI_INT_FLAG_MASK)
                           != SPI_ZERO)
               {
                 #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
                 /* Disable relevant interrupts */
                 SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
                 #endif

                 (*LpHWUnitInfo->pTxIntCntlAddress) &= SPI_CLR_INT_REQ;

                 /* Check if critical section protection is required */
                 #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
                 /* Enable relevant interrupts */
                 SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
                 #endif

                 Spi_TransmitISR(LddHWUnit, LucHWMemoryMode);
               }
               if(((*LpHWUnitInfo->pRxIntCntlAddress) & SPI_INT_FLAG_MASK)
                           != SPI_ZERO)
               {
                 /* Check if critical section protection is required */
                 #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
                 /* Disable relevant interrupts */
                 SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
                 #endif

                 (*LpHWUnitInfo->pRxIntCntlAddress) &= SPI_CLR_INT_REQ;

                 /* Check if critical section protection is required */
                 #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
                 /* Enable relevant interrupts */
                 SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
                 #endif

                 Spi_ReceiveISR(LddHWUnit, LucHWMemoryMode);
               }
               break;
          #endif

          #if (SPI_FIFO_MODE == STD_ON)
          case SPI_FIFO_MODE_CONFIGURED:
               if(((*LpHWUnitInfo->pTxIntCntlAddress) & SPI_INT_FLAG_MASK)
                           != SPI_ZERO)
               {
                 /* Check if critical section protection is required */
                 #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
                 /* Disable relevant interrupts */
                 SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
                 #endif

                 (*LpHWUnitInfo->pTxIntCntlAddress) &= SPI_CLR_INT_REQ;

                 /* Check if critical section protection is required */
                 #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
                 /* Enable relevant interrupts */
                 SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
                 #endif

                 Spi_TransmitISR(LddHWUnit, LucHWMemoryMode);
               }

               break;
          #endif

          #if (SPI_DUAL_BUFFER_MODE == STD_ON)
          case SPI_DUAL_BUFFER_MODE_CONFIGURED:
               if(((*LpHWUnitInfo->pTxIntCntlAddress) & SPI_INT_FLAG_MASK)
                           != SPI_ZERO)
               {

                /* Check if critical section protection is required */
                #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
                /* Disable relevant interrupts */
                SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
                #endif

                (*LpHWUnitInfo->pTxIntCntlAddress) &= SPI_CLR_INT_REQ;

                /* Check if critical section protection is required */
                #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
                /* Enable relevant interrupts */
                SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
                #endif

                 Spi_TransmitISR(LddHWUnit, LucHWMemoryMode);
               }
               break;
          #endif

          #if (SPI_TX_ONLY_MODE == STD_ON)
          case SPI_TX_ONLY_MODE_CONFIGURED:
               if(((*LpHWUnitInfo->pRxIntCntlAddress) & SPI_INT_FLAG_MASK)
                           != SPI_ZERO)
               {
                 /* Check if critical section protection is required */
                 #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
                 /* Disable relevant interrupts */
                 SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
                 #endif

                 (*LpHWUnitInfo->pRxIntCntlAddress) &= SPI_CLR_INT_REQ;

                 /* Check if critical section protection is required */
                 #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
                 /* Enable relevant interrupts */
                 SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
                 #endif

                 Spi_ReceiveISR(LddHWUnit, LucHWMemoryMode);
               }

               break;
          #endif

          default: break;
          /* MISRA Rule         : 15.4                                      */
          /* Message            : A switch expression shall not represent a */
          /*                      value that is effectively Boolean.        */
          /* Reason             : It is used to achieve better throughput   */
          /*                      when more number of swithes are on.       */
          /* Verification       : However, part of the code                 */
          /*                      is verified manually and                  */
          /*                      it is not having any impact               */
        }
      }
    }
    LddHWUnit++;
  } while(LddHWUnit < SPI_MAX_HW_UNIT);

}

#define SPI_STOP_SEC_PUBLIC_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of ((SPI_LEVEL_DELIVERED == SPI_ONE) ||
                             (SPI_LEVEL_DELIVERED == SPI_TWO))*/

/*******************************************************************************
** Function Name      : Spi_ComErrorISR
**
** Service ID         : Not Applicable
**
** Description        : This is the interrupt service routine for Error
**
** Sync/Async         : Synchronous
**
** Re-entrancy        : Non-Reentrant
**
** Input Parameters   : Spi_HWUnitType LddHWUnit
**                      uint8 LucHWMemoryMode
**
** InOut Parameters   : None
**
** Output Parameters  : None
**
** Return parameter   : void
**
** Preconditions      : None
**
** Remarks            : Global Variable:
**                      Spi_GaaHWIndexMap
**                      Spi_GstHWUnitInfo
**
**                      Function Invoked:
**                      None
**
*******************************************************************************/
#if ((SPI_LEVEL_DELIVERED == SPI_ONE) || (SPI_LEVEL_DELIVERED == SPI_TWO))

#define SPI_START_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

FUNC(void, SPI_PRIVATE_CODE) Spi_ComErrorISR
                 (Spi_HWUnitType LddHWUnit, uint8 LucHWMemoryMode)
{
  #if((SPI_DATA_CONSISTENCY_CHECK == STD_ON) || \
                ((SPI_FIFO_MODE == STD_ON) && (SPI_DEV_ERROR_DETECT == STD_ON)))
  P2CONST(Tdd_Spi_HWUnitInfo, AUTOMATIC, SPI_PRIVATE_CONST)LpHWUnitInfo;
  P2VAR(Tdd_Spi_MainUserRegs,AUTOMATIC,SPI_CONFIG_DATA) LpMainUserBaseAddr;
  #endif

  #if ((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON))
  P2CONST(Tdd_Spi_JobListType, AUTOMATIC, SPI_CONFIG_CONST) LpJobList;
  Spi_JobType LddJobIndex;
  #endif

  Spi_SequenceType LddSeqIndex;
  #if((SPI_DATA_CONSISTENCY_CHECK == STD_ON) || \
                ((SPI_FIFO_MODE == STD_ON) && (SPI_DEV_ERROR_DETECT == STD_ON)))
  LpHWUnitInfo = &Spi_GstHWUnitInfo[Spi_GaaHWIndexMap[LddHWUnit]];

   /* Get the main base address */
  LpMainUserBaseAddr = LpHWUnitInfo->pHwMainUserBaseAddress;
 #endif
 #if(SPI_DATA_CONSISTENCY_CHECK == STD_ON)
  if((LpMainUserBaseAddr->ulMainSTR0 & SPI_DATA_CONSISTENCY_ERROR) != SPI_ZERO)
  {
    /* MISRA Rule         : 3.1                                               */
    /* Message            : (6 277) [I] An integer constant expression with   */
    /*                      negative value is being converted to unsigned type*/
    /* Reason             : This is to address typecasting of the negated     */
    /*                      macro value.                                      */
    /* Verification       : However, part of the code  is verified manually   */
    /*                      and it is not having any impact                   */
    LpMainUserBaseAddr->usMainSTCR0 = (uint16)~(SPI_DATA_CONSISTENCY_ERROR_CLR);
    Dem_ReportErrorStatus(SPI_E_SEQ_FAILED, DEM_EVENT_STATUS_FAILED);
  }
  #endif
  #if (((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON)) \
        &&((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON)))
  if(LucHWMemoryMode < SPI_TWO)
  #endif
  {
    #if((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON))
    /* MISRA Rule         : 17.4                           */
    /* Message            : Performing pointer arithmetic  */
    /* Reason             : It is used to achieve          */
    /*                      better throughput.             */
    /* Verification       : However, part of the code      */
    /*                      is verified manually and       */
    /*                      it is not having any impact    */
    LpJobList = Spi_GpFirstJobList +
                 Spi_GaaJobQueue[Spi_GddQueueIndex[LucHWMemoryMode]];

    /* Get the index of the first job linked to this sequence */
    LddJobIndex = LpJobList->ddJobIndex;
    /* Get the index of the sequence */
    LddSeqIndex = LpJobList->ddSequenceIndex;

    /* Job is failed. Update job result as FAILED */
    Spi_GaaJobResult[LddJobIndex] = SPI_JOB_FAILED;
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Enter_Spi(SPI_REGISTER_PROTECTION);
    #endif
    #if((SPI_FIFO_MODE == STD_ON) && (SPI_DEV_ERROR_DETECT == STD_ON))
    if(LucHWMemoryMode == SPI_FIFO_MODE_CONFIGURED)
    {
      /* Check if overrun error has occured */
      if((LpMainUserBaseAddr->ulMainSTR0 & SPI_OVRFLW_OVRRUN_ERR) != SPI_ZERO)
      {
        LpMainUserBaseAddr->usMainSTCR0 = (uint16)~(SPI_OVRFLW_OVRRUN_ERR_CLR);
        /* Report to DET */
        Det_ReportError(SPI_MODULE_ID, SPI_INSTANCE_ID,
                           SPI_ASYNCTRANSMIT_SID, SPI_E_SEQ_FAILED);
      }
      else
      {
        /* No action required */
      }
    }
    #endif
    #if (SPI_CRITICAL_SECTION_PROTECTED == STD_ON)
    /* Disable relevant interrupts */
    SchM_Exit_Spi(SPI_REGISTER_PROTECTION);
    #endif
    #endif
  }
  #if (((SPI_DIRECT_ACCESS_MODE == STD_ON) || (SPI_FIFO_MODE == STD_ON)) \
      &&((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON)))
  else
  #endif
  {
    #if ((SPI_DUAL_BUFFER_MODE == STD_ON) || (SPI_TX_ONLY_MODE == STD_ON))
    LddSeqIndex = Spi_GaaSeqQueue[Spi_GddQueueIndex[LucHWMemoryMode]];
    #endif
  }
  /* Sequence is failed. Update sequence result as FAILED */
  Spi_GaaSeqResult[LddSeqIndex] = SPI_SEQ_FAILED;
}

#define SPI_STOP_SEC_PRIVATE_CODE
#include "MemMap.h"/* PRQA S 5087 */

#endif /* End of ((SPI_LEVEL_DELIVERED == SPI_ONE) ||
                             (SPI_LEVEL_DELIVERED == SPI_TWO))*/
/*******************************************************************************
**                          End of File                                       **
*******************************************************************************/
