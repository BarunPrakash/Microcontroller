/*************************************************************************//**
 *  Manitowoc Mid Range Project
 *****************************************************************************
 *  @file		RTX_CAN.c
 *  @brief	CAN Generic Layer Driver
 *  @date		2/01/2013
 *  @author	Keil, edited by Control Products
 ****************************************************************************/

#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <RTX_CAN.h>                  /* CAN Generic functions & defines     */
#include <LPC18xx.h>

#pragma diag_suppress 550

/* Declare memory pool for CAN messages, both transmit and receive           */
CAN_msgpool_declare(CAN_mpool,CAN_CTRL_MAX_NUM*(CAN_No_SendObjects+CAN_No_ReceiveObjects));

/* Declare mailbox, for CAN transmit messages                                */
mbx_arr_declare(MBX_tx_ctrl,CAN_CTRL_MAX_NUM,CAN_No_SendObjects);

/* Declare mailbox, for CAN receive messages                                 */
mbx_arr_declare(MBX_rx_ctrl,CAN_CTRL_MAX_NUM,CAN_No_ReceiveObjects);

/* Semaphores used for protecting writing to CAN hardware                    */
OS_SEM wr_sem[CAN_CTRL_MAX_NUM];


/*----------------------------------------------------------------------------
 *      CAN RTX Generic Driver Functions
 *----------------------------------------------------------------------------
 *  Functions implemented in this module:
 *           CAN_ERROR CAN_mem_init  (void);
 *           CAN_ERROR CAN_setup     (void)
 *           CAN_ERROR CAN_init      (U32 ctrl, U32 baudrate)
 *           CAN_ERROR CAN_start     (U32 ctrl)
 *    static CAN_ERROR CAN_push      (U32 ctrl, CAN_msg *msg, U16 timeout)
 *           CAN_ERROR CAN_send      (U32 ctrl, CAN_msg *msg, U16 timeout)
 *           CAN_ERROR CAN_request   (U32 ctrl, CAN_msg *msg, U16 timeout)
 *           CAN_ERROR CAN_set       (U32 ctrl, CAN_msg *msg, U16 timeout)
 *    static CAN_ERROR CAN_pull      (U32 ctrl, CAN_msg *msg, U16 timeout)
 *           CAN_ERROR CAN_receive   (U32 ctrl, CAN_msg *msg, U16 timeout)
 *           CAN_ERROR CAN_rx_object (U32 ctrl, U32 ch, U32 id, U32 object_para)
 *           CAN_ERROR CAN_tx_object (U32 ctrl, U32 ch,         U32 object_para)
 *---------------------------------------------------------------------------*/

/*******************************************************************//**
 * @brief			CAN_init: The first time this function is called, 
 *						initialize the memory pool for CAN messages and setup CAN
 *						controllers hardware.
 *  					Initialize mailboxes for CAN messages and initialize CAN controller
 * @param[in]	ctrl:       Index of the hardware CAN controller (1 .. x)
 * @param[in]	baudrate:   Baudrate
 * @return		CAN_ERROR:  Error code
 **********************************************************************/
CAN_ERROR CAN_init (U32 ctrl, U32 baudrate)  {
  static U8 first_run_flag = 0;
  CAN_ERROR error_code;
  U32 ctrl0 = ctrl-1;                 /* Controller index 0 .. x-1           */

  /* Initialize the Semaphore before the first use */
  os_sem_init (wr_sem[ctrl0], 1);

  /* When function is called for the first time it will initialize and setup 
     all of the resources that are common to CAN functionality               */
  if ((first_run_flag == 0)||(reset_poll == 1))  {
    first_run_flag = 1;
		reset_poll = 0;
    if (_init_box (CAN_mpool, sizeof(CAN_mpool), sizeof(CAN_msg)) == 1)
      return CAN_MEM_POOL_INIT_ERROR;
  }

  os_mbx_init (MBX_tx_ctrl[ctrl0], sizeof(MBX_tx_ctrl[ctrl0]));
  os_mbx_init (MBX_rx_ctrl[ctrl0], sizeof(MBX_rx_ctrl[ctrl0]));

  error_code = _CAN_hw_setup (ctrl);
  if (error_code != CAN_OK) 
    return error_code;

  return (CAN_hw_init (ctrl, baudrate));
}

/*******************************************************************//**
 * @brief			Start CAN controller (enable it to participate on CAN network)
 * @param[in]	ctrl:       Index of the hardware CAN controller (1 .. x)
 * @return		CAN_ERROR:  Error code
 **********************************************************************/
CAN_ERROR CAN_start (U32 ctrl)  {
  return (CAN_hw_start (ctrl));
}

/*******************************************************************//**
 * @brief			Send CAN_msg if hardware is free for sending, otherwise push message to 
 *  					message queue to be sent when hardware becomes free
 * @param[in]	ctrl:       Index of the hardware CAN controller (1 .. x)
 * @param[in] msg:        Pointer to CAN message to be sent
 * @param[in] timeout:    Timeout value for message sending
 * @return		CAN_ERROR:  Error code
 **********************************************************************/
static CAN_ERROR CAN_push (U32 ctrl, CAN_msg *msg, U16 timeout)  {
  CAN_msg *ptrmsg;
  U32 ctrl0 = ctrl-1;                 /* Controller index 0 .. x-1           */

  if (CAN_hw_tx_empty (ctrl) == CAN_OK) { /* Transmit hardware free for send */
		//disable interrupts
		NVIC_DisableIRQ(C_CAN0_IRQn);
    CAN_hw_wr (ctrl, msg);            /* Send message                        */
		//re-enable interrupts
		NVIC_EnableIRQ(C_CAN0_IRQn);
  }
  else {                              /* If hardware for sending is busy     */
    /* Write the message to send mailbox if there is room for it             */
    ptrmsg = _alloc_box (CAN_mpool);
    if (ptrmsg != NULL) {
      *ptrmsg = *msg;
      /* If message hasn't been sent but timeout expired, deallocate memory  */
      if (os_mbx_send (MBX_tx_ctrl[ctrl0], ptrmsg, timeout) == OS_R_TMO) {
        if (_free_box (CAN_mpool, ptrmsg) == 1)
          return CAN_DEALLOC_MEM_ERROR;

        return CAN_TIMEOUT_ERROR;
      } else {
        /* Check once again if transmit hardware is ready for transmission   */
        if (CAN_hw_tx_empty (ctrl) == CAN_OK) { /* Transmit hw free for send */ 
          if (os_mbx_wait (MBX_tx_ctrl[ctrl0], (void **)&ptrmsg, timeout) == OS_R_TMO) {
            return CAN_TIMEOUT_ERROR;
          }
          if (_free_box (CAN_mpool, ptrmsg) == 1)
            return CAN_DEALLOC_MEM_ERROR;
          
					//disable interrupts
					NVIC_DisableIRQ(C_CAN0_IRQn);
					CAN_hw_wr (ctrl, msg);      /* Send message                        */
					//re-enable interrupts
					NVIC_EnableIRQ(C_CAN0_IRQn);
        }
      }
    } else
      return CAN_ALLOC_MEM_ERROR;
  }
  return CAN_OK;
}

/*******************************************************************//**
 * @brief			Send DATA FRAME message, see CAN_push function comment
 * @param[in]	ctrl:       Index of the hardware CAN controller (1 .. x)
 * @param[in] msg:        Pointer to CAN message to be sent
 * @param[in] timeout:    Timeout value for message sending
 * @return		CAN_ERROR:  Error code
 **********************************************************************/
CAN_ERROR CAN_send (U32 ctrl, CAN_msg *msg, U16 timeout)  {
  msg->type = DATA_FRAME;

  return (CAN_push (ctrl, msg, timeout));
}

/*******************************************************************//**
 * @brief			Send REMOTE FRAME message, see CAN_push function comment
 * @param[in]	ctrl:       Index of the hardware CAN controller (1 .. x)
 * @param[in] msg:        Pointer to CAN message to be sent
 * @param[in] timeout:    Timeout value for message sending
 * @return		CAN_ERROR:  Error code
 **********************************************************************/
CAN_ERROR CAN_request (U32 ctrl, CAN_msg *msg, U16 timeout)  {
  msg->type = REMOTE_FRAME;

  return (CAN_push (ctrl, msg, timeout));
}

/*******************************************************************//**
 * @brief			Set a message that will automatically be sent as an answer to REMOTE 
 *						FRAME message
 * @param[in]	ctrl:       Index of the hardware CAN controller (1 .. x)
 * @param[in] msg:        Pointer to CAN message to be sent
 * @param[in] timeout:    Timeout value for message sending
 * @return		CAN_ERROR:  Error code
 **********************************************************************/
CAN_ERROR CAN_set (U32 ctrl, CAN_msg *msg, U16 timeout)  {
  S32 i = timeout;
  CAN_ERROR error_code;

  do {
    if (CAN_hw_tx_empty (ctrl) == CAN_OK)  {  /* Transmit hardware free      */
      error_code = CAN_hw_set (ctrl, msg);    /* Set message                 */
      os_sem_send (wr_sem[ctrl-1]);     /* Return a token back to semaphore  */
      return error_code;
    }
    if (timeout == 0xffff)              /* Indefinite wait                   */
      i++;
    i--;
    os_dly_wait (1);                    /* Wait 1 timer tick                 */
  }  while (i >= 0);

  return CAN_TIMEOUT_ERROR;             /* CAN message not set               */
}

/*******************************************************************//**
 * @brief			Pull first received and unread CAN_msg from receiving message queue
 * @param[in]	ctrl:       Index of the hardware CAN controller (1 .. x)
 * @param[in] msg:        Pointer to CAN message to be sent
 * @param[in] timeout:    Timeout value for message sending
 * @return		CAN_ERROR:  Error code
 **********************************************************************/
static CAN_ERROR CAN_pull (U32 ctrl, CAN_msg *msg, U16 timeout)  {
  CAN_msg *ptrmsg;
  U32 ctrl0 = ctrl-1;                 /* Controller index 0 .. x-1           */

  /* Wait for received message in mailbox                                    */
  if (os_mbx_wait (MBX_rx_ctrl[ctrl0], (void **)&ptrmsg, timeout) == OS_R_TMO)
    return CAN_TIMEOUT_ERROR;

  /* Copy received message from mailbox to address given in function parameter msg */
  *msg = *ptrmsg;

  /* Free box where message was kept                                         */
  if (_free_box (CAN_mpool, ptrmsg) == 1)
    return CAN_DEALLOC_MEM_ERROR;

  return CAN_OK;
}

/*******************************************************************//**
 * @brief			Read received message, see CAN_pull function comment
 * @param[in]	ctrl:       Index of the hardware CAN controller (1 .. x)
 * @param[in] msg:        Pointer where CAN message will be read
 * @param[in] timeout:    Timeout value for message receiving
 * @return		CAN_ERROR:  Error code
 **********************************************************************/
CAN_ERROR CAN_receive (U32 ctrl, CAN_msg *msg, U16 timeout)  {
  return (CAN_pull (ctrl, msg, timeout));
}

/*******************************************************************//**
 * @brief			Enable reception of messages on specified controller and channel 
 *						with specified identifier
 * @param[in]	ctrl:       Index of the hardware CAN controller (1 .. x)
 * @param[in] ch:         Channel for the message transmission
 * @param[in] id:         CAN message identifier
 * @param[in] object_para: Object parameters (standard or extended format, 
 *                          data or remote frame)
 * @return		CAN_ERROR:  Error code
 **********************************************************************/
CAN_ERROR CAN_rx_object (U32 ctrl, U32 ch, U32 id, U32 object_para)  {
  return (CAN_hw_rx_object (ctrl, ch, id, object_para));
}

/*******************************************************************//**
 * @brief			Enable transmission of messages on specified controller 
 *						and channel with specified identifier
 * @param[in]	ctrl:       Index of the hardware CAN controller (1 .. x)
 * @param[in] ch:         Channel for the message transmission
 * @param[in] object_para: Object parameters (standard or extended format, 
 *                          data or remote frame)
 * @return		CAN_ERROR:  Error code
 **********************************************************************/
CAN_ERROR CAN_tx_object (U32 ctrl, U32 ch, U32 object_para)  {
  return (CAN_hw_tx_object (ctrl, ch, object_para));
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
