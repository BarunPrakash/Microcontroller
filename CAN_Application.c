/*----------------------------------------------------------------------------
 *  Manitowoc Mid Range Project
 *----------------------------------------------------------------------------
 *  @file		Can_application.c
 *  @brief	source file for the UARTs including the ISR functions,
 *					and initialization routines
 *  @date		05/04/2013
 *  @author	barun
 *--------------------------------------------------------------------------*/
#define CMSIS_BITPOSITIONS 1	//!< Turn on the bit definitions in LPC18xx.h - this adds time to compiling
															//!< These are needed for the CAN bus test.
 #include "APP_CAN.h"
 #include "Can_application.h"
 #include "string.h"
 #include <LPC18xx.h>
 #include "NVM_DATA.h"
 #include "SWUPgrade.h"
 #include "CAN_RX_FW_Transfer.h"
#include "Can_RX_Messages.h"
 #include "CAN_RX_Blob_Transfer.h"

CAN_Message Can_Ring_Buffer[MAX_CAN_BUFFER];
CAN_Message Message_Tx;
CAN_Message Message_Rx;
OS_TID CAN_TX_ID;
OS_TID CAN_RX_ID;
uint16_t can_messanger_count;
uint8_t Can_Ring_Buffer_COunt;
//static uint8_t Number_of_Messages_for_tx;
static U64 CAN_TX_Stack[2048/8]; // usart recive(2kB)
static U64 CAN_RX_Stack[2048/8]; // usart recive(2kB)
CanCallbacks_t canRXCallBacks[] =
{
{FIB_SW_VERSION,CAN_Recive_FIB_SW_Version,8},
{SYSTEM_COMMUNICATION_ACK,CAN_Receive_System_and_Comm_ACK_Response,2},
{FIB_PUMP_RESPONSE,CAN_Receive_FIB_Pump_Response,4},
{FIB_VALVE_REQUEST,CAN_Receive_FIB_Valve_Request,5},
{FILTER_RESPONSE,CAN_Receive_Filter_Response,5},
{BUTTON_PRESS_REQUEST,CAN_Receive_Button_Press_Request,1},
{HEATER_REQUEST,CAN_Receive_Heater_Request,2},
{FIB_STATUS_ERROR_MESSAGE,CAN_Receive_FIB_ERROR_STATUS_MESSAGE,2},
{FIB_RESET_RESPONSE,CAN_Receive_FIB_Reset_Response_Message,2},
{UI_SYSTEM_CNFG_REQ,CAN_Receive_UI_Systemconfig_REQ,1},  //10
{FITER_PAD_SETTINGS,CAN_Receive_Filter_Pad_Setting,1},
{UI_DATE_TIME_STATUS_CMD,CAN_Received_Date_Time_Status_Cmd,7},
{DATE_TIME_REQ,CAN_Received_Date_Time_REQ,1},
{UI_SYSTEM_CONFIG,CAN_Received_SYSTEM_CNFG1,4},
{(UI_SYSTEM_CONFIG+1),CAN_Received_SYSTEM_CNFG2,4},
{(UI_SYSTEM_CONFIG+2),CAN_Received_SYSTEM_CNFG3,4},
{(UI_SYSTEM_CONFIG+3),CAN_Received_SYSTEM_CNFG4,4},
{(UI_SYSTEM_CONFIG+4),CAN_Received_SYSTEM_CNFG5,4},
{(UI_SYSTEM_CONFIG+5),CAN_Received_SYSTEM_CNFG6,4},
{UI_DISPLAY_FORMAT_RESPONSE_CMD,CAN_Receive_Display_Format_Response_Cmd,8}, //20
{UI_DISPLAY_LANGUAGE_RESPONSE_CMD,CAN_Receive_Display_LANGUAGE_Response_Cmd,2},
{UI_VIB_E36_COMM_LOSS,CAN_Receive_VIB_E36_ERROR,2},
{JIB_RESET_STATUS_MESSAGE,CAN_Receive_Jib_Reset_Message,2},
{FIB_ERROR_STATE,CAN_Receive_FIB_ERROR_message,2},
{TECH_MODE,CAN_Receive_TECH_MODE_SETTING,5},
{TECH_SWVERSION,CAN_Receive_TECH_MODE_SOFTWARE_VERSION,2},
{TECH_OIBSTATUS,CAN_Receive_TECH_MODE_OIBSTATUS_REQ,2},
{TECH_DRAINVALVESTATUS,CAN_Receive_TECH_MODE_DRAINVALVESTATUS_REQ,2},
{TECH_COOKTEMPSTATUS,CAN_Receive_TECH_MODE_COOKTEMP_REQ,2},
{TECH_RETURNVALVESTATUS,CAN_Receive_TECH_MODE_RETURNVALVESTATUS_REQ,2}, //30
{TECH_SET_RETURNVALVE,CAN_Receive_TECH_MODE_SETRETURNVALVE_REQ,3},
{TECH_SET_DRAINVALVE,CAN_Receive_TECH_MODE_SETDRAINVALVE_REQ,3},
{TECH_SETSPEAKER,CAN_Receive_TECH_MODE_SETSPEAKER_REQ,2},
{TECH_SETBLOWER,CAN_Receive_TECH_MODE_SETBLOWER_REQ,3},
{TECH_SETHEAT,CAN_Receive_TECH_MODE_SETHEAT_REQ,3},
{TECH_SETVRELAY,CAN_Receive_TECH_MODE_SETVRELAY_REQ,3},
{TECH_MODE_SELF_DIAG,CAN_Receive_TECH_MODE_SELF_DIAG,2},
{TECH_SETOIB_RELAY,CAN_Receive_TECH_MODE_SETOIB_RELAY,3 },
	{FIB_ERROR_STATE,CAN_Receive_FIB_ERROR_message,2},
	{UI_COOK_STATUS_CMD,CAN_Received_Cook_Status1,8},//40
  {(UI_COOK_STATUS_CMD+1),CAN_Received_Cook_Status2,8},
  {(UI_COOK_STATUS_CMD+2),CAN_Received_Cook_Status3,8},
  {(UI_COOK_STATUS_CMD+3),CAN_Received_Cook_Status4,8},
  {(UI_COOK_STATUS_CMD+4),CAN_Received_Cook_Status5,8},
  {(UI_COOK_STATUS_CMD+5),CAN_Received_Cook_Status6,8},
	{PWD_REPLICATION_MESSAGE,CAN_Receive_Password_Replication,8},//46
	{FIB_OIP_REQUEST,CAN_Receive_E51_Error_Status,6},
	{DST_REPLICATION_MESSAGE,CAN_Receive_DST_Parameters,5},
	{UI_FILTER_STATUS_BAT,Filter_Status_received,5},
	{UI_ATO_TEMP_SEND,CAN_Rec_ATO_Probe_Temp_Command,2},
	{CONFIG_WR_REQ_MESSAGE,Config_Write_Req_received,2},	
	{CONFIG_WR_VERSION_MESSAGE,Config_Write_Version_received,2},
	{UI_BLOWER_TYPE_MESSAGE,CAN_Receive_Blower_type,1},
	{UI_FILTR_TIME_SETTINGS_MESSAGE,CAN_Receive_FilterTimeSettings,8},	
	{UI_ID_REQUEST,CAN_Receive_UI_ID_Request,0},
	{UI_V24_REQUEST,CAN_Receive_UI_V24_Request,1},
	{UI_TECHMODE_OIB_REQUEST,CAN_Receive_UI_Techmode_OIB_Request,1},
	{UI_TECHMODE_BOARD_VER_REQUEST,CAN_Receive_UI_Techmode_Board_version_Request,1},
	{UI_SCREENSAVER_TIME_MESSAGE,CAN_ScreenSaver_Time_Recieve,1},
	{UI_E64_ERROR_CLEAR,CAN_Recieved_Clear_E64_Error,1},
	{UI_AIF_TEMP_REQUEST,CAN_Rec_AIF_Probe_Temp_Command,2},	//Newly added for AIF temp 27/07/2015
	{ATO_SUCCESS_MESSAGE,CAN_Receive_ATO_Success_Command,1},
	{GATEWAY_VERSION_IP_WIFI_STATUS,CAN_Recieved_Gateway_Software_Version,5},/** Newly added for Gateway software version,IP address,WiFi signal status		*/
	{OQS_MESSAGE,CAN_read_tpm_value,8},//read tpm value
	{UI_OQS_SETT_RESPONSE_CMD,CAN_Receive_OQS_Settings_Response_Cmd, 8},
	{UI_OQS_SENSOR_SOFTWARE_VERSION_MESSAGE,CAN_Receive_OQS_Software_Version_Response_Cmd,5},
	{UI_OQS_SENSOR_OIL_TYPE_MESSAGE,readOqsOiltype,8},
	{UI_OQS_SENSOR_CURRENT_OIL_TYPE_MESSAGE,CAN_receive_CurrentOilType,8},
  {FRYER_OPTIMIZATION_REQUEST,request_fryer_data,1},
	{FRYER_OQS_DATA_REQUEST,request_oqs_filter_data,1},
	{ENERGY_SAVER_REQ,enter_exit_cool_mode,3},
	{TURN_OFF_VAT_REQ,tun_off_fry_pot,2},
	{MENU_INFO_REQUEST,req_max_menucount,1},
	{PRODUCT_NAME_REQUEST,req_product_name,2},
	{ UI_OIL_DRAG_OUT_FLOW_RATE, CAN_Receive_Oil_DragOut_FlowRate_Cmd, 8 },
	{UI_DEMO_MODE_ENABLE_MESSAGE,CAN_Receive_DemoMode_Enable_Cmd,1},
	{TECH_CHECK_BASKET_LIFT,CAN_Receive_TECH_MODE_BASKET_LIFT,3},
};
CanCallbacksFW_t canFWRXCallBacks[] =
{
	{GLOBAL_FW_UPDATE_CMD_RX, GlobalFWUpdateCommand_Rx, 2},
	{GLOBAL_FW_UPDATE_FI_RSP_RX, GlobalFWUpdateFIResponse_Rx, 2},
	{GLOBAL_FW_UPDATE_UI2_RSP_RX, GlobalFWUpdateUI2Response_Rx, 6},
	{GLOBAL_FW_UPDATE_UI3_RSP_RX, GlobalFWUpdateUI3Response_Rx, 6},
	{GLOBAL_FW_UPDATE_UI4_RSP_RX, GlobalFWUpdateUI4Response_Rx, 6},
	{GLOBAL_FW_UPDATE_UI5_RSP_RX, GlobalFWUpdateUI5Response_Rx, 6},
	{GLOBAL_FW_UPDATE_UI6_RSP_RX, GlobalFWUpdateUI6Response_Rx, 6},
	{GLOBAL_FW_START_CMD_RX, GlobalFWStartCommand_Rx, 6},
	{GLOBAL_FW_START_FI_RSP_RX, GlobalFWStartFIResponse_Rx, 1},
	{GLOBAL_FW_START_UI2_RSP_RX, GlobalFWStartUI2Response_Rx, 3},
	{GLOBAL_FW_START_UI3_RSP_RX, GlobalFWStartUI3Response_Rx, 3},
	{GLOBAL_FW_START_UI4_RSP_RX, GlobalFWStartUI4Response_Rx, 3},
	{GLOBAL_FW_START_UI5_RSP_RX, GlobalFWStartUI5Response_Rx, 3},
	{GLOBAL_FW_START_UI6_RSP_RX, GlobalFWStartUI6Response_Rx, 3},
	{GLOBAL_FW_RECORD_START_CMD_RX, GlobalFWRecordStartCommand_Rx, 4},
	{GLOBAL_FW_RECORD_TRANSFER_PSEQ_RX, GlobalFWRecordTransferPacketSeq_Rx, 8},
	{GLOBAL_FW_RECORD_END_CMD_RX, GlobalFWRecordEndCommand_Rx, 2},
	{GLOBAL_FW_RECORD_END_FI_RSP_RX, GlobalFWRecordEndFIResponse_Rx, 1},
	{GLOBAL_FW_RECORD_END_UI2_RSP_RX, GlobalFWRecordEndUI2Response_Rx, 3},
	{GLOBAL_FW_RECORD_END_UI3_RSP_RX, GlobalFWRecordEndUI3Response_Rx, 3},
	{GLOBAL_FW_RECORD_END_UI4_RSP_RX, GlobalFWRecordEndUI4Response_Rx, 3},
	{GLOBAL_FW_RECORD_END_UI5_RSP_RX, GlobalFWRecordEndUI5Response_Rx, 3},
	{GLOBAL_FW_RECORD_END_UI6_RSP_RX, GlobalFWRecordEndUI6Response_Rx, 3},
	{GLOBAL_FW_END_CMD_RX, GlobalFWEndCommand_Rx, 2},
	{GLOBAL_FW_END_FI_RSP_RX, GlobalFWEndFIResponse_Rx, 1},
	{GLOBAL_FW_END_UI2_RSP_RX, GlobalFWEndUI2Response_Rx, 3},
	{GLOBAL_FW_END_UI3_RSP_RX, GlobalFWEndUI3Response_Rx, 3},
	{GLOBAL_FW_END_UI4_RSP_RX, GlobalFWEndUI4Response_Rx, 3},
	{GLOBAL_FW_END_UI5_RSP_RX, GlobalFWEndUI5Response_Rx, 3},
	{GLOBAL_FW_END_UI6_RSP_RX, GlobalFWEndUI6Response_Rx, 3},
	{GLOBAL_FW_IMAGE_END_CMD_RX, GlobalFWImageEndCommand_Rx, 1},
	{GLOBAL_FW_PERCENTAGE_COMPLETION_RX, GlobalFWPercentageCompletion_Rx, 1},
};

CanCallbacksBlob_t canBlobRXCallBacks[] =
{
	{UI_LON_UPDATE_REQUEST,UI_LON_Rec_Update_Request,0},
	{UI_LON_SERIAL_NUMBER_REQUEST,UI_LON_Model_Number_Send,1},
	{UI_LON_WINK_MESSAGE,UI_LON_Wink_Response,0}, 
	{BLOB_RTS_CMD_GIB_RX, Blob_RTS_Command_GIB_Rx, 2},
	{BLOB_CTS_CMD_GIB_RX, Blob_CTS_Command_GIB_Rx, 2},
	{BLOB_DATA_CMD_GIB_RX, Blob_Data_GIB_Rx, 8},
	{BLOB_END_CMD_GIB_RX, Blob_End_GIB_Rx, 2},
	{BLOB_ACK_CMD_GIB_RX, Blob_Ack_GIB_Rx, 1},
	
};
 /*********************************************************************
 * @brief	   function will send the tx complete event to CAN
 * @param[in]	void
 * @return	    init status
 **********************************************************************/
U8 reset_poll;
uint8_t can_TX_error_count;
__task void SendCanTxMessage(void)
{
	CAN_msg can_msg_send;
	CAN_ERROR can_tx_status;
	//static uint8_t error_count=0;

	static QEvt const CAN_TX_COMPLETE_REQ = {CAN_TX_COMPLETE_SIG, 0, 0};
  static QEvt const CAN_TX_ERROR_REQ = {CAN_SEND_ERROR_SIG, 0, 0};

can_TX_error_count = 0;

    /** Create Tx event to  be published */
	   while(1)
	  {
			os_evt_wait_or(CAN_TX_KEY, 0xffff);

			 /** Copy the application message and send to driver */
			Retransmit :
			              if(can_TX_error_count > MAX_ERROR_COUNT)
										{

                       QACTIVE_POST(AO_CAN, &CAN_TX_ERROR_REQ, 0);
										}
										else
										{

													can_msg_send.type   = DATA_FRAME;
													can_msg_send.format = STANDARD_FORMAT;
													can_msg_send.id     = Message_Tx.id;
													can_msg_send.len 	 = Message_Tx.len;
													can_msg_send.ch  	 = 1;
													memset(can_msg_send.data,0x00, 8);
													memcpy(can_msg_send.data,Message_Tx.data, Message_Tx.len);

													can_tx_status = CAN_send (1,									// Index of the hardware CAN controller (1 .. x)
																									&can_msg_send,	// Pointer to CAN message to be sent
																									0x0F00);

													 switch (can_tx_status)
													{
														case CAN_DEALLOC_MEM_ERROR:
															 reset_poll = true;
															CAN_REinit();
															 can_TX_error_count++;
														   UARTprintf("CAN_DEALLOC_MEM_ERROR\n");
															 goto Retransmit;
															break;
														case CAN_TIMEOUT_ERROR:
															reset_poll = true;
															CAN_REinit();
															UARTprintf("CAN_TIMEOUT_ERROR\n");
															 can_TX_error_count++;
														   goto Retransmit;
															break;
														case CAN_ALLOC_MEM_ERROR:
															reset_poll = true;
															CAN_REinit();
															UARTprintf("CAN_ALLOC_MEM_ERROR\n");
														 can_TX_error_count++;
															 goto Retransmit;
															break;
														case CAN_OK:
															QACTIVE_POST(AO_CAN, &CAN_TX_COMPLETE_REQ, 0);
															can_TX_error_count =0; //reset to 0
															break;
													}
									}



  	}

}

/*********************************************************************
 * @brief	    function will check recived rx frame if it valid id it will process the rx frame
 * @param[in]	void
 * @return	    init status
 **********************************************************************/

__task void CanRxMessage(void)
{

  static CAN_ERROR myCanStatus;
  uint8_t i;
  CAN_msg MSG_RECIVED;

	   while(1)
	  {
		//os_evt_wait_or(CAN_RX_KEY, 0xffff);
				myCanStatus = CAN_receive (1, &MSG_RECIVED, 20);
				if((nSWUP_Idle == SWUPGRADE_GetState()) && (nSWUP_Idle == SWUPGRADE_GetCANState()))
				{
			if(myCanStatus == CAN_OK)
			{
					//myPrintf("Received Message = %x \r\n", MSG_RECIVED.id );
					//if(cbus_traffic_cmd == 0x00)
					if (((MSG_RECIVED.id < 0x200) || (MSG_RECIVED.id == UI_OQS_SENSOR_SOFTWARE_VERSION_MESSAGE ) || 
					(MSG_RECIVED.id == UI_OQS_SENSOR_OIL_TYPE_MESSAGE ) || (MSG_RECIVED.id == UI_OQS_SENSOR_CURRENT_OIL_TYPE_MESSAGE)	)&& (cbus_traffic_cmd == false) )
					{
						for(i=0;i<RX_END;i++)
						{
							if((MSG_RECIVED.id == canRXCallBacks[i].id) &&(MSG_RECIVED.len == canRXCallBacks[i].len))
							{
								Message_Rx.id=MSG_RECIVED.id ;
								memcpy(Message_Rx.data,MSG_RECIVED.data,sizeof(Message_Rx.data));
								canRXCallBacks[i].func(&Message_Rx);
								break;
							}
						}
					}
					//else if(cbus_traffic_cmd == 0x01)
					else if ((MSG_RECIVED.id >= 0x200) && (MSG_RECIVED.id <= 0x21F))
					{
						for(i=0;i<NUM_CAN_RX_FW_MESSAGES;i++)
						{
							if((MSG_RECIVED.id == canFWRXCallBacks[i].id))
							{
								Message_Rx.id=MSG_RECIVED.id ;
								Message_Rx.len= MSG_RECIVED.len;
								memcpy(Message_Rx.data,MSG_RECIVED.data,sizeof(Message_Rx.data));
								canFWRXCallBacks[i].func(&Message_Rx);
								break;
							}
						}
					}
					else if ((MSG_RECIVED.id >= 0x240) && (MSG_RECIVED.id <= 0x278))
					{
						for(i=0;i<NUM_CAN_RX_BLOB_MESSAGES;i++)
						{
							if((MSG_RECIVED.id == canBlobRXCallBacks[i].id))
							{
								Message_Rx.id=MSG_RECIVED.id ;
								Message_Rx.len= MSG_RECIVED.len;
								memcpy(Message_Rx.data,MSG_RECIVED.data,sizeof(Message_Rx.data));
								canBlobRXCallBacks[i].func(&Message_Rx);
								break;
							}
						}
					}
					else
					{
						/* Do Nothing */
					}
				}
			}
		}
	}


 /*********************************************************************
 * @brief	    Can init
 * @param[in]	void
 * @return	    init status
 **********************************************************************/
// Get a pointer to the CAN0 registers.


uint8_t Can_init(void)
{
	// Our hardware uses the C_CAN0 controller.  C_CAN1 is not available.  The various CAN
	// routines in RTX_CAN.c, such as CAN_init, take a an index value which represents the
	// the CAN controller to be used.  This index starts at 1, so we use 1 for controller 0.
    if(CAN_init (1, 500000)!=CAN_OK)			// CAN controller 1 init, 125 kbit/s
	{
		return 0;
	}

    if((CAN_rx_object (1,						// Index of the hardware CAN controller
                               2,						// Index of object used for reception
															 0,// CAN message identifier (ourselves)
															 DATA_TYPE | STANDARD_TYPE)	// format and type
																)!=CAN_OK)
		{
			return 0;
		}


	if ( CAN_start (1)!= CAN_OK)					// check for error
	{
		return 0;										// Flag test done
	}

  CAN_TX_ID=os_tsk_create_user(SendCanTxMessage, 100, CAN_TX_Stack, sizeof(CAN_TX_Stack));
	CAN_RX_ID=os_tsk_create_user(CanRxMessage,80, CAN_RX_Stack, sizeof(CAN_RX_Stack));

		UARTprintf("can tx task id : %d \n ",CAN_TX_ID);
	  UARTprintf("can Rx task id : %d \n ",CAN_RX_ID);


	return 1;
} // End CAN_Test


 /*********************************************************************
 * @brief	    can message which need to be transmitted
 * @param[in]	CAN_Message
 * @return	    None
 **********************************************************************/
 void CAN_Transmit_message(CAN_Message * Message)
 {
    memcpy(&Can_Ring_Buffer[Can_Ring_Buffer_COunt],Message,sizeof(CAN_Message));
   if(can_messanger_count == Can_Ring_Buffer_COunt)
   {
    static QEvt const CAN_TX_REQ = {CAN_TX_SIG, 0, 0};
    memcpy(&Message_Tx,&Can_Ring_Buffer[Can_Ring_Buffer_COunt],sizeof(CAN_Message));
    QACTIVE_POST(AO_CAN, &CAN_TX_REQ, 0);
   }
   Can_Ring_Buffer_COunt++;

   if(Can_Ring_Buffer_COunt >=MAX_CAN_BUFFER)
   {
      Can_Ring_Buffer_COunt =0;
   }

//    Number_of_Messages_for_tx++;
//
// 	 if(Number_of_Messages_for_tx > MAX_CAN_BUFFER)
// 	 {
// 		static QEvt const CAN_TX_FULL= {CAN_TX_FULL_SIG, 0, 0};
// 		QACTIVE_POST(AO_CAN, &CAN_TX_FULL, 0);
// 	 }
//
 }

  /*********************************************************************
 * @brief	    check if any messages are in que
 * @param[in]	void
 * @return	    None
 **********************************************************************/
void Can_Frame_Send(void)
{
  can_messanger_count++;
  if(can_messanger_count >=MAX_CAN_BUFFER)
  {
      can_messanger_count =0;
  }
  if(can_messanger_count != Can_Ring_Buffer_COunt)
  {
    static QEvt const CAN_TX_REQ = {CAN_TX_SIG, 0, 0};
    memcpy(&Message_Tx,&Can_Ring_Buffer[can_messanger_count],sizeof(CAN_Message));
    QACTIVE_POST(AO_CAN, &CAN_TX_REQ, 0);
  }
// 	if(Number_of_Messages_for_tx)
// 	{
// 		Number_of_Messages_for_tx--;
// 	}
}
  /*********************************************************************
 * @brief	    send the can message to can hal layer
 * @param[in]	void
 * @return	    None
 **********************************************************************/
 void Process_Can_Message(void)
 {
	 os_evt_set(CAN_TX_KEY, CAN_TX_ID);
 }

 /*********************************************************************
 * @brief	    process the recived can message
 * @param[in]	void
 * @return	    None
 **********************************************************************/
 void Process_rx_Can_Message(void)
 {
   uint8_t i;
	  for(i = 0; i < RX_END; i++)
     {
                    if(canRXCallBacks[i].id == Message_Rx.id)
                    {
                        canRXCallBacks[i].func(&Message_Rx);
                    }
    }
 }


 /*********************************************************************
 * @brief	    reinit the can module
 * @param[in]	void
 * @return	    None
 **********************************************************************/
 CAN_ERROR CAN_REinit(void)
 {
	CAN_ERROR retVal = CAN_OK;
	 if(CAN_init (1, 500000)!=CAN_OK)			// CAN controller 1 init, 125 kbit/s
	{
		retVal = CAN_OK;
	}

    if((CAN_rx_object (1,						// Index of the hardware CAN controller
                               2,						// Index of object used for reception
															 0,// CAN message identifier (ourselves)
															 DATA_TYPE | STANDARD_TYPE)	// format and type
																)!=CAN_OK)
		{
			retVal = CAN_OK;
		}


	if ( CAN_start (1)!= CAN_OK)					// check for error
	{
		retVal = CAN_OK;										// Flag test done
	}
	return retVal;
 }


 /*********************************************************************
 * @brief	    reinit can ring buffer
 * @param[in]	void
 * @return	    None
 **********************************************************************/
 void Reinit_Can_buffer(void)
 {
	 uint8_t i;
   can_messanger_count=0;
   Can_Ring_Buffer_COunt=0;
	 for(i=0;i<MAX_CAN_BUFFER;i++)
	 {
		  memset(&Can_Ring_Buffer[i],0x00,sizeof(CAN_Message));
	 }
 }

