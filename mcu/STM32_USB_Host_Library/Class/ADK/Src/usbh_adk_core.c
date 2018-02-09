/**
  ******************************************************************************
  * @file    usbh_adk_core.c
  * @author  Yuuichi Akagawa
  * @version V1.0.0
  * @date    2012/03/05
  * @brief   Android Open Accessory implementation
  ******************************************************************************
  * @attention
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *      http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  * <h2><center>&copy; COPYRIGHT (C)2012 Yuuichi Akagawa</center></h2>
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbh_adk_core.h"
#include <stdlib.h>
#include <stdio.h>
#include "adk_uart.h"
/** @defgroup USBH_ADK_CORE_Private_Variables
* @{
*/
ADK_Machine_TypeDef         ADK_Machine = {
    .acc_manufacturer="ammlab.org",
    .acc_model="HelloADK",
    .acc_description="HelloADK for GR-SAKURA for STM32F4",
    .acc_version="1.0",
    .acc_uri="play.google.com/store/apps/details?adk",
    .acc_serial="1234567",
    .initstate = ADK_INIT_SETUP,
    .state     = ADK_INITIALIZING,
    .tx_state = 0,
		.rx_state = 0,
		.usr_cb = NULL,
};

/**
* @}
*/

/** @defgroup USBH_ADK_CORE_Private_FunctionPrototypes
* @{
*/
static USBH_StatusTypeDef USBH_ADK_InterfaceInit  (USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_ADK_InterfaceDeInit  (USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_ADK_Process(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_ADK_SOFProcess(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_ADK_ClassRequest(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_ADK_getProtocol (USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_ADK_sendString ( USBH_HandleTypeDef *phost, uint16_t index, uint8_t* buff);
static USBH_StatusTypeDef USBH_ADK_switch ( USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_ADK_configAndroid ( USBH_HandleTypeDef *phost);

USBH_ClassTypeDef ADK_Class =
{
    "ADK",
    USB_ADK_CLASS,
    USBH_ADK_InterfaceInit,
    USBH_ADK_InterfaceDeInit,
    USBH_ADK_ClassRequest,
    USBH_ADK_Process,
    USBH_ADK_SOFProcess,
    NULL,
};
/**
* @}
*/


/**
  * @brief  USBH_ADK_InterfaceInit
  *         Interface initialization for ADK class.
  * @param  pdev: Selected device
  * @param  hdev: Selected device property
  * @retval USBH_StatusTypeDef : Status of class request handled.
  */
static USBH_StatusTypeDef USBH_ADK_InterfaceInit ( USBH_HandleTypeDef *phost)
{
	USBH_StatusTypeDef status = USBH_OK; //USBH_OK ;

	ADK_Machine.inSize = 0;
	ADK_Machine.outSize = 0;
	return status ;
}

/**
  * @brief  USBH_ADK_InterfaceDeInit
  *         De-Initialize interface by freeing host channels allocated to interface
  * @param  pdev: Selected device
  * @param  hdev: Selected device property
  * @retval None
  */
USBH_StatusTypeDef USBH_ADK_InterfaceDeInit ( USBH_HandleTypeDef *phost)
{
	ADK_Machine.initstate = ADK_INIT_SETUP;
  ADK_Machine.state = ADK_INITIALIZING;
	ADK_Machine.tx_state = 0;
	ADK_Machine.rx_state = 0;
	/* Switch to accessory mode,  Re-enumeration */
//	if(ADK_Machine.state == ADK_INITIALIZING)
//	{
//		pdev->host.ConnSts = 1;
//	}
	/* close bulk transfer pipe */
	if ( ADK_Machine.hc_num_out)
	{
		  USBH_ClosePipe(phost, ADK_Machine.hc_num_out);
			USBH_FreePipe(phost, ADK_Machine.hc_num_out);
	    ADK_Machine.hc_num_out = 0;     /* Reset the Channel as Free */
	}
	if ( ADK_Machine.hc_num_in)
	{
	    USBH_ClosePipe(phost, ADK_Machine.hc_num_in);
			USBH_FreePipe(phost, ADK_Machine.hc_num_in);
	    ADK_Machine.hc_num_in = 0;     /* Reset the Channel as Free */
	}

	//restore NAK retry limit to default value
//	pdev->host.NakRetryLimit = USB_NAK_RETRY_ATTEMPTS;
	return USBH_OK;
}

/**
  * @brief  USBH_ADK_ClassRequest
  *         This function will only initialize the ADK state machine
  * @param  pdev: Selected device
  * @param  hdev: Selected device property
  * @retval USBH_StatusTypeDef : Status of class request handled.
  */
static USBH_StatusTypeDef USBH_ADK_ClassRequest(USBH_HandleTypeDef *phost)
{
	USBH_StatusTypeDef status = USBH_BUSY ;

	switch (ADK_Machine.initstate)
	{
		case ADK_INIT_SETUP:
			if(phost->device.DevDesc.idVendor == USB_ACCESSORY_VENDOR_ID &&
			   (phost->device.DevDesc.idProduct == USB_ACCESSORY_PRODUCT_ID ||
			    phost->device.DevDesc.idProduct == USB_ACCESSORY_ADB_PRODUCT_ID)
		    ){
				ADK_Machine.initstate = ADK_INIT_CONFIGURE_ANDROID;
			}else{
				ADK_Machine.initstate = ADK_INIT_GET_PROTOCOL;
				ADK_Machine.protocol = -1;
			}

			break;

		case ADK_INIT_GET_PROTOCOL:
			if ( USBH_ADK_getProtocol ( phost ) == USBH_OK ){
				if (ADK_Machine.protocol >= 1) {
					ADK_Machine.initstate = ADK_INIT_SEND_MANUFACTURER;
					UART_Log("p is %d\r\n",ADK_Machine.protocol);
				  
				} else {
					ADK_Machine.initstate = ADK_INIT_FAILED;
					UART_Log("ADK:could not read device protocol version\r\n");
				}
			}
			break;
	  case ADK_INIT_SEND_MANUFACTURER:
			if( USBH_ADK_sendString ( phost, ACCESSORY_STRING_MANUFACTURER, (uint8_t*)ADK_Machine.acc_manufacturer)== USBH_OK ){
				ADK_Machine.initstate = ADK_INIT_SEND_MODEL;
					UART_Log("ADK:SEND_MANUFACTURER\r\n");
			}
			break;
	  case ADK_INIT_SEND_MODEL:
			if( USBH_ADK_sendString ( phost, ACCESSORY_STRING_MODEL, (uint8_t*)ADK_Machine.acc_model)== USBH_OK ){
				ADK_Machine.initstate = ADK_INIT_SEND_DESCRIPTION;
					UART_Log("ADK:SEND_MODEL\r\n");
			}
			break;
	  case ADK_INIT_SEND_DESCRIPTION:
			if( USBH_ADK_sendString ( phost, ACCESSORY_STRING_DESCRIPTION, (uint8_t*)ADK_Machine.acc_description)== USBH_OK ){
				ADK_Machine.initstate = ADK_INIT_SEND_VERSION;
					UART_Log("ADK:SEND_DESCRIPTION\r\n");
			}
			break;
	  case ADK_INIT_SEND_VERSION:
			if( USBH_ADK_sendString ( phost, ACCESSORY_STRING_VERSION, (uint8_t*)ADK_Machine.acc_version)== USBH_OK ){
				ADK_Machine.initstate = ADK_INIT_SEND_URI;
				UART_Log("ADK:SEND_VERSION\r\n");
			}
			break;
	  case ADK_INIT_SEND_URI:
			if( USBH_ADK_sendString ( phost, ACCESSORY_STRING_URI, (uint8_t*)ADK_Machine.acc_uri)== USBH_OK ){
				ADK_Machine.initstate = ADK_INIT_SEND_SERIAL;
				UART_Log("ADK:SEND_URI\r\n");
			}
			break;
	  case ADK_INIT_SEND_SERIAL:
			if( USBH_ADK_sendString ( phost, ACCESSORY_STRING_SERIAL, (uint8_t*)ADK_Machine.acc_serial)== USBH_OK ){
				ADK_Machine.initstate = ADK_INIT_SWITCHING;
				UART_Log("ADK:SEND_SERIAL\r\n");
			}
			break;
	  case ADK_INIT_SWITCHING:
			if( USBH_ADK_switch (phost)== USBH_OK ){
				ADK_Machine.initstate = ADK_INIT_GET_DEVDESC;
				UART_Log("ADK:switch to accessory mode\r\n");
			}
			//HAL_Delay(500);
			break;

	  case ADK_INIT_GET_DEVDESC:
			if( USBH_Get_DevDesc(phost, USB_DEVICE_DESC_SIZE)== USBH_OK ){
				ADK_Machine.initstate = ADK_INIT_DONE;
				ADK_Machine.pid = phost->device.DevDesc.idProduct;
				//check vaild device
				if(phost->device.DevDesc.idVendor == USB_ACCESSORY_VENDOR_ID &&
				   (phost->device.DevDesc.idProduct == USB_ACCESSORY_PRODUCT_ID ||
				    phost->device.DevDesc.idProduct == USB_ACCESSORY_ADB_PRODUCT_ID)
			    ){
					ADK_Machine.initstate = ADK_INIT_CONFIGURE_ANDROID;
						UART_Log("ADK:success\r\n");
				}else{
					ADK_Machine.initstate = ADK_INIT_FAILED;
					UART_Log("ADK:failed\r\n");
				}
			}
			break;

	  case ADK_INIT_CONFIGURE_ANDROID:
		    USBH_ADK_configAndroid(phost);
		    ADK_Machine.initstate = ADK_INIT_DONE;
		  	break;

	  case ADK_INIT_DONE:
		  	status = USBH_OK;
		  	ADK_Machine.state = ADK_IDLE;
				UART_Log("ADK:configuration complete.\r\n");
			phost->pUser(phost, HOST_USER_CLASS_ACTIVE); 
		  	break;

	  case ADK_INIT_FAILED:
			status = USBH_UNRECOVERED_ERROR;
			break;

	  default:
		  break;
	  }

	return status;
}

/**
  * @brief  USBH_ADK_Process
  *         ADK state machine handler
  * @param  pdev: Selected device
  * @param  hdev: Selected device property
  * @retval USBH_StatusTypeDef
  */
static USBH_StatusTypeDef USBH_ADK_Process(USBH_HandleTypeDef *phost)
{
	USBH_StatusTypeDef status = USBH_BUSY;
	USBH_URBStateTypeDef URB_Status;
  uint16_t rx_len;
	switch (ADK_Machine.state)
	{
		case ADK_IDLE:
			ADK_Machine.state = ADK_GET_DATA;
      ADK_Machine.inSize = 0;
    case ADK_GET_DATA:
      if(ADK_Machine.rx_state == 0)
      {
        USBH_BulkReceiveData(phost, ADK_Machine.inbuff, ADK_Machine.BulkInEpSize, ADK_Machine.hc_num_in);
        ADK_Machine.rx_state = 1;
        UART_Log("rx_0\r\n");
      }
      else 
      {
        URB_Status = USBH_LL_GetURBState(phost , ADK_Machine.hc_num_in);
        
        if(URB_Status == USBH_URB_DONE )
        {
          rx_len = USBH_LL_GetLastXferSize(phost, ADK_Machine.hc_num_in);
          ADK_Machine.inSize += rx_len;
          ADK_Machine.rx_state = 0;
          UART_Log("rx_1 %d\r\n",ADK_Machine.inSize);
          if(rx_len <= ADK_Machine.BulkInEpSize)//no need to receive 0 pack
            ADK_Machine.state = ADK_CALLBACK;
        }
      }
      
      break;
    case ADK_CALLBACK:
      if(ADK_Machine.usr_cb != NULL)
      {
        ADK_Machine.outSize = ADK_Machine.usr_cb(ADK_Machine.inbuff,
                                                 ADK_Machine.inSize,
                                                 ADK_Machine.outbuff,
                                                 USBH_ADK_DATA_SIZE);
      }
      else
      {
        ADK_Machine.outSize = ADK_Machine.inSize;
        memcpy(ADK_Machine.outbuff, ADK_Machine.inbuff, ADK_Machine.inSize);
        UART_Log("no cb\r\n");
      }
      ADK_Machine.state = ADK_SEND_DATA;
      break;
		case ADK_SEND_DATA:
			if(ADK_Machine.tx_state == 0)//prepare to send
			{
				if(ADK_Machine.outSize>=ADK_Machine.BulkOutEpSize)
				{
					USBH_BulkSendData(phost, ADK_Machine.outbuff, ADK_Machine.BulkOutEpSize, ADK_Machine.hc_num_out,0);
					
				}
				else
				{
				  USBH_BulkSendData(phost, ADK_Machine.outbuff, ADK_Machine.outSize, ADK_Machine.hc_num_out,0);
				}
        ADK_Machine.tx_state = 1;
        UART_Log("tx_0 %d\r\n",ADK_Machine.outSize);
			}
			else
			{
				
				URB_Status = USBH_LL_GetURBState(phost , ADK_Machine.hc_num_out);
				if(URB_Status == USBH_URB_DONE )//wai until sent ok
				{
				  if(ADK_Machine.outSize >= ADK_Machine.BulkOutEpSize)//we have to receive 0 pack
					{
					  ADK_Machine.outSize -= ADK_Machine.BulkOutEpSize;
          }
          else
          {
            ADK_Machine.outSize = 0;
            ADK_Machine.state = ADK_IDLE;
          }
        	ADK_Machine.tx_state = 0;
          UART_Log("tx_1\r\n");
				}
				else if( URB_Status == USBH_URB_NOTREADY )
    		{
    			ADK_Machine.tx_state = 0;
				}
			}
			break;

		case ADK_BUSY:
			ADK_Machine.state = ADK_IDLE;
			ADK_Machine.outSize = 0;
			break;

		default:
			break;
	}
	status = USBH_OK;
	return status;
}

/**
  * @brief  USBH_ADK_getProtocol
  *         Inquiry protocol version number from Android device.
  * @param  pdev: Selected device
  * @param  hdev: Selected device property
  * @retval USBH_StatusTypeDef
  */
static USBH_StatusTypeDef USBH_ADK_getProtocol ( USBH_HandleTypeDef *phost)
{
	phost->Control.setup.b.bmRequestType = USB_D2H | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE;
	phost->Control.setup.b.bRequest = ACCESSORY_GET_PROTOCOL;
	phost->Control.setup.b.wValue.w = 0;
	phost->Control.setup.b.wIndex.w = 0;
	phost->Control.setup.b.wLength.w = 2;

	/* Control Request */
	return USBH_CtlReq(phost, (uint8_t*)&(ADK_Machine.protocol) , 2 );
}

/**
  * @brief  USBH_ADK_sendString
  *         Send identifying string information to the Android device.
  * @param  pdev: Selected device
  * @param  hdev: Selected device property
  * @param  index: String ID
  * @param  buff: Identifying string
  * @retval USBH_StatusTypeDef
  */
static USBH_StatusTypeDef USBH_ADK_sendString ( USBH_HandleTypeDef *phost, uint16_t index, uint8_t* buff)
{
	uint16_t length;
	length = (uint16_t)strlen(buff)+1;

	phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE;
	phost->Control.setup.b.bRequest = ACCESSORY_SEND_STRING;
	phost->Control.setup.b.wValue.w = 0;
	phost->Control.setup.b.wIndex.w = index;
	phost->Control.setup.b.wLength.w = length;

	/* Control Request */
	return USBH_CtlReq(phost, buff , length );
}

/**
  * @brief  USBH_ADK_switch
  *         Request the Android device start up in accessory mode.
  * @param  pdev: Selected device
  * @param  hdev: Selected device property
  * @retval USBH_StatusTypeDef
  */
static USBH_StatusTypeDef USBH_ADK_switch (USBH_HandleTypeDef *phost)
{
	phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE;
	phost->Control.setup.b.bRequest = ACCESSORY_START;
	phost->Control.setup.b.wValue.w = 0;
	phost->Control.setup.b.wIndex.w = 0;
	phost->Control.setup.b.wLength.w = 0;

	/* Control Request */
	return USBH_CtlReq(phost, 0 , 0);
}

/**
  * @brief  USBH_ADK_configAndroid
  *         Setup bulk transfer endpoint and open channel.
  * @param  pdev: Selected device
  * @param  hdev: Selected device property
  * @retval USBH_StatusTypeDef
  */
static USBH_StatusTypeDef USBH_ADK_configAndroid (USBH_HandleTypeDef *phost)
{
    if(phost->device.CfgDesc.Itf_Desc[0].Ep_Desc[0].bEndpointAddress & 0x80)
    {
      ADK_Machine.BulkInEp = (phost->device.CfgDesc.Itf_Desc[0].Ep_Desc[0].bEndpointAddress);
      ADK_Machine.BulkInEpSize  = phost->device.CfgDesc.Itf_Desc[0].Ep_Desc[0].wMaxPacketSize;
    }
    else
    {
      ADK_Machine.BulkOutEp = (phost->device.CfgDesc.Itf_Desc[0].Ep_Desc[0].bEndpointAddress);
      ADK_Machine.BulkOutEpSize  = phost->device.CfgDesc.Itf_Desc[0].Ep_Desc[0].wMaxPacketSize;
    }

    if(phost->device.CfgDesc.Itf_Desc[0].Ep_Desc[1].bEndpointAddress & 0x80)
    {
      ADK_Machine.BulkInEp = (phost->device.CfgDesc.Itf_Desc[0].Ep_Desc[1].bEndpointAddress);
      ADK_Machine.BulkInEpSize  = phost->device.CfgDesc.Itf_Desc[0].Ep_Desc[1].wMaxPacketSize;
    }
    else
    {
      ADK_Machine.BulkOutEp = (phost->device.CfgDesc.Itf_Desc[0].Ep_Desc[1].bEndpointAddress);
      ADK_Machine.BulkOutEpSize  = phost->device.CfgDesc.Itf_Desc[0].Ep_Desc[1].wMaxPacketSize;
    }

    ADK_Machine.hc_num_out = USBH_AllocPipe(phost,ADK_Machine.BulkOutEp);
    ADK_Machine.hc_num_in  = USBH_AllocPipe(phost, ADK_Machine.BulkInEp);

    /* Open the new channels */
    USBH_OpenPipe  (phost,
                      ADK_Machine.hc_num_out,
                      ADK_Machine.BulkOutEp,
                      phost->device.address,
                      phost->device.speed,
                      EP_TYPE_BULK,
                      ADK_Machine.BulkOutEpSize);  
    USBH_OpenPipe  (phost,
                      ADK_Machine.hc_num_in,
                      ADK_Machine.BulkInEp,
                      phost->device.address,
                      phost->device.speed,
                      EP_TYPE_BULK,
                      ADK_Machine.BulkInEpSize);
    UART_Log("config android in %d out %d\r\n",ADK_Machine.BulkInEpSize,ADK_Machine.BulkOutEpSize);
	return USBH_OK;
}

/**
  * @brief  USBH_ADK_write
  *         Send data to Android device.
  * @param  pdev: Selected device
  * @param  buff: send data
  * @param  len : send data length
  * @retval USBH_StatusTypeDef
  */
USBH_StatusTypeDef USBH_ADK_write(USBH_HandleTypeDef *phost,uint8_t *buff, uint16_t len)
{
	memcpy(ADK_Machine.outbuff, buff, len);
	ADK_Machine.outSize = len;
	return USBH_OK;
}

/**
  * @brief  USBH_ADK_read
  *         Receive data from  Android device.
  * @param  pdev: Selected device
  * @param  buff: receive data
  * @param  len : receive data buffer length
  * @retval received data length
  */
uint16_t USBH_ADK_read(USBH_HandleTypeDef *phost,uint8_t *buff, uint16_t len)
{
	uint16_t length = 0;
	length = USBH_LL_GetLastXferSize(phost, ADK_Machine.hc_num_in);
	if( length > 0 ){
		memcpy(buff, ADK_Machine.inbuff, (len<=length)?len:length);
		ADK_Machine.inSize = 0;
	}
	return length;
}

/**
  * @brief  USBH_ADK_getStatus
  *         Return ADK_Machine.state
  * @param  None
  * @retval ADK_Machine.state
  */
void USBH_ADK_SETcb(uint16_t(*usr_cb)        (char* rx_buf, uint16_t rx_size, char* tx_buf, uint16_t tx_buf_size))
{
	ADK_Machine.usr_cb = usr_cb;
}
static USBH_StatusTypeDef USBH_ADK_SOFProcess (USBH_HandleTypeDef *phost)
{
  return USBH_OK;  
}
