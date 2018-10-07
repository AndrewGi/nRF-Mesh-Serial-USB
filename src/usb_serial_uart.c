/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "serial_uart_usb.h"
#include "nrf_mesh_config_serial_uart.h"

#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "nrf_mesh_serial.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_defines.h"

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "log.h"

#define UART_IRQ_LEVEL NRF_MESH_IRQ_PRIORITY_LOWEST

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
				    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
			    cdc_acm_user_ev_handler,
			    CDC_ACM_COMM_INTERFACE,
			    CDC_ACM_DATA_INTERFACE,
			    CDC_ACM_COMM_EPIN,
			    CDC_ACM_DATA_EPIN,
			    CDC_ACM_DATA_EPOUT,
			    APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
static bool m_send_flag = 0;

/********** Static variables **********/
static bool volatile is_open;
static bool m_can_receive;
static serial_uart_rx_cb_t m_char_rx_cb;
static serial_uart_tx_cb_t m_char_tx_cb;

/********** Interrupt handlers **********/
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
				    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
	case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
	{
            is_open = true;
	    /*Setup first transfer*/
	    ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
						   m_rx_buffer,
						   READ_SIZE);
						   
	    /*
	    if(m_char_rx_cb)
	      m_char_rx_cb(m_rx_buffer[0]);
	    UNUSED_VARIABLE(ret);
	    */
	    break;
	}
	case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            is_open = false;
	    break;
	case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
	    if (m_char_tx_cb)
	      m_char_tx_cb();
	    __LOG(LOG_SRC_SERIAL, LOG_LEVEL_INFO, "Sent!\n");
	    break;
	case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
	{
	    ret_code_t ret;
	    if(m_can_receive){
	       do
	       {
		   /*Get amount of data transfered*/
		   size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
	   
		   /* Fetch data until internal buffer is empty */
		   ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
					       m_rx_buffer,
					       READ_SIZE);
		   if(m_char_rx_cb){
		       m_char_rx_cb(m_rx_buffer[0]);
		       __LOG(LOG_SRC_SERIAL, LOG_LEVEL_INFO, "%c (%d)\n", m_rx_buffer[0], m_rx_buffer[0]);
		   }
	       } while ((m_can_receive) && ret == NRF_SUCCESS);
	    }
	    break;
	}
	default:
	    break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
	case APP_USBD_EVT_DRV_SUSPEND:
	    break;
	case APP_USBD_EVT_DRV_RESUME:
	    break;
	case APP_USBD_EVT_STARTED:
	    __LOG(LOG_SRC_SERIAL, LOG_LEVEL_INFO, "Started USB driver\n");
	    break;
	case APP_USBD_EVT_STOPPED:
	    app_usbd_disable();
	    break;
	case APP_USBD_EVT_POWER_DETECTED:
	    __LOG(LOG_SRC_SERIAL, LOG_LEVEL_INFO, "Power detected\n");
	    if (!nrf_drv_usbd_is_enabled())
	    {
		app_usbd_enable();
	    }
	    break;
	case APP_USBD_EVT_POWER_REMOVED:
	    app_usbd_stop();
	    break;
	case APP_USBD_EVT_POWER_READY:
	    app_usbd_start();
	    break;
	default:
	    break;
    }
}


void UART0_IRQHandler(void)
{
    serial_uart_process();
}
static void check_power() {
  nrfx_power_usb_state_t power_state = nrfx_power_usbstatus_get();
  switch(power_state){
    case NRFX_POWER_USB_STATE_CONNECTED:
      __LOG(LOG_SRC_SERIAL, LOG_LEVEL_INFO, "USB Connected!\n");
      //Connected
      if(!nrf_drv_usbd_is_enabled()){
	app_usbd_enable();
	app_usbd_start();
      }
      break;
    case NRFX_POWER_USB_STATE_DISCONNECTED:
      __LOG(LOG_SRC_SERIAL, LOG_LEVEL_INFO, "USB Disconnected!\n");
      if(nrf_drv_usbd_is_enabled()){
	app_usbd_disable();
      }
    }

}
/********** Interface Functions **********/
uint32_t serial_uart_init(serial_uart_rx_cb_t rx_cb, serial_uart_tx_cb_t tx_cb)
{

    if (rx_cb == NULL || tx_cb == NULL)
    {
	return NRF_ERROR_NULL;
    }

    m_char_rx_cb = rx_cb;
    m_char_tx_cb = tx_cb;

    /* Set up GPIOs: */

    ret_code_t ret;
     const app_usbd_config_t usbd_config = {
	.ev_state_proc = usbd_user_ev_handler
    };

    app_usbd_serial_num_generate();

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    /*
    ret = app_usbd_power_events_enable();
    APP_ERROR_CHECK(ret);
   */
    //No USB power detection because of soft device
    app_usbd_enable();
    app_usbd_start();

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "USB inited\n");
    serial_uart_process(); //Process the USB events until none are left
    //Wait for usb port open
    while(is_open==false) 
	serial_uart_process();
    return NRF_SUCCESS;
}

void serial_uart_process(void)
{
    while(app_usbd_event_queue_process());
}

void serial_uart_receive_set(bool enable_rx)
{
    __LOG(LOG_SRC_SERIAL, LOG_LEVEL_INFO, "enable_rx: %d\n", enable_rx);
    m_can_receive = enable_rx;
}

void serial_uart_byte_send(uint8_t value)
{
    ret_code_t ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, &value, 1);
    serial_uart_process();
    __LOG(LOG_SRC_SERIAL, LOG_LEVEL_INFO, "send value %c (%d)  (%d)\n", value, value, ret);
}