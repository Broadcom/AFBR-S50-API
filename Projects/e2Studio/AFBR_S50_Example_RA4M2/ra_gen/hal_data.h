/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_flash_hp.h"
#include "r_flash_api.h"
#include "rm_vee_flash.h"
#include "r_usb_basic.h"
#include "r_usb_basic_api.h"
#include "r_can.h"
#include "r_can_api.h"
#include "r_gpt.h"
#include "r_timer_api.h"
#include "r_dtc.h"
#include "r_transfer_api.h"
#include "r_sci_uart.h"
#include "r_uart_api.h"
#include "r_spi.h"
FSP_HEADER
/* Flash on Flash HP Instance */
extern const flash_instance_t g_flash0;

/** Access the Flash HP instance using these structures when calling API functions directly (::p_api is not used). */
extern flash_hp_instance_ctrl_t g_flash0_ctrl;
extern const flash_cfg_t g_flash0_cfg;

#ifndef rm_vee_flash_callback
void rm_vee_flash_callback(flash_callback_args_t * p_args);
#endif
extern const rm_vee_instance_t g_vee0;
extern rm_vee_flash_instance_ctrl_t g_vee0_ctrl;
extern const rm_vee_cfg_t g_vee0_cfg;

/** Callback used by VEE Instance. */
#ifndef Flash_callback
void Flash_callback(rm_vee_callback_args_t * p_args);
#endif
/* Basic on USB Instance. */
extern const usb_instance_t g_basic0;

/** Access the USB instance using these structures when calling API functions directly (::p_api is not used). */
extern usb_instance_ctrl_t g_basic0_ctrl;
extern const usb_cfg_t g_basic0_cfg;

#ifndef NULL
void NULL(void*);
#endif

#if 2 == BSP_CFG_RTOS
#ifndef NULL
void NULL(usb_event_info_t *, usb_hdl_t, usb_onoff_t);
#endif
#endif
/** CAN on CAN Instance. */
extern const can_instance_t g_can0;
/** Access the CAN instance using these structures when calling API functions directly (::p_api is not used). */
extern can_instance_ctrl_t g_can0_ctrl;
extern const can_cfg_t g_can0_cfg;
extern const can_extended_cfg_t g_can0_extended_cfg;

#ifndef can_callback
void can_callback(can_callback_args_t * p_args);
#endif
#define CAN_NO_OF_MAILBOXES_g_can0 (32)
/** Timer on GPT Instance. */
extern const timer_instance_t g_upt;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_upt_ctrl;
extern const timer_cfg_t g_upt_cfg;

#ifndef usb_poll_callback
void usb_poll_callback(timer_callback_args_t * p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_pit;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_pit_ctrl;
extern const timer_cfg_t g_pit_cfg;

#ifndef user_timer1_callback
void user_timer1_callback(timer_callback_args_t * p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_ltc;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_ltc_ctrl;
extern const timer_cfg_t g_ltc_cfg;

#ifndef NULL
void NULL(timer_callback_args_t * p_args);
#endif
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer3;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer3_ctrl;
extern const transfer_cfg_t g_transfer3_cfg;
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer2;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer2_ctrl;
extern const transfer_cfg_t g_transfer2_cfg;
/** UART on SCI Instance. */
extern const uart_instance_t g_uart0;

/** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
extern sci_uart_instance_ctrl_t g_uart0_ctrl;
extern const uart_cfg_t g_uart0_cfg;
extern const sci_uart_extended_cfg_t g_uart0_cfg_extend;

#ifndef user_uart_callback
void user_uart_callback(uart_callback_args_t * p_args);
#endif
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer1;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer1_ctrl;
extern const transfer_cfg_t g_transfer1_cfg;
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer0;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer0_ctrl;
extern const transfer_cfg_t g_transfer0_cfg;
/** SPI on SPI Instance. */
extern const spi_instance_t g_spi0;

/** Access the SPI instance using these structures when calling API functions directly (::p_api is not used). */
extern spi_instance_ctrl_t g_spi0_ctrl;
extern const spi_cfg_t g_spi0_cfg;

/** Callback used by SPI Instance. */
#ifndef user_spi_callback
void user_spi_callback(spi_callback_args_t * p_args);
#endif

#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == g_transfer0)
    #define g_spi0_P_TRANSFER_TX (NULL)
#else
#define g_spi0_P_TRANSFER_TX (&g_transfer0)
#endif
#if (RA_NOT_DEFINED == g_transfer1)
    #define g_spi0_P_TRANSFER_RX (NULL)
#else
#define g_spi0_P_TRANSFER_RX (&g_transfer1)
#endif
#undef RA_NOT_DEFINED
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
