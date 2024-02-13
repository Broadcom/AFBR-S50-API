/* generated HAL source file - do not edit */
#include "hal_data.h"

flash_hp_instance_ctrl_t g_flash0_ctrl;
const flash_cfg_t g_flash0_cfg =
        {
          .data_flash_bgo = true,
          .p_callback = rm_vee_flash_callback,
          .p_context = &g_vee0_ctrl,
#if defined(VECTOR_NUMBER_FCU_FRDYI)
    .irq                 = VECTOR_NUMBER_FCU_FRDYI,
#else
          .irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_FCU_FIFERR)
    .err_irq             = VECTOR_NUMBER_FCU_FIFERR,
#else
          .err_irq = FSP_INVALID_VECTOR,
#endif
          .err_ipl = (13),
          .ipl = (13),
        };
/* Instance structure to use this module. */
const flash_instance_t g_flash0 =
        {
          .p_ctrl = &g_flash0_ctrl,
          .p_cfg = &g_flash0_cfg,
          .p_api = &g_flash_on_flash_hp
        };
rm_vee_flash_instance_ctrl_t g_vee0_ctrl;

const rm_vee_flash_cfg_t g_vee0_cfg_ext = {
                                            .p_flash = &g_flash0
};

static uint16_t g_vee0_record_offset[4 + 1] = { 0 };

const rm_vee_cfg_t g_vee0_cfg =
        {
          .start_addr = BSP_FEATURE_FLASH_DATA_FLASH_START,
          .num_segments = 2,
          .total_size = BSP_DATA_FLASH_SIZE_BYTES,
          .ref_data_size = 0,
          .record_max_id = 4,
          .rec_offset = &g_vee0_record_offset[0],
          .p_callback = Flash_callback,
          .p_context = NULL,
          .p_extend = &g_vee0_cfg_ext
        };

/* Instance structure to use this module. */
const rm_vee_instance_t g_vee0 =
        {
          .p_ctrl = &g_vee0_ctrl,
          .p_cfg = &g_vee0_cfg,
          .p_api = &g_rm_vee_on_flash
        };
usb_instance_ctrl_t g_basic0_ctrl;

#if !defined(g_usb_descriptor)
extern usb_descriptor_t g_usb_descriptor;
#endif
#define RA_NOT_DEFINED (1)
const usb_cfg_t g_basic0_cfg =
        {
          .usb_mode = USB_MODE_PERI,
          .usb_speed = USB_SPEED_FS,
          .module_number = 0,
          .type = USB_CLASS_PVND,
          #if defined(g_usb_descriptor)
                .p_usb_reg = g_usb_descriptor,
#else
          .p_usb_reg = &g_usb_descriptor,
#endif
          .usb_complience_cb = NULL,
#if defined(VECTOR_NUMBER_USBFS_INT)
                .irq       = VECTOR_NUMBER_USBFS_INT,
#else
          .irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_USBFS_RESUME)
                .irq_r     = VECTOR_NUMBER_USBFS_RESUME,
#else
          .irq_r = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_USBFS_FIFO_0)
                .irq_d0    = VECTOR_NUMBER_USBFS_FIFO_0,
#else
          .irq_d0 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_USBFS_FIFO_1)
                .irq_d1    = VECTOR_NUMBER_USBFS_FIFO_1,
#else
          .irq_d1 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_USBHS_USB_INT_RESUME)
                .hsirq     = VECTOR_NUMBER_USBHS_USB_INT_RESUME,
#else
          .hsirq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_USBHS_FIFO_0)
                .hsirq_d0  = VECTOR_NUMBER_USBHS_FIFO_0,
#else
          .hsirq_d0 = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_USBHS_FIFO_1)
                .hsirq_d1  = VECTOR_NUMBER_USBHS_FIFO_1,
#else
          .hsirq_d1 = FSP_INVALID_VECTOR,
#endif
          .ipl = (12),
          .ipl_r = (12),
          .ipl_d0 = (12),
          .ipl_d1 = (12),
          .hsipl = (BSP_IRQ_DISABLED),
          .hsipl_d0 = (BSP_IRQ_DISABLED),
          .hsipl_d1 = (BSP_IRQ_DISABLED),
#if (BSP_CFG_RTOS != 0)
                .p_usb_apl_callback = NULL,
#else
          .p_usb_apl_callback = NULL,
#endif
#if defined(NULL)
                .p_context = NULL,
#else
          .p_context = &NULL,
        #endif
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
#else
                .p_transfer_tx = &RA_NOT_DEFINED,
#endif
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
#else
                .p_transfer_rx = &RA_NOT_DEFINED,
#endif
        };
#undef RA_NOT_DEFINED

/* Instance structure to use this module. */
const usb_instance_t g_basic0 =
        {
          .p_ctrl = &g_basic0_ctrl,
          .p_cfg = &g_basic0_cfg,
          .p_api = &g_usb_on_usb,
        };
#ifndef CAN0_BAUD_SETTINGS_OVERRIDE
#define CAN0_BAUD_SETTINGS_OVERRIDE  (0)
#endif
#if CAN0_BAUD_SETTINGS_OVERRIDE
can_bit_timing_cfg_t g_can0_bit_timing_cfg =
{
    .baud_rate_prescaler = 1,
    .time_segment_1 = 4,
    .time_segment_2 = 2,
    .synchronization_jump_width = 1
};
#else
can_bit_timing_cfg_t g_can0_bit_timing_cfg =
        {
          /* Actual bitrate: 1000000 Hz. Actual Bit Time Ratio: 70 %. */.baud_rate_prescaler = 1 + 4 /* Division value of baud rate prescaler */,
          .time_segment_1 = 6, .time_segment_2 = 3, .synchronization_jump_width = 3,
        };
#endif

uint32_t g_can0_mailbox_mask[CAN_NO_OF_MAILBOXES_g_can0 / 4] =
        {
          0x1FFFFFFF,
        #if CAN_NO_OF_MAILBOXES_g_can0 > 4
0x1FFFFFFF,
#endif
#if CAN_NO_OF_MAILBOXES_g_can0 > 8
0x1FFFFFFF,
0x1FFFFFFF,
#endif
#if CAN_NO_OF_MAILBOXES_g_can0 > 16
0x1FFFFFFF,
0x1FFFFFFF,
#endif
#if CAN_NO_OF_MAILBOXES_g_can0 > 24
0x1FFFFFFF,
0x1FFFFFFF,
#endif
        };

can_mailbox_t g_can0_mailbox[CAN_NO_OF_MAILBOXES_g_can0] =
        {
          {
            .mailbox_id = 0,
            .id_mode = CAN_ID_MODE_STANDARD,
            .mailbox_type = CAN_MAILBOX_TRANSMIT,
            .frame_type = CAN_FRAME_TYPE_REMOTE
          },
          {
            .mailbox_id = 1,
            .id_mode = CAN_ID_MODE_STANDARD,
            .mailbox_type = CAN_MAILBOX_RECEIVE,
            .frame_type = CAN_FRAME_TYPE_DATA
          },
          {
            .mailbox_id = 2,
            .id_mode = CAN_ID_MODE_STANDARD,
            .mailbox_type = CAN_MAILBOX_RECEIVE,
            .frame_type = CAN_FRAME_TYPE_DATA,
          },
          {
            .mailbox_id = 3,
            .id_mode = CAN_ID_MODE_STANDARD,
            .mailbox_type = CAN_MAILBOX_RECEIVE,
            .frame_type = CAN_FRAME_TYPE_DATA
          },
        #if CAN_NO_OF_MAILBOXES_g_can0 > 4
    {
        .mailbox_id              =  4,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  5,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  6,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  7,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
#endif
#if CAN_NO_OF_MAILBOXES_g_can0 > 8
    {
        .mailbox_id              =  8,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_REMOTE
    },
    {
        .mailbox_id              =  9,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_REMOTE
    },
    {
        .mailbox_id              =  10,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_REMOTE
    },
    {
        .mailbox_id              =  11,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_REMOTE
    },
    {
        .mailbox_id              =  12,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA,
    },
    {
        .mailbox_id              =  13,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  14,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  15,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
#endif
#if CAN_NO_OF_MAILBOXES_g_can0 > 16
    {
        .mailbox_id              =  16,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },

    {
        .mailbox_id              =  17,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  18,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  19,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  20,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  21,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  22,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA,
    },
    {
        .mailbox_id              =  23,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
#endif
#if CAN_NO_OF_MAILBOXES_g_can0 > 24
    {
        .mailbox_id              =  24,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  25,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  26,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  27,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  28,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  29,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  30,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    },
    {
        .mailbox_id              =  31,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_DATA
    }
#endif
        };

#if CAN_CFG_FIFO_SUPPORT
const can_fifo_interrupt_cfg_t g_can0_fifo_int_cfg =
{
    .fifo_int_mode          = CAN_FIFO_INTERRUPT_MODE_RX_EVERY_FRAME | CAN_FIFO_INTERRUPT_MODE_TX_EVERY_FRAME,
    .rx_fifo_irq             = VECTOR_NUMBER_CAN0_FIFO_RX,
    .tx_fifo_irq             = VECTOR_NUMBER_CAN0_FIFO_TX,
};

can_rx_fifo_cfg_t g_can0_rx_fifo_cfg =
{
    .rx_fifo_mask1 = 0x1FFFFFFF,
    .rx_fifo_mask2 = 0x1FFFFFFF,

    .rx_fifo_id1 =
    {
        .mailbox_id              =  0,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_REMOTE
    },

    .rx_fifo_id2 =
    {
        .mailbox_id              =  0,
        .id_mode                 =  CAN_ID_MODE_STANDARD,
        .mailbox_type            =  CAN_MAILBOX_RECEIVE,
        .frame_type              =  CAN_FRAME_TYPE_REMOTE
    },
};
#endif

const can_extended_cfg_t g_can0_extended_cfg =
        {
          .clock_source = CAN_CLOCK_SOURCE_PCLKB,
          .p_mailbox_mask = g_can0_mailbox_mask,
          .p_mailbox = g_can0_mailbox,
          .global_id_mode = CAN_GLOBAL_ID_MODE_STANDARD,
          .mailbox_count = CAN_NO_OF_MAILBOXES_g_can0,
          .message_mode = CAN_MESSAGE_MODE_OVERWRITE,
          #if CAN_CFG_FIFO_SUPPORT
    .p_fifo_int_cfg         = &g_can0_fifo_int_cfg,
    .p_rx_fifo_cfg          = &g_can0_rx_fifo_cfg,
#else
          .p_fifo_int_cfg = NULL,
          .p_rx_fifo_cfg = NULL,
        #endif
        };

can_instance_ctrl_t g_can0_ctrl;
const can_cfg_t g_can0_cfg =
        {
          .channel = 0,
          .p_bit_timing = &g_can0_bit_timing_cfg,
          .p_callback = can_callback,
          .p_extend = &g_can0_extended_cfg,
          .p_context = NULL,
          .ipl = (3),
          #if defined(VECTOR_NUMBER_CAN0_MAILBOX_TX)
    .tx_irq             = VECTOR_NUMBER_CAN0_MAILBOX_TX,
#else
          .tx_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_CAN0_MAILBOX_RX)
    .rx_irq             = VECTOR_NUMBER_CAN0_MAILBOX_RX,
#else
          .rx_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_CAN0_ERROR)
    .error_irq             = VECTOR_NUMBER_CAN0_ERROR,
#else
          .error_irq = FSP_INVALID_VECTOR,
        #endif
        };
/* Instance structure to use this module. */
const can_instance_t g_can0 =
        {
          .p_ctrl = &g_can0_ctrl,
          .p_cfg = &g_can0_cfg,
          .p_api = &g_can_on_can
        };
gpt_instance_ctrl_t g_upt_ctrl;
#if 0
const gpt_extended_pwm_cfg_t g_upt_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT2_COUNTER_UNDERFLOW)
    .trough_irq          = VECTOR_NUMBER_GPT2_COUNTER_UNDERFLOW,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG0,
    .output_disable      = (gpt_output_disable_t) ( GPT_OUTPUT_DISABLE_NONE),
    .adc_trigger         = (gpt_adc_trigger_t) ( GPT_ADC_TRIGGER_NONE),
    .dead_time_count_up  = 0,
    .dead_time_count_down = 0,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
};
#endif
const gpt_extended_cfg_t g_upt_extend =
        {
          .gtioca = { .output_enabled = false,
                      .stop_level = GPT_PIN_LEVEL_LOW
          },
          .gtiocb = { .output_enabled = false,
                      .stop_level = GPT_PIN_LEVEL_LOW
          },
          .start_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .stop_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .clear_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .count_up_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .count_down_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .capture_a_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .capture_b_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .capture_a_ipl = (BSP_IRQ_DISABLED),
          .capture_b_ipl = (BSP_IRQ_DISABLED),
          #if defined(VECTOR_NUMBER_GPT2_CAPTURE_COMPARE_A)
    .capture_a_irq       = VECTOR_NUMBER_GPT2_CAPTURE_COMPARE_A,
#else
          .capture_a_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT2_CAPTURE_COMPARE_B)
    .capture_b_irq       = VECTOR_NUMBER_GPT2_CAPTURE_COMPARE_B,
#else
          .capture_b_irq = FSP_INVALID_VECTOR,
#endif
          .capture_filter_gtioca = GPT_CAPTURE_FILTER_NONE,
          .capture_filter_gtiocb = GPT_CAPTURE_FILTER_NONE,
          #if 0
    .p_pwm_cfg                   = &g_upt_pwm_extend,
#else
          .p_pwm_cfg = NULL,
#endif
#if 0
    .gtior_setting.gtior_b.gtioa  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.oadflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.oahld  = 0U,
    .gtior_setting.gtior_b.oae    = (uint32_t) false,
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) false,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfben  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsb  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
#else
          .gtior_setting.gtior = 0U,
        #endif
        };
const timer_cfg_t g_upt_cfg =
        {
          .mode = TIMER_MODE_PERIODIC,
          /* Actual period: 0.0001 seconds. Actual duty: 50%. */.period_counts = (uint32_t)0x2710, .duty_cycle_counts =
                  0x1388,
          .source_div = (timer_source_div_t)0,
          .channel = 2,
          .p_callback = usb_poll_callback,
          /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
          .p_context = &NULL,
#endif
          .p_extend = &g_upt_extend,
          .cycle_end_ipl = (4),
          #if defined(VECTOR_NUMBER_GPT2_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GPT2_COUNTER_OVERFLOW,
#else
          .cycle_end_irq = FSP_INVALID_VECTOR,
        #endif
        };
/* Instance structure to use this module. */
const timer_instance_t g_upt =
        {
          .p_ctrl = &g_upt_ctrl,
          .p_cfg = &g_upt_cfg,
          .p_api = &g_timer_on_gpt
        };
gpt_instance_ctrl_t g_pit_ctrl;
#if 0
const gpt_extended_pwm_cfg_t g_pit_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT1_COUNTER_UNDERFLOW)
    .trough_irq          = VECTOR_NUMBER_GPT1_COUNTER_UNDERFLOW,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG0,
    .output_disable      = (gpt_output_disable_t) ( GPT_OUTPUT_DISABLE_NONE),
    .adc_trigger         = (gpt_adc_trigger_t) ( GPT_ADC_TRIGGER_NONE),
    .dead_time_count_up  = 0,
    .dead_time_count_down = 0,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
};
#endif
const gpt_extended_cfg_t g_pit_extend =
        {
          .gtioca = { .output_enabled = false,
                      .stop_level = GPT_PIN_LEVEL_LOW
          },
          .gtiocb = { .output_enabled = false,
                      .stop_level = GPT_PIN_LEVEL_LOW
          },
          .start_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .stop_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .clear_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .count_up_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .count_down_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .capture_a_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .capture_b_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .capture_a_ipl = (BSP_IRQ_DISABLED),
          .capture_b_ipl = (BSP_IRQ_DISABLED),
          #if defined(VECTOR_NUMBER_GPT1_CAPTURE_COMPARE_A)
    .capture_a_irq       = VECTOR_NUMBER_GPT1_CAPTURE_COMPARE_A,
#else
          .capture_a_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT1_CAPTURE_COMPARE_B)
    .capture_b_irq       = VECTOR_NUMBER_GPT1_CAPTURE_COMPARE_B,
#else
          .capture_b_irq = FSP_INVALID_VECTOR,
#endif
          .capture_filter_gtioca = GPT_CAPTURE_FILTER_NONE,
          .capture_filter_gtiocb = GPT_CAPTURE_FILTER_NONE,
          #if 0
    .p_pwm_cfg                   = &g_pit_pwm_extend,
#else
          .p_pwm_cfg = NULL,
#endif
#if 0
    .gtior_setting.gtior_b.gtioa  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.oadflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.oahld  = 0U,
    .gtior_setting.gtior_b.oae    = (uint32_t) false,
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) false,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfben  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsb  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
#else
          .gtior_setting.gtior = 0U,
        #endif
        };
const timer_cfg_t g_pit_cfg =
        {
          .mode = TIMER_MODE_PERIODIC,
          /* Actual period: 1 seconds. Actual duty: 50%. */.period_counts = (uint32_t)0x5f5e100, .duty_cycle_counts =
                  0x2faf080,
          .source_div = (timer_source_div_t)0,
          .channel = 1,
          .p_callback = user_timer1_callback,
          /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
          .p_context = &NULL,
#endif
          .p_extend = &g_pit_extend,
          .cycle_end_ipl = (2),
          #if defined(VECTOR_NUMBER_GPT1_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GPT1_COUNTER_OVERFLOW,
#else
          .cycle_end_irq = FSP_INVALID_VECTOR,
        #endif
        };
/* Instance structure to use this module. */
const timer_instance_t g_pit =
        {
          .p_ctrl = &g_pit_ctrl,
          .p_cfg = &g_pit_cfg,
          .p_api = &g_timer_on_gpt
        };
gpt_instance_ctrl_t g_ltc_ctrl;
#if 0
const gpt_extended_pwm_cfg_t g_ltc_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT0_COUNTER_UNDERFLOW)
    .trough_irq          = VECTOR_NUMBER_GPT0_COUNTER_UNDERFLOW,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG0,
    .output_disable      = (gpt_output_disable_t) ( GPT_OUTPUT_DISABLE_NONE),
    .adc_trigger         = (gpt_adc_trigger_t) ( GPT_ADC_TRIGGER_NONE),
    .dead_time_count_up  = 0,
    .dead_time_count_down = 0,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
};
#endif
const gpt_extended_cfg_t g_ltc_extend =
        {
          .gtioca = { .output_enabled = false,
                      .stop_level = GPT_PIN_LEVEL_LOW
          },
          .gtiocb = { .output_enabled = false,
                      .stop_level = GPT_PIN_LEVEL_LOW
          },
          .start_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .stop_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .clear_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .count_up_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .count_down_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .capture_a_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .capture_b_source = (gpt_source_t)(GPT_SOURCE_NONE),
          .capture_a_ipl = (BSP_IRQ_DISABLED),
          .capture_b_ipl = (BSP_IRQ_DISABLED),
          #if defined(VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_A)
    .capture_a_irq       = VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_A,
#else
          .capture_a_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_B)
    .capture_b_irq       = VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_B,
#else
          .capture_b_irq = FSP_INVALID_VECTOR,
#endif
          .capture_filter_gtioca = GPT_CAPTURE_FILTER_NONE,
          .capture_filter_gtiocb = GPT_CAPTURE_FILTER_NONE,
          #if 0
    .p_pwm_cfg                   = &g_ltc_pwm_extend,
#else
          .p_pwm_cfg = NULL,
#endif
#if 0
    .gtior_setting.gtior_b.gtioa  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.oadflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.oahld  = 0U,
    .gtior_setting.gtior_b.oae    = (uint32_t) false,
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) false,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfben  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsb  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
#else
          .gtior_setting.gtior = 0U,
        #endif
        };
const timer_cfg_t g_ltc_cfg =
        {
          .mode = TIMER_MODE_PERIODIC,
          /* Actual period: 4000 seconds. Actual duty: 50%. */.period_counts = (uint32_t)0x5d21dba0,
          .duty_cycle_counts = 0x2e90edd0, .source_div = (timer_source_div_t)8,
          .channel = 0,
          .p_callback = NULL,
          /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
          .p_context = &NULL,
#endif
          .p_extend = &g_ltc_extend,
          .cycle_end_ipl = (BSP_IRQ_DISABLED),
          #if defined(VECTOR_NUMBER_GPT0_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GPT0_COUNTER_OVERFLOW,
#else
          .cycle_end_irq = FSP_INVALID_VECTOR,
        #endif
        };
/* Instance structure to use this module. */
const timer_instance_t g_ltc =
        {
          .p_ctrl = &g_ltc_ctrl,
          .p_cfg = &g_ltc_cfg,
          .p_api = &g_timer_on_gpt
        };
dtc_instance_ctrl_t g_transfer3_ctrl;

transfer_info_t g_transfer3_info =
        {
          .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
          .transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_DESTINATION,
          .transfer_settings_word_b.irq = TRANSFER_IRQ_END,
          .transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED,
          .transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED,
          .transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE,
          .transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL,
          .p_dest = (void*)NULL,
          .p_src = (void const*)NULL,
          .num_blocks = 0,
          .length = 0,
        };

const dtc_extended_cfg_t g_transfer3_cfg_extend =
        {
          .activation_source = VECTOR_NUMBER_SCI0_RXI,
        };
const transfer_cfg_t g_transfer3_cfg =
        {
          .p_info = &g_transfer3_info,
          .p_extend = &g_transfer3_cfg_extend,
        };

/* Instance structure to use this module. */
const transfer_instance_t g_transfer3 =
        {
          .p_ctrl = &g_transfer3_ctrl,
          .p_cfg = &g_transfer3_cfg,
          .p_api = &g_transfer_on_dtc
        };
dtc_instance_ctrl_t g_transfer2_ctrl;

transfer_info_t g_transfer2_info =
        {
          .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED,
          .transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE,
          .transfer_settings_word_b.irq = TRANSFER_IRQ_END,
          .transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED,
          .transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
          .transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE,
          .transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL,
          .p_dest = (void*)NULL,
          .p_src = (void const*)NULL,
          .num_blocks = 0,
          .length = 0,
        };

const dtc_extended_cfg_t g_transfer2_cfg_extend =
        {
          .activation_source = VECTOR_NUMBER_SCI0_TXI,
        };
const transfer_cfg_t g_transfer2_cfg =
        {
          .p_info = &g_transfer2_info,
          .p_extend = &g_transfer2_cfg_extend,
        };

/* Instance structure to use this module. */
const transfer_instance_t g_transfer2 =
        {
          .p_ctrl = &g_transfer2_ctrl,
          .p_cfg = &g_transfer2_cfg,
          .p_api = &g_transfer_on_dtc
        };
sci_uart_instance_ctrl_t g_uart0_ctrl;

baud_setting_t g_uart0_baud_setting =
        {
          /* Baud rate calculated with 0.469% error. */.semr_baudrate_bits_b.abcse = 0,
          .semr_baudrate_bits_b.abcs = 0, .semr_baudrate_bits_b.bgdm = 1, .cks = 0, .brr = 53, .mddr = (uint8_t)256,
          .semr_baudrate_bits_b.brme = false
        };

/** UART extended configuration for UARTonSCI HAL driver */
const sci_uart_extended_cfg_t g_uart0_cfg_extend =
        {
          .clock = SCI_UART_CLOCK_INT,
          .rx_edge_start = SCI_UART_START_BIT_FALLING_EDGE,
          .noise_cancel = SCI_UART_NOISE_CANCELLATION_DISABLE,
          .rx_fifo_trigger = SCI_UART_RX_FIFO_TRIGGER_MAX,
          .p_baud_setting = &g_uart0_baud_setting,
          .flow_control = SCI_UART_FLOW_CONTROL_RTS,
          #if 0xFF != 0xFF
                .flow_control_pin       = BSP_IO_PORT_FF_PIN_0xFF,
                #else
          .flow_control_pin = (bsp_io_port_pin_t)UINT16_MAX,
#endif
          .rs485_setting = {
                             .enable = SCI_UART_RS485_DISABLE,
                             .polarity = SCI_UART_RS485_DE_POLARITY_HIGH,
                             #if 0xFF != 0xFF
                    .de_control_pin = BSP_IO_PORT_FF_PIN_0xFF,
                #else
                             .de_control_pin = (bsp_io_port_pin_t)UINT16_MAX,
          #endif
                  },
        };

/** UART interface configuration */
const uart_cfg_t g_uart0_cfg =
        {
          .channel = 0,
          .data_bits = UART_DATA_BITS_8,
          .parity = UART_PARITY_OFF,
          .stop_bits = UART_STOP_BITS_1,
          .p_callback = user_uart_callback,
          .p_context = NULL,
          .p_extend = &g_uart0_cfg_extend,
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == g_transfer2)
                .p_transfer_tx       = NULL,
#else
          .p_transfer_tx = &g_transfer2,
#endif
#if (RA_NOT_DEFINED == g_transfer3)
                .p_transfer_rx       = NULL,
#else
          .p_transfer_rx = &g_transfer3,
#endif
#undef RA_NOT_DEFINED
          .rxi_ipl = (1),
          .txi_ipl = (1),
          .tei_ipl = (1),
          .eri_ipl = (1),
          #if defined(VECTOR_NUMBER_SCI0_RXI)
                .rxi_irq             = VECTOR_NUMBER_SCI0_RXI,
#else
          .rxi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI0_TXI)
                .txi_irq             = VECTOR_NUMBER_SCI0_TXI,
#else
          .txi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI0_TEI)
                .tei_irq             = VECTOR_NUMBER_SCI0_TEI,
#else
          .tei_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI0_ERI)
                .eri_irq             = VECTOR_NUMBER_SCI0_ERI,
#else
          .eri_irq = FSP_INVALID_VECTOR,
        #endif
        };

/* Instance structure to use this module. */
const uart_instance_t g_uart0 =
        {
          .p_ctrl = &g_uart0_ctrl,
          .p_cfg = &g_uart0_cfg,
          .p_api = &g_uart_on_sci
        };
dtc_instance_ctrl_t g_transfer1_ctrl;

transfer_info_t g_transfer1_info =
        {
          .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
          .transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_DESTINATION,
          .transfer_settings_word_b.irq = TRANSFER_IRQ_END,
          .transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED,
          .transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED,
          .transfer_settings_word_b.size = TRANSFER_SIZE_2_BYTE,
          .transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL,
          .p_dest = (void*)NULL,
          .p_src = (void const*)NULL,
          .num_blocks = 0,
          .length = 0,
        };

const dtc_extended_cfg_t g_transfer1_cfg_extend =
        {
          .activation_source = VECTOR_NUMBER_SPI0_RXI,
        };
const transfer_cfg_t g_transfer1_cfg =
        {
          .p_info = &g_transfer1_info,
          .p_extend = &g_transfer1_cfg_extend,
        };

/* Instance structure to use this module. */
const transfer_instance_t g_transfer1 =
        {
          .p_ctrl = &g_transfer1_ctrl,
          .p_cfg = &g_transfer1_cfg,
          .p_api = &g_transfer_on_dtc
        };
dtc_instance_ctrl_t g_transfer0_ctrl;

transfer_info_t g_transfer0_info =
        {
          .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED,
          .transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE,
          .transfer_settings_word_b.irq = TRANSFER_IRQ_END,
          .transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED,
          .transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
          .transfer_settings_word_b.size = TRANSFER_SIZE_2_BYTE,
          .transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL,
          .p_dest = (void*)NULL,
          .p_src = (void const*)NULL,
          .num_blocks = 0,
          .length = 0,
        };

const dtc_extended_cfg_t g_transfer0_cfg_extend =
        {
          .activation_source = VECTOR_NUMBER_SPI0_TXI,
        };
const transfer_cfg_t g_transfer0_cfg =
        {
          .p_info = &g_transfer0_info,
          .p_extend = &g_transfer0_cfg_extend,
        };

/* Instance structure to use this module. */
const transfer_instance_t g_transfer0 =
        {
          .p_ctrl = &g_transfer0_ctrl,
          .p_cfg = &g_transfer0_cfg,
          .p_api = &g_transfer_on_dtc
        };
#define RA_NOT_DEFINED (UINT32_MAX)
#if (RA_NOT_DEFINED) != (RA_NOT_DEFINED)

/* If the transfer module is DMAC, define a DMAC transfer callback. */
#include "r_dmac.h"
extern void spi_tx_dmac_callback(spi_instance_ctrl_t const * const p_ctrl);

void g_spi0_tx_transfer_callback (dmac_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    spi_tx_dmac_callback(&g_spi0_ctrl);
}
#endif

#if (RA_NOT_DEFINED) != (RA_NOT_DEFINED)

/* If the transfer module is DMAC, define a DMAC transfer callback. */
#include "r_dmac.h"
extern void spi_rx_dmac_callback(spi_instance_ctrl_t const * const p_ctrl);

void g_spi0_rx_transfer_callback (dmac_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    spi_rx_dmac_callback(&g_spi0_ctrl);
}
#endif
#undef RA_NOT_DEFINED

spi_instance_ctrl_t g_spi0_ctrl;

/** SPI extended configuration for SPI HAL driver */
const spi_extended_cfg_t g_spi0_ext_cfg =
        {
          .spi_clksyn = SPI_SSL_MODE_CLK_SYN,
          .spi_comm = SPI_COMMUNICATION_FULL_DUPLEX,
          .ssl_polarity = SPI_SSLP_LOW,
          .ssl_select = SPI_SSL_SELECT_SSL0,
          .mosi_idle = SPI_MOSI_IDLE_VALUE_FIXING_HIGH,
          .parity = SPI_PARITY_MODE_DISABLE,
          .byte_swap = SPI_BYTE_SWAP_DISABLE,
          .spck_div = {
                        /* Actual calculated bitrate: 25000000. */.spbr = 1,
                        .brdv = 0
          },
          .spck_delay = SPI_DELAY_COUNT_1,
          .ssl_negation_delay = SPI_DELAY_COUNT_1,
          .next_access_delay = SPI_DELAY_COUNT_1
        };

/** SPI configuration for SPI HAL driver */
const spi_cfg_t g_spi0_cfg =
        {
          .channel = 0,

#if defined(VECTOR_NUMBER_SPI0_RXI)
    .rxi_irq             = VECTOR_NUMBER_SPI0_RXI,
#else
          .rxi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI0_TXI)
    .txi_irq             = VECTOR_NUMBER_SPI0_TXI,
#else
          .txi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI0_TEI)
    .tei_irq             = VECTOR_NUMBER_SPI0_TEI,
#else
          .tei_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI0_ERI)
    .eri_irq             = VECTOR_NUMBER_SPI0_ERI,
#else
          .eri_irq = FSP_INVALID_VECTOR,
#endif

          .rxi_ipl = (4),
          .txi_ipl = (4),
          .tei_ipl = (4),
          .eri_ipl = (4),

          .operating_mode = SPI_MODE_MASTER,

          .clk_phase = SPI_CLK_PHASE_EDGE_EVEN,
          .clk_polarity = SPI_CLK_POLARITY_HIGH,

          .mode_fault = SPI_MODE_FAULT_ERROR_DISABLE,
          .bit_order = SPI_BIT_ORDER_MSB_FIRST,
          .p_transfer_tx = g_spi0_P_TRANSFER_TX,
          .p_transfer_rx = g_spi0_P_TRANSFER_RX,
          .p_callback = user_spi_callback,

          .p_context = NULL,
          .p_extend = (void*)&g_spi0_ext_cfg,
        };

/* Instance structure to use this module. */
const spi_instance_t g_spi0 =
        {
          .p_ctrl = &g_spi0_ctrl,
          .p_cfg = &g_spi0_cfg,
          .p_api = &g_spi_on_spi
        };
void g_hal_init(void)
{
    g_common_init();
}
