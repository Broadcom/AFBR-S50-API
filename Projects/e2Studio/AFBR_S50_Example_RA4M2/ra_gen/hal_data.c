/* generated HAL source file - do not edit */
#include "hal_data.h"

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
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         =  GPT_ADC_TRIGGER_NONE,
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
          /* Actual period: 1 seconds. Actual duty: 50%. */.period_counts = (uint32_t)0x3d09000, .duty_cycle_counts =
                  0x1e84800,
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
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         =  GPT_ADC_TRIGGER_NONE,
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
          /* Actual period: 4000 seconds. Actual duty: 50%. */.period_counts = (uint32_t)0xee6b2800,
          .duty_cycle_counts = 0x77359400, .source_div = (timer_source_div_t)6,
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
icu_instance_ctrl_t g_external_irq0_ctrl;
const external_irq_cfg_t g_external_irq0_cfg =
        {
          .channel = 1,
          .trigger = EXTERNAL_IRQ_TRIG_FALLING,
          .filter_enable = false,
          .pclk_div = EXTERNAL_IRQ_PCLK_DIV_BY_64,
          .p_callback = user_irq_callback,
          /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
          .p_context = &NULL,
#endif
          .p_extend = NULL,
          .ipl = (3),
          #if defined(VECTOR_NUMBER_ICU_IRQ1)
    .irq                 = VECTOR_NUMBER_ICU_IRQ1,
#else
          .irq = FSP_INVALID_VECTOR,
        #endif
        };
/* Instance structure to use this module. */
const external_irq_instance_t g_external_irq0 =
        {
          .p_ctrl = &g_external_irq0_ctrl,
          .p_cfg = &g_external_irq0_cfg,
          .p_api = &g_external_irq_on_icu
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
          /* Baud rate calculated with 2.124% error. */.semr_baudrate_bits_b.abcse = 0,
          .semr_baudrate_bits_b.abcs = 0, .semr_baudrate_bits_b.bgdm = 1, .cks = 0, .brr = 33, .mddr = (uint8_t)256,
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
                        /* Actual calculated bitrate: 10666667. */.spbr = 2,
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
