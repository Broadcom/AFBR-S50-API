/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = spi_rxi_isr, /* SPI0 RXI (Receive buffer full) */
            [1] = spi_txi_isr, /* SPI0 TXI (Transmit buffer empty) */
            [2] = spi_tei_isr, /* SPI0 TEI (Transmission complete event) */
            [3] = spi_eri_isr, /* SPI0 ERI (Error) */
            [4] = sci_uart_rxi_isr, /* SCI0 RXI (Receive data full) */
            [5] = sci_uart_txi_isr, /* SCI0 TXI (Transmit data empty) */
            [6] = sci_uart_tei_isr, /* SCI0 TEI (Transmit end) */
            [7] = sci_uart_eri_isr, /* SCI0 ERI (Receive error) */
            [8] = r_icu_isr, /* ICU IRQ1 (External pin interrupt 1) */
            [9] = gpt_counter_overflow_isr, /* GPT1 COUNTER OVERFLOW (Overflow) */
            [10] = can_error_isr, /* CAN0 ERROR (Error interrupt) */
            [11] = can_rx_isr, /* CAN0 MAILBOX RX (Reception complete interrupt) */
            [12] = can_tx_isr, /* CAN0 MAILBOX TX (Transmission complete interrupt) */
            [13] = can_rx_isr, /* CAN0 FIFO RX (Receive FIFO interrupt) */
            [14] = can_tx_isr, /* CAN0 FIFO TX (Transmit FIFO interrupt) */
            [15] = usbfs_interrupt_handler, /* USBFS INT (USBFS interrupt) */
            [16] = usbfs_resume_handler, /* USBFS RESUME (USBFS resume interrupt) */
            [17] = usbfs_d0fifo_handler, /* USBFS FIFO 0 (DMA transfer request 0) */
            [18] = usbfs_d1fifo_handler, /* USBFS FIFO 1 (DMA transfer request 1) */
            [19] = dmac_int_isr, /* DMAC0 INT (DMAC transfer end 0) */
            [20] = dmac_int_isr, /* DMAC1 INT (DMAC transfer end 1) */
        };
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_IELS_ENUM(EVENT_SPI0_RXI), /* SPI0 RXI (Receive buffer full) */
            [1] = BSP_PRV_IELS_ENUM(EVENT_SPI0_TXI), /* SPI0 TXI (Transmit buffer empty) */
            [2] = BSP_PRV_IELS_ENUM(EVENT_SPI0_TEI), /* SPI0 TEI (Transmission complete event) */
            [3] = BSP_PRV_IELS_ENUM(EVENT_SPI0_ERI), /* SPI0 ERI (Error) */
            [4] = BSP_PRV_IELS_ENUM(EVENT_SCI0_RXI), /* SCI0 RXI (Receive data full) */
            [5] = BSP_PRV_IELS_ENUM(EVENT_SCI0_TXI), /* SCI0 TXI (Transmit data empty) */
            [6] = BSP_PRV_IELS_ENUM(EVENT_SCI0_TEI), /* SCI0 TEI (Transmit end) */
            [7] = BSP_PRV_IELS_ENUM(EVENT_SCI0_ERI), /* SCI0 ERI (Receive error) */
            [8] = BSP_PRV_IELS_ENUM(EVENT_ICU_IRQ1), /* ICU IRQ1 (External pin interrupt 1) */
            [9] = BSP_PRV_IELS_ENUM(EVENT_GPT1_COUNTER_OVERFLOW), /* GPT1 COUNTER OVERFLOW (Overflow) */
            [10] = BSP_PRV_IELS_ENUM(EVENT_CAN0_ERROR), /* CAN0 ERROR (Error interrupt) */
            [11] = BSP_PRV_IELS_ENUM(EVENT_CAN0_MAILBOX_RX), /* CAN0 MAILBOX RX (Reception complete interrupt) */
            [12] = BSP_PRV_IELS_ENUM(EVENT_CAN0_MAILBOX_TX), /* CAN0 MAILBOX TX (Transmission complete interrupt) */
            [13] = BSP_PRV_IELS_ENUM(EVENT_CAN0_FIFO_RX), /* CAN0 FIFO RX (Receive FIFO interrupt) */
            [14] = BSP_PRV_IELS_ENUM(EVENT_CAN0_FIFO_TX), /* CAN0 FIFO TX (Transmit FIFO interrupt) */
            [15] = BSP_PRV_IELS_ENUM(EVENT_USBFS_INT), /* USBFS INT (USBFS interrupt) */
            [16] = BSP_PRV_IELS_ENUM(EVENT_USBFS_RESUME), /* USBFS RESUME (USBFS resume interrupt) */
            [17] = BSP_PRV_IELS_ENUM(EVENT_USBFS_FIFO_0), /* USBFS FIFO 0 (DMA transfer request 0) */
            [18] = BSP_PRV_IELS_ENUM(EVENT_USBFS_FIFO_1), /* USBFS FIFO 1 (DMA transfer request 1) */
            [19] = BSP_PRV_IELS_ENUM(EVENT_DMAC0_INT), /* DMAC0 INT (DMAC transfer end 0) */
            [20] = BSP_PRV_IELS_ENUM(EVENT_DMAC1_INT), /* DMAC1 INT (DMAC transfer end 1) */
        };
        #endif
