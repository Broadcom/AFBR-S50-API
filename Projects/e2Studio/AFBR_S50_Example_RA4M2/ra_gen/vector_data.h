/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#ifdef __cplusplus
        extern "C" {
        #endif
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (22)
#endif
/* ISR prototypes */
void spi_rxi_isr(void);
void spi_txi_isr(void);
void spi_tei_isr(void);
void spi_eri_isr(void);
void sci_uart_rxi_isr(void);
void sci_uart_txi_isr(void);
void sci_uart_tei_isr(void);
void sci_uart_eri_isr(void);
void r_icu_isr(void);
void gpt_counter_overflow_isr(void);
void can_error_isr(void);
void can_rx_isr(void);
void can_tx_isr(void);
void usbfs_interrupt_handler(void);
void usbfs_resume_handler(void);
void usbfs_d0fifo_handler(void);
void usbfs_d1fifo_handler(void);
void fcu_frdyi_isr(void);
void fcu_fiferr_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_SPI0_RXI ((IRQn_Type) 0) /* SPI0 RXI (Receive buffer full) */
#define SPI0_RXI_IRQn          ((IRQn_Type) 0) /* SPI0 RXI (Receive buffer full) */
#define VECTOR_NUMBER_SPI0_TXI ((IRQn_Type) 1) /* SPI0 TXI (Transmit buffer empty) */
#define SPI0_TXI_IRQn          ((IRQn_Type) 1) /* SPI0 TXI (Transmit buffer empty) */
#define VECTOR_NUMBER_SPI0_TEI ((IRQn_Type) 2) /* SPI0 TEI (Transmission complete event) */
#define SPI0_TEI_IRQn          ((IRQn_Type) 2) /* SPI0 TEI (Transmission complete event) */
#define VECTOR_NUMBER_SPI0_ERI ((IRQn_Type) 3) /* SPI0 ERI (Error) */
#define SPI0_ERI_IRQn          ((IRQn_Type) 3) /* SPI0 ERI (Error) */
#define VECTOR_NUMBER_SCI0_RXI ((IRQn_Type) 4) /* SCI0 RXI (Receive data full) */
#define SCI0_RXI_IRQn          ((IRQn_Type) 4) /* SCI0 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI0_TXI ((IRQn_Type) 5) /* SCI0 TXI (Transmit data empty) */
#define SCI0_TXI_IRQn          ((IRQn_Type) 5) /* SCI0 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI0_TEI ((IRQn_Type) 6) /* SCI0 TEI (Transmit end) */
#define SCI0_TEI_IRQn          ((IRQn_Type) 6) /* SCI0 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI0_ERI ((IRQn_Type) 7) /* SCI0 ERI (Receive error) */
#define SCI0_ERI_IRQn          ((IRQn_Type) 7) /* SCI0 ERI (Receive error) */
#define VECTOR_NUMBER_ICU_IRQ1 ((IRQn_Type) 8) /* ICU IRQ1 (External pin interrupt 1) */
#define ICU_IRQ1_IRQn          ((IRQn_Type) 8) /* ICU IRQ1 (External pin interrupt 1) */
#define VECTOR_NUMBER_GPT1_COUNTER_OVERFLOW ((IRQn_Type) 9) /* GPT1 COUNTER OVERFLOW (Overflow) */
#define GPT1_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 9) /* GPT1 COUNTER OVERFLOW (Overflow) */
#define VECTOR_NUMBER_GPT2_COUNTER_OVERFLOW ((IRQn_Type) 10) /* GPT2 COUNTER OVERFLOW (Overflow) */
#define GPT2_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 10) /* GPT2 COUNTER OVERFLOW (Overflow) */
#define VECTOR_NUMBER_CAN0_ERROR ((IRQn_Type) 11) /* CAN0 ERROR (Error interrupt) */
#define CAN0_ERROR_IRQn          ((IRQn_Type) 11) /* CAN0 ERROR (Error interrupt) */
#define VECTOR_NUMBER_CAN0_MAILBOX_RX ((IRQn_Type) 12) /* CAN0 MAILBOX RX (Reception complete interrupt) */
#define CAN0_MAILBOX_RX_IRQn          ((IRQn_Type) 12) /* CAN0 MAILBOX RX (Reception complete interrupt) */
#define VECTOR_NUMBER_CAN0_MAILBOX_TX ((IRQn_Type) 13) /* CAN0 MAILBOX TX (Transmission complete interrupt) */
#define CAN0_MAILBOX_TX_IRQn          ((IRQn_Type) 13) /* CAN0 MAILBOX TX (Transmission complete interrupt) */
#define VECTOR_NUMBER_CAN0_FIFO_RX ((IRQn_Type) 14) /* CAN0 FIFO RX (Receive FIFO interrupt) */
#define CAN0_FIFO_RX_IRQn          ((IRQn_Type) 14) /* CAN0 FIFO RX (Receive FIFO interrupt) */
#define VECTOR_NUMBER_CAN0_FIFO_TX ((IRQn_Type) 15) /* CAN0 FIFO TX (Transmit FIFO interrupt) */
#define CAN0_FIFO_TX_IRQn          ((IRQn_Type) 15) /* CAN0 FIFO TX (Transmit FIFO interrupt) */
#define VECTOR_NUMBER_USBFS_INT ((IRQn_Type) 16) /* USBFS INT (USBFS interrupt) */
#define USBFS_INT_IRQn          ((IRQn_Type) 16) /* USBFS INT (USBFS interrupt) */
#define VECTOR_NUMBER_USBFS_RESUME ((IRQn_Type) 17) /* USBFS RESUME (USBFS resume interrupt) */
#define USBFS_RESUME_IRQn          ((IRQn_Type) 17) /* USBFS RESUME (USBFS resume interrupt) */
#define VECTOR_NUMBER_USBFS_FIFO_0 ((IRQn_Type) 18) /* USBFS FIFO 0 (DMA transfer request 0) */
#define USBFS_FIFO_0_IRQn          ((IRQn_Type) 18) /* USBFS FIFO 0 (DMA transfer request 0) */
#define VECTOR_NUMBER_USBFS_FIFO_1 ((IRQn_Type) 19) /* USBFS FIFO 1 (DMA transfer request 1) */
#define USBFS_FIFO_1_IRQn          ((IRQn_Type) 19) /* USBFS FIFO 1 (DMA transfer request 1) */
#define VECTOR_NUMBER_FCU_FRDYI ((IRQn_Type) 20) /* FCU FRDYI (Flash ready interrupt) */
#define FCU_FRDYI_IRQn          ((IRQn_Type) 20) /* FCU FRDYI (Flash ready interrupt) */
#define VECTOR_NUMBER_FCU_FIFERR ((IRQn_Type) 21) /* FCU FIFERR (Flash access error interrupt) */
#define FCU_FIFERR_IRQn          ((IRQn_Type) 21) /* FCU FIFERR (Flash access error interrupt) */
#ifdef __cplusplus
        }
        #endif
#endif /* VECTOR_DATA_H */
