#ifndef _ZYNQ_UART_H_
#define _ZYNQ_UART_H_

enum {
    UART_DATA_8_BIT = 1,
    UART_DATA_7_BIT,
    UART_DATA_6_BIT,
    UART_DATA_5_BIT,
};

enum {
    UART_EVEN_PARITY,
    UART_ODD_PARITY,
    UART_SPACE_PARITY,
    UART_MARK_PARITY,
    UART_NO_PARITY,
};

enum {
    UART_1_STOP_BIT,
    UART_1_5_STOP_BIT,
    UART_2_STOP_BIT,
};

typedef struct UartDataFormat
{
    uint32_t iBaudRate;
	uint32_t iDataBits;	/**< Number of data bits */
	uint32_t iParity;	/**< Parity */
	uint8_t iStopBits;	/**< Number of stop bits */

}UartDataFormat_t;

#define UART_MODE_LOOP (1)

uint32_t uartps_init(int32_t iUartId);

#endif
