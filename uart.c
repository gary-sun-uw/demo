#include "uart.h"
#include <stdio.h>

volatile static const unsigned char* print_buf;
volatile static size_t print_buf_size;
volatile static size_t print_buf_idx;

#define PRINTF_BUF_SIZE 64
//static char printf_buf[PRINTF_BUF_SIZE];

void uart_printf(char* format, ...)
{
    va_list args;
    va_start(args, format);
    char printf_buf[PRINTF_BUF_SIZE];
    int count = vsnprintf(printf_buf, PRINTF_BUF_SIZE, format, args);
    if (count > 0) {
        uart_write(0, printf_buf, count);
    }
    va_end(args);
}

size_t uart_write(int handle, const unsigned char *buf, size_t bufSize) {
    // while (print_buf_idx < print_buf_size);
    if (buf == NULL) {
        return 0;
    }

    print_buf_size = bufSize;
    print_buf_idx = 0;
    print_buf = buf;
    EUSCI_A_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);

    // Enter LPM0, with interrupts enabled, and wait for transmit interrupt
    __bis_SR_register(LPM0_bits + GIE);

    EUSCI_A_UART_disableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);

    return bufSize;
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A1_VECTOR)))
#endif
void EUSCI_A1_ISR(void)
{
  switch(__even_in_range(UCA1IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG: break;
    case USCI_UART_UCTXIFG:
        EUSCI_A_UART_transmitData(EUSCI_A1_BASE,
                           print_buf[print_buf_idx]);
        print_buf_idx += 1;
        if (print_buf_idx == print_buf_size) {
            __bic_SR_register_on_exit(LPM0_bits); // leave low power mode
        }
        break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
  }
}