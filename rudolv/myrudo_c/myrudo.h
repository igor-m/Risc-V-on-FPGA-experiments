#ifndef MYRUDO_H
#define MYRUDO_H


#include <stdint.h>

#define read_csr(r) ({ unsigned long t; \
  asm volatile ("csrr %0, %1" : "=r"(t) : "i"(r)); t; })

#define write_csr(r, v) ({ \
  asm volatile ("csrw %0, %1" :: "i"(r), "rK"(v)); })

#define swap_csr(r, v) ({ unsigned long t; \
  asm volatile ("csrrw %0, %1, %2" : "=r"(t) : "i"(r), "rK"(v)); t; })

#define set_csr(r, v) ({ unsigned long t; \
  asm volatile ("csrrs %0, %1, %2" : "=r"(t) : "i"(r), "rK"(v)); t; })

#define clear_csr(r, v) ({ unsigned long t; \
  asm volatile ("csrrc %0, %1, %2" : "=r"(t) : "i"(r), "rK"(v)); t; })


// CSR mapped character interface, RudolV specific

#define UART_CSR 0xbc0
#define UART_RECV_BUF_EMPTY 0x100
#define UART_SEND_BUF_FULL 0x200

static inline unsigned long read_cycle()
{
    return read_csr(0xc00); // cycle
}

int uart_nonblocking_receive()
{
    return (read_csr(UART_CSR) & UART_RECV_BUF_EMPTY) 
        ? -1
        : set_csr(UART_CSR, UART_RECV_BUF_EMPTY);
}

int uart_blocking_receive()
{
    unsigned long ch;
    do {
        ch = set_csr(UART_CSR, UART_RECV_BUF_EMPTY);
    } while (ch & UART_RECV_BUF_EMPTY);
    return ch & 0xff;
}

void uart_send(unsigned ch)
{
    while (swap_csr(UART_CSR, ch) & UART_SEND_BUF_FULL);
}

#endif

// SPDX-License-Identifier: ISC


