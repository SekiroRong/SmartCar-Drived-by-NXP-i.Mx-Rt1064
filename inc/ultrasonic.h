#ifndef ultrasonic_h
#define ultrasonic_h

#include "common.h"
#include "fsl_lpuart.h"
#include "fsl_common.h"

extern void example_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);

#endif