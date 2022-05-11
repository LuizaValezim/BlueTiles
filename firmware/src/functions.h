
#ifndef INIT_FUNCTIONS_H_
#define INIT_FUNCTIONS_H_

#include <asf.h>
#include "conf_board.h"
#include <string.h>
#include "variables.h"

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

void io_init(void);
static void configure_console(void);
uint32_t usart_puts(uint8_t *pstring);
void usart_put_string(Usart *usart, char str[]);
int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms);
void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout);
void config_usart0(void);
int hc05_init(void);

void but1_callback(void);

#endif /* INIT_FUNCTIONS_H_ */