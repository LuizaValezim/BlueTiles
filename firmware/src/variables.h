
#ifndef VARIABLES_H_
#define VARIABLES_H_

// Botão
#define BUT1_PIO      PIOA
#define BUT1_PIO_ID   ID_PIOA
#define BUT1_IDX      11
#define BUT1_IDX_MASK (1 << BUT1_IDX)

//#define DEBUG_SERIAL


#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

// RTOS
#define TASK_BLUETOOTH_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY        (tskIDLE_PRIORITY)

// Queue
QueueHandle_t xQueueBotoes;

// Struct
typedef struct {
	int id;
	char status;
} Botao;
 
#endif /* VARIABLES_H_ */