#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************/
/* defines                                                              */
/************************/

// Botoes
#define BUT_1_PIO      PIOB
#define BUT_1_PIO_ID   ID_PIOB
#define BUT_1_IDX      2
#define BUT_1_IDX_MASK (1 << BUT_1_IDX)

#define BUT_2_PIO      PIOD
#define BUT_2_PIO_ID   ID_PIOD
#define BUT_2_IDX      28
#define BUT_2_IDX_MASK (1 << BUT_2_IDX)

#define BUT_3_PIO      PIOA
#define BUT_3_PIO_ID   ID_PIOA
#define BUT_3_IDX      3
#define BUT_3_IDX_MASK (1 << BUT_3_IDX)

#define BUT_4_PIO      PIOD
#define BUT_4_PIO_ID   ID_PIOD
#define BUT_4_IDX      11
#define BUT_4_IDX_MASK (1 << BUT_4_IDX)

#define BUT_5_PIO      PIOC
#define BUT_5_PIO_ID   ID_PIOC
#define BUT_5_IDX      19
#define BUT_5_IDX_MASK (1 << BUT_5_IDX)

#define BUT_6_PIO      PIOB
#define BUT_6_PIO_ID   ID_PIOB
#define BUT_6_IDX      4
#define BUT_6_IDX_MASK (1 << BUT_6_IDX)

#define BUT_7_PIO      PIOB
#define BUT_7_PIO_ID   ID_PIOB
#define BUT_7_IDX      3
#define BUT_7_IDX_MASK (1 << BUT_7_IDX)

#define BUT_8_PIO      PIOC
#define BUT_8_PIO_ID   ID_PIOC
#define BUT_8_IDX      17
#define BUT_8_IDX_MASK (1 << BUT_8_IDX)

#define BUT_9_PIO      PIOA
#define BUT_9_PIO_ID   ID_PIOA
#define BUT_9_IDX      4
#define BUT_9_IDX_MASK (1 << BUT_9_IDX)

#define BUT_10_PIO      PIOC
#define BUT_10_PIO_ID   ID_PIOC
#define BUT_10_IDX      13
#define BUT_10_IDX_MASK (1 << BUT_10_IDX)

#define BUT_11_PIO      PIOD
#define BUT_11_PIO_ID   ID_PIOD
#define BUT_11_IDX      26
#define BUT_11_IDX_MASK (1 << BUT_11_IDX)

#define BUT_12_PIO      PIOA
#define BUT_12_PIO_ID   ID_PIOA
#define BUT_12_IDX      21
#define BUT_12_IDX_MASK (1 << BUT_12_IDX)

#define BUT_13_PIO      PIOD
#define BUT_13_PIO_ID   ID_PIOD
#define BUT_13_IDX      27
#define BUT_13_IDX_MASK (1 << BUT_13_IDX)

#define LED_PIO      PIOB
#define LED_PIO_ID   ID_PIOB
#define LED_IDX      20
#define LED_IDX_MASK (1 << LED_IDX)

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

//#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

/************************/
/* RTOS                                                                 */
/************************/

#define TASK_BLUETOOTH_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_HANDSHAKE_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_HANDSHAKE_STACK_PRIORITY        (tskIDLE_PRIORITY)

QueueHandle_t xQueueBlueTiles;
QueueHandle_t xQueueADC;
QueueHandle_t xQueueHandshake;

/************************/
/* prototypes                                                           */
/************************/

void but_1_callback(void);
void but_2_callback(void);
void but_3_callback(void);
void but_4_callback(void);
void but_5_callback(void);
void but_6_callback(void);
void but_7_callback(void);
void but_8_callback(void);
void but_9_callback(void);
void but_10_callback(void);
void but_11_callback(void);
void but_12_callback(void);
void but_13_callback(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

void task_handshake(void);
void task_bluetooth(void);

static void USART1_init(void);
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback);
static void configure_console(void);

/************************/
/* constants                                                            */
/************************/

typedef struct {
	uint value;
} adcData;

/************************/
/* variaveis globais                                                    */
/************************/

/************************/
/* RTOS application HOOK                                                */
/************************/

/* Called if stack overflow during execution */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/* This function is called by FreeRTOS idle task */
extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/* This function is called by FreeRTOS each tick */
extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
afec_callback_t callback) {
	/*************
	* Ativa e configura AFEC
	*************/
	/* Ativa AFEC - 0 */
	afec_enable(afec);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(afec, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(afec, AFEC_TRIG_SW);

	/* Configuracao específica do canal AFEC */
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	down to 0.
	*/
	afec_channel_set_analog_offset(afec, afec_channel, 0x200);

	/*  Configura sensor de temperatura */
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

	/* configura IRQ */
	afec_set_callback(afec, afec_channel, callback, 1);
	NVIC_SetPriority(afec_id, 4);
	NVIC_EnableIRQ(afec_id);
}

/************************/
/* handlers / callbacks                                                 */
/************************/

void but_1_callback(void) {
	char id;
	
	if (!pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK)){
		id = 'A';
		} else{
		id = 'a';
	}
	xQueueSendFromISR(xQueueBlueTiles, (void *)&id, 0);
	
}

void but_2_callback(void) {
	char id;
	
	if (!pio_get(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK)){
		id = 'B';
		} else{
		id = 'b';
	}
	xQueueSendFromISR(xQueueBlueTiles, (void *)&id, 0);
}

void but_3_callback(void) {
	char id;
	
	if (!pio_get(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK)){
		id = 'C';
		} else{
		id = 'c';
	}
	xQueueSendFromISR(xQueueBlueTiles, (void *)&id, 0);
}

void but_4_callback(void) {
	char id;
	
	if (!pio_get(BUT_4_PIO, PIO_INPUT, BUT_4_IDX_MASK)){
		id = 'D';
		} else{
		id = 'd';
	}
	xQueueSendFromISR(xQueueBlueTiles, (void *)&id, 0);
}

void but_5_callback(void) {
	char id;
	
	if (!pio_get(BUT_5_PIO, PIO_INPUT, BUT_5_IDX_MASK)){
		id = 'E';
		} else{
		id = 'e';
	}
	xQueueSendFromISR(xQueueBlueTiles, (void *)&id, 0);
}

void but_6_callback(void) {
	char id;
	
	if (!pio_get(BUT_6_PIO, PIO_INPUT, BUT_6_IDX_MASK)){
		id = 'F';
		} else{
		id = 'f';
	}
	xQueueSendFromISR(xQueueBlueTiles, (void *)&id, 0);
}

void but_7_callback(void) {
	char id;
	
	if (!pio_get(BUT_7_PIO, PIO_INPUT, BUT_7_IDX_MASK)){
		id = 'G';
		} else{
		id = 'g';
	}
	xQueueSendFromISR(xQueueBlueTiles, (void *)&id, 0);
}

void but_8_callback(void) {
	char id;
	
	if (!pio_get(BUT_8_PIO, PIO_INPUT, BUT_8_IDX_MASK)){
		id = 'I';
		} else{
		id = 'i';
	}
	xQueueSendFromISR(xQueueBlueTiles, (void *)&id, 0);
}

void but_9_callback(void) {
	char id;
	
	if (!pio_get(BUT_9_PIO, PIO_INPUT, BUT_9_IDX_MASK)){
		id = 'J';
		} else{
		id = 'j';
	}
	xQueueSendFromISR(xQueueBlueTiles, (void *)&id, 0);
}

void but_10_callback(void) {
	char id;
	
	if (!pio_get(BUT_10_PIO, PIO_INPUT, BUT_10_IDX_MASK)){
		id = 'K';
		} else{
		id = 'k';
	}
	xQueueSendFromISR(xQueueBlueTiles, (void *)&id, 0);
}

void but_11_callback(void) {
	char id;
	
	if (!pio_get(BUT_11_PIO, PIO_INPUT, BUT_11_IDX_MASK)){
		id = 'L';
		} else{
		id = 'l';
	}
	xQueueSendFromISR(xQueueBlueTiles, (void *)&id, 0);
}

void but_12_callback(void) {
	char id;
	
	if (!pio_get(BUT_12_PIO, PIO_INPUT, BUT_12_IDX_MASK)){
		id = 'M';
		} else{
		id = 'm';
	}
	xQueueSendFromISR(xQueueBlueTiles, (void *)&id, 0);
}

void but_13_callback(void) {
	char id;
	
	if (!pio_get(BUT_13_PIO, PIO_INPUT, BUT_13_IDX_MASK)){
		id = 'H';
		} else{
		id = 'H';
	}
	xQueueSendFromISR(xQueueHandshake, (void *)&id, 0);
}

static void AFEC_pot_callback(void) {
	adcData adc;
	adc.value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueADC, &adc, &xHigherPriorityTaskWoken);
}

void TC1_Handler(void) {
	volatile uint32_t ul_dummy;

	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	afec_start_software_conversion(AFEC_POT);
}

/************************/
/* funcoes                                                              */
/************************/

void io_init(void) {

	pmc_enable_periph_clk(BUT_1_PIO_ID);
	pmc_enable_periph_clk(BUT_2_PIO_ID);
	pmc_enable_periph_clk(BUT_3_PIO_ID);
	pmc_enable_periph_clk(BUT_4_PIO_ID);
	pmc_enable_periph_clk(BUT_5_PIO_ID);
	pmc_enable_periph_clk(BUT_6_PIO_ID);
	pmc_enable_periph_clk(BUT_7_PIO_ID);
	pmc_enable_periph_clk(BUT_8_PIO_ID);
	pmc_enable_periph_clk(BUT_9_PIO_ID);
	pmc_enable_periph_clk(BUT_10_PIO_ID);
	pmc_enable_periph_clk(BUT_11_PIO_ID);
	pmc_enable_periph_clk(BUT_12_PIO_ID);
	pmc_enable_periph_clk(BUT_13_PIO_ID);
	pmc_enable_periph_clk(LED_PIO_ID);
	
	pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_4_PIO, PIO_INPUT, BUT_4_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_5_PIO, PIO_INPUT, BUT_5_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_6_PIO, PIO_INPUT, BUT_6_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_7_PIO, PIO_INPUT, BUT_7_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_8_PIO, PIO_INPUT, BUT_8_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_9_PIO, PIO_INPUT, BUT_9_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_10_PIO, PIO_INPUT, BUT_10_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_11_PIO, PIO_INPUT, BUT_11_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_12_PIO, PIO_INPUT, BUT_12_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_13_PIO, PIO_INPUT, BUT_13_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(LED_PIO, PIO_OUTPUT_1, LED_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	
	pio_set_debounce_filter(BUT_1_PIO, BUT_1_IDX_MASK, 60);
	pio_set_debounce_filter(BUT_2_PIO, BUT_2_IDX_MASK, 60);
	pio_set_debounce_filter(BUT_3_PIO, BUT_3_IDX_MASK, 60);
	pio_set_debounce_filter(BUT_4_PIO, BUT_4_IDX_MASK, 60);
	pio_set_debounce_filter(BUT_5_PIO, BUT_5_IDX_MASK, 60);
	pio_set_debounce_filter(BUT_6_PIO, BUT_6_IDX_MASK, 60);
	pio_set_debounce_filter(BUT_7_PIO, BUT_7_IDX_MASK, 60);
	pio_set_debounce_filter(BUT_8_PIO, BUT_8_IDX_MASK, 60);
	pio_set_debounce_filter(BUT_9_PIO, BUT_9_IDX_MASK, 60);
	pio_set_debounce_filter(BUT_10_PIO, BUT_10_IDX_MASK, 60);
	pio_set_debounce_filter(BUT_11_PIO, BUT_11_IDX_MASK, 60);
	pio_set_debounce_filter(BUT_12_PIO, BUT_12_IDX_MASK, 60);
	pio_set_debounce_filter(BUT_13_PIO, BUT_13_IDX_MASK, 60);

	pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_EDGE, but_1_callback);
	pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_EDGE, but_2_callback);
	pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_EDGE, but_3_callback);
	pio_handler_set(BUT_4_PIO, BUT_4_PIO_ID, BUT_4_IDX_MASK, PIO_IT_EDGE, but_4_callback);
	pio_handler_set(BUT_5_PIO, BUT_5_PIO_ID, BUT_5_IDX_MASK, PIO_IT_EDGE, but_5_callback);
	pio_handler_set(BUT_6_PIO, BUT_6_PIO_ID, BUT_6_IDX_MASK, PIO_IT_EDGE, but_6_callback);
	pio_handler_set(BUT_7_PIO, BUT_7_PIO_ID, BUT_7_IDX_MASK, PIO_IT_EDGE, but_7_callback);
	pio_handler_set(BUT_8_PIO, BUT_8_PIO_ID, BUT_8_IDX_MASK, PIO_IT_EDGE, but_8_callback);
	pio_handler_set(BUT_9_PIO, BUT_9_PIO_ID, BUT_9_IDX_MASK, PIO_IT_EDGE, but_9_callback);
	pio_handler_set(BUT_10_PIO, BUT_10_PIO_ID, BUT_10_IDX_MASK, PIO_IT_EDGE, but_10_callback);
	pio_handler_set(BUT_11_PIO, BUT_11_PIO_ID, BUT_11_IDX_MASK, PIO_IT_EDGE, but_11_callback);
	pio_handler_set(BUT_12_PIO, BUT_12_PIO_ID, BUT_12_IDX_MASK, PIO_IT_EDGE, but_12_callback);
	pio_handler_set(BUT_13_PIO, BUT_13_PIO_ID, BUT_13_IDX_MASK, PIO_IT_EDGE, but_13_callback);
	
	pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
	pio_get_interrupt_status(BUT_1_PIO);
	
	pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
	pio_get_interrupt_status(BUT_2_PIO);
	
	pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);
	pio_get_interrupt_status(BUT_3_PIO);
	
	pio_enable_interrupt(BUT_4_PIO, BUT_4_IDX_MASK);
	pio_get_interrupt_status(BUT_4_PIO);

	pio_enable_interrupt(BUT_5_PIO, BUT_5_IDX_MASK);
	pio_get_interrupt_status(BUT_5_PIO);

    pio_enable_interrupt(BUT_6_PIO, BUT_6_IDX_MASK);
	pio_get_interrupt_status(BUT_6_PIO);

    pio_enable_interrupt(BUT_7_PIO, BUT_7_IDX_MASK);
	pio_get_interrupt_status(BUT_7_PIO);

    pio_enable_interrupt(BUT_8_PIO, BUT_8_IDX_MASK);
	pio_get_interrupt_status(BUT_8_PIO);

    pio_enable_interrupt(BUT_9_PIO, BUT_9_IDX_MASK);
	pio_get_interrupt_status(BUT_9_PIO);

    pio_enable_interrupt(BUT_10_PIO, BUT_10_IDX_MASK);
	pio_get_interrupt_status(BUT_10_PIO);

    pio_enable_interrupt(BUT_11_PIO, BUT_11_IDX_MASK);
	pio_get_interrupt_status(BUT_11_PIO);
    
    pio_enable_interrupt(BUT_12_PIO, BUT_12_IDX_MASK);
	pio_get_interrupt_status(BUT_12_PIO);

    pio_enable_interrupt(BUT_13_PIO, BUT_13_IDX_MASK);
	pio_get_interrupt_status(BUT_13_PIO);
		
	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 4);
	
	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 4);
	
	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 4);
	
	NVIC_EnableIRQ(BUT_4_PIO_ID);
	NVIC_SetPriority(BUT_4_PIO_ID, 4);
	
	NVIC_EnableIRQ(BUT_5_PIO_ID);
	NVIC_SetPriority(BUT_5_PIO_ID, 4);

    NVIC_EnableIRQ(BUT_6_PIO_ID);
	NVIC_SetPriority(BUT_6_PIO_ID, 4);

    NVIC_EnableIRQ(BUT_7_PIO_ID);
	NVIC_SetPriority(BUT_7_PIO_ID, 4);

    NVIC_EnableIRQ(BUT_8_PIO_ID);
	NVIC_SetPriority(BUT_8_PIO_ID, 4);

    NVIC_EnableIRQ(BUT_9_PIO_ID);
	NVIC_SetPriority(BUT_9_PIO_ID, 4);

    NVIC_EnableIRQ(BUT_10_PIO_ID);
	NVIC_SetPriority(BUT_10_PIO_ID, 4);

    NVIC_EnableIRQ(BUT_11_PIO_ID);
	NVIC_SetPriority(BUT_11_PIO_ID, 4);

    NVIC_EnableIRQ(BUT_12_PIO_ID);
	NVIC_SetPriority(BUT_12_PIO_ID, 4);

    NVIC_EnableIRQ(BUT_13_PIO_ID);
	NVIC_SetPriority(BUT_13_PIO_ID, 4);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	#if defined(_GNUC_)
	setbuf(stdout, NULL);
	#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	* emits one character at a time.
	*/
	#endif
}

uint32_t usart_puts(uint8_t *pstring) {
	uint32_t i ;

	while(*(pstring + i))
	if(uart_is_tx_empty(USART_COM))
	usart_serial_putchar(USART_COM, *(pstring+i++));
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
	uint timecounter = timeout_ms;
	uint32_t rx;
	uint32_t counter = 0;

	while( (timecounter > 0) && (counter < bufferlen - 1)) {
		if(usart_read(usart, &rx) == 0) {
			buffer[counter++] = rx;
		}
		else{
			timecounter--;
			vTaskDelay(1);
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen,
char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void config_usart0(void) {
	sysclk_enable_peripheral_clock(ID_USART0);
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);

	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_init(void) {
	char buffer_rx[128];
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEBlueTiles", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+PIN0000", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
}

void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq) {
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	pmc_enable_periph_clk(ID_TC);

	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	NVIC_SetPriority((IRQn_Type)ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type)ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

/************************/
/* TASKS                                                                */
/************************/

void task_handshake(void) {
	
	config_usart0();
	hc05_init();
	io_init();

	char id;
	char botao = '0';
	char eof = 'X';
	char handshake;
	
	for (;;) {
		
		if (xQueueReceive(xQueueHandshake, &id, 0) == pdTRUE) {
			while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
			usart_write(USART_COM, 'H');
			botao = id;
			while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
			usart_write(USART_COM, botao);

			while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
			usart_write(USART_COM, eof);
			
		}
		
 		if(usart_is_rx_ready(USART_COM)){
 			usart_read(USART_COM, &handshake);
 			if(handshake == 'h'){
 				pio_set_output(LED_PIO, LED_IDX_MASK, 1, 0, 0);
 				break;
 			}
 		}	
		
	}
	
	xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, NULL);
	
	while(1){
		vTaskDelay(1000000);
	}
}

void task_bluetooth(void) {
	config_usart0();
	hc05_init();
	io_init();
	
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_callback);
	TC_init(TC0, ID_TC1, 1, 10);
	tc_start(TC0, 1);

	char eof = 'X';
	char id;
	char botao = '0';
	
	adcData adc;
	int old_adc = 0;
	
	for (;;) {
		
		if (xQueueReceive(xQueueBlueTiles, &id, 0) == pdTRUE) {
			while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
			usart_write(USART_COM, 'B');
			
			botao = id;
			while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
			usart_write(USART_COM, botao);

			while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
			usart_write(USART_COM, eof);
		}
		
		if (xQueueReceive(xQueueADC, &(adc), 0)) {
			if ((adc.value >> 8)!= old_adc) {
				while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
				usart_write(USART_COM, 'V');
				
				while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
				usart_write(USART_COM, (adc.value >> 8));
				
				while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
				usart_write(USART_COM, eof);
				
				old_adc = adc.value >> 8;
			}
		}
		
	}
}

/************************/
/* main                                                                 */
/************************/

int main(void) {
	sysclk_init();
	board_init();

	configure_console();
	
	xQueueBlueTiles = xQueueCreate(32, sizeof(uint32_t));
	xQueueADC = xQueueCreate(100, sizeof(adcData));
	xQueueHandshake = xQueueCreate(100, sizeof(adcData));

	xTaskCreate(task_handshake, "HSK", TASK_HANDSHAKE_STACK_SIZE, NULL, TASK_HANDSHAKE_STACK_PRIORITY, NULL);
	
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
