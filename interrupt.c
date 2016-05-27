#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "espressif/esp_common.h"
#include "espressif/sdk_private.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

  // for interrupt handling
  //#include "esp8266.h"

#include "ssid_config.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include <esp/uart.h>

// needed for interrupt code in rtos
#include <stdint.h>

#include "lisp.h"

#include "compat.h"


//#ifdef UNIX
//  typedef enum {
//      GPIO_INTTYPE_NONE       = 0,
//      GPIO_INTTYPE_EDGE_POS   = 1,
//      GPIO_INTTYPE_EDGE_NEG   = 2,
//      GPIO_INTTYPE_EDGE_ANY   = 3,
//      GPIO_INTTYPE_LEVEL_LOW  = 4,
//      GPIO_INTTYPE_LEVEL_HIGH = 5,
//  } gpio_inttype_t;

//  typedef void * xQueueHandle;

//typedef unsigned   uint32_t;
//#ifdef ___int8_t_defined
//typedef __int8_t int8_t ;
//typedef __uint8_t uint8_t ;
//#define __int8_t_defined 1
//#endif

//typedef uint32_t portTickType;
//
//  #ifndef configTICK_RATE_HZ
//    #define configTICK_RATE_HZ			( ( portTickType ) 100 )
//  #endif
//
//  #define portMAX_DELAY ( portTickType ) 0xffffffff
//  #define portTICK_RATE_MS			( ( portTickType ) 1000 / configTICK_RATE_HZ )

//#endif

// code from interrupt example
int gpio = 0; // 4;
const int active = 0; // active == 0 for active low
const gpio_inttype_t int_type = GPIO_INTTYPE_EDGE_NEG; // GPIO_INTTYPE_LEVEL_LOW; // GPIO_INTTYPE_EDGE_NEG;
// #define GPIO_HANDLER gpio00_interrupt_handler
//#define GPIO_HANDLER gpio02_interrupt_handler
//#define GPIO_HANDLER gpio04_interrupt_handler

//#define GPIO_HANDLER gpio_interrupt_handler

//#define gpio00_interrupt_handler gpio_interrupt_handler

 #define GPIO_HANDLER_00 gpio00_interrupt_handler
 //#define GPIO_HANDLER_00 gpio02_interrupt_handler
 #define GPIO_HANDLER_04 gpio04_interrupt_handler

typedef void (*pdTASK_CODE)( void * );


//#define portBASE_TYPE           long
//typedef portBASE_TYPE (*pdTASK_HOOK_CODE)( void * );
//#define pdFALSE		( ( portBASE_TYPE ) 0 )

// #define xQueueReceive( xQueue, pvBuffer, xTicksToWait ) xQueueGenericReceive( ( xQueue ), ( pvBuffer ), ( xTicksToWait ), pdFALSE )

// signed portBASE_TYPE xQueueGenericReceive( xQueueHandle xQueue, const void * const pvBuffer, portTickType xTicksToWait, portBASE_TYPE xJustPeeking ) {}

// NOTE: need to support multiple buttons -
// does this need multiple fns for more vars?

// flag for count change is set here, and reset when lisp env var is updated
int buttonCountChanged = 0;
// add button press count
int buttonPressCount = 0;

//void buttonIntTask(void *pvParameters)
void int00Task(void *pvParameters)
{
    printf("Waiting for button press interrupt on gpio %d...\r\n", gpio);
    xQueueHandle *tsqueue = (xQueueHandle *)pvParameters;
    gpio_set_interrupt(gpio, int_type);

    uint32_t last = 0;
    while(1) {
        uint32_t button_ts;
        xQueueReceive(*tsqueue, &button_ts, portMAX_DELAY);
        button_ts *= portTICK_RATE_MS;

        buttonCountChanged = 1;
        buttonPressCount = buttonPressCount + 1;

        //if (buttonPressCount >= 1) {
//          printf("still waiting - %d", button_ts);
        //	buttonPressCount = 0;
        //}

        if(last < button_ts-200) {
            printf("interrupt 0 fired at %dms\r\n", button_ts);
            last = button_ts;
        }
    }
}

void int04Task(void *pvParameters)
{
    printf("Waiting for button press interrupt on gpio %d...\r\n", gpio);
    xQueueHandle *tsqueue = (xQueueHandle *)pvParameters;
    gpio_set_interrupt(gpio, int_type);

    uint32_t last = 0;
    while(1) {
        uint32_t button_ts;
        xQueueReceive(*tsqueue, &button_ts, portMAX_DELAY);
        button_ts *= portTICK_RATE_MS;

        buttonCountChanged = 1;
        buttonPressCount = buttonPressCount + 1;

        //if (buttonPressCount >= 1) {
//          printf("still waiting - %d", button_ts);
        //	buttonPressCount = 0;
        //}

        if(last < button_ts-200) {
            printf("interrupt 4 fired at %dms\r\n", button_ts);
            last = button_ts;
        }
    }
}

static xQueueHandle tsqueue = NULL;
static xQueueHandle tsqueue00 = NULL;
static xQueueHandle tsqueue04 = NULL;

void GPIO_HANDLER(void)
{
  uint32_t now = xTaskGetTickCountFromISR();
  xQueueSendToBackFromISR(tsqueue, &now, NULL);
}

void GPIO_HANDLER_00(void)
{
printf("00 handler");
  uint32_t now = xTaskGetTickCountFromISR();
  xQueueSendToBackFromISR(tsqueue00, &now, NULL);
}

void GPIO_HANDLER_04(void)
{
 uint32_t now = xTaskGetTickCountFromISR();
   xQueueSendToBackFromISR(tsqueue04, &now, NULL);
  // xQueueSendToBackFromISR(tsqueue, &now, NULL);
}

int setUpInterruptTask (pdTASK_CODE intFn, signed char *pFnName, int priority) {
	  xTaskHandle xHandle = NULL;

	printf("creating task \r\n");
	  xTaskCreate(intFn, pFnName, 256, &tsqueue, priority, &xHandle);

	// printf("q - %ld", q);
	 printf("handle - %ld", xHandle);
}

//void user_init(void)
void interrupt_init(int pin, int changeType)
{
  gpio = pin;

  uart_set_baud(0, 115200);
  gpio_enable(gpio, GPIO_INPUT);

  signed char *pFnName = NULL;

  pdTASK_CODE intFn = NULL;

  pdTASK_CODE int00 = int00Task;
  pdTASK_CODE int04 = int04Task;

  int priority = 2;

//  xQueueHandle tsqueue = NULL;

  int retVal = 0;


  if (tsqueue == NULL ) {
	  //tsqueue = xQueueCreate(2, sizeof(uint32_t));
	  tsqueue = xQueueCreate(20, sizeof(uint32_t));
  }

//  if (gpio == 0) {

  printf("setting up for int 0, tsQueue - %ld", tsqueue);

	  pFnName 	= (signed char *)"int00Task";
	  intFn   	= int00;

     priority 	= changeType; // 1;
      tsqueue00	= tsqueue;

      xTaskCreate(int00Task, (signed char *)"int00Task", 256, &tsqueue, priority, &xHandle); //NULL);
      //long q = xTaskCreate(int04Task, (signed char *)"int04Task", 256, &tsqueue, priority, &xHandle);

      retVal = setUpInterruptTask (pdTASK_CODE intFn, signed char *pFnName, int priority);

//  }
//  else {
	  pFnName 	= (signed char *)"int04Task";
	  intFn   	= int04;

      tsqueue04	= tsqueue;

//      xTaskCreate(int04Task, (signed char *)"int04Task", 256, &tsqueue04, 2, NULL);
//        xTaskCreate(intFn, pFnName, 256, &tsqueue04, 2, NULL);
//      xTaskCreate(intFn, pFnName, 256, &tsqueue, priority, NULL);

//      return;
//  }

//printf("creating task \r\n");
//  xTaskCreate(intFn, pFnName, 256, &tsqueue, priority, NULL);

	retVal = setUpInterruptTask (intFn, pFnName, priority);

  // just for a breakpoint
  priority = 2;
}

static inline void gpio_set_interrupt2(const uint8_t gpio_num, const gpio_inttype_t int_type)
{
    GPIO.CONF[gpio_num] = SET_FIELD(GPIO.CONF[gpio_num], GPIO_CONF_INTTYPE, int_type);
    if(int_type != GPIO_INTTYPE_NONE) {
        _xt_isr_attach(INUM_GPIO, gpio_interrupt_handler);
        _xt_isr_unmask(1<<INUM_GPIO);
    }
}

