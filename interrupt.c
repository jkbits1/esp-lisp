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
 #define GPIO_HANDLER_02 gpio02_interrupt_handler
 #define GPIO_HANDLER_04 gpio04_interrupt_handler

typedef void (*pdTASK_CODE)( void * );

// NOTE: need to support multiple buttons -
// does this need multiple fns for more vars?

// flag for count change is set here, and reset when lisp env var is updated
int buttonCountChanged = 0;
// add button press count
int buttonPressCount = 0;

//void buttonIntTask(void *pvParameters)
void int00Task(void *pvParameters)
{
    printf("Waiting for button press interrupt on gpio 0 \r\n");
    xQueueHandle *tsqueue = (xQueueHandle *)pvParameters;
    //gpio_set_interrupt(gpio, int_type);
    //gpio_set_interrupt(0, int_type);
    //gpio_set_interrupt(2, int_type);

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
//            printf("interrupt 0 fired at %dms\r\n", button_ts);
            last = button_ts;
        }
    }
}

void int02Task(void *pvParameters)
{
    printf("Waiting for button press interrupt on gpio 2 \r\n");
    xQueueHandle *tsqueue = (xQueueHandle *)pvParameters;
    //gpio_set_interrupt(gpio, int_type);
    //gpio_set_interrupt(0, int_type);
    //gpio_set_interrupt(2, int_type);

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
  //          printf("interrupt 0 fired at %dms\r\n", button_ts);
            last = button_ts;
        }
    }
}

void int04Task(void *pvParameters)
{
    printf("Waiting for button press interrupt on gpio 4\r\n");
    xQueueHandle *tsqueue = (xQueueHandle *)pvParameters;
  //  gpio_set_interrupt(4, int_type);

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
    //        printf("interrupt 4 fired at %dms\r\n", button_ts);
            last = button_ts;
        }
    }
}

static xQueueHandle tsqueue = NULL;

void GPIO_HANDLER(void)
{
  uint32_t now = xTaskGetTickCountFromISR();
  xQueueSendToBackFromISR(tsqueue, &now, NULL);
}

void GPIO_HANDLER_00(void)
{
printf("00 handler");
  uint32_t now = xTaskGetTickCountFromISR();
  xQueueSendToBackFromISR(tsqueue, &now, NULL);
}

void GPIO_HANDLER_02(void)
{
printf("02 handler");
  uint32_t now = xTaskGetTickCountFromISR();
  xQueueSendToBackFromISR(tsqueue, &now, NULL);
}

void GPIO_HANDLER_04(void)
{
	printf("04 handler");
 uint32_t now = xTaskGetTickCountFromISR();
   xQueueSendToBackFromISR(tsqueue, &now, NULL);
  // xQueueSendToBackFromISR(tsqueue, &now, NULL);
}
//
//int setUpInterruptTask (pdTASK_CODE intFn, signed char *pFnName, int priority) {
//	  xTaskHandle xHandle = NULL;
//
//	printf("creating task %s \r\n", pFnName);
//int retVal =	  xTaskCreate(intFn, pFnName, 256, &tsqueue, priority, &xHandle);
//
//	// printf("q - %ld", q);
//	 printf("handle - %ld", xHandle);
//return retVal;
//}

void interrupt_init(int pin, int changeType)
{
  gpio = pin;

  uart_set_baud(0, 115200);
  //gpio_enable(gpio, GPIO_INPUT);
  gpio_enable(0, GPIO_INPUT);
  gpio_enable(2, GPIO_INPUT);
  gpio_enable(4, GPIO_INPUT);

  signed char *pFnName = NULL;

  int retVal = 0;

  if (tsqueue == NULL ) {
	  tsqueue = xQueueCreate(2, sizeof(uint32_t));
  }

//  xTaskCreate(&int02Task, (signed char *)"int02Task", 256, &tsqueue, 1, NULL);
  xTaskCreate(&int04Task, (signed char *)"int04Task", 256, &tsqueue, 2, NULL);
}

static inline void gpio_set_interrupt2(const uint8_t gpio_num, const gpio_inttype_t int_type)
{
    GPIO.CONF[gpio_num] = SET_FIELD(GPIO.CONF[gpio_num], GPIO_CONF_INTTYPE, int_type);
    if(int_type != GPIO_INTTYPE_NONE) {
        _xt_isr_attach(INUM_GPIO, gpio_interrupt_handler);
        _xt_isr_unmask(1<<INUM_GPIO);
    }
}

