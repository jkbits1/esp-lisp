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

static xQueueHandle tsqueue = NULL;
//static xQueueHandle tsqueue02 = NULL;
//static xQueueHandle tsqueue04 = NULL;

// NOTE: need to support multiple buttons -
// does this need multiple fns for more vars?

// flag for count change is set here, and reset when lisp env var is updated
int button00CountChanged = 0;
// add button press count
int button00PressCount = 0;

// flag for count change is set here, and reset when lisp env var is updated
int button02CountChanged = 0;
// add button press count
int button02PressCount = 0;

// flag for count change is set here, and reset when lisp env var is updated
int button04CountChanged = 0;
// add button press count
int button04PressCount = 0;

struct ButtonMessage {           
        uint32_t now;            
        uint32_t buttonNumber;   
};

static checkCount = 0;
static uint32_t last = 0;

void checkInterruptQueue()
{
	uint32_t button_ts;

	struct ButtonMessage btnMsg;

	if (tsqueue == NULL) {
		return;
	}

//printf("x");
	// debugging
//    	 xQueueReceive(*tsqueue, &btnMsg, portMAX_DELAY);

	 if (xQueueReceive(tsqueue, &btnMsg, 0)) {
		 checkCount = 0;

//    		 printf("item in queue");

		button_ts = btnMsg.now;
		button_ts *= portTICK_RATE_MS;
//
//        //if (button04PressCount >= 1) {
////          printf("still waiting - %d", button_ts);
//        //	button04PressCount = 0;
//        //}
//
		if(last < button_ts-200) {
			printf("interrupt %d fired at %dms\r\n", btnMsg.buttonNumber, button_ts);
			last = button_ts;
//
			if (btnMsg.buttonNumber == 0) {
				button00CountChanged = 1;
				button00PressCount = button00PressCount + 1;
			}
			if (btnMsg.buttonNumber == 2) {
				button02CountChanged = 1;
				button02PressCount = button02PressCount + 1;
			}
			if (btnMsg.buttonNumber == 4) {
				button04CountChanged = 1;
				button04PressCount = button04PressCount + 1;
			}
		}
	 }
//	 else {
//		checkCount = checkCount + 1;
//
//		if (checkCount > 1000000) {
//			// printf("q");
//
//			checkCount = 0;
//		}
//	 }
}



//void buttonIntTask(void *pvParameters)
void int04Task(void *pvParameters)
{
//    printf("Waiting for button press interrupt on gpio 4\r\n");
    xQueueHandle *tsqueue = (xQueueHandle *)pvParameters;
    gpio_set_interrupt(0, int_type);
//    gpio_set_interrupt(2, int_type);
//    gpio_set_interrupt(4, int_type);

    uint32_t last = 0;
    while(1) {
        uint32_t button_ts;

    	struct ButtonMessage btnMsg;

//printf("x");
    	// debugging
    	 xQueueReceive(*tsqueue, &btnMsg, portMAX_DELAY);

//        button_ts = btnMsg.now;
//        button_ts *= portTICK_RATE_MS;
//
//        //if (button04PressCount >= 1) {
////          printf("still waiting - %d", button_ts);
//        //	button04PressCount = 0;
//        //}
//
//        if(last < button_ts-200) {
////            printf("interrupt %d fired at %dms\r\n", btnMsg.buttonNumber, button_ts);
//            last = button_ts;
//
//            if (btnMsg.buttonNumber == 0) {
//            	button00CountChanged = 1;
//            	button00PressCount = button00PressCount + 1;
//            }
//            if (btnMsg.buttonNumber == 2) {
//            	button02CountChanged = 1;
//            	button02PressCount = button02PressCount + 1;
//            }
//            if (btnMsg.buttonNumber == 4) {
//            	button04CountChanged = 1;
//            	button04PressCount = button04PressCount + 1;
//            }
//        }
    }
}

void GPIO_HANDLER(void)
{
  uint32_t now = xTaskGetTickCountFromISR();
  xQueueSendToBackFromISR(tsqueue, &now, NULL);
}

// NOTE probably refactor these fns to share code
void GPIO_HANDLER_00(void)
{
//	printf("00 handler");
	uint32_t now = xTaskGetTickCountFromISR();

	struct ButtonMessage btnMsg00;

	btnMsg00.now 			= now;
	btnMsg00.buttonNumber   = 0;

  // debugging
  xQueueSendToBackFromISR(tsqueue, &btnMsg00, NULL);
}

void GPIO_HANDLER_02(void)
{
//printf("02 handler");
  uint32_t now = xTaskGetTickCountFromISR();

	struct ButtonMessage btnMsg02;

	btnMsg02.now 			= now;
	btnMsg02.buttonNumber   = 2;

  //xQueueSendToBackFromISR(tsqueue02, &now, NULL);
  xQueueSendToBackFromISR(tsqueue, &btnMsg02, NULL);
}

void GPIO_HANDLER_04(void)
{
//	printf("04 handler");
	uint32_t now = xTaskGetTickCountFromISR();

 	 struct ButtonMessage btnMsg04;

	btnMsg04.now 			= now;
	btnMsg04.buttonNumber   = 4;

   //xQueueSendToBackFromISR(tsqueue04, &now, NULL);
   xQueueSendToBackFromISR(tsqueue, &btnMsg04, NULL);
}
//
// multiple tasks seem to cause an error
// will multiplex clicks through a single task
//
// NOTE - task needs to run at same or higher priority as lisptask
void interrupt_init(int pin, int changeType)
{
printf("pin %d chgType %d", pin, changeType);

  gpio = pin;

  uart_set_baud(0, 115200);
  gpio_enable(0, GPIO_INPUT);
  gpio_enable(2, GPIO_INPUT);
  gpio_enable(4, GPIO_INPUT);

  gpio_set_interrupt(0, int_type);

  if (tsqueue == NULL ) {
	  tsqueue = xQueueCreate(2,
			  //sizeof(uint32_t)
			  sizeof(struct ButtonMessage)
			  );
  }

  unsigned long priority = (unsigned long) changeType;
printf("p - %lu", priority);

    // debugging
//    xTaskCreate(&int04Task, (signed char *)"int04Task", 256, &tsqueue, priority, NULL);
}

static inline void gpio_set_interrupt2(const uint8_t gpio_num, const gpio_inttype_t int_type)
{
    GPIO.CONF[gpio_num] = SET_FIELD(GPIO.CONF[gpio_num], GPIO_CONF_INTTYPE, int_type);
    if(int_type != GPIO_INTTYPE_NONE) {
        _xt_isr_attach(INUM_GPIO, gpio_interrupt_handler);
        _xt_isr_unmask(1<<INUM_GPIO);
    }
}

