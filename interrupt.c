#include <string.h>

#ifndef UNIX
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
#endif

#include "lisp.h"

#include "compat.h"

#ifdef UNIX
  typedef enum {
      GPIO_INTTYPE_NONE       = 0,
      GPIO_INTTYPE_EDGE_POS   = 1,
      GPIO_INTTYPE_EDGE_NEG   = 2,
      GPIO_INTTYPE_EDGE_ANY   = 3,
      GPIO_INTTYPE_LEVEL_LOW  = 4,
      GPIO_INTTYPE_LEVEL_HIGH = 5,
  } gpio_inttype_t;

  typedef void * xQueueHandle;

  typedef unsigned   uint32_t;
#ifdef ___int8_t_defined
typedef __int8_t int8_t ;
typedef __uint8_t uint8_t ;
#define __int8_t_defined 1
#endif

  typedef uint32_t portTickType;

  #ifndef configTICK_RATE_HZ
    #define configTICK_RATE_HZ			( ( portTickType ) 100 )
  #endif

  #define portMAX_DELAY ( portTickType ) 0xffffffff
  #define portTICK_RATE_MS			( ( portTickType ) 1000 / configTICK_RATE_HZ )

#endif

// code from interrupt example
const int gpio = 4;
const int active = 0; // active == 0 for active low
const gpio_inttype_t int_type = GPIO_INTTYPE_EDGE_NEG;

void buttonIntTask(void *pvParameters)
{
    printf("Waiting for button press interrupt on gpio %d...\r\n", gpio);
    xQueueHandle *tsqueue = (xQueueHandle *)pvParameters;
    gpio_set_interrupt(gpio, int_type);

    uint32_t last = 0;
    while(1) {
        uint32_t button_ts;
        xQueueReceive(*tsqueue, &button_ts, portMAX_DELAY);
        button_ts *= portTICK_RATE_MS;
        if(last < button_ts-200) {
            printf("Button interrupt fired at %dms\r\n", button_ts);
            last = button_ts;
        }
    }
}

static xQueueHandle tsqueue;

//void user_init(void)
void interrupt_init(void)
{
  tsqueue = xQueueCreate(1, sizeof(uint32_t));
  xTaskCreate(buttonIntTask, (signed char *)"buttonIntTask", 256, &tsqueue, 2, NULL);
}


