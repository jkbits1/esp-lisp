#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "espressif/esp_common.h"
#include "espressif/sdk_private.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ssid_config.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include <esp/uart.h>

#include "lisp.h"

#include "compat.h"

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


