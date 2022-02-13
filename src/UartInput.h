#pragma once

/* UART Events Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <freertos/ringbuf.h>
#include <stdio.h>
#include <string.h>

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM                                                        \
  (3) /*!< Set the number of consecutive and identical characters received by  \
         receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

class UartInput {
  RingbufHandle_t mRingBufferHandle;
  QueueHandle_t mQueueHandleForDelayInUs;

public:
  UartInput();
  ~UartInput();

private:
  static constexpr char const *TAG = "myUart";
  static void uart_event_task(void *pvParameters);
  void initQueueAndRingBuffer() {
    mRingBufferHandle = xRingbufferCreate(1028, RINGBUF_TYPE_NOSPLIT);
    mQueueHandleForDelayInUs = xQueueCreateSet(4);
    if (xRingbufferAddToQueueSetRead(mRingBufferHandle,
                                     mQueueHandleForDelayInUs) != pdTRUE) {
      ESP_LOGI(TAG, "ring buffer creation failed");
    }

#if 0
    //send
    xRingbufferSend(buf_handle, tx_item, sizeof(tx_item), pdMS_TO_TICKS(1000));
    if (res != pdTRUE) {
        printf("Failed to send item\n");
    }

#endif
  }

public:
  void init();
};

extern UartInput myUart;