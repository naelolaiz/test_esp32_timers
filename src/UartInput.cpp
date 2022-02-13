
/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#include "UartInput.h"

static QueueHandle_t uart0_queue;
UartInput myUart = UartInput();

UartInput::UartInput() {}

UartInput::~UartInput() {}

void UartInput::uart_event_task(void *pvParameters) {
  uart_event_t event;
  size_t buffered_size;
  uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
  for (;;) {
    // Waiting for UART event.
    if (xQueueReceive(uart0_queue, (void *)&event,
                      (portTickType)portMAX_DELAY)) {
      bzero(dtmp, RD_BUF_SIZE);
      ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
      switch (event.type) {
      // Event of UART receving data
      /*We'd better handler data event fast, there would be much more data
      events than other types of events. If we take too much time on data event,
      the queue might be full.*/
      case UART_DATA:
        ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
        uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
        ESP_LOGI(TAG, "[DATA EVT]:");
        uart_write_bytes(EX_UART_NUM, (const char *)dtmp, event.size);
        break;
      // Event of HW FIFO overflow detected
      case UART_FIFO_OVF:
        ESP_LOGI(TAG, "hw fifo overflow");
        // If fifo overflow happened, you should consider adding flow control
        // for your application. The ISR has already reset the rx FIFO, As an
        // example, we directly flush the rx buffer here in order to read more
        // data.
        uart_flush(EX_UART_NUM);
        xQueueReset(uart0_queue);
        break;
      // Event of UART ring buffer full
      case UART_BUFFER_FULL:
        ESP_LOGI(TAG, "ring buffer full");
        // If buffer full happened, you should consider encreasing your buffer
        // size As an example, we directly flush the rx buffer here in order to
        // read more data.
        uart_flush(EX_UART_NUM);
        xQueueReset(uart0_queue);
        break;
      // Event of UART RX break detected
      case UART_BREAK:
        ESP_LOGI(TAG, "uart rx break");
        break;
      // Event of UART parity check error
      case UART_PARITY_ERR:
        ESP_LOGI(TAG, "uart parity error");
        break;
      // Event of UART frame error
      case UART_FRAME_ERR:
        ESP_LOGI(TAG, "uart frame error");
        break;
      // UART_PATTERN_DET
      case UART_PATTERN_DET:
        ESP_LOGI(TAG, "UART PATTERN DETECTED\n");
        break;
      // Others
      default:
        ESP_LOGI(TAG, "uart event type: %d", event.type);
        break;
      }
    }
  }
  free(dtmp);
  dtmp = NULL;
  vTaskDelete(NULL);
}

void UartInput::init() {
  esp_log_level_set(TAG, ESP_LOG_INFO);

  /* Configure parameters of an UART driver,
   * communication pins and install the driver */
  uart_config_t uart_config = {.baud_rate = 115200,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  uart_param_config(EX_UART_NUM, &uart_config);

  // Set UART log level
  esp_log_level_set(TAG, ESP_LOG_INFO);
  // Set UART pins (using UART0 default pins ie no changes.)
  uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  // Install UART driver, and get the queue.
  // uart0_queue = xQueueCreate(5, sizeof(uint8_t)); //initialize the queue

  uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue,
                      0);

  // Set uart pattern detect function.
  uart_enable_pattern_det_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 10000, 10,
                               10);
  // Reset the pattern queue length to record at most 20 pattern positions.
  // uart_pattern_queue_reset(EX_UART_NUM, 20);
  // Create a task to handler UART event from ISR
  xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}