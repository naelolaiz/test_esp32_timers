#include <array>
#include <driver/gpio.h>
#include <driver/timer.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <esp_intr_alloc.h>
#include <soc/soc.h>
// ETS_TIMER1_INTR_SOURCE;
//  ETS_INTERNAL_TIMER0_INTR_SOURCE;
//   https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/intr_alloc.html
//   Internal interrupt sources are defined in esp_intr_alloc.h as
//   ETS_INTERNAL_*_INTR_SOURCE.

// The remaining interrupt sources are from external peripherals. These are
// defined in soc/soc.h as ETS_*_INTR_SOURCE.

// hw_timer_t *timer = NULL;
// timer_config_t timerConfig;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  //  Critical code here
  portEXIT_CRITICAL_ISR(&timerMux);
  //  Give a semaphore that we can check in the loop
  //  TIMERG0.hw_timer[0].config.alarm_en = 1;
  xSemaphoreGiveFromISR(timerSemaphore, nullptr);
}

class Led {
  const std::array<gpio_num_t, 2> mLedGpioNumbers{GPIO_NUM_5, GPIO_NUM_22};
  const bool mInitialState{false};
  bool mCurrentState{false};

public:
  Led() = default;
  void setup() {
    for (auto gpioNumber : mLedGpioNumbers) {
      gpio_reset_pin(gpioNumber);
      gpio_set_direction(gpioNumber, GPIO_MODE_OUTPUT);
    }
  }
  void setValue(bool value) {
    for (auto gpioNumber : mLedGpioNumbers) {
      gpio_set_level(gpioNumber, value);
    }
  }
  void toggle() {
    mCurrentState = !mCurrentState;
    setValue(mCurrentState);
  }
};
static void periodic_timer_callback(void *arg) {
  static_cast<Led *>(arg)->toggle();
  // xSemaphoreGiveFromISR(timerSemaphore, nullptr);
  //  xSemaphoreGive(timerSemaphore);
}

static void myTask(void *arg) {
  auto led = static_cast<Led *>(arg);
  while (1) {
    while (true) {
      led->toggle();
      // vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    /*
// Consume CPU cycles
for (int i = 0; i < SPIN_ITER; i++) {
  __asm__ __volatile__("NOP");
}
printf("delay begin\n");
printf("......\n");
printf("delay task end\n");

vTaskDelay(pdMS_TO_TICKS(10000));
*/
  }
}

extern "C" {
void app_main();
}
#if 0
void setTimer() {

  timer_config_t config = {.alarm_en = TIMER_ALARM_DIS,
                           .counter_en = TIMER_PAUSE,
                           .intr_type = TIMER_INTR_LEVEL,
                           .counter_dir = countDir,
                           .auto_reload = TIMER_AUTORELOAD_DIS,
                           .divider = divider};

  timerSemaphore = xSemaphoreCreateBinary();
  timerConfig.divider = 80; // Set prescaler for 1 MHz clock
  timerConfig.counter_dir = TIMER_COUNT_UP;
  timerConfig.alarm_en = 1;
  timerConfig.intr_type = TIMER_INTR_LEVEL;
  timerConfig.auto_reload =
      TIMER_AUTORELOAD_EN; // Reset timer to 0 when end condition is triggered
  timerConfig.counter_en = TIMER_PAUSE;
  timer_init(TIMER_GROUP_0, 0, &timerConfig);   // start timer 0 at group 0
  timer_set_counter_value(TIMER_GROUP_0, 0, 0); // set timer for 0
  timer_isr_register(TIMER_GROUP_0, 0, &timer_group0_isr, &spkr_pin,
                     ESP_INTR_FLAG_IRAM, NULL);
  timer_set_alarm_value(TIMER_GROUP_0, 0, 5000000);
  timer_enable_intr(TIMER_GROUP_0, 0);

  timer_start(TIMER_GROUP_0, 0);
}
#endif
static Led led;
void app_main() {
  led.setup();
  timerSemaphore = xSemaphoreCreateBinary();
  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &periodic_timer_callback,
      .arg = &led,
      /* name is optional, but may help identify the timer when
         debugging */
      .name = "periodic",
      .skip_unhandled_events = false};

  esp_timer_handle_t periodic_timer;

  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1));
  /* The timer has been created but is not running yet */

  // xTaskCreate(myTask, "blinking_led_task", 4096, &led, 5, NULL);
}
