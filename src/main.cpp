#include <array>
#include <cmath>
#include <driver/gpio.h>
#include <driver/timer.h>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <esp_intr_alloc.h>
#include <esp_task_wdt.h>
#include <soc/soc.h>

struct TaskData {
  struct OutputConfig {
    gpio_num_t mGpioPin;
    bool mCurrentOutputStatus = false;
  };
  OutputConfig mOutputConfigForISR;
  OutputConfig mOutputConfigForTask;
  SemaphoreHandle_t mSemaphore;
  portMUX_TYPE mTimerMux;
  void *mExtraCtx;
};

namespace HighResolutionTimerTest {
class HRTimerTest {
  static TaskData mTaskData;

public:
  static TaskData &getTaskData() { return mTaskData; }

private:
  bool mCurrentLedStatus{true};
  static void periodic_timer_callback(void *arg) {

    auto taskData = static_cast<TaskData *>(arg);

    gpio_set_level(taskData->mOutputConfigForISR.mGpioPin,
                   taskData->mOutputConfigForISR.mCurrentOutputStatus);
    taskData->mOutputConfigForISR.mCurrentOutputStatus ^= 1;

    xSemaphoreGiveFromISR(taskData->mSemaphore, nullptr);
    //  xSemaphoreGive(timerSemaphore);
  }

public:
  HRTimerTest() {
    mTaskData.mSemaphore = xSemaphoreCreateBinary();
    gpio_reset_pin(mTaskData.mOutputConfigForISR.mGpioPin);
    gpio_set_direction(mTaskData.mOutputConfigForISR.mGpioPin,
                       GPIO_MODE_OUTPUT);
    gpio_set_level(mTaskData.mOutputConfigForISR.mGpioPin,
                   mTaskData.mOutputConfigForISR.mCurrentOutputStatus);
    gpio_reset_pin(mTaskData.mOutputConfigForTask.mGpioPin);
    gpio_set_direction(mTaskData.mOutputConfigForTask.mGpioPin,
                       GPIO_MODE_OUTPUT);
    gpio_set_level(mTaskData.mOutputConfigForTask.mGpioPin,
                   mTaskData.mOutputConfigForTask.mCurrentOutputStatus);
  }
  void initPeriodicTimer(size_t us) {
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .arg = &mTaskData,
        /* name is optional, but may help identify the timer when
           debugging */
        .name = "periodic_timer",
        .skip_unhandled_events = false};

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, us));
    /* The timer has been created but is not running yet */
  }
};
TaskData HRTimerTest::mTaskData = {
    .mOutputConfigForISR = {.mGpioPin = GPIO_NUM_19},
    .mOutputConfigForTask = {.mGpioPin = GPIO_NUM_18},
    .mTimerMux = portMUX_INITIALIZER_UNLOCKED};
} // namespace HighResolutionTimerTest
/////////////
namespace GeneralPurposeTimerTest {
typedef struct {
  timer_group_t timer_group;
  timer_idx_t timer_idx;
  size_t alarm_interval;
  timer_autoreload_t auto_reload;
} example_timer_info_t;

static bool IRAM_ATTR timer_group_isr_callback(void *args) {
  BaseType_t high_task_awoken = pdFALSE;
  ///
  TaskData *taskData = static_cast<TaskData *>(args);
  example_timer_info_t *info =
      static_cast<example_timer_info_t *>(taskData->mExtraCtx);

  uint64_t timer_counter_value =
      timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);
  //
  /* Now just send the event data back to the main program task */
  portENTER_CRITICAL_ISR(&taskData->mTimerMux);

  gpio_set_level(taskData->mOutputConfigForISR.mGpioPin,
                 taskData->mOutputConfigForISR.mCurrentOutputStatus);

  taskData->mOutputConfigForISR.mCurrentOutputStatus ^= 1;

  //  Critical code here
  portEXIT_CRITICAL_ISR(&taskData->mTimerMux);
  if (!info->auto_reload) {
    timer_counter_value += info->alarm_interval;
    timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx,
                                       timer_counter_value);
  }
  //  Give a semaphore that we can check in the loop
  //  TIMERG0.hw_timer[0].config.alarm_en = 1;
  xSemaphoreGiveFromISR(taskData->mSemaphore, &high_task_awoken);

  return high_task_awoken ==
         pdTRUE; // return whether we need to yield at the end of ISR
}
class TestGeneralPurposeTimerOldAPI {

  static TaskData mTaskData;

public:
  static constexpr uint8_t TIMER_DIVIDER = 64u;
  static constexpr size_t TIMER_SCALER_S = TIMER_BASE_CLK / TIMER_DIVIDER;
  static constexpr double TIMER_SCALER_US =
      0.000001 * TIMER_BASE_CLK / TIMER_DIVIDER;
  static TaskData &getTaskData() { return mTaskData; }

private:
  void example_tg_timer_init(timer_group_t group, timer_idx_t timer,
                             timer_autoreload_t auto_reload,
                             size_t timer_interval_usec) {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload =
            auto_reload, // ? TIMER_AUTORELOAD_EN : TIMER_AUTORELOAD_DIS,
        .divider = TIMER_DIVIDER}; // default clock source is APB
    timer_init(group, timer, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on
       alarm */
    timer_set_counter_value(group, timer, 0);

    const size_t scaledInterval =
        std::round(timer_interval_usec * TIMER_SCALER_US);
    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(group, timer, scaledInterval);
    timer_enable_intr(group, timer);

    example_timer_info_t *timer_info = static_cast<example_timer_info_t *>(
        calloc(1, sizeof(example_timer_info_t)));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = scaledInterval;
    mTaskData.mExtraCtx = static_cast<void *>(timer_info);
    timer_isr_callback_add(group, timer,
                           GeneralPurposeTimerTest::timer_group_isr_callback,
                           &mTaskData, 0);

    timer_start(group, timer);
  }

public:
  void initGenericTimerOldAPI(size_t timeout) {
    example_tg_timer_init(TIMER_GROUP_0, TIMER_0, TIMER_AUTORELOAD_EN, timeout);
  }
  TestGeneralPurposeTimerOldAPI() {
    mTaskData.mSemaphore = xSemaphoreCreateBinary();
    gpio_reset_pin(mTaskData.mOutputConfigForISR.mGpioPin);
    gpio_set_direction(mTaskData.mOutputConfigForISR.mGpioPin,
                       GPIO_MODE_OUTPUT);
    gpio_set_level(mTaskData.mOutputConfigForISR.mGpioPin,
                   mTaskData.mOutputConfigForISR.mCurrentOutputStatus);
    gpio_reset_pin(mTaskData.mOutputConfigForTask.mGpioPin);
    gpio_set_direction(mTaskData.mOutputConfigForTask.mGpioPin,
                       GPIO_MODE_OUTPUT);
    gpio_set_level(mTaskData.mOutputConfigForTask.mGpioPin,
                   mTaskData.mOutputConfigForTask.mCurrentOutputStatus);
  }
};

TaskData TestGeneralPurposeTimerOldAPI::mTaskData = {
    .mOutputConfigForISR = {.mGpioPin = GPIO_NUM_21},
    .mOutputConfigForTask = {.mGpioPin = GPIO_NUM_22},
    .mTimerMux = portMUX_INITIALIZER_UNLOCKED};
} // namespace GeneralPurposeTimerTest

class Led {
  const std::array<gpio_num_t, 2> mLedGpioNumbers{GPIO_NUM_5};
  const bool mInitialState{false};
  bool mCurrentState{false};

public:
  Led() = default;
  bool getState() const { return mCurrentState; }
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
static void myTask(void *arg) {
  auto *taskData = static_cast<TaskData *>(arg);
  while (1) {
    while (true) {
      if (xSemaphoreTake(taskData->mSemaphore, 0) == pdTRUE) {
        gpio_set_level(taskData->mOutputConfigForTask.mGpioPin,
                       taskData->mOutputConfigForTask.mCurrentOutputStatus);
        taskData->mOutputConfigForTask.mCurrentOutputStatus ^= 1;
      }
      // vTaskDelay(1);
    }
  }
}

extern "C" {
void app_main();
}
static Led led;
void app_main() {
  led.setup();

  constexpr size_t timeInUs = 500;
  /////
  auto *gpTimerTest =
      new GeneralPurposeTimerTest::TestGeneralPurposeTimerOldAPI;
  gpTimerTest->initGenericTimerOldAPI(timeInUs);
  ////
  auto *hrTimerTest = new HighResolutionTimerTest::HRTimerTest;
  hrTimerTest->initPeriodicTimer(timeInUs);

  ////
  xTaskCreate(myTask, "blinking_led_task_gp", 4096, &gpTimerTest->getTaskData(),
              6, nullptr);
  xTaskCreate(myTask, "blinking_led_task_gp", 4096, &hrTimerTest->getTaskData(),
              5, nullptr);

  // static constexpr int taskCore = 0;
  // xTaskCreatePinnedToCore(myTask, /* Function to implement the task */
  //                         "blinking_led_task", /* Name of the task */
  //                         4096,                /* Stack size in words */
  //                         &led,                /* Task input parameter */
  //                         10,                  /* Priority of the task */
  //                         nullptr,             /* Task handle. */
  //                         taskCore); /* Core where the task should run */
}
