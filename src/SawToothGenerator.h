#pragma once
#include "TaskData.h"
#include <driver/dac_common.h>
#include <driver/gpio.h>

class SawToothGenerator {
  const gpio_num_t mGpioNumber;

public:
  SawToothGenerator(gpio_num_t gpioNumber);
  void start(size_t frequency) {}
};