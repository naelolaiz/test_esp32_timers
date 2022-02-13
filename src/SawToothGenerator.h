#pragma once
#include "TaskData.h"
#include <driver/dac_common.h>
#include <driver/gpio.h>

class SawToothGenerator {
  const gpio_num_t mGpioNumber;

public:
  SawToothGenerator(gpio_num_t gpioNumber) : mGpioNumber(gpioNumber) {}
};
extern SawToothGenerator sawToothGenerator(GPIO_NUM_25);