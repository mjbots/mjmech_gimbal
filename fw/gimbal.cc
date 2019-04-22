// Copyright 2015-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdlib.h>

#include <cstring>

#include "mjlib/base/visitor.h"

#include "stm32f4xx_hal.h"

#include "mjlib/micro/async_exclusive.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"
#include "mjlib/multiplex/micro_server.h"

#include "fw/as5048_driver.h"
#include "fw/bldc_encoder.h"
#include "fw/bmi160_driver.h"
#include "fw/fire_control.h"
#include "fw/gimbal_stabilizer.h"
#include "fw/mahony_imu.h"
#include "fw/millisecond_timer.h"
#include "fw/stm32_analog_sampler.h"
#include "fw/stm32_bldc_pwm.h"
#include "fw/stm32_flash.h"
#include "fw/stm32_gpio_pin.h"
#include "fw/stm32_hal_spi.h"
#include "fw/stm32_pwm.h"
#include "fw/stm32_raw_i2c.h"
#include "fw/stm32f446_async_uart.h"
#include "fw/system_info.h"

// #include "gimbal_herkulex_operations.h"
// #include "herkulex_protocol.h"

namespace {
struct SystemStatus {
  uint32_t timestamp = 0;
  bool command_manager_init = false;
  bool bmi160_init = false;
  int32_t bmi160_error = 0;
  bool herkulex_init = false;
  int32_t herkulex_error = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(command_manager_init));
    a->Visit(MJ_NVP(bmi160_init));
    a->Visit(MJ_NVP(bmi160_error));
    a->Visit(MJ_NVP(herkulex_init));
    a->Visit(MJ_NVP(herkulex_error));
  }
};

void UpdateLEDs(uint32_t count) {
  int cycle = count / 250;
  if (cycle & 1) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  }
  if (cycle & 2) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
  }
  if (cycle & 4) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  }
}

void InitGpio() {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC14 PC15 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void InitTimers(TIM_HandleTypeDef* htim2,
                TIM_HandleTypeDef* htim3,
                TIM_HandleTypeDef* htim4,
                TIM_HandleTypeDef* htim5) {
  // This is copied from CubeMX auto-generated crud.
  {
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim2->Instance = TIM2;
    htim2->Init.Prescaler = 0;
    htim2->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2->Init.Period = 2048;
    htim2->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(htim2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(htim2, &sClockSourceConfig);

    HAL_TIM_PWM_Init(htim2);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(htim2, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim2, &sConfigOC, TIM_CHANNEL_1);

    HAL_TIM_PWM_ConfigChannel(htim2, &sConfigOC, TIM_CHANNEL_2);

    HAL_TIM_PWM_ConfigChannel(htim2, &sConfigOC, TIM_CHANNEL_3);
  }

  {
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim3->Instance = TIM3;
    htim3->Init.Prescaler = 0;
    htim3->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3->Init.Period = 2048;
    htim3->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(htim3);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(htim3, &sClockSourceConfig);

    HAL_TIM_PWM_Init(htim3);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(htim3, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim3, &sConfigOC, TIM_CHANNEL_1);

    HAL_TIM_PWM_ConfigChannel(htim3, &sConfigOC, TIM_CHANNEL_2);

    HAL_TIM_PWM_ConfigChannel(htim3, &sConfigOC, TIM_CHANNEL_3);

    HAL_TIM_PWM_ConfigChannel(htim3, &sConfigOC, TIM_CHANNEL_4);
  }

  {
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim4->Instance = TIM4;
    htim4->Init.Prescaler = 0;
    htim4->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4->Init.Period = 2048;
    htim4->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(htim4);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(htim4, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim4, &sConfigOC, TIM_CHANNEL_3);
  }

  {
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim5->Instance = TIM5;
    htim5->Init.Prescaler = 4800;
    htim5->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5->Init.Period = 4294967295u;
    htim5->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(htim5);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(htim5, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(htim5, &sMasterConfig);
  }
}
}


using namespace fw;
using namespace mjlib;

int main(void) {
  InitGpio();
  TIM_HandleTypeDef htim2;
  TIM_HandleTypeDef htim3;
  TIM_HandleTypeDef htim4;
  TIM_HandleTypeDef htim5;
  InitTimers(&htim2, &htim3, &htim4, &htim5);

  MillisecondTimer clock;
  micro::SizedPool<12288> pool;

  Stm32F446AsyncUart rs485(&pool, &clock, []() {
      Stm32F446AsyncUart::Options options;
      options.tx = PC_6;
      options.rx = PC_7;
      options.dir = PC_8;
      options.baud_rate = 3000000;
      return options;
    }());

  multiplex::MicroServer multiplex_protocol(&pool, &rs485, {});
  micro::AsyncStream* serial = multiplex_protocol.MakeTunnel(1);

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(serial);
  micro::CommandManager command_manager(&pool, serial, &write_stream);
  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream);
  Stm32Flash flash_interface;
  micro::PersistentConfig persistent_config(pool, command_manager, flash_interface);

  Stm32RawI2C::Parameters parameters;
  parameters.speed = 400000;

  Stm32GpioPin i2c1_sda(GPIOB, GPIO_PIN_7);
  Stm32GpioPin i2c1_scl(GPIOB, GPIO_PIN_6);
  Stm32RawI2C i2c1(pool, 1, i2c1_scl, i2c1_sda, parameters, clock);

  Stm32GpioPin i2c2_sda(GPIOB, GPIO_PIN_9);
  Stm32GpioPin i2c2_scl(GPIOB, GPIO_PIN_10);
  Stm32RawI2C i2c2(pool, 2, i2c2_scl, i2c2_sda, parameters, clock);

  Stm32GpioPin i2c3_sda(GPIOC, GPIO_PIN_9);
  Stm32GpioPin i2c3_scl(GPIOA, GPIO_PIN_8);
  Stm32RawI2C i2c3(pool, 3, i2c3_scl, i2c3_sda, parameters, clock);

  Stm32HalSPI spi1(pool, 3, GPIOC, GPIO_PIN_15);

  SystemInfo system_info(pool, telemetry_manager, clock);

  Stm32AnalogSampler analog_sampler(pool, clock, persistent_config,
                                    telemetry_manager);
  Bmi160Driver bmi160(pool, "pimu",
                      i2c3, clock, persistent_config, telemetry_manager);
  As5048Driver yaw_encoder(pool, "yawenc",
                           nullptr, &spi1, clock,
                           persistent_config, telemetry_manager);
  BldcEncoder yaw_bldc_encoder(pool, "yawblenc",
                               yaw_encoder, clock,
                               persistent_config, telemetry_manager);

  As5048Driver pitch_encoder(pool, "pitchenc",
                             &i2c1, nullptr, clock,
                             persistent_config, telemetry_manager);
  BldcEncoder pitch_bldc_encoder(pool, "pitchblenc",
                                 pitch_encoder, clock,
                                 persistent_config, telemetry_manager);

  Stm32BldcPwm motor1(&htim2, TIM_CHANNEL_1,
                      &htim2, TIM_CHANNEL_2,
                      &htim2, TIM_CHANNEL_3);
  Stm32BldcPwm motor2(&htim3, TIM_CHANNEL_1,
                      &htim3, TIM_CHANNEL_2,
                      &htim4, TIM_CHANNEL_3);
  MahonyImu imu(pool, clock,
                persistent_config, telemetry_manager,
                *bmi160.data_signal());

  Stm32GpioPin bldc_sleep(GPIOA, GPIO_PIN_6, true);
  Stm32GpioPin bldc_reset(GPIOA, GPIO_PIN_7, true);
  bldc_sleep.Set(false);
  bldc_reset.Set(false);

  Stm32GpioPin boost_enable(GPIOC, GPIO_PIN_3);
  Stm32GpioPin motor_enable(GPIOC, GPIO_PIN_13);
  Stm32GpioPin torque_led(GPIOB, GPIO_PIN_14, true);
  GimbalStabilizer stabilizer(pool, clock,
                              persistent_config, telemetry_manager,
                              *imu.data_signal(),
                              boost_enable, motor_enable, motor1, motor2,
                              *yaw_bldc_encoder.data_signal(),
                              *pitch_bldc_encoder.data_signal(),
                              torque_led);


  Stm32GpioPin laser_enable(GPIOA, GPIO_PIN_10);
  Stm32GpioPin pwm_enable(GPIOC, GPIO_PIN_14, true);
  Stm32Pwm aeg_pwm(&htim3, TIM_CHANNEL_3);
  Stm32Pwm agitator_pwm(&htim3, TIM_CHANNEL_4);
  Stm32GpioPin arm_switch(GPIOB, GPIO_PIN_12);
  Stm32GpioPin arm_led(GPIOB, GPIO_PIN_13, true);
  FireControl fire_control(pool, clock,
                           persistent_config, telemetry_manager,
                           laser_enable, pwm_enable, aeg_pwm, agitator_pwm,
                           arm_switch, arm_led);

  command_manager.Register(
      "imu", [&imu](auto&& _1, auto&& _2) { imu.Command(_1, _2); });
  command_manager.Register(
      "gim", [&stabilizer](auto&& _1, auto&& _2) {
        stabilizer.Command(_1, _2);
      });
  command_manager.Register(
      "fire", [&fire_control](auto&& _1, auto&& _2) {
        fire_control.Command(_1, _2);
      });


  persistent_config.Register("id", multiplex_protocol.config(), [](){});
  persistent_config.Load();

  SystemStatus system_status;
  telemetry_manager.Register("system_status", &system_status);

  command_manager.AsyncStart();
  multiplex_protocol.Start(nullptr);

  bmi160.AsyncStart([&](auto error) {
      if (!error) {
        system_status.bmi160_init = true;
        system_status.bmi160_error = error.value();
      }
    });

  auto old_time = clock.read_ms();

  for (;;) {
    rs485.Poll();
    i2c1.Poll();
    i2c2.Poll();
    i2c3.Poll();
    spi1.Poll();
    bmi160.Poll();
    yaw_encoder.Poll();
    pitch_encoder.Poll();
    system_info.MainLoopCount();

    const auto new_time = clock.read_ms();
    if (new_time != old_time) {
      system_info.PollMillisecond();
      analog_sampler.PollMillisecond();
      telemetry_manager.PollMillisecond();
      yaw_bldc_encoder.PollMillisecond();
      pitch_bldc_encoder.PollMillisecond();
      stabilizer.PollMillisecond();
      fire_control.PollMillisecond();
      system_status.timestamp = clock.read_us();

      UpdateLEDs(new_time);

      old_time = new_time;
    }
  }


//   GimbalHerkulexOperations operations(
//       stabilizer, imu, yaw_bldc_encoder, fire_control);
//   HerkulexProtocol herkulex(pool, herkulex_stream, operations);


//   herkulex.AsyncStart([&](int error) {
//       if (!error) {
//         system_status.herkulex_init = true;
//         system_status.herkulex_error = error;
//       }
//     });

//   uint32_t old_tick = 0;
//   while (1) {
//     if (new_tick != old_tick) {
//       HAL_IWDG_Refresh(&hiwdg);

//     }

//   }

  return 0;
}

extern "C" {
  void abort() {
    mbed_die();
  }
}
