# -*- python -*-

# Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("@com_github_ARMmbed_mbed-os//:rules.bzl", "mbed_binary")

mbed_binary(
    name = "gimbal",
    srcs = [
        "ahrs_data.h",
        "as5048_driver.h",
        "as5048_driver.cc",
        "async_i2c.h",
        "async_spi.h",
        "atomic_event_queue.h",
        "bldc_encoder.h",
        "bldc_encoder.cc",
        "bldc_encoder_data.h",
        "bldc_pwm.h",
        "bmi160_driver.h",
        "bmi160_driver.cc",
        "circular_buffer.h",
        "error.h",
        "error.cc",
        "euler.h",
        "fire_control.h",
        "fire_control.cc",
        "gimbal.cc",
        "gimbal_stabilizer.h",
        "gimbal_stabilizer.cc",
        "gpio_pin.h",
        "imu_data.h",
        "irq_callback_table.h",
        "irq_callback_table.cc",
        "mahony_imu.h",
        "mahony_imu.cc",
        "math_util.h",
        "millisecond_timer.h",
        "point3d.h",
        "pwm_pin.h",
        "quaternion.h",
        "static_signal.h",
        "stm32_analog_sampler.h",
        "stm32_analog_sampler.cc",
        "stm32_serial.cc",
        "stm32_serial.h",
        "stm32f446_async_uart.cc",
        "stm32f446_async_uart.h",
    ],
    copts = [
        "-Wno-int-in-bool-context",
    ],
    deps = [
        "@eigen",
        "@moteus//mjlib/base:assert",
        "@moteus//mjlib/base:pid",
        "@moteus//mjlib/base:tokenizer",
        "@moteus//mjlib/base:visitor",
        "@moteus//mjlib/micro:async_stream",
        "@moteus//mjlib/micro:async_types",
        "@moteus//mjlib/micro:persistent_config",
        "@moteus//mjlib/micro:pool_ptr",
        "@moteus//mjlib/micro:static_function",
        "@moteus//mjlib/micro:telemetry_manager",
    ],
)

OCD = (
    "openocd " +
    "-f /usr/share/openocd/scripts/interface/stlink-v2.cfg " +
    "-f /usr/share/openocd/scripts/target/stm32f0x_stlink.cfg "
)

genrule(
    name = "flash",
    tags = ["manual"],
    srcs = ["gimbal.bin"],
    outs = ["flash.stamp"],
    cmd = (OCD + "-c init -c \"reset_config none separate; program $(location gimbal.bin) verify 0x8000000 reset exit 0x8000000\" && touch $@"),
)
