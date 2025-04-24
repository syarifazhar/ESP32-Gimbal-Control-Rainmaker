/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize the Pan/Tilt motor drivers, IR sensors, and servo (MCPWM).
 *         Configures GPIOs for outputs (motors), inputs (IR), and sets up PWM for servo.
 */
void app_driver_init(void);

/**
 * @brief  Control additional named GPIOs (e.g., RGB LED) by string identifier.
 *
 * @param  name   Name of the GPIO to set (e.g., "Red").
 * @param  state  True = HIGH, False = LOW.
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if unknown name.
 */
esp_err_t app_driver_set_gpio(const char *name, bool state);

#ifdef __cplusplus
}
#endif
