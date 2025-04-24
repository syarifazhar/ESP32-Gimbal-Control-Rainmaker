/*
 * app_driver.c
 * ESP-IDF driver for Pan/Tilt motors, IR sensors, and servo
 */
 #include <sdkconfig.h>
 #include <string.h>
 #include <esp_err.h>
 #include <esp_log.h>
 #include <driver/gpio.h>
 #include <driver/mcpwm.h>
 
 #include <app_reset.h>
 //#include "app_driver.h"
 
 #define TAG "app_driver"
 
 // Reset button config
 #define WIFI_RESET_BUTTON_TIMEOUT    3
 #define FACTORY_RESET_BUTTON_TIMEOUT 10
 #define BUTTON_GPIO                   CONFIG_EXAMPLE_BOARD_BUTTON_GPIO
 #define BUTTON_ACTIVE_LEVEL          0
 
 // Motor GPIOs (set via menuconfig)
 #define PAN_M1A_PIN   CONFIG_PAN_M1A_GPIO
 #define PAN_M1B_PIN   CONFIG_PAN_M1B_GPIO
 #define TILT_M1A_PIN  CONFIG_TILT_M1A_GPIO
 #define TILT_M1B_PIN  CONFIG_TILT_M1B_GPIO
 
 // IR sensor GPIOs
 #define IR_LEFT_PIN   CONFIG_IR_LEFT_GPIO
 #define IR_RIGHT_PIN  CONFIG_IR_RIGHT_GPIO
 
 // Servo GPIO (MCPWM)
 #define SERVO_PIN     CONFIG_SERVO_GPIO
 
 // Initialize MCPWM for servo control
 static void servo_init(gpio_num_t gpio_num)
 {
     mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, gpio_num);
     mcpwm_config_t pwm_cfg = {
         .frequency = 50,
         .cmpr_a = 0,
         .cmpr_b = 0,
         .duty_mode = MCPWM_DUTY_MODE_0,
         .counter_mode = MCPWM_UP_COUNTER,
     };
     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_cfg);
     ESP_LOGI(TAG, "Servo MCPWM initialized at GPIO %d", gpio_num);
 }
 
 // Convert angle [0..180] to pulse width [500..2500]us
 static void servo_set_angle(int angle)
 {
     uint32_t pulse = (angle * (2500 - 500) / 180) + 500;
     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulse);
     ESP_LOGI(TAG, "Servo angle set to %d°, pulse %dus", (int)angle, (unsigned)pulse);
 }
 
 void app_driver_init()
 {
     // Register Wi-Fi and factory reset button
     app_reset_button_register(
         app_reset_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL),
         WIFI_RESET_BUTTON_TIMEOUT,
         FACTORY_RESET_BUTTON_TIMEOUT);
 
     // Configure motor GPIOs as outputs
     gpio_config_t io_conf = {
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = GPIO_PULLUP_ENABLE,
     };
     io_conf.pin_bit_mask = (1ULL<<PAN_M1A_PIN) | (1ULL<<PAN_M1B_PIN)
                          | (1ULL<<TILT_M1A_PIN)| (1ULL<<TILT_M1B_PIN);
     gpio_config(&io_conf);
     gpio_set_level(PAN_M1A_PIN, 0);
     gpio_set_level(PAN_M1B_PIN, 0);
     gpio_set_level(TILT_M1A_PIN, 0);
     gpio_set_level(TILT_M1B_PIN, 0);
 
     // Configure IR sensor GPIOs as inputs
     io_conf.mode = GPIO_MODE_INPUT;
     io_conf.pin_bit_mask = (1ULL<<IR_LEFT_PIN) | (1ULL<<IR_RIGHT_PIN);
     gpio_config(&io_conf);
 
     // Initialize servo control
     servo_init((gpio_num_t)SERVO_PIN);
     servo_set_angle(180); // default on boot
 
     ESP_LOGI(TAG, "Driver initialized: motors, IR sensors, servo");
 }
 
 esp_err_t app_driver_set_gpio(const char *name, bool state)
 {
     // Example placeholder if you need to handle additional named GPIOs
     if (strcmp(name, "Red") == 0) {
         gpio_set_level(CONFIG_EXAMPLE_OUTPUT_GPIO_RED, state);
         return ESP_OK;
     }
     ESP_LOGW(TAG, "Unknown GPIO name '%s'", name);
     return ESP_ERR_NOT_SUPPORTED;
 }
 