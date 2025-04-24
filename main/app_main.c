/*
 * main.c
 * ESP-IDF PanTilt RainMaker + ESP-NOW with legacy MCPWM API
 */
 #include <stdio.h>
 #include <string.h>            // for strcmp()
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_system.h"
 #include "esp_log.h"
 #include "nvs_flash.h"
 #include "esp_err.h"
 #include "driver/gpio.h"
 #include "driver/mcpwm.h"      // legacy driver
 #include "esp_wifi.h"
 #include "esp_event.h"
 
 /* RainMaker & provisioning */
 #include "esp_rmaker_core.h"
 #include "esp_rmaker_standard_types.h"
 #include "app_network.h"
 #include "app_insights.h"
 #include "app_priv.h"
 
 /* ESP-NOW */
 #include "esp_now.h"
 
 static const char *TAG = "Camera Control";
 
 // Motor GPIOs
 #define PAN_M1A    GPIO_NUM_12
 #define PAN_M1B    GPIO_NUM_13
 #define TILT_M1A   GPIO_NUM_10
 #define TILT_M1B   GPIO_NUM_11
 // Servo GPIO
 #define SERVO_PIN  GPIO_NUM_8
 // IR sensor GPIOs
 #define IR_LEFT    GPIO_NUM_1
 #define IR_RIGHT   GPIO_NUM_0
 
 // Global state
 static bool autoTracking = false;
 static bool useCloud     = true;               // Master switch: true=RainMaker, false=ESP-NOW
 static const int centerValue = 2048;
 static esp_rmaker_param_t *pan_param;
 static esp_rmaker_param_t *tilt_param;
 static esp_rmaker_param_t *master_param;
 
 // Ramp control ticks
 static TickType_t lastPanTime = 0;
 static TickType_t lastTiltTime = 0;
 static const TickType_t rampTicks = pdMS_TO_TICKS(30);
 
 //------------------------------------------------------------------------------
 // Servo initialization and control (legacy)
 //------------------------------------------------------------------------------
 static void servo_init(gpio_num_t gpio)
 {
     mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, gpio);
     mcpwm_config_t cfg = {
         .frequency    = 50,
         .cmpr_a       = 0,
         .cmpr_b       = 0,
         .duty_mode    = MCPWM_DUTY_MODE_0,
         .counter_mode = MCPWM_UP_COUNTER,
     };
     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);
 }
 
 static void servo_set_angle(int angle)
 {
     if (angle < 0)   angle = 0;
     if (angle > 180) angle = 180;
     uint32_t pulse_us = (angle * (2500 - 500) / 180) + 500;
     mcpwm_set_duty_in_us(
         MCPWM_UNIT_0,
         MCPWM_TIMER_0,
         MCPWM_OPR_A,
         pulse_us
     );
     ESP_LOGI(TAG,
              "Servo angle -> %d°, pulse %u us",
              (int)angle, (unsigned)pulse_us);
 }
 
 //------------------------------------------------------------------------------
 // Motor control
 //------------------------------------------------------------------------------
 static void movePanLeft(void) {
     if (xTaskGetTickCount() - lastPanTime >= rampTicks) {
         gpio_set_level(PAN_M1A, 1);
         gpio_set_level(PAN_M1B, 0);
         lastPanTime = xTaskGetTickCount();
     }
 }
 static void movePanRight(void) {
     if (xTaskGetTickCount() - lastPanTime >= rampTicks) {
         gpio_set_level(PAN_M1A, 0);
         gpio_set_level(PAN_M1B, 1);
         lastPanTime = xTaskGetTickCount();
     }
 }
 static void stopPanMotor(void) {
     gpio_set_level(PAN_M1A, 0);
     gpio_set_level(PAN_M1B, 0);
 }
 static void moveTiltUp(void) {
     if (xTaskGetTickCount() - lastTiltTime >= rampTicks) {
         gpio_set_level(TILT_M1A, 1);
         gpio_set_level(TILT_M1B, 0);
         lastTiltTime = xTaskGetTickCount();
     }
 }
 static void moveTiltDown(void) {
     if (xTaskGetTickCount() - lastTiltTime >= rampTicks) {
         gpio_set_level(TILT_M1A, 0);
         gpio_set_level(TILT_M1B, 1);
         lastTiltTime = xTaskGetTickCount();
     }
 }
 static void stopTiltMotor(void) {
     gpio_set_level(TILT_M1A, 0);
     gpio_set_level(TILT_M1B, 0);
 }
 
 //------------------------------------------------------------------------------
 // ESP-NOW receive callback (used when useCloud == false)
 //------------------------------------------------------------------------------
 static void on_data_received(const esp_now_recv_info_t *info,
                              const uint8_t *data, int len)
 {
     if (useCloud) return;   // ignore ESP-NOW while in cloud mode
     // parse data[] and drive motors as before
 }
 
 //------------------------------------------------------------------------------
 // RainMaker write callback
 //------------------------------------------------------------------------------
 static esp_err_t write_cb(const esp_rmaker_device_t *device,
                           const esp_rmaker_param_t *param,
                           const esp_rmaker_param_val_t val,
                           void *priv_data, esp_rmaker_write_ctx_t *ctx)
 {
     const char *nm = esp_rmaker_param_get_name(param);
 
     // Master toggle
     if (strcmp(nm, "Master") == 0) {
         useCloud = val.val.b;
         ESP_LOGI(TAG, "Master → %s", useCloud ? "RainMaker" : "ESP-NOW");
         if (useCloud) {
             esp_now_deinit();
         } else {
             esp_now_init();
             esp_now_register_recv_cb(on_data_received);
         }
         esp_rmaker_param_update(master_param, val);
         return ESP_OK;
     }
     if (!useCloud) {
         // ignore other cloud writes when master==OFF
         return ESP_OK;
     }
 
     // Follow toggle
     if (strcmp(nm, "Follow") == 0) {
         autoTracking = val.val.b;
         if (autoTracking) {
             esp_now_deinit();
             ESP_LOGI(TAG, "ESP-NOW DISABLED (cloud control)");
         } else {
             esp_wifi_set_mode(WIFI_MODE_STA);
             esp_wifi_start();
             esp_now_init();
             esp_now_register_recv_cb(on_data_received);
             ESP_LOGI(TAG, "ESP-NOW ENABLED (follow mode)");
         }
     }
     // Pan slider
     else if (strcmp(nm, "Pan") == 0) {
         int p = val.val.i;
         if (p < centerValue-200)      movePanLeft();
         else if (p > centerValue+200) movePanRight();
         else                       stopPanMotor();
         esp_rmaker_param_update(pan_param, esp_rmaker_int(centerValue));
     }
     // Tilt slider
     else if (strcmp(nm, "Tilt") == 0) {
         int t = val.val.i;
         if (t < centerValue-200)      moveTiltUp();
         else if (t > centerValue+200) moveTiltDown();
         else                       stopTiltMotor();
         esp_rmaker_param_update(tilt_param, esp_rmaker_int(centerValue));
     }
     // Orientation dropdown
     else if (strcmp(nm, "Orientation") == 0) {
         const char *s = val.val.s;
         if (strcmp(s, "LANDSCAPE") == 0) {
             servo_set_angle(180);
         } else {
             servo_set_angle(90);
         }
     }
 
     esp_rmaker_param_update(param, val);
     return ESP_OK;
 }
 
 //------------------------------------------------------------------------------
 // Application entry point
 //------------------------------------------------------------------------------
 void app_main(void)
 {
     // NVS init
     esp_err_t err = nvs_flash_init();
     if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
         err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         ESP_ERROR_CHECK(nvs_flash_erase());
         ESP_ERROR_CHECK(nvs_flash_init());
     }
 
     // Motor GPIOs
     gpio_config_t io_conf = {0};
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.pin_bit_mask = (1ULL<<PAN_M1A) | (1ULL<<PAN_M1B)
                         | (1ULL<<TILT_M1A) | (1ULL<<TILT_M1B);
     gpio_config(&io_conf);
     stopPanMotor(); stopTiltMotor();
 
     // IR sensor GPIOs
     io_conf.mode = GPIO_MODE_INPUT;
     io_conf.pin_bit_mask = (1ULL<<IR_LEFT) | (1ULL<<IR_RIGHT);
     gpio_config(&io_conf);
 
     // Initialize servo
     servo_init(SERVO_PIN);
     servo_set_angle(180);
 
     // RainMaker + provisioning
     app_network_init();
     esp_rmaker_config_t rm_cfg = { .enable_time_sync = false };
     esp_rmaker_node_t *node = esp_rmaker_node_init(&rm_cfg,
                                                    "PanTilt Node",
                                                    "Controller");
     esp_rmaker_device_t *dev = esp_rmaker_device_create(
         "Camera Control", NULL, NULL);
     esp_rmaker_device_add_cb(dev, write_cb, NULL);
 
     // 1) Master toggle
     master_param = esp_rmaker_param_create(
         "Master", NULL,
         esp_rmaker_bool(true),
         PROP_FLAG_READ | PROP_FLAG_WRITE
     );
     esp_rmaker_param_add_ui_type(master_param, ESP_RMAKER_UI_TOGGLE);
     esp_rmaker_device_add_param(dev, master_param);
 
     // 2) Follow toggle
     esp_rmaker_param_t *mode_param = esp_rmaker_param_create(
         "Follow", NULL,
         esp_rmaker_bool(false),
         PROP_FLAG_READ | PROP_FLAG_WRITE
     );
     esp_rmaker_param_add_ui_type(mode_param, ESP_RMAKER_UI_TOGGLE);
     esp_rmaker_device_add_param(dev, mode_param);
 
     // 3) Pan slider
     pan_param = esp_rmaker_param_create(
         "Pan", NULL,
         esp_rmaker_int(centerValue),
         PROP_FLAG_READ | PROP_FLAG_WRITE
     );
     esp_rmaker_param_add_ui_type(pan_param, ESP_RMAKER_UI_SLIDER);
     esp_rmaker_param_add_bounds(
         pan_param,
         esp_rmaker_int(0),
         esp_rmaker_int(4095),
         esp_rmaker_int(1)
     );
     esp_rmaker_device_add_param(dev, pan_param);
 
     // 4) Tilt slider
     tilt_param = esp_rmaker_param_create(
         "Tilt", NULL,
         esp_rmaker_int(centerValue),
         PROP_FLAG_READ | PROP_FLAG_WRITE
     );
     esp_rmaker_param_add_ui_type(tilt_param, ESP_RMAKER_UI_SLIDER);
     esp_rmaker_param_add_bounds(
         tilt_param,
         esp_rmaker_int(0),
         esp_rmaker_int(4095),
         esp_rmaker_int(1)
     );
     esp_rmaker_device_add_param(dev, tilt_param);
 
     // 5) Orientation dropdown
     esp_rmaker_param_t *servo_param = esp_rmaker_param_create(
         "Orientation", NULL,
         esp_rmaker_str("LANDSCAPE"),
         PROP_FLAG_READ | PROP_FLAG_WRITE
     );
     esp_rmaker_param_add_ui_type(servo_param, ESP_RMAKER_UI_DROPDOWN);
     const char *orient_list[] = { "PORTRAIT", "LANDSCAPE" };
     esp_rmaker_param_add_valid_str_list(servo_param, orient_list, 2);
     esp_rmaker_device_add_param(dev, servo_param);
 
     esp_rmaker_node_add_device(node, dev);
     esp_rmaker_ota_enable_default();
     app_insights_enable();
     esp_rmaker_start();
     ESP_ERROR_CHECK(app_network_start(POP_TYPE_RANDOM));
 
     // ESP-NOW init
     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
     ESP_ERROR_CHECK(esp_wifi_start());
     ESP_ERROR_CHECK(esp_now_init());
     esp_now_register_recv_cb(on_data_received);
 
     // Main loop
     while (1) {
         if (autoTracking) {
             bool L = (gpio_get_level(IR_LEFT) == 0);
             bool R = (gpio_get_level(IR_RIGHT) == 0);
             if (L && R) stopPanMotor();
             else if (L && !R) movePanLeft();
             else if (!L && R) movePanRight();
             else {
                 movePanLeft();
                 vTaskDelay(pdMS_TO_TICKS(100));
                 if ((gpio_get_level(IR_LEFT)==0) ||
                     (gpio_get_level(IR_RIGHT)==0)) {
                     stopPanMotor();
                 }
             }
         }
         vTaskDelay(pdMS_TO_TICKS(50));
     }
 }
 