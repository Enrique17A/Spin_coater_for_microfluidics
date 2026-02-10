#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define PWM_GPIO       18
#define PWM_FREQ_HZ    1000
#define PWM_RES        LEDC_TIMER_10_BIT
#define PWM_CHANNEL    LEDC_CHANNEL_0
#define PWM_TIMER      LEDC_TIMER_0

#define PWM_DUTY       512   // Cambia esto para variar velocidad

void app_main(void)
{
    // 1. Configurar el timer del PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution  = PWM_RES,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 2. Configurar el canal PWM
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = PWM_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM_GPIO,
        .duty           = PWM_DUTY,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // 3. Aplicar el duty
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, PWM_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL));

    printf("PWM iniciado: freq=%d Hz, duty=%d\n", PWM_FREQ_HZ, PWM_DUTY);

    // 4. Loop vacío (PWM queda corriendo solo)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

