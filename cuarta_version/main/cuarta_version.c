#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define ESC_GPIO       18      // GPIO conectado a S del ESC
#define PWM_FREQ       50      // 50 Hz (20 ms)
#define PWM_RES        LEDC_TIMER_16_BIT

// Valores típicos ESC (en microsegundos)
#define ESC_MIN_US     1000
#define ESC_MAX_US     2000
#define ESC_ARM_US     1000
#define ESC_RUN_US     2000    // <<< VELOCIDAD FIJA QUE TÚ SETEAS

#define ENCODER_GPIO     34      // cambia si usas otro pin
#define PULSES_PER_REV  10       // 10 huecos = 10 pulsos por vuelta
volatile uint32_t pulse_count = 0;

static void IRAM_ATTR encoder_isr_handler(void *arg)
{
    pulse_count++;
}

static uint32_t us_to_duty(uint32_t us)
{
    uint32_t period_us = 1000000 / PWM_FREQ;  // 20,000 us
    uint32_t max_duty = (1 << 16) - 1;

    return (us * max_duty) / period_us;
}

void esc_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = PWM_RES,
        .freq_hz          = PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .gpio_num       = ESC_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

void esc_set_pulse_us(uint32_t pulse_us)
{
    uint32_t duty = us_to_duty(pulse_us);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void encoder_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENCODER_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // típico para encoders
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE      // flanco de subida
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_GPIO, encoder_isr_handler, NULL);
}

void rpm_task(void *arg)
{
    uint32_t last_count = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));   // ventana de 1 segundo

        uint32_t current_count = pulse_count;
        uint32_t pulses = current_count - last_count;
        last_count = current_count;

        float rpm = (pulses * 60.0f) / PULSES_PER_REV;

        printf("RPM: %.2f\n", rpm);
    }
}
void app_main(void)
{
    esc_init();

    // 🔹 ARMADO DEL ESC
    esc_set_pulse_us(ESC_ARM_US);
    vTaskDelay(pdMS_TO_TICKS(3000));  // espera típica de armado

    // 🔹 VELOCIDAD FIJA
    esc_set_pulse_us(ESC_RUN_US);

    encoder_init();

    xTaskCreate(rpm_task, "rpm_task", 2048, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

