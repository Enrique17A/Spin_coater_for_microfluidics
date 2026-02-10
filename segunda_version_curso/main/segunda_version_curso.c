#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <stdint.h>
#include "esp_log.h"

#define AIN1 27
#define AIN2 26
#define PWMA 25
#define ENCODER 34

#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_FREQ 20000
#define PWM_RES LEDC_TIMER_10_BIT
#define PWM_MAX ((1 << 10) - 1)
#define TAG "CONTADOR_DE_PULSOS"

volatile uint32_t contadorPulsos = 0;
float dutyActual = 50.0f;
float porcentajeMax = 100.0f;
float porcentajeMinimo = 10.0f;

float integral = 0;
float ultimoError = 0;
float kp = 0.05f;
float ki = 0.002f;
float kd = 0.00f;

static void IRAM_ATTR encoderIsr(void *arg)
{
    contadorPulsos = contadorPulsos + 1;
}

void inicioEncoder()
{
    gpio_config_t puertoEncoder = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << 34),
        .pull_down_en = 1
    };
    gpio_config(&puertoEncoder);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER,encoderIsr,NULL);
}

void inicioMotorPwm()
{
    printf("Configuracion GPIO:  \n");
    gpio_config_t io_driver = {
        .pin_bit_mask = ((1 << AIN1) | (1 << AIN2)),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_driver);
    gpio_set_level(AIN1,1);
    gpio_set_level(AIN2,0);
    printf("Fin configuracion GPIO:  \n");

    printf("Iniciando el timer:  \n");
    ledc_timer_config_t ledc_timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ch_conf = {
        .gpio_num = PWMA,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel =PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&ch_conf);
}

void motorRun(int porcentaje)
{
    if(porcentaje < porcentajeMinimo)
    {
        porcentaje = porcentajeMinimo;
    }

    if(porcentaje > porcentajeMax)
    {
        porcentaje = porcentajeMax;
    }

    int pwmDuty = ((PWM_MAX * porcentaje) / 100);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL, pwmDuty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
    dutyActual = porcentaje;
}

float controladorPid(int ref, int valorActual)
{
    float error = ref - valorActual;
    integral = integral + error;
    float derivada = error - ultimoError;
    float u = kp*error + ki*integral + kd*derivada;
    int nuevoCicloTrabajo = (int)(dutyActual + u);
    motorRun(nuevoCicloTrabajo);
    ultimoError = error;
    return nuevoCicloTrabajo;
}

void app_main(void)
{
    inicioMotorPwm();
    motorRun(50);
    inicioEncoder();
    int refPulsos = 15;
    
    while(1) {
        contadorPulsos = 0;
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        uint32_t pulsos = contadorPulsos;
        float accionControl = controladorPid(refPulsos, pulsos);
        ESP_LOGI(TAG, "REF = %d | ACTUAL = %d | DUTY= %.1f%% | ACCION = %.2f", refPulsos, pulsos, dutyActual, accionControl);
    }
}
