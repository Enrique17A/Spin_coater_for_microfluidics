#include "freertos/task.h"
#include "driver/rmt.h"
#include "esp_log.h"
#include "esp_err.h"

#define TAG "DSHOT"

// -----------------------------
// Configuración RMT y GPIO
// -----------------------------
#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_GPIO    5
#define RMT_CLK_DIV    2   // Tick = 80MHz / 2 = 40MHz = 25ns

// -----------------------------
// Temporización DSHOT600
// -----------------------------
#define DSHOT_T0H 25   // 0.625us
#define DSHOT_T0L 42   // 1.05us
#define DSHOT_T1H 50   // 1.25us
#define DSHOT_T1L 34   // 0.85us

// -----------------------------
// Parámetros de motor y ESC
// -----------------------------
#define ESC_SPEED_ZERO   0     // Throttle 0
#define ESC_SPEED_START  400   // Throttle para armar ESC
#define ESC_SPEED_MAX    700   // Throttle máximo seguro para fuente 5A
#define RAMP_STEP        10
#define RAMP_DELAY_MS    50

// -----------------------------
// Funciones DSHOT
// -----------------------------
uint8_t dshot_crc(uint16_t value) {
    uint8_t csum = 0;
    uint16_t csum_data = value;
    for(int i=0;i<3;i++){
        csum ^= csum_data;
        csum_data >>=4;
    }
    return csum & 0xF;
}

uint16_t dshot_build_packet(uint16_t throttle) {
    uint16_t packet = throttle << 1; // Telemetry bit = 0
    uint8_t crc = dshot_crc(packet);
    packet = (packet << 4) | crc;
    return packet;
}

void dshot_rmt_send(uint16_t packet) {
    rmt_item32_t items[16];

    for(int i=0;i<16;i++){
        if(packet & 0x8000){
            items[i].level0 = 1; items[i].duration0 = DSHOT_T1H;
            items[i].level1 = 0; items[i].duration1 = DSHOT_T1L;
        } else {
            items[i].level0 = 1; items[i].duration0 = DSHOT_T0H;
            items[i].level1 = 0; items[i].duration1 = DSHOT_T0L;
        }
        packet <<=1;
    }

    rmt_write_items(RMT_TX_CHANNEL, items, 16, true);
}

void send_throttle(uint16_t throttle, int count, int delay_ms){
    for(int i=0;i<count;i++){
        dshot_rmt_send(dshot_build_packet(throttle));
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

// -----------------------------
// Task principal motor
// -----------------------------
void motor_task(void* arg) {
    ESP_LOGI(TAG, "Arming ESC...");
    // 1. 50 paquetes de 0 para asegurar armado
    send_throttle(ESC_SPEED_ZERO, 100, 20);

    // 2. 50 paquetes de ESC_SPEED_START
    send_throttle(ESC_SPEED_START, 100, 20);

    ESP_LOGI(TAG, "Ramp-up gradual...");
    // 3. Ramp-up gradual a velocidad deseada
    uint16_t speed = ESC_SPEED_START;
    while(speed <= ESC_SPEED_MAX){
        dshot_rmt_send(dshot_build_packet(speed));
        ESP_LOGI(TAG, "Throttle: %d", speed);
        speed += RAMP_STEP;
        vTaskDelay(pdMS_TO_TICKS(RAMP_DELAY_MS));
    }

    ESP_LOGI(TAG, "Motor a velocidad fija: %d", ESC_SPEED_MAX);

    // 4. Mantener velocidad constante
    while(1){
        dshot_rmt_send(dshot_build_packet(ESC_SPEED_MAX));
        vTaskDelay(pdMS_TO_TICKS(50)); // ~50Hz
    }
}

// -----------------------------
// Función main
// -----------------------------
void app_main(void) {
    // Configurar RMT
    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_TX_CHANNEL,
        .gpio_num = RMT_TX_GPIO,
        .clk_div = RMT_CLK_DIV,
        .mem_block_num = 1,
        .tx_config = {
            .loop_en = false,
            .carrier_en = false,
            .idle_output_en = true,
            .idle_level = RMT_IDLE_LEVEL_LOW
        }
    };
    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);

    // Crear tarea del motor
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
}
