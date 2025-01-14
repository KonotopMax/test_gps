#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "string.h"
#include "esp_timer.h"
#include "time.h"
#include "sys/time.h"

#define CONFIG_ESP_CONSOLE_UART_NUM 0
#define GPS_UART UART_NUM_2
#define GPS_TXD (GPIO_NUM_13)
#define GPS_RXD (GPIO_NUM_12)
#define BUF_SIZE 1024

static const char *TAG = "GPS";

void gps_task(void *pvParameters) {
    char* data = (char*) malloc(BUF_SIZE);
    char line[256];
    int line_pos = 0;
    
    // Настройка вывода
    setvbuf(stdout, NULL, _IONBF, 0);
    esp_log_level_set("*", ESP_LOG_INFO);
    
    while (1) {
        int len = uart_read_bytes(GPS_UART, (uint8_t*)data, 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            if (data[0] == '\n' || data[0] == '\r') {
                if (line_pos > 0) {
                    line[line_pos] = 0;
                    
                    // Получаем точное время в микросекундах
                    int64_t time_us = esp_timer_get_time();
                    int msec = (time_us / 1000) % 1000;  // Правильный расчет миллисекунд
                    int64_t time_s = time_us / 1000000;  // Перевод в секунды
                    int sec = time_s % 60;
                    int min = (time_s / 60) % 60;
                    int hour = (time_s / 3600) % 24;
                    
                    // Выводим с временной меткой и гарантированными тремя цифрами миллисекунд
                    printf("%02d:%02d:%02d.%03d -> %s\n", 
                           hour, min, sec, msec, line);
                    fflush(stdout);
                    line_pos = 0;
                }
            } else {
                if (line_pos < sizeof(line)-1) {
                    line[line_pos++] = data[0];
                }
            }
        }
    }
    free(data);
}

void app_main(void)
{
    // Инициализация UART для консоли
    setvbuf(stdout, NULL, _IONBF, 0);
    
    ESP_LOGI(TAG, "GPS Test Starting...");
    
    // Инициализация GPS UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, GPS_TXD, GPS_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, BUF_SIZE * 2, 0, 0, NULL, 0));
    
    // Создание задачи для чтения GPS данных
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
} 