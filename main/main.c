// Подключение необходимых библиотек
#include <stdio.h>                    // Стандартная библиотека ввода-вывода
#include "freertos/FreeRTOS.h"       // Основная библиотека RTOS
#include "freertos/task.h"           // Для работы с задачами
#include "driver/uart.h"             // Драйвер UART
#include "driver/gpio.h"             // Драйвер GPIO
#include "esp_log.h"                 // Для логирования
#include "string.h"                  // Для работы со строками
#include "esp_timer.h"               // Для работы с таймером ESP
#include "time.h"                    // Для работы со временем
#include "sys/time.h"                // Для системного времени

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
    
    // Отключение буферизации stdout
    setvbuf(stdout, NULL, _IONBF, 0);
    // Установка уровня логирования
    esp_log_level_set("*", ESP_LOG_INFO);
    
    while (1) {
        // Чтение данных из UART по одному байту
        int len = uart_read_bytes(GPS_UART, (uint8_t*)data, 1, 20 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            // Обработка конца строки
            if (data[0] == '\n' || data[0] == '\r') {
                if (line_pos > 0) {
                    line[line_pos] = 0;  // Завершающий нуль
                    
                    // Получение текущего времени
                    int64_t time_us = esp_timer_get_time();
                    // Расчет компонентов времени
                    int msec = (time_us / 1000) % 1000;
                    int64_t time_s = time_us / 1000000;
                    int sec = time_s % 60;
                    int min = (time_s / 60) % 60;
                    int hour = (time_s / 3600) % 24;
                    
                    // Вывод времени и данных
                    printf("%02d:%02d:%02d.%03d -> %s\n", 
                           hour, min, sec, msec, line);
                    fflush(stdout);   // Принудительный вывод
                    line_pos = 0;     // Сброс позиции
                }
            } else {
                // Добавление символа в буфер
                if (line_pos < sizeof(line)-1) {
                    line[line_pos++] = data[0];
                }
            }
        }
    }
    free(data);  // Освобождение памяти (никогда не выполнится в данном коде)
}

void app_main(void)
{
    // Отключение буферизации stdout
    setvbuf(stdout, NULL, _IONBF, 0);
    
    ESP_LOGI(TAG, "GPS Test Starting...");
    
    // Конфигурация UART
    uart_config_t uart_config = {
        .baud_rate = 115200,         // Скорость передачи
        .data_bits = UART_DATA_8_BITS, // 8 бит данных
        .parity    = UART_PARITY_DISABLE, // Без четности
        .stop_bits = UART_STOP_BITS_1,    // 1 стоп-бит
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE // Без управления потоком
    };

    // Применение конфигурации UART
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &uart_config));
    // Назначение пинов для UART
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, GPS_TXD, GPS_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Установка драйвера UART
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, BUF_SIZE * 2, 0, 0, NULL, 0));
    
    // Создание задачи FreeRTOS для обработки GPS данных
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
} 