/*
 *  Por: Wilton Lacerda Silva
 *  Data: 10/05/2025
 *
 *  Exemplo do uso de Filas queue no FreeRTOS com Raspberry Pi Pico
 *
 *  Descrição: Leitura do valor do joystick e exibição no display OLED SSD1306
 *  com comunicação I2C. O valor do joystick é lido a cada 100ms e enviado para a fila.
 *  A task de exibição recebe os dados da fila e atualiza o display a cada 100ms.
 *  Os leds são controlados por PWM, com brilho proporcional ao desvio do joystick.
 *  O led verde controla o eixo X e o led azul controla o eixo Y.
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define ADC_JOYSTICK_X 27
#define ADC_JOYSTICK_Y 26//para versão da BitDogLab representa eixo Y
#define LED_BLUE 12
#define LED_GREEN  11
#define tam_quad 10

typedef struct
{
    uint16_t x_pos;
    uint16_t y_pos;
} joystick_data_t;

QueueHandle_t xQueueJoystickData;

void vJoystickTask(void *params)
{
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);
    adc_init();

    joystick_data_t joydata;

    while (true)
    {
        adc_select_input(0); // GPIO 26 = ADC0
        joydata.y_pos = adc_read();

        adc_select_input(1); // GPIO 27 = ADC1
        joydata.x_pos = adc_read();

        xQueueSend(xQueueJoystickData, &joydata, 0); // Envia o valor do joystick para a fila
        vTaskDelay(pdMS_TO_TICKS(100));              // 10 Hz de leitura
    }
}

void vDisplayTask(void *params)
{
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    joystick_data_t joydata;
    bool cor = true;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            uint8_t x = (joydata.x_pos * (128 - tam_quad)) / 4095;
            uint8_t y = (joydata.y_pos * (64 - tam_quad)) / 4095;
            y = (64 - tam_quad) - y;                                 // Inverte o eixo Y
            ssd1306_fill(&ssd, !cor);                                // Limpa a tela
            ssd1306_rect(&ssd, y, x, tam_quad, tam_quad, cor, !cor); // Quadrado 5x5
            ssd1306_send_data(&ssd);
        }
    }
}

void vLedGreenTask(void *params)
{
    gpio_set_function(LED_GREEN, GPIO_FUNC_PWM);   // Configura GPIO como PWM
    uint slice = pwm_gpio_to_slice_num(LED_GREEN); // Obtém o slice de PWM
    pwm_set_wrap(slice, 100);                     // Define resolução (0–100)
    pwm_set_chan_level(slice, PWM_CHAN_B, 0);     // Duty inicial
    pwm_set_enabled(slice, true);                 // Ativa PWM

    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            // Brilho proporcional ao desvio do centro
            int16_t desvio_centro = (int16_t)joydata.x_pos - 2000;
            if (desvio_centro < 0)
                desvio_centro = -desvio_centro;
            uint16_t pwm_value = (desvio_centro * 100) / 2048;
            pwm_set_chan_level(slice, PWM_CHAN_B, pwm_value);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}

void vLedBlueTask(void *params)
{
    gpio_set_function(LED_BLUE, GPIO_FUNC_PWM);   // Configura GPIO como PWM
    uint slice = pwm_gpio_to_slice_num(LED_BLUE); // Obtém o slice de PWM
    pwm_set_wrap(slice, 100);                     // Define resolução (0–100)
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);     // Duty inicial
    pwm_set_enabled(slice, true);                 // Ativa PWM

    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            // Brilho proporcional ao desvio do centro
            int16_t desvio_centro = (int16_t)joydata.y_pos - 2048;
            if (desvio_centro < 0)
                desvio_centro = -desvio_centro;
            uint16_t pwm_value = (desvio_centro * 100) / 2048;
            pwm_set_chan_level(slice, PWM_CHAN_A, pwm_value);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}


// Modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

int main()
{
    // Ativa BOOTSEL via botão
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();

    // Cria a fila para compartilhamento de valor do joystick
    xQueueJoystickData = xQueueCreate(5, sizeof(joystick_data_t));

    // Criação das tasks
    xTaskCreate(vJoystickTask, "Joystick Task", 256, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 512, NULL, 1, NULL);
    xTaskCreate(vLedGreenTask, "LED red Task", 256, NULL, 1, NULL);
    xTaskCreate(vLedBlueTask, "LED blue Task", 256, NULL, 1, NULL);
    // Inicia o agendador
    vTaskStartScheduler();
    panic_unsupported();
}
