#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "src/ssd1306.h"
#include "src/pin_configuration.h"
#include "src/font.h"
#include "src/np_led.h"
#include "src/handle_alert.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Structure for sensor data
typedef struct
{
    uint16_t nivel_agua;   // Simulado pelo eixo Y do joystick (0-100%)
    uint16_t volume_chuva; // Simulado pelo eixo X do joystick (0-100%)
    char status_message[20];
    uint8_t alert_source; // 0 = nenhum, 1 = água, 2 = chuva, 3 = ambos
} SensorData_t;

// Structure for buzzer control
typedef struct
{
    bool active;
    uint32_t interval;
} BuzzerControl_t;

// Queues
QueueHandle_t xSensorQueue;
QueueHandle_t xLEDQueue;
QueueHandle_t xBuzzerQueue;
QueueHandle_t xDisplayQueue;

// Global variables
ssd1306_t ssd;
uint32_t current_time;
static volatile uint32_t last_time_SW = 0;
uint slice_led_r, slice_led_b, slice_buzzer;
uint LED_ON = 500;
uint LED_OFF = 0;

// Function prototypes
void vJoystickTask(void *pvParameters);
void vRedLEDTask(void *pvParameters);
void vBlueLEDTask(void *pvParameters);
void vBuzzerTask(void *pvParameters);
void vDisplayTask(void *pvParameters);
void vMatrixLEDTask(void *pvParameters);
void setup_pwm(uint gpio, uint *slice, uint16_t level);
void setup_matrix_led();
void setup_joystick_and_button();
void setup_display();

void setup_matrix_led()
{
    npInit(MATRIX_LED_PIN);
    npClear();
    npWrite();
}

// PWM setup function (usada apenas para o buzzer)
void setup_pwm(uint gpio, uint *slice, uint16_t level)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    *slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(*slice, DIVIDER_PWM);
    pwm_set_wrap(*slice, PERIOD);
    pwm_set_gpio_level(gpio, level);
    pwm_set_enabled(*slice, true);
}

// Joystick and button setup
void setup_joystick_and_button()
{
    adc_init();
    adc_gpio_init(VRX); // Volume de chuva (eixo X)
    adc_gpio_init(VRY); // Nível de água (eixo Y)

    gpio_init(SW);
    gpio_set_dir(SW, GPIO_IN);
    gpio_pull_up(SW);
}

// Display setup
void setup_display()
{
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ADDRESS, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

// Initial setup
void setup()
{
    stdio_init_all();
    setup_joystick_and_button();
    setup_display();

    // Configuração do LED Vermelho (PWM_CHAN_B)
    gpio_set_function(LED_R, GPIO_FUNC_PWM);
    slice_led_r = pwm_gpio_to_slice_num(LED_R);
    pwm_set_clkdiv(slice_led_r, 4.0f);
    pwm_set_wrap(slice_led_r, 100);
    pwm_set_chan_level(slice_led_r, PWM_CHAN_B, 0);
    pwm_set_enabled(slice_led_r, true);

    // Configuração do LED Azul (PWM_CHAN_A)
    gpio_set_function(LED_B, GPIO_FUNC_PWM);
    slice_led_b = pwm_gpio_to_slice_num(LED_B);
    pwm_set_clkdiv(slice_led_b, 4.0f);
    pwm_set_wrap(slice_led_b, 100);
    pwm_set_chan_level(slice_led_b, PWM_CHAN_A, 0);
    pwm_set_enabled(slice_led_b, true);

    // Configuração do Buzzer com a função setup_pwm
    setup_pwm(BUZZER_A, &slice_buzzer, 0);

    // Configuração do LED Neopixel
    setup_matrix_led();

    // Create queues
    xSensorQueue = xQueueCreate(4, sizeof(SensorData_t));
    xBuzzerQueue = xQueueCreate(1, sizeof(BuzzerControl_t));
    xDisplayQueue = xQueueCreate(1, sizeof(SensorData_t));

    // Create tasks
    xTaskCreate(vJoystickTask, "Joystick", 256, NULL, 1, NULL);
    xTaskCreate(vBlueLEDTask, "BlueLED", 256, NULL, 1, NULL);
    xTaskCreate(vRedLEDTask, "RedLED", 256, NULL, 1, NULL);
    xTaskCreate(vBuzzerTask, "Buzzer", 256, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display", 512, NULL, 1, NULL);
    xTaskCreate(vMatrixLEDTask, "MatrixLED", 256, NULL, 1, NULL);
}

// Joystick task - simula leitura de sensores
void vJoystickTask(void *pvParameters)
{
    SensorData_t sensorData;

    while (1)
    {
        // Lê eixo Y (nivel de água) - convertido para 0-100%
        adc_select_input(ADC_CHANNEL_0); // VRY
        sensorData.nivel_agua = (adc_read() * 100) / 4095;

        // Lê eixo X (volume de chuva) - convertido para 0-100%
        adc_select_input(ADC_CHANNEL_1); // VRX
        sensorData.volume_chuva = (adc_read() * 100) / 4095;

        // Determina o status e a fonte do alerta
        bool agua_alerta = (sensorData.nivel_agua >= 70);
        bool chuva_alerta = (sensorData.volume_chuva >= 80);

        if (agua_alerta && chuva_alerta)
        {
            strcpy(sensorData.status_message, "ALERTA CRITICO");
            sensorData.alert_source = 3; // Ambos
        }
        else if (agua_alerta)
        {
            strcpy(sensorData.status_message, "ALERTA AGUA");
            sensorData.alert_source = 1; // Água
        }
        else if (chuva_alerta)
        {
            strcpy(sensorData.status_message, "ALERTA CHUVA");
            sensorData.alert_source = 2; // Chuva
        }
        else
        {
            strcpy(sensorData.status_message, "NORMAL");
            sensorData.alert_source = 0; // Nenhum
        }

        // Envia dados para as outras tarefas
        xQueueSend(xSensorQueue, &sensorData, 0);
        xQueueSend(xDisplayQueue, &sensorData, 0);

        vTaskDelay(pdMS_TO_TICKS(10)); // Atualiza a cada 10ms
    }
}
// LED Matrix task
void vMatrixLEDTask(void *pvParameters)
{
    float brightnessScale = 0.05; // Escala de brilho (1%)
    SensorData_t sensorData;

    applyBrightnessToMatrix(LETRA_S, brightnessScale);
    applyBrightnessToMatrix(LETRA_O, brightnessScale);
    applyBrightnessToMatrix(SORRISO_NORMAL, brightnessScale);

    int state = 0;

    while (1)
    {
        if (xQueueReceive(xSensorQueue, &sensorData, portMAX_DELAY) == pdTRUE)
        {
            if (strcmp(sensorData.status_message, "NORMAL") == 0)
            {
                state = -1;          // Reseta o estado
                handle_alert(state); // Sorriso verde
            }
            else
            {
                state = (state + 1) % 4; // Alterna entre os estados
                handle_alert(state);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Atualiza a cada 500ms
    }
}

// Blue LED task
void vBlueLEDTask(void *pvParameters)
{
    SensorData_t sensorData;
    bool blue_state = false;

    while (1)
    {
        if (xQueueReceive(xSensorQueue, &sensorData, portMAX_DELAY) == pdTRUE)
        {
            if (strcmp(sensorData.status_message, "NORMAL") == 0)
            {
                blue_state = true; // Modo normal - LED azul ligado

                // Envia sinal para desativar o buzzer
                BuzzerControl_t buzzerControl = {false, 0};
                xQueueSend(xBuzzerQueue, &buzzerControl, 0);
            }
            else
            {
                blue_state = false; // Modo alerta - LED azul desligado
            }

            // Atualiza LED Azul (PWM_CHAN_A)
            pwm_set_chan_level(slice_led_b, PWM_CHAN_A, blue_state ? LED_ON : LED_OFF);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Red LED task
void vRedLEDTask(void *pvParameters)
{
    SensorData_t sensorData;
    bool red_state = false;

    while (1)
    {
        if (xQueueReceive(xSensorQueue, &sensorData, portMAX_DELAY) == pdTRUE)
        {
            if (sensorData.alert_source > 0)
            {                     // Se há algum alerta
                red_state = true; // Liga LED vermelho

                BuzzerControl_t buzzerControl;
                buzzerControl.active = true;

                // Define intervalo baseado na fonte do alerta
                if (sensorData.alert_source == 1)
                {                                 // Água
                    buzzerControl.interval = 300; // 300ms para alerta de água
                }
                else if (sensorData.alert_source == 2)
                {                                 // Chuva
                    buzzerControl.interval = 500; // 500ms para alerta de chuva
                }
                else
                {                                 // Ambos (crítico)
                    buzzerControl.interval = 200; // 200ms mais rápido para alerta crítico
                }

                xQueueSend(xBuzzerQueue, &buzzerControl, 0);
            }
            else
            {
                red_state = false; // Desliga LED vermelho

                // Desativa o buzzer
                BuzzerControl_t buzzerControl = {false, 0};
                xQueueSend(xBuzzerQueue, &buzzerControl, 0);
            }

            // Atualiza LED Vermelho
            pwm_set_chan_level(slice_led_r, PWM_CHAN_B, red_state ? LED_ON : LED_OFF);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Buzzer task
void vBuzzerTask(void *pvParameters)
{
    BuzzerControl_t buzzerControl;
    uint32_t last_buzzer_time = 0;
    bool buzzer_state = false;

    while (1)
    {
        if (xQueueReceive(xBuzzerQueue, &buzzerControl, portMAX_DELAY) == pdTRUE)
        {
            if (buzzerControl.active)
            {
                uint32_t now = time_us_32();
                if (now - last_buzzer_time >= buzzerControl.interval * 1000)
                {
                    last_buzzer_time = now;
                    buzzer_state = !buzzer_state;
                    pwm_set_gpio_level(BUZZER_A, buzzer_state ? 300 : 0);
                }
            }
            else
            {
                // Desativa o buzzer
                pwm_set_gpio_level(BUZZER_A, 0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Display task
void vDisplayTask(void *pvParameters)
{
    SensorData_t sensorData;
    char str_agua[20], str_chuva[20];

    while (1)
    {
        if (xQueueReceive(xDisplayQueue, &sensorData, portMAX_DELAY) == pdTRUE)
        {
            // Formata strings para exibição
            snprintf(str_agua, sizeof(str_agua), "Agua: %d%%", sensorData.nivel_agua);
            snprintf(str_chuva, sizeof(str_chuva), "Chuva: %d%%", sensorData.volume_chuva);

            // Limpa e desenha borda
            ssd1306_fill(&ssd, !COLOR);
            ssd1306_rect(&ssd, 3, 3, 120, 56, COLOR, !COLOR, 1);

            // Calcula posições
            uint8_t center_agua = (WIDTH - (strlen(str_agua) * 8)) / 2;
            uint8_t center_chuva = (WIDTH - (strlen(str_chuva) * 8)) / 2;
            uint8_t center_y = HEIGHT / 2 - 8;
            uint8_t center_status = (WIDTH - (strlen(sensorData.status_message) * 8)) / 2;

            // Desenha strings
            ssd1306_draw_string(&ssd, str_agua, center_agua, center_y - 8);
            ssd1306_draw_string(&ssd, str_chuva, center_chuva, center_y + 2);

            ssd1306_draw_string(&ssd, sensorData.status_message, center_status, center_y + 14);

            // Envia para o display
            ssd1306_send_data(&ssd);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main()
{
    setup();
    vTaskStartScheduler();

    while (1)
    {
        // Não deve chegar aqui
    }
}