#ifndef PIN_CONFIGURATION_H
#define PIN_CONFIGURATION_H

// Definição dos pinos
const uint VRX = 26;
const uint VRY = 27;
const uint ADC_CHANNEL_0 = 0;
const uint ADC_CHANNEL_1 = 1;
const uint SW = 22;
const uint BUZZER_A = 21;

// Constantes para configurações PWM e LED
const float DIVISOR_CLK_PWM = 16.0;      // Divisor de clock para o PWM
const uint16_t MAX_WRAP_DIV_BUZZER = 16; // Valor máximo para wrap divisor do buzzer
const uint16_t MIN_WRAP_DIV_BUZZER = 2;  // Valor mínimo para wrap divisor do buzzer

// LEDs RGB
const uint LED_B = 12;
const uint LED_R = 13;
const float DIVIDER_PWM = 16.0;
const uint16_t PERIOD = 4096;

// Pino do LED neopixel
#define MATRIX_LED_PIN 7

// Configuração do display OLED via I2C
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define ADDRESS 0x3C
bool COLOR = true;

#endif