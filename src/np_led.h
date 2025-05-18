#ifndef NP_LED_H
#define NP_LED_H

#include "hardware/pio.h"
#include "pico/stdlib.h"

// Define o tipo pixel e renomeia para npLED_t
typedef struct
{
    uint8_t G, R, B;
} npLED_t;

// Número total de LEDs (você pode mudar esse valor aqui também)
#define LED_COUNT 25

// Buffer de LEDs
extern npLED_t leds[LED_COUNT];

// Variáveis globais de controle do PIO
extern PIO np_pio;
extern uint sm;

// Prototipação das funções
void npInit(uint pin);
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b);
void npClear();
void npWrite();
int getIndex(int x, int y);
void updateMatrix(int matriz[5][5][3]);
void applyBrightnessToMatrix(int matriz[5][5][3], float scale);

#endif // NP_LED_H
