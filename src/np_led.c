#include "np_led.h"
#include "ws2818b.pio.h"

npLED_t leds[LED_COUNT]; // Buffer de LEDs
PIO np_pio;
uint sm;

/**
 * Inicializa a máquina PIO para controle da matriz de LEDs.
 */
void npInit(uint pin)
{
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;

    sm = pio_claim_unused_sm(np_pio, false);
    if (sm < 0)
    {
        np_pio = pio1;
        sm = pio_claim_unused_sm(np_pio, true);
    }

    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

    for (uint i = 0; i < LED_COUNT; ++i)
    {
        leds[i].R = leds[i].G = leds[i].B = 0;
    }
}

/**
 * Atribui uma cor RGB a um LED.
 */
void npSetLED(uint index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index < LED_COUNT)
    {
        leds[index].R = r;
        leds[index].G = g;
        leds[index].B = b;
    }
}

/**
 * Limpa o buffer de pixels.
 */
void npClear()
{
    for (uint i = 0; i < LED_COUNT; ++i)
    {
        npSetLED(i, 0, 0, 0);
    }
}

/**
 * Escreve os dados do buffer nos LEDs.
 */
void npWrite()
{
    for (uint i = 0; i < LED_COUNT; ++i)
    {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100); // Espera 100us, sinal de RESET do datasheet.
}

/**
 * Aplica um fator de escala para brilho na matriz.
 */
void applyBrightnessToMatrix(int matriz[5][5][3], float scale)
{
    for (int linha = 0; linha < 5; linha++)
    {
        for (int coluna = 0; coluna < 5; coluna++)
        {
            matriz[linha][coluna][0] = (uint8_t)(matriz[linha][coluna][0] * scale);
            matriz[linha][coluna][1] = (uint8_t)(matriz[linha][coluna][1] * scale);
            matriz[linha][coluna][2] = (uint8_t)(matriz[linha][coluna][2] * scale);
        }
    }
}

/**
 * Converte uma posição na matriz para índice no vetor de LEDs.
 */
int getIndex(int x, int y)
{
    return (y % 2 == 0) ? 24 - (y * 5 + x) : 24 - (y * 5 + (4 - x));
}

/**
 * Atualiza a matriz com base em uma configuração de cores.
 */
void updateMatrix(int matriz[5][5][3])
{
    for (int linha = 0; linha < 5; linha++)
    {
        for (int coluna = 0; coluna < 5; coluna++)
        {
            int posicao = getIndex(coluna, linha);
            npSetLED(posicao, matriz[linha][coluna][0], matriz[linha][coluna][1], matriz[linha][coluna][2]);
        }
    }
    npWrite();
}