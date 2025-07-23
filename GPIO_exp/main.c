#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>

#define LEDR 12
#define LEDG 11
#define LEDB 13

#define HIGH 1
#define LOW 0

#define BUTTON_A 5 // GPIO conectado ao Botão A
#define BUTTON_B 6 // GPIO conectado ao Botão A

//=========================================
//  PROTOTIPOS
//=========================================

void gpio_callback(uint gpio, uint32_t events);

//=========================================
//  MAIN
//=========================================
int main()
{
    stdio_init_all();

    gpio_init_mask((1 << LEDR) | (1 << LEDG) |
                   (1 << BUTTON_A) | (1 << BUTTON_B));

    gpio_set_dir_out_masked((1 << LEDR) | (1 << LEDG));
    gpio_set_dir_in_masked((1 << BUTTON_A) | (1 << BUTTON_B));

    gpio_pull_up(BUTTON_A);
    gpio_pull_up(BUTTON_B);

    // Configura a interrupção no GPIO do botão para borda de descida
    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL,
                                       true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL,
                                       true, &gpio_callback);

    while (true)
    {
    }
}

//=========================================
//  FUNCOES
//=========================================

void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == BUTTON_A)
    {
        if (gpio_get_out_level(LEDR))
            gpio_put(LEDR, LOW);
        else
            gpio_put(LEDR, HIGH);
    }

    if (gpio == BUTTON_B)
    {
        if (gpio_get_out_level(LEDG))
            gpio_put(LEDG, LOW);
        else
            gpio_put(LEDG, HIGH);
    }
}
