#include <stdio.h>

#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/clocks.h>
#include <hardware/pll.h>
#include <hardware/irq.h>

#include "FreeRTOS.h"
#include "task.h"

#include "ssd1306.h"

#define LEDG 13

// #define ENCODER_PIN_A 18
#define ENCODER_PIN_A 5 // Teste de encoder

#define I2C_SDA 14
#define I2C_SCL 15
#define I2C_FREQ_BASE 100000UL

// Macro para colocar a escrita no inicoio do OLED
#define RETURN_HOME_SSD(_ssd) memset(_ssd, 0, ssd1306_buffer_length)

#define MAX_LEN_BUFFER UINT8_MAX // Tamanho maximo do buffer

//======================================
//  VARS GLOBAIS
//======================================

// Variavel para calcular a frequencia do sistema
uint f_clk_sys = -1;

// Variavel para contar os pulsos do encoder
volatile int32_t i32PulsosEncoder = 0;

// Preparar área de renderização para o display
// (ssd1306_width pixels por ssd1306_n_pages páginas)
struct render_area frame_area = {
    start_column : 0,
    end_column : ssd1306_width - 1,
    start_page : 0,
    end_page : ssd1306_n_pages - 1
};

uint8_t ssd[ssd1306_buffer_length];

char buffer[MAX_LEN_BUFFER];

//======================================
//  PROTOTIPOS
//======================================

/// @brief Configuracao do I2C
void setup_i2c(void);

/// @brief Configuracao do PWM
void setup_pwm(void);

/// @brief Configuracao do clock do sistemas
void setup_clock(void);

/// @brief Configuracao das GPIOS
void setup_gpio(void);

void gpio_callback(uint gpio, uint32_t events);

/// @brief Task para escrita do OLED
/// @param pvArgs Argumentos
void vTaskOLED(void *pvArgs);

//======================================
//  MAIN
//======================================
int main(void)
{

    setup_clock();
    setup_pwm();
    setup_i2c();
    setup_gpio();

    stdio_init_all();

    xTaskCreate(vTaskOLED, "TaskOLED", 128, NULL, 1, NULL);

    vTaskStartScheduler();

    return 0;
}

//======================================
//  FUNCS
//======================================

void setup_i2c(void)
{
    // Inicialização do i2c
    i2c_init(i2c1, I2C_FREQ_BASE);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void setup_pwm(void)
{
    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(LEDG, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(LEDG);

    // Set period of 4 cycles (0 to 3 inclusive)
    pwm_set_wrap(slice_num, 3);
    // Set initial B output high for three cycles before dropping
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 3);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);
    pwm_set_enabled(slice_num, true);

    // Set the PWM running
    pwm_init(slice_num, &config, true);

    // Note we could also use pwm_set_gpio_level(gpio, x) which looks up the
    // correct slice and channel for a given GPIO.
}

void setup_clock(void)
{
    // Change clk_sys to be 48MHz. The simplest way is to take this from PLL_USB
    // which has a source frequency of 48MHz
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    // Turn off PLL sys for good measure
    pll_deinit(pll_sys);

    // CLK peri is clocked from clk_sys so need to change clk_peri's freq
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    48 * MHZ,
                    48 * MHZ);

    f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
}

void setup_gpio(void)
{
    gpio_set_dir(ENCODER_PIN_A, GPIO_IN);

    gpio_pull_up(ENCODER_PIN_A);

    // Configura a interrupção no GPIO do botão para borda de descida
    gpio_set_irq_enabled_with_callback(ENCODER_PIN_A, GPIO_IRQ_EDGE_FALL,
                                       true, &gpio_callback);
}

void gpio_callback(uint gpio, uint32_t events)
{
    i32PulsosEncoder++;
}

//======================================
//  TASK
//======================================
void vTaskOLED(void *pvArgs)
{
    // Inicia o display oled
    ssd1306_init();
    calculate_render_area_buffer_length(&frame_area);

    // coloca no home do display
    RETURN_HOME_SSD(ssd);
    render_on_display(ssd, &frame_area);

    sprintf(&buffer[0], "Dados do motor");
    ssd1306_draw_string(ssd, 5, 10, buffer);
    sprintf(&buffer[0], "RPM ");
    ssd1306_draw_string(ssd, 5, 25, buffer);
    render_on_display(ssd, &frame_area);

    for (;;)
    {
        sprintf(&buffer[0], "%i", i32PulsosEncoder);
        ssd1306_draw_string(ssd, 35, 25, buffer);
        render_on_display(ssd, &frame_area);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}