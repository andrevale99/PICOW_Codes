#include "pico/stdlib.h"

#include "FreeRTOS.h"

#include "task.h"

#include <stdio.h>

const uint led_pin_red = 12;

void vBlinkTask()
{

    for (;;)
    {

        gpio_put(led_pin_red, 1);

        vTaskDelay(pdMS_TO_TICKS(1000));

        gpio_put(led_pin_red, 0);

        vTaskDelay(pdMS_TO_TICKS(1000));

        printf("Blinking\n");
    }
}

void main()
{

    stdio_init_all();

    gpio_init(led_pin_red);

    gpio_set_dir(led_pin_red, GPIO_OUT);

    xTaskCreate(vBlinkTask, "Blink Task", 128, NULL, 1, NULL);

    vTaskStartScheduler();
}