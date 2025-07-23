#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define LEDR 12
#define LEDG 13

volatile bool timer_fired = false;

int64_t alarm_callback(alarm_id_t id, __unused void *user_data)
{
    if (gpio_get(LEDR))
        gpio_put(LEDR, 0);
    else
        gpio_put(LEDR, 1);

    timer_fired = true;
    // Can return a value here in us to fire in the future
    return 0;
}

bool repeating_timer_callback(__unused struct repeating_timer *t)
{
    if (gpio_get(LEDG))
        gpio_put(LEDG, 0);
    else
        gpio_put(LEDG, 1);

    return true;
}

int main()
{
    stdio_init_all();

    gpio_init(LEDR);
    gpio_set_dir(LEDR, true);

    gpio_init(LEDG);
    gpio_set_dir(LEDG, true);

    gpio_put(LEDR, 1);
    // Call alarm_callback in 2 seconds
    add_alarm_in_ms(2000, alarm_callback, NULL, false);

    // Wait for alarm callback to set timer_fired
    while (!timer_fired)
    {
        tight_loop_contents();
    }

    // Create a repeating timer that calls repeating_timer_callback.
    // If the delay is > 0 then this is the delay between the previous callback ending and the next starting.
    // If the delay is negative (see below) then the next call to the callback will be exactly 500ms after the
    // start of the call to the last callback
    struct repeating_timer timer;
    gpio_put(LEDR, 0);
    add_repeating_timer_ms(500, repeating_timer_callback, NULL, &timer);
    sleep_ms(3500);
    bool cancelled = cancel_repeating_timer(&timer);
    sleep_ms(2000);

    while (1)
    {
        // if (gpio_get(LEDR))
        //     gpio_put(LEDR, 0);
        // else
        //     gpio_put(LEDR, 1);

        // busy_wait_ms(1000);
    }

    return 0;
}
