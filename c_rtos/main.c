#include "pico/stdlib.h"

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 250
#endif

// Perform initialisation
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}

// Turn the led on or off
void pico_set_led(int led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

int main() {
    int rc = pico_led_init();
    //hard_assert(rc == PICO_OK);
    while (1) {
        pico_set_led(1);
        sleep_ms(LED_DELAY_MS);
        pico_set_led(0);
        sleep_ms(LED_DELAY_MS);
    }
}




//#include "FreeRTOS.h"
//#include "task.h"

//#include <stdio.h>
//#include "pico/stdlib.h"


//void led_task()
//{   
//    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
//    gpio_init(LED_PIN);
//    gpio_set_dir(LED_PIN, GPIO_OUT);
//    while (true) {
//        gpio_put(LED_PIN, 1);
//        sleep_ms(250);
        //vTaskDelay(100);
//        gpio_put(LED_PIN, 0);
//        sleep_ms(250);
        //vTaskDelay(100);
//    }
//}

//int main()
//{
//    stdio_init_all();
//
    //xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    //vTaskStartScheduler();
//    led_task();
//
//    while(1){};
//}