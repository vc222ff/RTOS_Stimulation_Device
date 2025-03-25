#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    sleep_ms(3000);  // Wait for USB to enumerate

    while (1) {
        printf("✅ USB Serial Works!\n");
        sleep_ms(1000);
    }
}
