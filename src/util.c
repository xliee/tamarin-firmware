#include "util.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "pico/stdio.h"
#include "pico/multicore.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/timer.h"

#include "bsp/board.h"
#include "tusb.h"
#include "serial.h"

// send [CLEAR] to clear terminal screen
void clear_screen() {
    tud_cdc_n_write_str(ITF_CONSOLE, "\033[2J\033[H");
    tud_cdc_n_write_flush(ITF_CONSOLE);

    // "[CLEAR]" command
    const char* clear_cmd = "[CLEAR]\r\n";
    // echo to web serial
    if (web_serial_connected) {
        tud_vendor_write(clear_cmd, strlen(clear_cmd));
        tud_vendor_write_flush();
    }

}

void serprint(const char* format, ...) {
    char buf[128];
    va_list args;
    va_start (args, format);
    vsnprintf(buf, 128, format, args);
    va_end (args);

    uint count = strlen(buf);

    // echo to web serial
    if (web_serial_connected) {
        tud_vendor_write(buf, count);
        tud_vendor_write_flush();
    }

    tud_cdc_n_write_str(ITF_CONSOLE, buf);
    tud_cdc_n_write_flush(ITF_CONSOLE);
    tud_task();
}
