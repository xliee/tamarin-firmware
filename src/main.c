#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

#include "pico/stdio.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"

#include "lightning_rx.pio.h"
#include "lightning_tx.pio.h"

#include "bsp/board.h"
#include "bsp/board_api.h"
#include "usb_descriptors.h"
#include "tusb.h"

#include "tamarin_probe.h"
#include "serial.h"
#include "util.h"
#include "crc.h"

#define PIN_SDQ 3
#define PRODUCT_URL "example.tinyusb.org/webusb-serial/index.html"
#ifdef PICO_DEFAULT_LED_PIN
#define PIN_LED PICO_DEFAULT_LED_PIN
#else
#define PIN_LED 25
#endif

const tusb_desc_webusb_url_t desc_url =
{
  .bLength         = 3 + sizeof(PRODUCT_URL) - 1,
  .bDescriptorType = 3, // TAMARIN URL type
  .bScheme         = 1, // 0: http, 1: https
  .url             = PRODUCT_URL
};

bool web_serial_connected = false;


/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED     = 1000,
  BLINK_SUSPENDED   = 2500,

  BLINK_ALWAYS_ON   = UINT32_MAX,
  BLINK_ALWAYS_OFF  = 0
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;


volatile bool serialEnabled = false;
volatile bool jtagInited = false;
volatile bool jtagEnabled = false;
volatile bool wantLeaveJtag = false;

void configure_rx(PIO pio, uint sm) {
    pio_sm_set_enabled(pio, sm, false);
    pio_clear_instruction_memory(pio);
    uint offset = pio_add_program(pio, &lightning_rx_program);
    pio_sm_config c = lightning_rx_program_init(pio, sm, offset, PIN_SDQ, 125.0/2.0);
}

void leave_dcsd() {
    serialEnabled = false;

    uart_deinit(uart0);
}

void leave_jtag() {
    jtagEnabled = false;
    jtagInited  = false;

    tamarin_probe_deinit();
    wantLeaveJtag = false;
}

void lightning_send_wake() {
    gpio_init(PIN_SDQ);
    gpio_set_dir(PIN_SDQ, GPIO_OUT);
    gpio_put(PIN_SDQ, 0);

    sleep_us(200);

    gpio_set_dir(PIN_SDQ, GPIO_IN);

    sleep_us(50);
}

void tamarin_reset_tristar(PIO pio, uint sm) {
    leave_dcsd();
    wantLeaveJtag = true;
    while (wantLeaveJtag);

    lightning_send_wake();

    configure_rx(pio, sm);
}

unsigned char reverse_byte(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

enum state {
    RESTART_ENUMERATION,
    WAITING_FOR_INIT,
    READING_TRISTAR_REQUEST,
    HANDLE_TRISTAR_REQUEST,
    READING_POWER_REQUEST,
    HANDLE_POWER_REQUEST,
    READING_TRISTAR_UNKNOWN_76,
    HANDLE_TRISTAR_UNKNOWN_76,
    HANDLE_JTAG,
    // Force JTAG mode if the cable is already in JTAG mode
    // (e.g. after the tamarin cable was reset but not the device)
    FORCE_JTAG,
    FORCE_SPAM
};

volatile enum state gState = RESTART_ENUMERATION;

enum command {
    CMD_DEFAULT,
    CMD_RESET,
    CMD_AUTO_DFU,
    // Used as 'second stage' for the DFU command. Saves us a state machine.
    CMD_INTERNAL_AUTO_DFU_2,
    CMD_MAX,
};

enum default_command {
    DEFAULT_CMD_DCSD = 0,
    DEFAULT_CMD_JTAG,
    DEFAULT_CMD_SPAM,
    DEFAULT_CMD_CHARGING
};

// Command that should be sent automatically (on reboot, plugin, etc.)
// Automatically changed when selecting DCSD/JTAG
enum default_command gDefaultCommand = DEFAULT_CMD_DCSD;

// Next command to send
enum command gCommand = CMD_DEFAULT;

enum responses {
    // JTAG mode
    RSP_USB_UART_JTAG,
    // JTAG mode
    RSP_USB_SPAM_JTAG,
    // DCSD mode
    RSP_USB_UART,
    // Reset
    RSP_RESET,
    // DFU
    RSP_DFU,
    RSP_USB_A_CHARGING_CABLE,
    RSP_MAX
};

enum TRISTAR_REQUESTS {
    TRISTAR_POLL = 0x74,
    TRISTAR_POWER = 0x70,
    TRISTAR_UNKNOWN_76 = 0x76,
};

// To generate CRCs:
// hex(pwnlib.util.crc.generic_crc(b"\x75\x00\x00\x02\x00\x00\x00", 0x31, 8, 0xff, True, True, False))
const uint8_t bootloader_response[RSP_MAX][7] = {
    [RSP_USB_UART_JTAG] = {0x75, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00},
    [RSP_USB_SPAM_JTAG] = {0x75, 0xa0, 0x08, 0x10, 0x00, 0x00, 0x00},
    [RSP_USB_UART] = {0x75, 0x20, 0x00, 0x10, 0x00, 0x00, 0x00},
    [RSP_RESET] = {0x75, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00},
    [RSP_DFU] = {0x75, 0x20, 0x00, 0x02, 0x00, 0x00, 0x00},
    [RSP_USB_A_CHARGING_CABLE] = {0x75, 0x10, 0x0c, 0x00, 0x00, 0x00, 0x00},
};

void set_idbus_high_impedance() {
    gpio_pull_up (PIN_SDQ);
    gpio_init(PIN_SDQ);
    gpio_set_dir(PIN_SDQ, GPIO_IN);
}

#define DCSD_UART uart0
#define DCSD_TX_PIN 0
#define DCSD_RX_PIN 1

void dcsd_mode(PIO pio, uint sm) {
    uart_init(uart0, 115200);
    gpio_set_function(DCSD_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DCSD_RX_PIN, GPIO_FUNC_UART);

    serprint("DCSD mode active.\r\n");
    serprint("Connect to the second serial port of the\r\n");
    serprint("Tamarin Cable to access the monitor.\r\n");

    serialEnabled = true;
}

void jtag_mode(PIO pio, uint sm) {
    set_idbus_high_impedance();
    pio_sm_set_enabled(pio, sm, false);
    serprint("JTAG mode active, ID pin in Hi-Z.\r\n");
    serprint("You can now connect with an SWD debugger.\r\n");

    jtagInited  = false;
    jtagEnabled = true;
}

uint8_t crc_data(const uint8_t *data, size_t len) {
    crc_t crc = crc_init();
    crc = crc_update(crc, data, len);
    crc = crc_finalize(crc);
    return crc;
}

void respond_lightning(PIO pio, uint sm, const uint8_t *data, size_t data_length) {
    // Static buffer
    static uint8_t response_buffer[64];
    if(data_length > 63) {
        serprint("Lightning response too large, not sending.\r\n");
        return;
    }
    memcpy(response_buffer, data, data_length);
    response_buffer[data_length] = crc_data(data, data_length);

    pio_sm_set_enabled(pio, sm, false);
    pio_clear_instruction_memory(pio);
    uint offset = pio_add_program(pio, &lightning_tx_program);
    pio_sm_config c = lightning_tx_program_init(pio, sm, offset, PIN_SDQ, 125.0/2.0);
    // We fill in 1 byte first, then the second byte, only then do we start to do
    // "get_blocking" to avoid the state machine ever running empty.
    pio_sm_put_blocking(pio, sm, response_buffer[0]);
    for(size_t i=1; i < data_length+1; i++) {
        pio_sm_put_blocking(pio, sm, response_buffer[i]);
        pio_sm_get_blocking(pio, sm);
    }
    pio_sm_get_blocking(pio, sm);
}

void output_state_machine() {
    char DFU[8];
    PIO pio = pio1;
    uint sm = pio_claim_unused_sm(pio, true);

    uint8_t i = 0;
    uint8_t buf[4];

    uint32_t value, value_b;
    while(1) {
        switch(gState) {
            case RESTART_ENUMERATION:
                serprint("Restarting enumeration!\r\n");
                tamarin_reset_tristar(pio, sm);
                serprint("Done restarting enumeration!\r\n");
                // led_blink(3, 100);
                gState = WAITING_FOR_INIT;
                break;

            case WAITING_FOR_INIT:
                if (pio_sm_is_rx_fifo_empty(pio, sm)) break;
                serprint("State: WAITING_FOR_INIT\r\n");
                value = pio_sm_get_blocking(pio, sm);
                value_b = reverse_byte(value & 0xFF);
                if(value_b == TRISTAR_POLL) {
                    leave_dcsd();
                    gState = READING_TRISTAR_REQUEST;
                    buf[0] = value_b;
                    i = 1;
                } else if(value_b == TRISTAR_POWER) {
                    gState = READING_POWER_REQUEST;
                    buf[0] = value_b;
                    i = 1;
                } else if(value_b == TRISTAR_UNKNOWN_76) {
                    gState = READING_TRISTAR_UNKNOWN_76;
                    buf[0] = value_b;
                    i = 1;
                } else {
                    serprint("Tristar >> 0x%x (unknown, ignoring)\r\n", value_b);
                }
                // led_blink(1, 100);
                sleep_us(100); // Breaks without this...
                break;
            case READING_TRISTAR_REQUEST:
                if (pio_sm_is_rx_fifo_empty(pio, sm)) break;
                serprint("State: READING_TRISTAR_REQUEST\r\n");
                value = pio_sm_get_blocking(pio, sm);
                value_b = reverse_byte(value & 0xFF);
                // led_blink(1, 100);
                buf[i++] = value_b;
                if(i == 4) {
                    gState = HANDLE_TRISTAR_REQUEST;
                    i = 0;
                }
                sleep_us(10); // Breaks without this...
                break;
            case READING_TRISTAR_UNKNOWN_76:
                if (pio_sm_is_rx_fifo_empty(pio, sm)) break;
                serprint("State: READING_TRISTAR_UNKNOWN_76\r\n");
                value = pio_sm_get_blocking(pio, sm);
                value_b = reverse_byte(value & 0xFF);

                buf[i++] = value_b;
                if(i == 2) {
                    gState = HANDLE_TRISTAR_UNKNOWN_76;
                    i = 0;
                }
                // led_blink(1, 100);
                sleep_us(10); // Breaks without this...
                break;
            case HANDLE_TRISTAR_UNKNOWN_76:
                respond_lightning(pio, sm, "\x77\x02\x01\x02\x80\x60\x01\x39\x3a\x44\x3e\xc9", 11);
                serprint("76 request received: %02X %02X\r\n", buf[0], buf[1]);
                gState = WAITING_FOR_INIT;
                break;
            case READING_POWER_REQUEST:
                if (pio_sm_is_rx_fifo_empty(pio, sm)) break;
                serprint("State: READING_POWER_REQUEST\r\n");
                value = pio_sm_get_blocking(pio, sm);
                value_b = reverse_byte(value & 0xFF);

                buf[i++] = value_b;
                if(i == 4) {
                    gState = HANDLE_POWER_REQUEST;
                    i = 0;
                }

                sleep_us(10); // Breaks without this...

                break;
            case HANDLE_POWER_REQUEST:
                respond_lightning(pio, sm, "\x71\x93", 1);
                serprint("Power request received: %02X %02X %02X %02X\r\n", buf[0], buf[1], buf[2], buf[3]);
                gState = WAITING_FOR_INIT;
                break;
            case HANDLE_TRISTAR_REQUEST:
                serprint("Tristar request received: %02X %02X %02X %02X\r\n", buf[0], buf[1], buf[2], buf[3]);
                switch(gCommand) {
                    case CMD_DEFAULT:
                        switch (gDefaultCommand) {
                            case DEFAULT_CMD_DCSD:
                                respond_lightning(pio, sm, bootloader_response[RSP_USB_UART], 7);
                                dcsd_mode(pio, sm);
                                break;

                            case DEFAULT_CMD_JTAG:
                                respond_lightning(pio, sm, bootloader_response[RSP_USB_UART_JTAG], 7);
                                gState = FORCE_JTAG;
                                continue;

                            case DEFAULT_CMD_SPAM:
                                respond_lightning(pio, sm, bootloader_response[RSP_USB_SPAM_JTAG], 7);
                                gState = FORCE_SPAM;
                                continue;

                            case DEFAULT_CMD_CHARGING:
                                respond_lightning(pio, sm, bootloader_response[RSP_USB_A_CHARGING_CABLE], 7);
                                serprint("Sent charging\r\n");
                                gState = WAITING_FOR_INIT;
                                break;
                        }
                        break;
                    case CMD_RESET:
                        respond_lightning(pio, sm, bootloader_response[RSP_RESET], 7);
                        sleep_us(1000);
                        gCommand = CMD_DEFAULT;
                        break;
                    case CMD_AUTO_DFU:
                        respond_lightning(pio, sm, bootloader_response[RSP_RESET], 7);
                        gCommand = CMD_INTERNAL_AUTO_DFU_2;
                        break;
                    case CMD_INTERNAL_AUTO_DFU_2:
                        respond_lightning(pio, sm, bootloader_response[RSP_DFU], 7);
                        serprint("Device should now be in DFU mode.\r\n");
                        gCommand = CMD_DEFAULT;
                        break;
                    default:
                        serprint("UNKNOWN MODE. Send help. Locking up.\r\n");
                        while(1) {}
                        break;
                }
                // led_blink(1, 50);
                gState = WAITING_FOR_INIT;
                configure_rx(pio, sm);
                break;

            case HANDLE_JTAG:
                // Nothing to do
                break;

            case FORCE_JTAG:
                // led_blink(2, 50);
                dcsd_mode(pio, sm); // Also init serial
            case FORCE_SPAM:
                // led_blink(2, 50);
                jtag_mode(pio, sm);
                gState = HANDLE_JTAG;
                break;
        }
    }
}

void print_menu() {
    clear_screen();
    serprint("Good morning!\r\n\r\n");
    serprint("1: JTAG mode\r\n");
    serprint("2: DCSD mode\r\n");
    serprint("3: Reset device\r\n");
    serprint("4: Reset and enter DFU mode (iPhone X and up only)\r\n");
    serprint("5: Reenumerate\r\n\r\n");
    serprint("C: Charging cable mode\r\n");
    serprint("F: Force JTAG mode without sending command\r\n");
    serprint("J: Force SPAM-JTAG mode without sending command\r\n");
    serprint("R: Reset Tamarin cable\r\n");
    serprint("S: SPAM mode (Apple Watch UART)\r\n");
    serprint("U: Go into firmware update mode\r\n");
    serprint("> ");
}

void menu_action(char c) {
    switch(c) {
        case '1':
            serprint("\r\nEnabling JTAG mode.\r\n");
            gCommand = CMD_DEFAULT;
            gDefaultCommand = DEFAULT_CMD_JTAG;
            gState = RESTART_ENUMERATION;
            break;
        case '2':
            serprint("\r\nEnabling DCSD mode.\r\n");
            gCommand = CMD_DEFAULT;
            gDefaultCommand = DEFAULT_CMD_DCSD;
            gState = RESTART_ENUMERATION;
            break;
        case '3':
            serprint("\r\nResetting.\r\n");
            gCommand = CMD_RESET;
            gState = RESTART_ENUMERATION;
            break;
        case '4':
            serprint("\r\nEnabling DFU mode.\r\n");
            gCommand = CMD_AUTO_DFU;
            gState = RESTART_ENUMERATION;
            break;
        case '5':
            serprint("\r\nReenumerate\r\n");
            gCommand = CMD_DEFAULT;
            gState = RESTART_ENUMERATION;
            break;
        case 'c':
        case 'C':
            // Charging
            serprint("\r\nEnabling charging mode.\r\n");
            gCommand = CMD_DEFAULT;
            gDefaultCommand = DEFAULT_CMD_CHARGING;
            gState = RESTART_ENUMERATION;
            break;
        case 'f':
        case 'F':
            serprint("\r\nForcing JTAG mode.\r\n");
            gDefaultCommand = DEFAULT_CMD_JTAG;
            gState = FORCE_JTAG;
            break;
        case 'j':
        case 'J':
            serprint("\r\nForcing SPAM mode.\r\n");
            gDefaultCommand = DEFAULT_CMD_SPAM;
            gState = FORCE_SPAM;
            break;
        case 'R':
        case 'r':
            watchdog_enable(100, 1);
            break;
        case 's':
        case 'S':
            serprint("\r\nEnabling SPAM mode.\r\n");
            gCommand = CMD_DEFAULT;
            gDefaultCommand = DEFAULT_CMD_SPAM;
            gState = RESTART_ENUMERATION;
            break;
        case 'U':
        case 'u':
            reset_usb_boot(0, 0);
        default:
            print_menu();
            break;
    }
}

void shell_task() {
    // CDC interface
    if (tud_cdc_n_available(ITF_CONSOLE)) {
        char c = tud_cdc_n_read_char(ITF_CONSOLE);
        tud_cdc_n_write_char(ITF_CONSOLE, c);
        menu_action(c);
    }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  (void)itf;
  if (itf != ITF_CONSOLE) return;

  // connected
  if (dtr && rts) {
    // print initial message when connected
    tud_cdc_n_write_str(ITF_CONSOLE, "\r\n### Tamarin Cable ###\r\n");
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf) {
  (void)itf;
}


//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}


//--------------------------------------------------------------------+
// WebUSB use vendor class
//--------------------------------------------------------------------+

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
  // nothing to with DATA & ACK stage
  if (stage != CONTROL_STAGE_SETUP) return true;

  switch (request->bmRequestType_bit.type) {
    case TUSB_REQ_TYPE_VENDOR:
      switch (request->bRequest) {
        case VENDOR_REQUEST_WEBUSB:
          // match vendor request in BOS descriptor
          // Get landing page url
          return tud_control_xfer(rhport, request, (void*)(uintptr_t)&desc_url, desc_url.bLength);

        case VENDOR_REQUEST_MICROSOFT:
          if (request->wIndex == 7) {
            // Get Microsoft OS 2.0 compatible descriptor
            uint16_t total_len;
            memcpy(&total_len, desc_ms_os_20 + 8, 2);

            return tud_control_xfer(rhport, request, (void*)(uintptr_t)desc_ms_os_20, total_len);
          } else {
            return false;
          }

        default: break;
      }
      break;

    case TUSB_REQ_TYPE_CLASS:
      if (request->bRequest == 0x22) {
        // Webserial simulate the CDC_REQUEST_SET_CONTROL_LINE_STATE (0x22) to connect and disconnect.
        web_serial_connected = (request->wValue != 0);

        // Always lit LED if connected
        if (web_serial_connected) {
          board_led_write(true);
          blink_interval_ms = BLINK_ALWAYS_ON;

          tud_vendor_write_str("\r\nWebUSB interface connected\r\n");
          tud_vendor_write_flush();
        } else {
          blink_interval_ms = BLINK_MOUNTED;
        }

        // response with status OK
        return tud_control_status(rhport, request);
      }
      break;

    default: break;
  }

  // stall unknown request
  return false;
}

void tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint16_t bufsize) {
  (void) itf;
  // echo_all(buffer, bufsize);

  char c = buffer[0];
  if (c == '\r' || c == '\n') {
    tud_vendor_write_str("\r\n");
  } else {
    tud_vendor_write(buffer, bufsize);
  }
  menu_action(c);

  // if using RX buffered is enabled, we need to flush the buffer to make room for new data
  #if CFG_TUD_VENDOR_RX_BUFSIZE > 0
  tud_vendor_read_flush();
  #endif
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if (board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}

int buttonCNT = 0;

int main() {
    // Power enable for Tamarin Cable. Not required on regular Pico,
    // but also doesn't hurt.
    gpio_init(19);
    gpio_set_dir(19, GPIO_OUT);
    gpio_put(19, 1);

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);

    board_init();
    tusb_init();
    if (board_init_after_tusb) {
        board_init_after_tusb();
    }
    multicore_launch_core1(output_state_machine);

    while(1) {
        tud_task();
        shell_task();
        led_blinking_task();

        // Handle serial
        if (serialEnabled) {
            if (uart_is_readable(uart0)) {
                tud_cdc_n_write_char(ITF_DCSD, uart_getc(uart0));
                tud_cdc_n_write_flush(ITF_DCSD);
            }

            if (tud_cdc_n_available(ITF_DCSD)) {
                uart_putc_raw(uart0, tud_cdc_n_read_char(ITF_DCSD));
            }
        }

        tud_task();
        tud_vendor_flush();
        tud_task();
        if (jtagEnabled) {
            if (!jtagInited) {
                tamarin_probe_init();
                tud_task();
                jtagInited = true;
            }

            tamarin_probe_task(!serialEnabled);
        }
        tud_task();
        tud_vendor_flush();
        tud_task();
        if (wantLeaveJtag){
            leave_jtag();
        }
    }
}