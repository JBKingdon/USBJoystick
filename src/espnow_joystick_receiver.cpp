/**
 * Joystick 
 *
 * Next:
 *   Test unicast vs broadcast
 *   Add a nonce to the OTA and get stats on dropped/late packets
 *   Create a GIT repo
 *   Refactor structs into header file
 *   Create a user config file?
 *   Add a webserver for config?
*/

// #define EJ_TRANSMITTER
#define EJ_RECEIVER

#define ENABLE_USB_HID

#define ENABLE_USB_CDC

#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "class/cdc/cdc_device.h"
#include "tusb_cdc_acm.h"

#include "driver/gpio.h"

#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

#include "sdkconfig.h"

#include "esp_wifi.h"
#include "esp_mac.h"

#include "nvs_flash.h"
#include <esp_now.h>
#include "esp_timer.h"

#include "crc.h"

// The CRC code uses the UID from ELRS, so need it to match the transmitter
// TODO keep this or get rid of it?
uint8_t UID[6] = {MY_UID};

#define ELRS_CRC14_POLY 0x2E57 // 0x372B
static GENERIC_CRC14 ota_crc(ELRS_CRC14_POLY);


// RGB LED stuff
// #define RMT_LED_STRIP_GPIO_NUM      GPIO_NUM_48

#ifdef RMT_LED_STRIP_GPIO_NUM
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

#define EXAMPLE_LED_NUMBERS         1
#define EXAMPLE_CHASE_SPEED_MS      10

static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];

rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;
#endif // RMT_LED_STRIP_GPIO_NUM


#ifdef EJ_RECEIVER

// Variables to track connectedness
bool isConnected = false;
uint32_t tLastRX = 0;

#endif


// ESP NOW STUFF

#define CHANNEL 1

// interval in ms for usb updates
// try reducing the rate to 125hz to see if we're overwhelming Windows/velocidrone
#define USB_POLLING_INTERVAL 8
// #define USB_POLLING_INTERVAL 4

enum Messages
{
    PING,
    ECHO
};

static const char *TAG = "Joystick";

/**
 * Write directly to the CDC usb serial output stream
 * 
 * TOOD replace this by remapping stdout/stderr
*/
void debugWrite(const char *str)
{
    #ifndef ENABLE_USB_CDC
    return;
    #endif

    const int len = strlen(str);

    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (uint8_t *)str, len);
    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 10);
}


static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read((tinyusb_cdcacm_itf_t)itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Data from channel %d:", itf);
        ESP_LOG_BUFFER_HEXDUMP(TAG, buf, rx_size, ESP_LOG_INFO);
    } else {
        ESP_LOGE(TAG, "Read error");
    }

    /* write back */
    // tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)itf, buf, rx_size);
    // tinyusb_cdcacm_write_flush((tinyusb_cdcacm_itf_t)itf, 0);
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
}




// Joystick Report Descriptor Template
// with 8 buttons, 2 joysticks

#ifdef NOSUCHDEFINE
// 8 bit version
// | X | Y | Rx | Ry (1 byte each) | Button Map (1 byte) |
#define TUD_HID_REPORT_DESC_JOYSTICK(...) \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )                 ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_JOYSTICK )                 ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )                 ,\
    /* Report ID if any */\
    __VA_ARGS__ \
    /* 8 bit X, Y, Z, Rz, Rx, Ry (min -127, max 127 ) */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_X                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_Y                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RX                   ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RY                   ) ,\
    HID_LOGICAL_MIN    ( 0x81                                   ) ,\
    HID_LOGICAL_MAX    ( 0x7f                                   ) ,\
    HID_REPORT_COUNT   ( 4                                      ) ,\
    HID_REPORT_SIZE    ( 8                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    /* 8 bit Button Map */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_BUTTON                  ) ,\
    HID_USAGE_MIN      ( 1                                      ) ,\
    HID_USAGE_MAX      ( 8                                      ) ,\
    HID_LOGICAL_MIN    ( 0                                      ) ,\
    HID_LOGICAL_MAX    ( 1                                      ) ,\
    HID_REPORT_COUNT   ( 8                                      ) ,\
    HID_REPORT_SIZE    ( 1                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
  HID_COLLECTION_END

// TODO if this works, see if we can increase the resolution
typedef struct TU_ATTR_PACKED
{
  int8_t  x;         
  int8_t  y;         
  int8_t  rx;        
  int8_t  ry;        
  uint8_t buttons;
} hid_joystick_report_t;

#endif


// 16 bit version - which endianess?
// | X | Y | Rx | Ry (2 bytes each) | Button Map (1 byte) |
#define TUD_HID_REPORT_DESC_JOYSTICK(...) \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )                 ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_JOYSTICK )                 ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )                 ,\
    /* Report ID if any */\
    __VA_ARGS__ \
    /* 16 bit X, Y, Z, Rz, Rx, Ry (min 0, max 2047 - replace with CRSF range or scale to +/-?) */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_X                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_Y                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RX                   ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RY                   ) ,\
    HID_LOGICAL_MIN    ( 172                                    ) ,\
    HID_LOGICAL_MAX_N  ( 1811, 2                                ) ,\
    HID_REPORT_COUNT   ( 4                                      ) ,\
    HID_REPORT_SIZE    ( 16                                     ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    /* 8 bit Button Map */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_BUTTON                  ) ,\
    HID_USAGE_MIN      ( 1                                      ) ,\
    HID_USAGE_MAX      ( 8                                      ) ,\
    HID_LOGICAL_MIN    ( 0                                      ) ,\
    HID_LOGICAL_MAX    ( 1                                      ) ,\
    HID_REPORT_COUNT   ( 8                                      ) ,\
    HID_REPORT_SIZE    ( 1                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
  HID_COLLECTION_END \

// struct for passing the data over usb
typedef struct TU_ATTR_PACKED
{
  uint16_t  x;
  uint16_t  y;
  uint16_t  rx;
  uint16_t  ry;
  uint8_t buttons;
} hid_joystick_report_t;

// struct for passing the data over esp_now
typedef struct TU_ATTR_PACKED
{
    hid_joystick_report_t report;
    uint16_t crc;
} hid_joystick_ota_t;

#define HID_REPORT_LENGTH sizeof(hid_joystick_report_t)

uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

esp_now_peer_info_t peerInfo;

#define CRSF_CHANNEL_VALUE_MID  992

hid_joystick_report_t joystickData =
{
    .x = CRSF_CHANNEL_VALUE_MID,
    .y = CRSF_CHANNEL_VALUE_MID,
    .rx = CRSF_CHANNEL_VALUE_MID,
    .ry = CRSF_CHANNEL_VALUE_MID,
    .buttons = 0
};

// uint8_t rxData[ESP_NOW_MAX_DATA_LEN], txData[ESP_NOW_MAX_DATA_LEN];
// uint8_t rxLen = 0;

// local mac address
uint8_t mac[8]; // mac is probably 6 bytes, but might be 8

uint64_t txTime = 0;

TaskHandle_t rxTaskHandle;

uint8_t const desc_hid_report[] =
{
    // TUD_HID_REPORT_DESC_GAMEPAD()
    TUD_HID_REPORT_DESC_JOYSTICK()
};


#ifdef EJ_RECEIVER

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)
static const uint8_t hid_configuration_descriptor[] = 
{
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    // XXX what is the size parameter the size of?
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(desc_hid_report), 0x81, 16, USB_POLLING_INTERVAL),
};

#define TUSB_COMBINED_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_CDC_DESC_LEN)
#define EPNUM_0_CDC_NOTIF 1
#define EPNUM_0_CDC 2

// define string descriptors to include HID
tusb_desc_strarray_device_t descriptor_strings = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},                // 0: is supported language is English (0x0409)
    // CONFIG_TINYUSB_DESC_MANUFACTURER_STRING, // 1: Manufacturer
    "JBK Test Manu",
    CONFIG_TINYUSB_DESC_PRODUCT_STRING,      // 2: Product
    CONFIG_TINYUSB_DESC_SERIAL_STRING,       // 3: Serials, should use chip ID

    // The next two seem to be ignored
    CONFIG_TINYUSB_DESC_CDC_STRING,          // 4: CDC Interface
    "ELRS Joystick Device",
    "",
    ""
};

static const uint8_t combined_configuration_descriptor[] = 
{
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 3, 0, TUSB_COMBINED_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 5, false, sizeof(desc_hid_report), 0x85, 16, USB_POLLING_INTERVAL),


    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(1, 4, 0x80 | EPNUM_0_CDC_NOTIF, 8, EPNUM_0_CDC, 0x80 | EPNUM_0_CDC, CFG_TUD_CDC_EP_BUFSIZE),
    
};


#endif // EJ_RECEIVER

/**
 * Send the joystick data over usb
 * 
 * When does this get used?
*/
bool sendJoystickReport(hid_joystick_report_t *report)
{
    const uint8_t instance = 0;
    const uint8_t report_id = 0;

    return tud_hid_n_report(instance, report_id, report, sizeof(hid_joystick_report_t));
}


uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    return desc_hid_report;
}

// TODO needed?
// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    printf("tud_hid_get_report_cb called\n");

    return 0;
}

// TODO needed?
// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    printf("tud_hid_set_report_cb called\n");
}


#ifdef EJ_TRANSMITTER
/**
 * Generate synthetic joystick data
 * 
 * NB This generates 8 bit data
*/
static void dataSourceTestTask(void *pvParameter)
{
    uint8_t c = 0;    // 8 bits to allow implicit wrapping to handle range of xy axis

    // Give the system time to start up before we start spamming test data
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    TickType_t lastWakeTime = xTaskGetTickCount();

    while(true)
    {
        xTaskDelayUntil(&lastWakeTime, USB_POLLING_INTERVAL / portTICK_PERIOD_MS);

        // generate synthetic data

        int8_t x, y, rx, ry;
        uint8_t buttons;

        x = c - 128;
        y = 127 - c;

        rx = x;
        ry = y;

        buttons = 1 << ((c/32) % 8);

        c++;

        joystickData.x = x;
        joystickData.y = y;
        joystickData.rx = rx;
        joystickData.ry = ry;
        joystickData.buttons = buttons;

        esp_now_send(broadcastAddress, (uint8_t *)&joystickData, sizeof(joystickData));
    }

}
#endif // EJ_TRANSMITTER


#ifdef EJ_RECEIVER
/**
 * Send the joystick data over usb to the host
*/
static void usbUpdateTask(void *pvParameter)
{
    // uint16_t c = 0;

    // Give the system time to start up before we start sending joystick data
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    TickType_t lastWakeTime = xTaskGetTickCount();

    while(true)
    {
        xTaskDelayUntil(&lastWakeTime, USB_POLLING_INTERVAL / portTICK_PERIOD_MS);

        // joystickData.x = c;
        // c++;
        // if (c > 2047) c = 0;

        // // XXX Test - toggle one of the buttons on every update
        // joystickData.buttons ^= 1 << 4;

        // Can only send the data when the usb is properly connected and configured
        if (tud_mounted())
        {
            sendJoystickReport(&joystickData);
            // debugWrite("J");
        } else {
            // debugWrite("U");
        }
    }
}

#endif // EJ_RECEIVER

/**
 * Test code for measuring round trip times
*/
// static void receiverTask(void *pvParameter)
// {
//     while(true)
//     {
//         xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

//         // get the time in microseconds
//         uint64_t rxTime = esp_timer_get_time();

//         // printf("rxTask notified\n");

//         if (rxLen > 0) {
//             if (rxLen == 1) 
//             {
//                 if (rxData[0] == PING) {
//                     uint8_t data[] = {ECHO};
//                     esp_now_send(broadcastAddress, data, 1);
//                     printf("Sending ECHO\n");
//                 }  else if (rxData[0] == ECHO) {
//                     uint32_t rtt = (rxTime-txTime)/1000;
//                     printf("Received ECHO, trip time %lums\n", rtt);
//                 }
//             }
//             printf("Received packet: ");
//             for(int i=0; i<rxLen; i++) {
//                 printf("%d ", rxData[i]);
//             }
//             printf("\n");

//         }

//     }
// }


int sendCalls = 0, rcvCalls = 0;

/**
 * This gets called once per sent message, but exactly when?
*/
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    sendCalls++;
}

/**
 * Invoked by esp-now after a packet is received.
 * 
 * Is this called in an ISR context?
*/
static void joystick_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    uint64_t rxTime = esp_timer_get_time();
    uint32_t rxTimeMs = rxTime / 1000;

    // Need to set the led colour for connected?

    if (!isConnected && ((rxTimeMs - tLastRX) < 1000)) {
        isConnected = true;
        #ifdef RMT_LED_STRIP_GPIO_NUM
        // for rgb led
        rmt_transmit_config_t tx_config;
        memset(&tx_config, 0, sizeof(tx_config));
        tx_config.loop_count = 0; // no transfer loop
        // set the rgb led to green (order is green, blue, red)
        led_strip_pixels[0] = 20;
        led_strip_pixels[1] = 0;
        led_strip_pixels[2] = 0;

        // Flush RGB values to LEDs
        rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config);
        #endif // RMT_LED_STRIP_GPIO_NUM
    }

    tLastRX = rxTimeMs;

    rcvCalls++;

    // validate the data and copy into local buffer
    // TODO We could check the address of the sender and implement a simple binding model (without the ack/retry that comes with unicast)
    if (len == sizeof(hid_joystick_ota_t))
    {
        hid_joystick_ota_t *rcvdData = (hid_joystick_ota_t *)data;

        // Calculate the expected crc, HID_REPORT_LENGTH is the length of the payload, not including the crc
        uint16_t expectedCrc = ota_crc.calc((uint8_t*)data, HID_REPORT_LENGTH);

        if (expectedCrc == rcvdData->crc)
        {
            // filter out single event glitches in the buttons
            static uint8_t previous_requested_buttons = 0;

            const uint8_t requested_buttons = rcvdData->report.buttons;

            const uint8_t changed_buttons = previous_requested_buttons ^ requested_buttons;

            // buttons that change from request to request are ignored, those that are consistent with the previous
            // request are used

            const uint8_t currentButtons = joystickData.buttons;

            uint8_t newButtons = currentButtons & changed_buttons;
            newButtons |= requested_buttons & ~changed_buttons;

            previous_requested_buttons = requested_buttons;
            
            memcpy(&joystickData, data, HID_REPORT_LENGTH);

            joystickData.buttons = newButtons;

            // printf("good packet x is %u\n", joystickData.x);
            // debugWrite(".");
        } else {
            printf("crc mismatch %u vs %u\n", expectedCrc, rcvdData->crc);
            debugWrite("C");
        }
    } else {
        printf("bad len %u vs %u\n", len, sizeof(hid_joystick_report_t));
        debugWrite("L");
    }

    // Test code for measuring round trip times without using a task
    // if (data[0] == PING) {
    //     txData[0] = ECHO;
    //     esp_now_send(broadcastAddress, txData, 5);
    //     // printf("Sending ECHO\n");
    // }  else if (data[0] == ECHO) {
    //     // uint32_t rtt = (rxTime-txTime)/1000;
    //     uint32_t rtt = (rxTime-txTime);
    //     printf("Received ECHO, trip time %lu us\n", rtt);
    // }

    // Original code using a task to handle sending to usb
    // if (len > 16) {
    //     printf("rx data too long %d\n", len);
    //     return;
    // }

    // memcpy(rxData, data, len);
    // rxLen = len;

    // // TODO - can we do the check to see if the task has been woken and jump straight to it?
    // BaseType_t taskWoken = pdFALSE;
    // xTaskNotifyFromISR(rxTaskHandle, 0, eSetValueWithOverwrite, &taskWoken);

    // if (taskWoken) {
    //     portYIELD_FROM_ISR();
    // }
}


static esp_err_t joystick_espnow_init(void)
{
    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(joystick_recv_cb) );

    /* Set primary master key. */
    // ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = (esp_now_peer_info_t*)malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        // vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CHANNEL;
    peer->ifidx = WIFI_IF_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcastAddress, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    // set the mode and rate

    esp_now_rate_config_t peerConfig;

    // WIFI_PHY_MODE_LR,   /**< PHY mode for Low Rate */
    // WIFI_PHY_MODE_11B,  /**< PHY mode for 11b */
    // WIFI_PHY_MODE_11G,  /**< PHY mode for 11g */
    // WIFI_PHY_MODE_HT20, /**< PHY mode for Bandwidth HT20 */
    // WIFI_PHY_MODE_HT40, /**< PHY mode for Bandwidth HT40 */
    // WIFI_PHY_MODE_HE20, /**< PHY mode for Bandwidth HE20 */

    peerConfig.phymode = WIFI_PHY_MODE_HT20;    // needs a coordinated change with secondChannel to use HT40. HE20 is refused

    // rates are MCS0 through MCS7. Long or short guard interval LGI vs SGI
    peerConfig.rate = WIFI_PHY_RATE_MCS0_LGI;
    // peerConfig.rate = WIFI_PHY_RATE_MCS4_LGI;

    peerConfig.ersu = false;
    ESP_ERROR_CHECK( esp_now_set_peer_rate_config(broadcastAddress, &peerConfig) );

    return ESP_OK;
}


// app_main needs to use C binding

extern "C"
{

void app_main(void) 
{
    // Setup gpio for LED

    // TODO make conditional and add a new config for the usb dongle which doesn't have a gpio led

    gpio_config_t ioConfig{
        .pin_bit_mask = 1 << GPIO_NUM_15,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&ioConfig);

    // Set led on
    gpio_set_level(GPIO_NUM_15, 1);


    // uint32_t red = 0;
    // uint32_t green = 0;
    // uint32_t blue = 0;
    // uint16_t hue = 0;
    // uint16_t start_rgb = 0;

    // No rgb on the S2 mini
    #ifdef RMT_LED_STRIP_GPIO_NUM

    printf("Init RGB\n");
    rmt_tx_channel_config_t tx_chan_config;
    memset(&tx_chan_config, 0, sizeof(tx_chan_config));
    tx_chan_config.gpio_num = RMT_LED_STRIP_GPIO_NUM;
    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT; // select source clock
    tx_chan_config.resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ;
    tx_chan_config.mem_block_symbols = 64; // increase the block size can make the LED less flickering
    tx_chan_config.trans_queue_depth = 4; // set the number of transactions that can be pending in the background

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_ERROR_CHECK(rmt_enable(led_chan));

    rmt_transmit_config_t tx_config;
    memset(&tx_config, 0, sizeof(tx_config));
    // Nothing specific to set

    // hue = 360 + start_rgb;
    // led_strip_hsv2rgb(hue, 80, 15, &red, &green, &blue);
    // Pixel colour order is: green, red, blue
    led_strip_pixels[0] = 0;
    led_strip_pixels[1] = 20;
    led_strip_pixels[2] = 0;

    // Flush RGB values to LEDs
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));

    vTaskDelay(500 / portTICK_PERIOD_MS);

    led_strip_pixels[0] = 20;
    led_strip_pixels[1] = 0;
    led_strip_pixels[2] = 0;

    // Flush RGB values to LEDs
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));

    vTaskDelay(500 / portTICK_PERIOD_MS);

    led_strip_pixels[0] = 0;
    led_strip_pixels[1] = 0;
    led_strip_pixels[2] = 20;

    // Flush RGB values to LEDs
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    #endif // RMT_LED_STRIP_GPIO_NUM

    #ifdef EJ_RECEIVER

    #ifdef ENABLE_USB_HID

    ESP_LOGI(TAG, "USB initialization");
    tinyusb_config_t tusb_cfg;
    memset(&tusb_cfg, 0, sizeof(tusb_cfg));

    #ifdef ENABLE_USB_CDC
    tusb_cfg.configuration_descriptor = combined_configuration_descriptor;
    #else
    tusb_cfg.configuration_descriptor = hid_configuration_descriptor;
    #endif

    tusb_cfg.string_descriptor = descriptor_strings;
    
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    #ifdef ENABLE_USB_CDC

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_0,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    #endif // ENABLE_USB_CDC

    ESP_LOGI(TAG, "USB initialization DONE");

    debugWrite("USB up!\n");


    // Start the rx task (not used, all receive code now running from the esp now callback)
    // xTaskCreate(receiverTask, "receiverTask", 2048, NULL, 5, &rxTaskHandle);

    xTaskCreate(usbUpdateTask, "usbUpdateTask", 2048, NULL, 10, NULL);

    #endif // ENABLE_USB_HID

    #else   // EJ_TRANSMITTER

    // Task that creates and sends synthetic joystick data
    xTaskCreate(dataSourceTestTask, "dataTask", 2048, NULL, 10, NULL);


    #endif // EJ_RECEIVER


    // setup esp-now/wifi

    // wifi depends on nvs, so we need to init it first
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // now bring up the wifi support
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );                // station or AP?

    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE));
    
    printf("wifi init done\n");

    // I'm not currently expecting the receiver to do any transmitting, so this is a bit pedantic
    // (but I don't know if the wifi stack is broadcasting something under the covers)
    esp_wifi_set_max_tx_power(40);

    // get the mac address
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    printf("mac: ");
    for(int i=0; i<6; i++) {
        printf("%02x ", mac[i]);
    }
    printf("\n");


    // Bring up esp-now

    joystick_espnow_init();

    printf("esp-now init done\n");


    // monitor what's happening
    while(true)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // XXX where does printf go when the usb hid is running?
        // print the number of send and receive callbacks
        // printf("Send Calls: %d, Receive Calls: %d\n", sendCalls, rcvCalls);

        // debugWrite("loop active\n");


        #ifdef EJ_RECEIVER

        uint32_t now = esp_timer_get_time() / 1000;

        if (isConnected && ((now - tLastRX) > 1000)) {
            isConnected = false;

            #ifdef RMT_LED_STRIP_GPIO_NUM
            // set the rgb led to red
            led_strip_pixels[0] = 0;
            led_strip_pixels[1] = 20;
            led_strip_pixels[2] = 0;

            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            #endif // RMT_LED_STRIP_GPIO_NUM
        }

        static bool ledState = false;
        if (isConnected) {
            gpio_set_level(GPIO_NUM_15, 1);
        } else {
            gpio_set_level(GPIO_NUM_15, ledState);
            ledState = !ledState;
        }

        #endif // EJ_RECEIVER

        // // Send a 'ping'
        // // uint8_t data[] = {PING};
        // txData[0] = PING;

        // txTime = esp_timer_get_time();
        // if (esp_now_send(broadcastAddress, txData, 5) == ESP_OK) // 5 bytes is likely the needed size for the joystick
        // {
        //     uint64_t t1 = esp_timer_get_time();
        //     uint32_t dt = t1-txTime;
        //     printf("Sent packet in %lu us\n", dt);
        // } else {
        //     printf("Failed to send packet!\n");
        // }

    }

} // app_main

} // extern C
