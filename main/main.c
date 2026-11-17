#include "board.h"
#include "led.h"
#include "can_interface.h"
#include "can_manager.h"
#include "can_filter.h"
#include "dbc.h"
#include "automation_manager.h"
#include "vehicle_control.h"
#include "mcp251xfd.h"
#include "ble_server.h"
#include "ble_ota.h"
#include "ble_appchan.h"
#include "tesla_ble_adapter.h"
#include "tesla_ble_client.h"
#include "tesla_pairing.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "can_diag.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#if !defined(CONFIG_BT_NIMBLE_NVS_PERSIST) || (CONFIG_BT_NIMBLE_NVS_PERSIST != 1)
#error "CONFIG_BT_NIMBLE_NVS_PERSIST must be enabled (bonds would be RAM-only)"
#endif

#if !defined(CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE) || (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE != 1)
#error "CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE must be enabled (a crashing OTA image would brick the device)"
#endif

// Tesla BLE central client relies on the NimBLE central role (connect,
// discovery, write/indicate against the vehicle). No observer: the app
// provisions the car's address over the app channel.
#if defined(CONFIG_DASHKIT_TESLA_BLE) && \
    (!defined(CONFIG_BT_NIMBLE_ROLE_CENTRAL) || (CONFIG_BT_NIMBLE_ROLE_CENTRAL != 1))
#error "CONFIG_DASHKIT_TESLA_BLE requires CONFIG_BT_NIMBLE_ROLE_CENTRAL (set CONFIG_BT_NIMBLE_ROLE_CENTRAL=y)"
#endif

static const char *TAG = "main";

// Commit the running image after it has survived a stable boot window (OTA
// rollback: PENDING_VERIFY -> VALID, so the next reboot does not roll back).
#define MARK_VALID_DELAY_MS  10000

static void mark_app_valid_cb(void *arg)
{
    (void)arg;
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mark app valid failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "app confirmed valid (OTA rollback cleared)");
    }
}

#define BLE_BATCH_MAX_FRAMES  46   // compact frames in a 512-byte MTU
#define BLE_BATCH_TIMEOUT_MS  20
#define BLE_COMPACT_MARK      0x80 // first byte of a compact packet; the original format starts with a count < 0x80

// Compact frames carry an 11-bit id and a bus below 32. Others (the firmware's diagnostic records) keep the
// original format.
static bool fits_compact(const can_tagged_frame_t *f)
{
    return !f->frame.extended && f->frame.id <= 0x7FF && f->bus_id < 32;
}

// Packs frames [*from, count) until the buffer is full and moves *from past the ones packed (or skipped).
//   original: [count][timestamp_us LE32, bus, id LE32, len, data]...   18 bytes per 8-byte frame
//   compact:  [0x80][id | bus << 11 (LE16), len, data]...              11 bytes, no timestamp (the app never read it)
// A compact packet skips frames that don't fit the format; the caller then sends only those (rest) in the original.
static size_t build_ble_packet(const can_tagged_frame_t *frames, int count, int *from, uint8_t *buf, size_t buf_size,
                               bool compact, bool rest)
{
    size_t pos = 1;
    uint8_t n = 0;
    for (; *from < count; (*from)++) {
        const can_tagged_frame_t *f = &frames[*from];
        if (compact ? !fits_compact(f) : rest && fits_compact(f)) continue;
        uint8_t len = f->frame.dlc > 8 ? 8 : f->frame.dlc;  // DashPilot expects classic CAN (max 8)
        if (pos + (compact ? 3 : 10) + len > buf_size) break;
        if (compact) {
            uint16_t h = (uint16_t)(f->frame.id | f->bus_id << 11);
            buf[pos++] = (uint8_t)h;
            buf[pos++] = (uint8_t)(h >> 8);
        } else {
            uint32_t ts = f->timestamp_us, id = f->frame.id;
            memcpy(&buf[pos], &ts, 4);  // little-endian CPU
            buf[pos + 4] = f->bus_id;
            memcpy(&buf[pos + 5], &id, 4);
            pos += 9;
        }
        buf[pos++] = len;
        memcpy(&buf[pos], f->frame.data, len);
        pos += len;
        n++;
    }
    buf[0] = compact ? BLE_COMPACT_MARK : n;
    return n ? pos : 0;
}

// A busy link returns ESP_ERR_NO_MEM: wait for it (up to 200 ms) instead of dropping the packet. Meanwhile the CAN
// queue fills and its drops are counted, so loss shows in the diagnostics.
static uint32_t s_tx_pkts, s_tx_bytes, s_tx_waits, s_tx_dropped;

static void notify_wait(const uint8_t *buf, size_t len)
{
    if (!len) return;
    esp_err_t err;
    for (int t = 0; (err = ble_server_notify(buf, len)) == ESP_ERR_NO_MEM && t < 200; t++) {
        s_tx_waits++;
        vTaskDelay(1);
    }
    if (err == ESP_OK) { s_tx_pkts++; s_tx_bytes += len; } else s_tx_dropped++;
}

// Link records for the app's CAN log, every 5 s (no-ops while diagnostics are off).
static void link_diag(void)
{
    static int64_t last;
    int64_t now = esp_timer_get_time();
    if (now - last < 5000000) return;
    last = now;
    uint8_t d[8];
    ble_server_link_diag(d);
    can_diag_emit(0, CAN_DIAG_ID_LINK, d);
    can_diag_pack4(d, s_tx_pkts / 5, s_tx_bytes / 80, s_tx_waits, s_tx_dropped);
    can_diag_emit(0, CAN_DIAG_ID_LINK_TX, d);
    can_diag_pack4(d, ble_server_pings(), can_manager_bulk_gap_ms(), 0, 0);
    can_diag_emit(0, CAN_DIAG_ID_LINK_RX, d);
    s_tx_pkts = s_tx_bytes = s_tx_waits = 0;
}

// Task that bridges CAN frames to BLE notifications
static void can_to_ble_task(void *arg)
{
    (void)arg;
    static can_tagged_frame_t batch[BLE_BATCH_MAX_FRAMES];
    static uint8_t ble_buf[512];

    while (true) {
        if (can_manager_receive(&batch[0], 100) != ESP_OK) continue;
        // As many frames as one notification to this phone holds.
        size_t max = ble_server_max_payload();
        if (max > sizeof(ble_buf)) max = sizeof(ble_buf);
        bool compact = ble_server_compact();
        int want = max < 19 ? 1 : (int)((max - 1) / (compact ? 11 : 18));
        if (want > BLE_BATCH_MAX_FRAMES) want = BLE_BATCH_MAX_FRAMES;

        // Wait for frames only up to the original 16 per packet (latency as before); beyond that take just what
        // is already queued, so packets grow only when the link is behind.
        int count = 1;
        while (count < want && can_manager_receive(&batch[count], count < 16 ? BLE_BATCH_TIMEOUT_MS : 0) == ESP_OK) count++;
        if (!max) continue;  // no phone
        link_diag();

        for (int from = 0; from < count; ) {
            int before = from;
            notify_wait(ble_buf, build_ble_packet(batch, count, &from, ble_buf, max, compact, false));
            if (from == before) break;  // a frame larger than the MTU: drop the rest
        }
        if (compact) {
            for (int from = 0; from < count; ) {
                int before = from;
                notify_wait(ble_buf, build_ble_packet(batch, count, &from, ble_buf, max, false, true));
                if (from == before) break;
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "DashKit firmware starting... (reset reason=%d)",
             (int)esp_reset_reason());

    // Initialize NVS (required for BLE)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Erases the bond store: a bonded phone then hits the iOS stale-bond wall
        // and can't re-pair to this address on its own. Loud on purpose.
        ESP_LOGE(TAG, "NVS re-init (%s): erasing — all bonds will be lost",
                 esp_err_to_name(err));
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(led_init());
    led_set_color(LED_COLOR_BLUE);

    ESP_ERROR_CHECK(can_manager_init());

    ESP_ERROR_CHECK(can_filter_init());

    dbc_init();

    ESP_ERROR_CHECK(automation_manager_init());

    ESP_ERROR_CHECK(vehicle_control_init());

    // CAN interface 0 (MCP2518FD)
    mcp251xfd_config_t can0_cfg = {
        .spi_host     = CAN0_SPI_HOST,
        .pin_mosi     = CAN0_PIN_MOSI,
        .pin_sclk     = CAN0_PIN_SCLK,
        .pin_miso     = CAN0_PIN_MISO,
        .pin_cs       = CAN0_PIN_CS,
        .pin_int      = CAN0_PIN_INT,
        .spi_clock_hz = CAN0_SPI_CLOCK_HZ,
        .osc_freq_hz  = CAN0_OSC_FREQ_HZ,
        .bitrate      = 500000,
        .bitrate_data = 0,
        .bus_id       = 0,
    };
    can_interface_t *can0 = mcp251xfd_create(&can0_cfg);
    if (!can0) {
        ESP_LOGE(TAG, "Failed to create CAN0 interface");
        led_set_color(LED_COLOR_RED);
        return;
    }
    ESP_ERROR_CHECK(can_manager_add_interface(can0));

    // CAN interface 1 (MCP2518FD, shares SPI2 bus with CAN0)
    mcp251xfd_config_t can1_cfg = {
        .spi_host     = CAN1_SPI_HOST,
        .pin_mosi     = CAN1_PIN_MOSI,
        .pin_sclk     = CAN1_PIN_SCLK,
        .pin_miso     = CAN1_PIN_MISO,
        .pin_cs       = CAN1_PIN_CS,
        .pin_int      = CAN1_PIN_INT,
        .spi_clock_hz = CAN1_SPI_CLOCK_HZ,
        .osc_freq_hz  = CAN1_OSC_FREQ_HZ,
        .bitrate      = 500000,
        .bitrate_data = 0,
        .bus_id       = 1,
    };
    can_interface_t *can1 = mcp251xfd_create(&can1_cfg);
    if (!can1) {
        ESP_LOGE(TAG, "Failed to create CAN1 interface");
        led_set_color(LED_COLOR_RED);
        return;
    }
    ESP_ERROR_CHECK(can_manager_add_interface(can1));

    // Start CAN
    err = can_manager_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN manager start failed");
        led_set_color(LED_COLOR_RED);
        return;
    }

    // BLE
    ESP_ERROR_CHECK(ble_server_init());
    ESP_ERROR_CHECK(ble_ota_init());
    ESP_ERROR_CHECK(ble_appchan_init());
    ESP_ERROR_CHECK(ble_server_start());

#if defined(CONFIG_DASHKIT_TESLA_BLE)
    // Boot canary for the Tesla link: role state must be visible and a missing
    // key/link must not be silent. The app provisions the car (opcode 0x04)
    // and starts enrollment (0x01); until then the firmware only reports
    // "never enrolled".
    ESP_LOGI(TAG, "Tesla BLE: enabled (central=%d). Persistent VCSEC status "
                  "poll; enrollment is provisioned + started from the app.",
             CONFIG_BT_NIMBLE_ROLE_CENTRAL);
    ESP_ERROR_CHECK(tesla_pairing_init());
    ESP_ERROR_CHECK(tesla_ble_client_init());
#endif

    // Bridge task: CAN -> BLE
    xTaskCreatePinnedToCore(can_to_ble_task, "can2ble", 8192, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "DashKit firmware ready");

    // Commit this image once it has survived the boot window (OTA rollback).
    const esp_timer_create_args_t mv_args = {
        .callback = mark_app_valid_cb,
        .name = "ota_mark_valid",
    };
    esp_timer_handle_t mv_timer;
    ESP_ERROR_CHECK(esp_timer_create(&mv_args, &mv_timer));
    ESP_ERROR_CHECK(esp_timer_start_once(mv_timer, MARK_VALID_DELAY_MS * 1000ULL));

    led_set_color(LED_COLOR_GREEN);
}
