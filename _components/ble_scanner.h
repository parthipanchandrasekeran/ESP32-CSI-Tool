#ifndef ESP32_CSI_BLE_SCANNER_H
#define ESP32_CSI_BLE_SCANNER_H

// BLE passive scanner for the CSI-Tool firmware.
//
// Uses NimBLE (the lightweight Bluetooth host stack) to passively listen for
// BLE advertisements in range. For every advertisement we receive, we send a
// UDP datagram of the form:
//     <board_mac> BLE <bt_addr> <rssi> <name?>\n
//
// The host-side csi_udp_receiver parses these and stores them in
// ble_observations.

#include <string.h>
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

// Forward declaration — implemented in csi_component.h
static void udp_send_typed(const char *type, const char *fmt, ...);

static int _ble_gap_event_cb(struct ble_gap_event *event, void *arg);
static uint8_t _ble_own_addr_type = 0;

static void _ble_start_scan(void) {
    struct ble_gap_disc_params disc_params = {0};
    disc_params.passive = 1;        // we don't want to actively probe
    disc_params.itvl = 0;           // default
    disc_params.window = 0;         // default
    disc_params.filter_duplicates = 0;  // see every advert (we de-dupe host-side)
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    int rc = ble_gap_disc(_ble_own_addr_type, BLE_HS_FOREVER, &disc_params,
                          _ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE("ble", "ble_gap_disc failed: %d", rc);
    } else {
        ESP_LOGI("ble", "BLE scan started");
    }
}

static int _ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type != BLE_GAP_EVENT_DISC) return 0;
    char addr_str[18];
    snprintf(addr_str, sizeof(addr_str),
             "%02x:%02x:%02x:%02x:%02x:%02x",
             event->disc.addr.val[5], event->disc.addr.val[4],
             event->disc.addr.val[3], event->disc.addr.val[2],
             event->disc.addr.val[1], event->disc.addr.val[0]);
    int rssi = (int) event->disc.rssi;

    // Try to extract the complete-local-name from advertisement data.
    // BLE adv data is a sequence of (length, type, data...) records.
    char name_buf[33] = {0};
    const uint8_t *adv = event->disc.data;
    int len = event->disc.length_data;
    int i = 0;
    while (i < len) {
        int field_len = adv[i];
        if (field_len == 0 || i + field_len >= len) break;
        int field_type = adv[i + 1];
        if (field_type == 0x09 || field_type == 0x08) {  // Complete or Shortened Local Name
            int name_len = field_len - 1;
            if (name_len > 32) name_len = 32;
            memcpy(name_buf, &adv[i + 2], name_len);
            name_buf[name_len] = '\0';
            // strip non-printable chars to keep the UDP message clean
            for (int j = 0; j < name_len; j++) {
                if (name_buf[j] < 0x20 || name_buf[j] >= 0x7f) name_buf[j] = '?';
            }
            break;
        }
        i += field_len + 1;
    }

    if (name_buf[0]) {
        udp_send_typed("BLE", "%s %d %s", addr_str, rssi, name_buf);
    } else {
        udp_send_typed("BLE", "%s %d", addr_str, rssi);
    }
    return 0;
}

static void _ble_on_sync(void) {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE("ble", "ensure_addr failed: %d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &_ble_own_addr_type);
    if (rc != 0) {
        ESP_LOGE("ble", "infer_auto failed: %d", rc);
        return;
    }
    _ble_start_scan();
}

static void _ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void start_ble_scanner(void) {
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE("ble", "nimble_port_init failed: %s", esp_err_to_name(ret));
        return;
    }
    ble_hs_cfg.sync_cb = _ble_on_sync;
    // We're a passive scanner only — no advertising or GATT services. Skipping
    // ble_svc_gap_init() / device_name_set() because those expect a fully
    // initialized GATT server and crashed our boot.
    nimble_port_freertos_init(_ble_host_task);
    ESP_LOGI("ble", "NimBLE host started");
}

#endif // ESP32_CSI_BLE_SCANNER_H
