#ifndef ESP32_CSI_CSI_COMPONENT_H
#define ESP32_CSI_CSI_COMPONENT_H

#include "time_component.h"
#include "math.h"
#include <sstream>
#include <iostream>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdarg.h>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_IDF_TARGET_ESP32S3
#include "driver/temperature_sensor.h"
#define HAS_INTERNAL_TEMP_SENSOR 1
#endif

#include "ble_scanner.h"

#ifndef CSI_UDP_HOST_IP
#define CSI_UDP_HOST_IP "10.0.0.70"
#endif
#ifndef CSI_UDP_HOST_PORT
#define CSI_UDP_HOST_PORT 5051
#endif

static int udp_csi_fd = -1;
static char board_mac_str[18] = {0};
static struct sockaddr_in udp_csi_dst = {0};

static void udp_csi_init() {
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(board_mac_str, sizeof(board_mac_str),
             "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    udp_csi_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_csi_fd < 0) {
        printf("udp_csi: socket() failed\n");
        return;
    }
    udp_csi_dst.sin_family = AF_INET;
    udp_csi_dst.sin_port = htons(CSI_UDP_HOST_PORT);
    inet_aton(CSI_UDP_HOST_IP, &udp_csi_dst.sin_addr);
    printf("udp_csi: ready, board=%s -> " CSI_UDP_HOST_IP ":%d\n",
           board_mac_str, CSI_UDP_HOST_PORT);
}

static void udp_csi_send(const char *line) {
    if (udp_csi_fd < 0) return;
    char buf[2048];
    int n = snprintf(buf, sizeof(buf), "%s %s", board_mac_str, line);
    if (n > 0 && n < (int) sizeof(buf)) {
        sendto(udp_csi_fd, buf, n, 0,
               (struct sockaddr *) &udp_csi_dst, sizeof(udp_csi_dst));
    }
}

// Helper: send a custom datagram with a typed prefix (TEMP, BLE, etc.)
static void udp_send_typed(const char *type, const char *fmt, ...) {
    if (udp_csi_fd < 0) return;
    char buf[256];
    int prefix = snprintf(buf, sizeof(buf), "%s %s ", board_mac_str, type);
    if (prefix < 0 || prefix >= (int) sizeof(buf)) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + prefix, sizeof(buf) - prefix, fmt, ap);
    va_end(ap);
    if (n > 0 && prefix + n < (int) sizeof(buf)) {
        sendto(udp_csi_fd, buf, prefix + n, 0,
               (struct sockaddr *) &udp_csi_dst, sizeof(udp_csi_dst));
    }
}

#ifdef HAS_INTERNAL_TEMP_SENSOR
static temperature_sensor_handle_t cpu_temp_handle = NULL;

static void cpu_temp_task(void *arg) {
    // Initialize the on-chip temperature sensor (range -10..80 C is the
    // narrowest, lowest-noise range — adequate for indoor use).
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&cfg, &cpu_temp_handle) != ESP_OK) {
        printf("cpu_temp: install failed\n");
        vTaskDelete(NULL);
        return;
    }
    if (temperature_sensor_enable(cpu_temp_handle) != ESP_OK) {
        printf("cpu_temp: enable failed\n");
        vTaskDelete(NULL);
        return;
    }
    while (1) {
        float c = 0.0f;
        if (temperature_sensor_get_celsius(cpu_temp_handle, &c) == ESP_OK) {
            udp_send_typed("TEMP", "%.2f", c);
        }
        vTaskDelay(pdMS_TO_TICKS(10000));  // every 10 sec
    }
}

static void start_cpu_temp_reporter() {
    xTaskCreate(cpu_temp_task, "cpu_temp", 3072, NULL, 1, NULL);
}
#else
static void start_cpu_temp_reporter() {}
#endif

// ---- WiFi neighbor scanner --------------------------------------------------
// Periodically scan all SSIDs in range and report each AP via UDP.
// The scan briefly takes the radio off our STA channel (~1-2 sec gap in
// CSI capture), so we run it infrequently (every 60 sec) and use passive
// scan (no probe requests = quieter on the air).
static void wifi_scan_task(void *arg) {
    // Stagger initial scan per-board so multiple boards don't all scan
    // at the same instant. Use last MAC byte to spread across ~50 sec.
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    vTaskDelay(pdMS_TO_TICKS(15000 + (mac[5] % 50) * 1000));

    while (1) {
        wifi_scan_config_t scan_cfg = {};
        scan_cfg.ssid = NULL;
        scan_cfg.bssid = NULL;
        scan_cfg.channel = 0;                       // 0 = all channels
        scan_cfg.show_hidden = true;
        scan_cfg.scan_type = WIFI_SCAN_TYPE_PASSIVE;
        scan_cfg.scan_time.passive = 120;           // ms per channel

        esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);  // blocking
        if (err == ESP_OK) {
            uint16_t ap_count = 0;
            esp_wifi_scan_get_ap_num(&ap_count);
            if (ap_count > 0) {
                if (ap_count > 32) ap_count = 32;   // cap to avoid OOM
                wifi_ap_record_t *records = (wifi_ap_record_t *)
                    calloc(ap_count, sizeof(wifi_ap_record_t));
                if (records != NULL) {
                    if (esp_wifi_scan_get_ap_records(&ap_count, records) == ESP_OK) {
                        for (int i = 0; i < ap_count; i++) {
                            wifi_ap_record_t *r = &records[i];
                            // Sanitize SSID: line-based UDP, so strip CR/LF
                            // and replace non-printable with '?'.
                            char ssid_clean[33] = {0};
                            for (int j = 0; j < 32 && r->ssid[j]; j++) {
                                char c = (char) r->ssid[j];
                                ssid_clean[j] = (c >= 32 && c < 127 && c != ' ') ? c
                                              : (c == ' ' ? '_' : '?');
                            }
                            char bssid_str[18];
                            snprintf(bssid_str, sizeof(bssid_str),
                                     "%02x:%02x:%02x:%02x:%02x:%02x",
                                     r->bssid[0], r->bssid[1], r->bssid[2],
                                     r->bssid[3], r->bssid[4], r->bssid[5]);
                            // "WIFI <bssid> <rssi> <channel> <auth> <ssid>"
                            udp_send_typed("WIFI", "%s %d %d %d %s",
                                           bssid_str, r->rssi, r->primary,
                                           (int) r->authmode,
                                           ssid_clean[0] ? ssid_clean : "-");
                            vTaskDelay(pdMS_TO_TICKS(5));  // stagger packets
                        }
                    }
                    free(records);
                }
            }
        } else {
            printf("wifi_scan: scan_start failed: %d\n", err);
        }
        vTaskDelay(pdMS_TO_TICKS(60000));  // next scan in 60s
    }
}

static void start_wifi_scanner() {
    xTaskCreate(wifi_scan_task, "wifi_scan", 4096, NULL, 1, NULL);
}

char *project_type;

#define CSI_RAW 1
#define CSI_AMPLITUDE 0
#define CSI_PHASE 0

#define CSI_TYPE CSI_RAW

SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

void _wifi_csi_cb(void *ctx, wifi_csi_info_t *data) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    std::stringstream ss;

    wifi_csi_info_t d = data[0];
    char mac[20] = {0};
    sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X", d.mac[0], d.mac[1], d.mac[2], d.mac[3], d.mac[4], d.mac[5]);

    ss << "CSI_DATA,"
       << project_type << ","
       << mac << ","
       // https://github.com/espressif/esp-idf/blob/9d0ca60398481a44861542638cfdc1949bb6f312/components/esp_wifi/include/esp_wifi_types.h#L314
       << d.rx_ctrl.rssi << ","
       << d.rx_ctrl.rate << ","
       << d.rx_ctrl.sig_mode << ","
       << d.rx_ctrl.mcs << ","
       << d.rx_ctrl.cwb << ","
       << d.rx_ctrl.smoothing << ","
       << d.rx_ctrl.not_sounding << ","
       << d.rx_ctrl.aggregation << ","
       << d.rx_ctrl.stbc << ","
       << d.rx_ctrl.fec_coding << ","
       << d.rx_ctrl.sgi << ","
       << d.rx_ctrl.noise_floor << ","
       << d.rx_ctrl.ampdu_cnt << ","
       << d.rx_ctrl.channel << ","
       << d.rx_ctrl.secondary_channel << ","
       << d.rx_ctrl.timestamp << ","
       << d.rx_ctrl.ant << ","
       << d.rx_ctrl.sig_len << ","
       << d.rx_ctrl.rx_state << ","
       << real_time_set << ","
       << get_steady_clock_timestamp() << ","
       << data->len << ",[";

#if CONFIG_SHOULD_COLLECT_ONLY_LLTF
    int data_len = 128;
#else
    int data_len = data->len;
#endif

int8_t *my_ptr;
#if CSI_RAW
    my_ptr = data->buf;
    for (int i = 0; i < data_len; i++) {
        ss << (int) my_ptr[i] << " ";
    }
#endif
#if CSI_AMPLITUDE
    my_ptr = data->buf;
    for (int i = 0; i < data_len / 2; i++) {
        ss << (int) sqrt(pow(my_ptr[i * 2], 2) + pow(my_ptr[(i * 2) + 1], 2)) << " ";
    }
#endif
#if CSI_PHASE
    my_ptr = data->buf;
    for (int i = 0; i < data_len / 2; i++) {
        ss << (int) atan2(my_ptr[i*2], my_ptr[(i*2)+1]) << " ";
    }
#endif
    ss << "]\n";

    printf(ss.str().c_str());
    udp_csi_send(ss.str().c_str());
    fflush(stdout);
    vTaskDelay(0);
    xSemaphoreGive(mutex);
}

void _print_csi_csv_header() {
    char *header_str = (char *) "type,role,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_state,real_time_set,real_timestamp,len,CSI_DATA\n";
    outprintf(header_str);
}

void csi_init(char *type) {
    project_type = type;

#ifdef CONFIG_SHOULD_COLLECT_CSI
    ESP_ERROR_CHECK(esp_wifi_set_csi(1));

    // @See: https://github.com/espressif/esp-idf/blob/master/components/esp_wifi/include/esp_wifi_types.h#L401
    wifi_csi_config_t configuration_csi;
    configuration_csi.lltf_en = 1;
    configuration_csi.htltf_en = 1;
    configuration_csi.stbc_htltf2_en = 1;
    configuration_csi.ltf_merge_en = 1;
    configuration_csi.channel_filter_en = 0;
    configuration_csi.manu_scale = 0;

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&configuration_csi));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(&_wifi_csi_cb, NULL));

    _print_csi_csv_header();
    udp_csi_init();
    start_cpu_temp_reporter();
    start_ble_scanner();
    start_wifi_scanner();
#endif
}

#endif //ESP32_CSI_CSI_COMPONENT_H
