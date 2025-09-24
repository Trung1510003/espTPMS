#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "bt_hci_common.h"
#include "driver/uart.h"

static const char *TAG = "BLE_ADV_SCAN";

// Cấu hình UART cho DFPlayer Mini
#define TXD_PIN 2   // ESP32-C3 TX → MP3 RX
#define RXD_PIN 3   // ESP32-C3 RX → MP3 TX
#define UART_PORT_NUM UART_NUM_1
#define BUF_SIZE (1024)

// Thời gian thực thi mỗi file MP3 (6 giây)
#define AUDIO_EXECUTION_TIME_MS 6000

// Kích thước hàng đợi âm thanh (bây giờ là 1)
#define AUDIO_QUEUE_LENGTH 1

// Thời gian cooldown (10 giây)
#define AUDIO_COOLDOWN_MS 10000

// Ngưỡng áp suất (14.6 * 10 = 146)
#define PRESSURE_THRESHOLD 146

// Biến trạng thái để kiểm tra xem âm thanh đang phát hay không
static volatile bool is_audio_playing = false;

// Hàng đợi âm thanh
static QueueHandle_t audio_queue;

// Thời điểm chấp nhận phần tử cuối cùng vào queue
static volatile TickType_t last_accept_time = 0;

typedef enum { VOICE_MALE = 0, VOICE_FEMALE = 1 } voice_gender_t;
typedef enum { PSI_UNIT = 0, BAR_UNIT = 1 } unit_pressure_t;
typedef enum { TIRE_SWAP_INITIAL = 0, TIRE_SWAP_VERTICAL, TIRE_SWAP_CROSS, TIRE_SWAP_HORIZONTAL } TireSwapMode;

typedef struct {
    uint8_t  warm_up_greetings;
    voice_gender_t warning_settings;
    float    front_tire_press_Upper_limit;
    float    front_tire_press_Lower_limit;
    float    rear_tire_press_Upper_limit;
    float    rear_tire_press_Lower_limit;
    float    leak;
    uint8_t  high_temp_warning;
    TireSwapMode tire_swap;
    char     short_name[16];
    char     addressB_TT[18];   // Front Left
    char     addressB_TP[18];   // Front Right
    char     addressB_ST[18];   // Rear Left
    char     addressB_SP[18];   // Rear Right
    unit_pressure_t tire_pressure_unit;
    uint8_t  version;
} TPMS_Config;

static TPMS_Config g_tpms_config = {
    .warm_up_greetings = 1,
    .warning_settings = VOICE_FEMALE,
    .front_tire_press_Upper_limit = 30.0,
    .front_tire_press_Lower_limit = 24.0,
    .rear_tire_press_Upper_limit = 30.0,
    .rear_tire_press_Lower_limit = 24.0,
    .leak = 16.0,
    .high_temp_warning = 30,
    .tire_swap = TIRE_SWAP_VERTICAL,
    .short_name = "AI-8000",
    .addressB_TT = "12:30:af:00:01:59",
    .addressB_TP = "12:30:af:00:01:46",
    .addressB_ST = "12:30:af:00:01:5e",
    .addressB_SP = "12:30:af:00:00:f0",
    .tire_pressure_unit = PSI_UNIT,
    .version = 1
};

// Structure for AI-8000 devices
typedef struct {
    char address[18];
    char name[4]; // "TT", "TP", "ST", "SP" - chỉ cần 3 bytes + null
} ai_device_t;

// Global device array (will be updated based on swap mode)
static ai_device_t g_ai_devices[4];

// Initialize device names (fixed order: TT, TP, ST, SP)
static const char* const device_names[] = {"TT", "TP", "ST", "SP"};

// Function to update devices based on tire swap mode
static void update_ai_devices_by_swap_mode(TireSwapMode mode) {
    const char* addresses[4];

    switch(mode) {
        case TIRE_SWAP_INITIAL:
            addresses[0] = g_tpms_config.addressB_TT;
            addresses[1] = g_tpms_config.addressB_TP;
            addresses[2] = g_tpms_config.addressB_ST;
            addresses[3] = g_tpms_config.addressB_SP;
            break;
        case TIRE_SWAP_VERTICAL:
            addresses[0] = g_tpms_config.addressB_ST;
            addresses[1] = g_tpms_config.addressB_SP;
            addresses[2] = g_tpms_config.addressB_TT;
            addresses[3] = g_tpms_config.addressB_TP;
            break;
        case TIRE_SWAP_CROSS:
            addresses[0] = g_tpms_config.addressB_SP;
            addresses[1] = g_tpms_config.addressB_ST;
            addresses[2] = g_tpms_config.addressB_TP;
            addresses[3] = g_tpms_config.addressB_TT;
            break;
        case TIRE_SWAP_HORIZONTAL:
            addresses[0] = g_tpms_config.addressB_TP;
            addresses[1] = g_tpms_config.addressB_TT;
            addresses[2] = g_tpms_config.addressB_SP;
            addresses[3] = g_tpms_config.addressB_ST;
            break;
    }

    for(int i = 0; i < 4; i++) {
        strncpy(g_ai_devices[i].address, addresses[i], sizeof(g_ai_devices[i].address));
        strncpy(g_ai_devices[i].name, device_names[i], sizeof(g_ai_devices[i].name));
        g_ai_devices[i].address[sizeof(g_ai_devices[i].address)-1] = '\0';
        g_ai_devices[i].name[sizeof(g_ai_devices[i].name)-1] = '\0';
    }
}

typedef struct {
    char scan_local_name[32];
    uint8_t name_len;
} ble_scan_local_name_t;

typedef struct {
    uint8_t temperature;
    float battery_level;
    uint16_t pressure;
    char device_name[4]; // Reduced size to match actual need
    char address[18];
} sensor_data_t;

// Hàm gửi lệnh đến DFPlayer Mini
static void send_command(uint8_t cmd, uint16_t param)
{
    uint8_t buf[10];
    uint16_t checksum = (uint16_t)(0xFFFF - (0xFF + 0x06 + cmd + (param >> 8) + (param & 0xFF)) + 1);

    buf[0] = 0x7E;          // Start byte
    buf[1] = 0xFF;          // Version
    buf[2] = 0x06;          // Length
    buf[3] = cmd;           // Command
    buf[4] = 0x00;          // No feedback
    buf[5] = (param >> 8);  // Parameter high
    buf[6] = (param & 0xFF);// Parameter low
    buf[7] = (checksum >> 8);
    buf[8] = (checksum & 0xFF);
    buf[9] = 0xEF;          // End byte

    uart_write_bytes(UART_PORT_NUM, (const char*)buf, 10);
}

// Khởi tạo hàng đợi âm thanh
static void init_audio_queue(void)
{
    audio_queue = xQueueCreate(AUDIO_QUEUE_LENGTH, sizeof(uint16_t));
    if (audio_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create audio queue");
    } else {
        ESP_LOGI(TAG, "Audio queue initialized with length %d", AUDIO_QUEUE_LENGTH);
    }
}

// Task xử lý hàng đợi âm thanh
static void audio_task(void *pvParameters)
{
    uint16_t mp3_file;
    while (1) {
        if (xQueueReceive(audio_queue, &mp3_file, portMAX_DELAY) == pdTRUE) {
            is_audio_playing = true;
            ESP_LOGI(TAG, "Playing MP3 file: %d", mp3_file);
            send_command(0x03, mp3_file);
            vTaskDelay(pdMS_TO_TICKS(AUDIO_EXECUTION_TIME_MS));
            is_audio_playing = false;
        }
    }
}

// Task khởi tạo DFPlayer Mini
static void dfplayer_init_task(void *pvParameters)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    send_command(0x06, 25);
    ESP_LOGI(TAG, "Đặt âm lượng = 25");

    // Chọn thư mục mp3 để phát
    send_command(0x0F, 1);  // Chọn thư mục 01 (mp3)
    ESP_LOGI(TAG, "Chọn thư mục mp3 (01)");

    vTaskDelay(pdMS_TO_TICKS(1000));
    // Phát 14.mp3 ngay sau khi khởi tạo
    send_command(0x03, 14);

    ESP_LOGI(TAG, "DFPlayer initialization completed");
    vTaskDelete(NULL);
}

// Task quản lý cooldown timer
static void cooldown_manager_task(void *pvParameters)
{
    while (1) {
        // Có thể thêm logic quản lý cooldown phức tạp hơn ở đây
        // Ví dụ: điều chỉnh thời gian cooldown động dựa trên tần suất cảnh báo
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Kiểm tra mỗi giây
    }
}

// Phát âm thanh theo ngưỡng áp suất và vị trí lốp
static void play_sound_based_on_pressure(const sensor_data_t *sensor_data)
{
    if (sensor_data == NULL) {
        ESP_LOGW(TAG, "Sensor data is NULL");
        return;
    }

    uint16_t mp3_file = 0;

    // Kiểm tra áp suất và vị trí lốp
    if (sensor_data->pressure > g_tpms_config.front_tire_press_Upper_limit * 10) {
        ESP_LOGI(TAG, "Pressure %u > %.1f for %s", sensor_data->pressure, g_tpms_config.front_tire_press_Upper_limit, sensor_data->device_name);
        if (strcmp(sensor_data->device_name, "TP") == 0) {
            mp3_file = 15; // Phải-Trước áp suất cao: 15.mp3
        } else if (strcmp(sensor_data->device_name, "TT") == 0) {
            mp3_file = 16; // Trái-Trước áp suất cao: 16.mp3
        } else if (strcmp(sensor_data->device_name, "SP") == 0) {
            mp3_file = 17; // Phải-Sau áp suất cao: 17.mp3
        } else if (strcmp(sensor_data->device_name, "ST") == 0) {
            mp3_file = 18; // Trái-Sau áp suất cao: 18.mp3
        }
    } else if (sensor_data->pressure < g_tpms_config.front_tire_press_Lower_limit * 10) {
        ESP_LOGI(TAG, "Pressure %u < %.1f for %s", sensor_data->pressure, g_tpms_config.front_tire_press_Lower_limit, sensor_data->device_name);
        if (strcmp(sensor_data->device_name, "TP") == 0) {
            mp3_file = 19; // Phải-Trước áp suất thấp: 19.mp3
        } else if (strcmp(sensor_data->device_name, "TT") == 0) {
            mp3_file = 20; // Trái-Trước áp suất thấp: 20.mp3
        } else if (strcmp(sensor_data->device_name, "SP") == 0) {
            mp3_file = 21; // Phải-Sau áp suất thấp: 21.mp3
        } else if (strcmp(sensor_data->device_name, "ST") == 0) {
            mp3_file = 22; // Trái-Sau áp suất thấp: 22.mp3
        }
    }
    if (sensor_data->pressure <= g_tpms_config.leak * 13 && sensor_data->pressure >= g_tpms_config.leak * 10) {
        // ESP_LOGI(TAG, "Pressure %u < %.1f for %s", sensor_data->pressure, g_tpms_config.front_tire_press_Lower_limit, sensor_data->device_name);
        if (strcmp(sensor_data->device_name, "TP") == 0) {
            mp3_file = 23; // Phải-Trước áp suất thấp: 19.mp3
        } else if (strcmp(sensor_data->device_name, "TT") == 0) {
            mp3_file = 24; // Trái-Trước áp suất thấp: 20.mp3
        } else if (strcmp(sensor_data->device_name, "SP") == 0) {
            mp3_file = 25; // Phải-Sau áp suất thấp: 21.mp3
        } else if (strcmp(sensor_data->device_name, "ST") == 0) {
            mp3_file = 26; // Trái-Sau áp suất thấp: 22.mp3
        }
    }

    // Nếu có file MP3 cần phát, kiểm tra cooldown trước khi thêm vào hàng đợi
    if (mp3_file != 0) {
        TickType_t now = xTaskGetTickCount();
        if (now - last_accept_time >= pdMS_TO_TICKS(AUDIO_COOLDOWN_MS)) {
            if (sensor_data->pressure != PRESSURE_THRESHOLD) {
                if (xQueueSend(audio_queue, &mp3_file, 0) == pdTRUE) {
                    last_accept_time = now;
                    ESP_LOGI(TAG, "Accepted and added MP3 file %d to audio queue for %s", mp3_file, sensor_data->device_name);
                } else {
                    ESP_LOGW(TAG, "Audio queue full, could not add request for %s", sensor_data->device_name);
                }
            } else {
                ESP_LOGW(TAG, "Pressure %u equals threshold %d, discarding request for %s", sensor_data->pressure, PRESSURE_THRESHOLD, sensor_data->device_name);
            }
        } else {
            ESP_LOGW(TAG, "Discarding audio request for %s during cooldown", sensor_data->device_name);
        }
    }
}



static uint8_t hci_cmd_buf[128];

// Optimized address conversion
static void address_to_string(const uint8_t *addr, char *str, size_t str_len) {
    snprintf(str, str_len, "%02x:%02x:%02x:%02x:%02x:%02x",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

// Optimized device lookup with pointer arithmetic
static const ai_device_t* get_device_by_address(const uint8_t *addr) {
    char addr_str[18];
    address_to_string(addr, addr_str, sizeof(addr_str));

    for (int i = 0; i < 4; i++) {
        if (strcmp(addr_str, g_ai_devices[i].address) == 0) {
            return &g_ai_devices[i];
        }
    }
    return NULL;
}

// Optimized sensor data parsing
static sensor_data_t parse_sensor_data(const uint8_t *raw_data, uint8_t data_len, const uint8_t *addr) {
    sensor_data_t sensor_data = {0};

    if (data_len < 21) {
        ESP_LOGW(TAG, "Insufficient data length: %d", data_len);
        return sensor_data;
    }

    const uint8_t *sensor_bytes = &raw_data[data_len - 21];
    sensor_data.temperature = sensor_bytes[1];
    sensor_data.battery_level = sensor_bytes[2] / 10.0f;
    sensor_data.pressure = (sensor_bytes[3] << 8) | sensor_bytes[4];

    const ai_device_t *device = get_device_by_address(addr);
    if (device != NULL) {
        strncpy(sensor_data.device_name, device->name, sizeof(sensor_data.device_name));
    } else {
        strcpy(sensor_data.device_name, "UNK"); // Unknown
    }

    address_to_string(addr, sensor_data.address, sizeof(sensor_data.address));
    return sensor_data;
}

// Optimized local name extraction
static esp_err_t get_local_name(const uint8_t *data_msg, uint8_t data_len, ble_scan_local_name_t *scanned_packet) {
    uint8_t curr_ptr = 0;

    while (curr_ptr < data_len) {
        uint8_t curr_len = data_msg[curr_ptr++];
        if (curr_len == 0) return ESP_FAIL;

        uint8_t curr_type = data_msg[curr_ptr++];
        if (curr_type == 0x08 || curr_type == 0x09) {
            uint8_t name_len = (curr_len - 1 < sizeof(scanned_packet->scan_local_name)) ?
                              curr_len - 1 : sizeof(scanned_packet->scan_local_name) - 1;

            memcpy(scanned_packet->scan_local_name, &data_msg[curr_ptr], name_len);
            scanned_packet->scan_local_name[name_len] = '\0';
            scanned_packet->name_len = name_len;
            return ESP_OK;
        }
        curr_ptr += curr_len - 1;
    }
    return ESP_FAIL;
}

// Packet structure to avoid multiple allocations
typedef struct {
    uint8_t event_type;
    uint8_t addr_type;
    uint8_t addr[6];
    uint8_t data_len;
    int8_t rssi;
} adv_report_t;

static void controller_rcv_pkt_ready(void) {
    // Keep it minimal
}

// Optimized packet processing with single allocation
static int host_rcv_pkt(uint8_t *data, uint16_t len) {
    if (data[1] == 0x0e) {
        if (data[6] != 0) {
            ESP_LOGE(TAG, "Event opcode 0x%02x fail: 0x%02x", data[4], data[6]);
            return ESP_FAIL;
        }
    }

    if (data[3] == HCI_LE_ADV_REPORT) {
        uint8_t num_responses = data[4];
        if (num_responses == 0) return ESP_OK;

        // Single allocation for all reports
        adv_report_t *reports = malloc(num_responses * sizeof(adv_report_t));
        if (!reports) return ESP_FAIL;

        uint16_t data_ptr = 5;
        uint16_t total_data_len = 0;

        // Parse all reports first
        for (uint8_t i = 0; i < num_responses; i++) {
            reports[i].event_type = data[data_ptr++];
            reports[i].addr_type = data[data_ptr++];
            memcpy(reports[i].addr, &data[data_ptr], 6);
            data_ptr += 6;
            reports[i].data_len = data[data_ptr++];
            total_data_len += reports[i].data_len;
        }

        // Single allocation for all data
        uint8_t *all_data = malloc(total_data_len);
        if (!all_data) {
            free(reports);
            return ESP_FAIL;
        }

        // Copy all data
        uint16_t data_msg_ptr = 0;
        for (uint8_t i = 0; i < num_responses; i++) {
            memcpy(&all_data[data_msg_ptr], &data[data_ptr], reports[i].data_len);
            data_ptr += reports[i].data_len;
            data_msg_ptr += reports[i].data_len;
        }

        // Process RSSI
        for (uint8_t i = 0; i < num_responses; i++) {
            reports[i].rssi = -(0xFF - data[data_ptr++]);
        }

        // Process each report
        data_msg_ptr = 0;
        for (uint8_t i = 0; i < num_responses; i++) {
            ble_scan_local_name_t scanned_name = {0};

            if (get_local_name(&all_data[data_msg_ptr], reports[i].data_len, &scanned_name) == ESP_OK) {
                if (strcmp(scanned_name.scan_local_name, "AI-8000") == 0) {
                    sensor_data_t sensor_data = parse_sensor_data(data, len, reports[i].addr);

                    ESP_LOGI(TAG, "Device: %s, Temp: %u°C, Battery: %.1f, Pressure: %u Pa",
                             sensor_data.device_name, sensor_data.temperature,
                             sensor_data.battery_level, sensor_data.pressure);

                    // Phát âm thanh dựa trên áp suất
                    play_sound_based_on_pressure(&sensor_data);

                    // Pressure checking with configurable thresholds
                    if (sensor_data.pressure > g_tpms_config.front_tire_press_Upper_limit * 10) {
                        ESP_LOGW(TAG, "HIGH Pressure: %u > %.1f for %s",
                                sensor_data.pressure, g_tpms_config.front_tire_press_Upper_limit, sensor_data.device_name);
                    } else if (sensor_data.pressure < g_tpms_config.front_tire_press_Lower_limit * 10) {
                        ESP_LOGW(TAG, "LOW Pressure: %u < %.1f for %s",
                                sensor_data.pressure, g_tpms_config.front_tire_press_Lower_limit, sensor_data.device_name);
                    }

                    // Compact printf output
                    printf("=== AI-8000 [%s] ===\n", sensor_data.device_name);
                    printf("Addr: %s\nRSSI: %ddB\nTemp: %u°C\nBatt: %.1f\nPress: %u Pa\n",
                           sensor_data.address, reports[i].rssi, sensor_data.temperature,
                           sensor_data.battery_level, sensor_data.pressure);

                    // Debug parsing
                    const uint8_t *sensor_bytes = &data[len - 21];
                    printf("Data: ");
                    for (int k = 0; k < 5; k++) printf("%02x ", sensor_bytes[k]);
                    printf("\n====================\n");
                }
            }
            data_msg_ptr += reports[i].data_len;
        }

        free(all_data);
        free(reports);
    }
    return ESP_OK;
}

static esp_vhci_host_callback_t vhci_host_cb = {
    .notify_host_send_available = controller_rcv_pkt_ready,
    .notify_host_recv = host_rcv_pkt
};

// Command sending functions remain the same
static void hci_cmd_send_reset(void) {
    uint16_t sz = make_cmd_reset(hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_set_evt_mask(void) {
    uint8_t evt_mask[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20};
    uint16_t sz = make_cmd_set_evt_mask(hci_cmd_buf, evt_mask);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_scan_params(void) {
    uint16_t sz = make_cmd_ble_set_scan_params(hci_cmd_buf, 0x01, 0x50, 0x30, 0x00, 0x00);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_scan_start(void) {
    uint16_t sz = make_cmd_ble_set_scan_enable(hci_cmd_buf, 0x01, 0x00);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
    ESP_LOGI(TAG, "BLE Scanning started");
}

// Task xử lý sequence command BLE
static void ble_cmd_sequence_task(void *pvParameters)
{
    const int total_commands = 4;
    
    for (int cmd_cnt = 0; cmd_cnt < total_commands; cmd_cnt++) {
        while (!esp_vhci_host_check_send_available()) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        switch (cmd_cnt) {
            case 0: hci_cmd_send_reset(); break;
            case 1: hci_cmd_send_set_evt_mask(); break;
            case 2: hci_cmd_send_ble_scan_params(); break;
            case 3: hci_cmd_send_ble_scan_start(); break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(TAG, "BLE command sequence completed");
    vTaskDelete(NULL);
}

static void hci_evt_process(void *pvParameters) {
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize AI devices based on current swap mode
    update_ai_devices_by_swap_mode(g_tpms_config.tire_swap);

    // Khởi tạo hàng đợi âm thanh trước
    init_audio_queue();
    
    // Tạo các task
    xTaskCreate(dfplayer_init_task, "dfplayer_init", 2048, NULL, 4, NULL);
    xTaskCreate(audio_task, "audio_task", 2048, NULL, 5, NULL);
    xTaskCreate(cooldown_manager_task, "cooldown_mgr", 2048, NULL, 2, NULL);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_vhci_host_register_callback(&vhci_host_cb));

    // Chạy sequence command trong task riêng
    xTaskCreate(ble_cmd_sequence_task, "ble_cmd_seq", 2048, NULL, 3, NULL);

    xTaskCreatePinnedToCore(hci_evt_process, "hci_evt_process", 2048, NULL, 6, NULL, 0);
    
    ESP_LOGI(TAG, "All tasks started successfully");
}