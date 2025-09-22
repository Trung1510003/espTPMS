/*
 * BLE Combined Advertising and Scanning Example with Sensor Data Parsing.
 * Added device identification and BLE forwarding functionality.
 *
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
 #include <stdio.h>
 #include <string.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_bt.h"
 #include "esp_log.h"
 #include "esp_timer.h"
 #include "nvs_flash.h"
 #include "freertos/queue.h"
 #include "bt_hci_common.h"
 
 static const char *TAG = "BLE_ADV_SCAN";
 
 // Định nghĩa cấu trúc cho thiết bị AI-8000
 typedef struct {
     char address[18];  // Địa chỉ Bluetooth dạng string
     char name[16];     // Tên hiển thị
     int queue_index;   // Chỉ số queue tương ứng
 } ai_device_t;
 
 // Danh sách các thiết bị AI-8000
 static const ai_device_t ai_devices[] = {
     {"12:30:af:00:01:5e", "Trái-Sau", 0},
     {"12:30:af:00:00:f0", "Phải-Sau", 1},
     {"12:30:af:00:01:59", "Trái-Trước", 2},
     {"12:30:af:00:01:46", "Phải-Trước", 3}
 };
 
 typedef struct {
     char scan_local_name[32];
     uint8_t name_len;
 } ble_scan_local_name_t;
 
 typedef struct {
     uint8_t *q_data;
     uint16_t q_data_len;
 } host_rcv_data_t;
 
 typedef struct {
     uint8_t temperature;
     uint64_t pressure;
     char device_name[16];  // Tên thiết bị (Trái-Sau, Phải-Sau, etc.)
     char address[18];      // Địa chỉ thiết bị
 } sensor_data_t;
 
 // Hàng đợi để chuyển tiếp dữ liệu cảm biến
 static QueueHandle_t sensor_data_queue;
 
 // Hàng đợi riêng cho từng thiết bị
 static QueueHandle_t device_queues[4]; // 4 queues tương ứng với 4 thiết bị
 static const int QUEUE_SIZE_PER_DEVICE = 5; // Kích thước queue cho mỗi thiết bị
 
 static uint8_t hci_cmd_buf[128];
 
 static uint16_t scanned_count = 0;
 static QueueHandle_t adv_queue;
 
 // Biến để theo dõi thời gian cập nhật cuối cùng cho mỗi thiết bị
 static uint32_t last_update_time[4] = {0};
 
 static void periodic_timer_callback(void *arg)
 {
     ESP_LOGI(TAG, "Number of received advertising reports: %d", scanned_count);
     
     // Hiển thị thông tin về số lượng phần tử trong mỗi queue
     for (int i = 0; i < 4; i++) {
         UBaseType_t items_in_queue = uxQueueMessagesWaiting(device_queues[i]);
         ESP_LOGI(TAG, "Queue %s: %d items", ai_devices[i].name, items_in_queue);
     }
 }
 
 /*
  * @brief: Chuyển đổi địa chỉ từ mảng byte sang chuỗi
  */
 static void address_to_string(uint8_t *addr, char *str, size_t str_len)
 {
     snprintf(str, str_len, "%02x:%02x:%02x:%02x:%02x:%02x",
              addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
 }
 
 /*
  * @brief: Xác định tên thiết bị dựa trên địa chỉ
  */
 static const ai_device_t* get_device_by_address(uint8_t *addr)
 {
     char addr_str[18];
     address_to_string(addr, addr_str, sizeof(addr_str));
     
     for (int i = 0; i < sizeof(ai_devices)/sizeof(ai_devices[0]); i++) {
         if (strcmp(addr_str, ai_devices[i].address) == 0) {
             return &ai_devices[i];
         }
     }
     
     return NULL;
 }
 
 /*
  * @brief: Parse sensor data from the last 21 bytes of raw data
  * Format: a0 a1 a2 a3 a4 a5 a6 a7 a8 a9 a10 a11 a12 a13 a14 a15 a16 a17 a18 a19 a20 a21
  * Temperature: a1 (hex to decimal)
  * Pressure: a6-a21 (16 bytes, hex to decimal)
  */
 static sensor_data_t parse_sensor_data(uint8_t *raw_data, uint8_t data_len, uint8_t *addr)
 {
     sensor_data_t sensor_data = {0};
     
     // Ensure we have at least 21 bytes
     if (data_len < 21) {
         ESP_LOGW(TAG, "Insufficient data length: %d, expected at least 21 bytes", data_len);
         return sensor_data;
     }
     
     // Get the last 21 bytes
     uint8_t *sensor_bytes = &raw_data[data_len - 21];
     
     // Parse temperature from a1 (second byte of the 21-byte sequence)
     sensor_data.temperature = sensor_bytes[1];
     
     // Parse pressure from a6-a21 (bytes 5-20 of the 21-byte sequence, 16 bytes total)
     sensor_data.pressure = 0;
     for (int i = 0; i < 16; i++) {
         sensor_data.pressure = (sensor_data.pressure << 8) | sensor_bytes[5 + i];
     }
     
     // Lấy thông tin thiết bị từ địa chỉ
     const ai_device_t *device = get_device_by_address(addr);
     if (device != NULL) {
         strncpy(sensor_data.device_name, device->name, sizeof(sensor_data.device_name) - 1);
         sensor_data.device_name[sizeof(sensor_data.device_name) - 1] = '\0';
     } else {
         strcpy(sensor_data.device_name, "Unknown");
     }
     
     // Lưu địa chỉ thiết bị
     address_to_string(addr, sensor_data.address, sizeof(sensor_data.address));
     
     return sensor_data;
 }
 
 /*
  * @brief: Gửi dữ liệu cảm biến đến queue tương ứng của thiết bị
  */
 static void send_to_device_queue(const sensor_data_t *sensor_data, int queue_index)
 {
     if (queue_index >= 0 && queue_index < 4) {
         if (xQueueSend(device_queues[queue_index], sensor_data, pdMS_TO_TICKS(100)) != pdTRUE) {
             ESP_LOGW(TAG, "Queue for %s is full, dropping data", sensor_data->device_name);
         } else {
             ESP_LOGI(TAG, "Data sent to %s queue successfully", sensor_data->device_name);
         }
     } else {
         ESP_LOGE(TAG, "Invalid queue index: %d", queue_index);
     }
 }
 
 /*
  * @brief: BT controller callback function, used to notify the upper layer that
  *         controller is ready to receive command
  */
 static void controller_rcv_pkt_ready(void)
 {
     ESP_LOGI(TAG, "controller rcv pkt ready");
 }
 
 /*
  * @brief: BT controller callback function to transfer data packet to
  *         the host
  */
 static int host_rcv_pkt(uint8_t *data, uint16_t len)
 {
     host_rcv_data_t send_data;
     uint8_t *data_pkt;
     /* Check second byte for HCI event. If event opcode is 0x0e, the event is
      * HCI Command Complete event. Sice we have received "0x0e" event, we can
      * check for byte 4 for command opcode and byte 6 for it's return status. */
     if (data[1] == 0x0e) {
         if (data[6] == 0) {
             esp_rom_printf("Event opcode 0x%02x success.", data[4]);
         } else {
             esp_rom_printf("Event opcode 0x%02x fail with reason: 0x%02x.", data[4], data[6]);
             return ESP_FAIL;
         }
     }
 
     data_pkt = (uint8_t *)malloc(sizeof(uint8_t) * len);
     if (data_pkt == NULL) {
         esp_rom_printf("Malloc data_pkt failed!");
         return ESP_FAIL;
     }
     memcpy(data_pkt, data, len);
     send_data.q_data = data_pkt;
     send_data.q_data_len = len;
     if (xQueueSend(adv_queue, (void *)&send_data, ( TickType_t ) 0) != pdTRUE) {
         esp_rom_printf("Failed to enqueue advertising report. Queue full.");
         /* If data sent successfully, then free the pointer in `xQueueReceive'
          * after processing it. Or else if enqueue in not successful, free it
          * here. */
         free(data_pkt);
     }
     return ESP_OK;
 }
 
 static esp_vhci_host_callback_t vhci_host_cb = {
     controller_rcv_pkt_ready,
     host_rcv_pkt
 };
 
 static void hci_cmd_send_reset(void)
 {
     uint16_t sz = make_cmd_reset (hci_cmd_buf);
     esp_vhci_host_send_packet(hci_cmd_buf, sz);
 }
 
 static void hci_cmd_send_set_evt_mask(void)
 {
     /* Set bit 61 in event mask to enable LE Meta events. */
     uint8_t evt_mask[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20};
     uint16_t sz = make_cmd_set_evt_mask(hci_cmd_buf, evt_mask);
     esp_vhci_host_send_packet(hci_cmd_buf, sz);
 }
 
 static void hci_cmd_send_ble_scan_params(void)
 {
     /* Set scan type to 0x01 for active scanning and 0x00 for passive scanning. */
     uint8_t scan_type = 0x01;
 
     /* Scan window and Scan interval are set in terms of number of slots. Each slot is of 625 microseconds. */
     uint16_t scan_interval = 0x50; /* 50 ms */
     uint16_t scan_window = 0x30; /* 30 ms */
 
     uint8_t own_addr_type = 0x00; /* Public Device Address (default). */
     uint8_t filter_policy = 0x00; /* Accept all packets except directed advertising packets (default). */
     uint16_t sz = make_cmd_ble_set_scan_params(hci_cmd_buf, scan_type, scan_interval, scan_window, own_addr_type, filter_policy);
     esp_vhci_host_send_packet(hci_cmd_buf, sz);
 }
 
 static void hci_cmd_send_ble_scan_start(void)
 {
     uint8_t scan_enable = 0x01; /* Scanning enabled. */
     uint8_t filter_duplicates = 0x00; /* Duplicate filtering disabled. */
     uint16_t sz = make_cmd_ble_set_scan_enable(hci_cmd_buf, scan_enable, filter_duplicates);
     esp_vhci_host_send_packet(hci_cmd_buf, sz);
     ESP_LOGI(TAG, "BLE Scanning started..");
 }
 
 static void hci_cmd_send_ble_adv_start(void)
 {
     uint16_t sz = make_cmd_ble_set_adv_enable (hci_cmd_buf, 1);
     esp_vhci_host_send_packet(hci_cmd_buf, sz);
     ESP_LOGI(TAG, "BLE Advertising started..");
 }
 
 static void hci_cmd_send_ble_set_adv_param(void)
 {
     /* Minimum and maximum Advertising interval are set in terms of slots. Each slot is of 625 microseconds. */
     uint16_t adv_intv_min = 0x100;
     uint16_t adv_intv_max = 0x100;
 
     /* Connectable undirected advertising (ADV_IND). */
     uint8_t adv_type = 0;
 
     /* Own address is public address. */
     uint8_t own_addr_type = 0;
 
     /* Public Device Address */
     uint8_t peer_addr_type = 0;
     uint8_t peer_addr[6] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85};
 
     /* Channel 37, 38 and 39 for advertising. */
     uint8_t adv_chn_map = 0x07;
 
     /* Process scan and connection requests from all devices (i.e., the White List is not in use). */
     uint8_t adv_filter_policy = 0;
 
     uint16_t sz = make_cmd_ble_set_adv_param(hci_cmd_buf,
                   adv_intv_min,
                   adv_intv_max,
                   adv_type,
                   own_addr_type,
                   peer_addr_type,
                   peer_addr,
                   adv_chn_map,
                   adv_filter_policy);
     esp_vhci_host_send_packet(hci_cmd_buf, sz);
 }
 
 static void hci_cmd_send_ble_set_adv_data(void)
 {
     char *adv_name = "ESP-BLE-1";
     uint8_t name_len = (uint8_t)strlen(adv_name);
     uint8_t adv_data[31] = {0x02, 0x01, 0x06, 0x0, 0x09};
     uint8_t adv_data_len;
 
     adv_data[3] = name_len + 1;
     for (int i = 0; i < name_len; i++) {
         adv_data[5 + i] = (uint8_t)adv_name[i];
     }
     adv_data_len = 5 + name_len;
 
     uint16_t sz = make_cmd_ble_set_adv_data(hci_cmd_buf, adv_data_len, (uint8_t *)adv_data);
     esp_vhci_host_send_packet(hci_cmd_buf, sz);
     ESP_LOGI(TAG, "Starting BLE advertising with name \"%s\"", adv_name);
 }
 
 static esp_err_t get_local_name (uint8_t *data_msg, uint8_t data_len, ble_scan_local_name_t *scanned_packet)
 {
     uint8_t curr_ptr = 0, curr_len, curr_type;
     while (curr_ptr < data_len) {
         curr_len = data_msg[curr_ptr++];
         curr_type = data_msg[curr_ptr++];
         if (curr_len == 0) {
             return ESP_FAIL;
         }
 
         /* Search for current data type and see if it contains name as data (0x08 or 0x09). */
         if (curr_type == 0x08 || curr_type == 0x09) {
             for (uint8_t i = 0; i < curr_len - 1; i += 1) {
                 scanned_packet->scan_local_name[i] = data_msg[curr_ptr + i];
             }
             scanned_packet->name_len = curr_len - 1;
             return ESP_OK;
         } else {
             /* Search for next data. Current length includes 1 octate for AD Type (2nd octate). */
             curr_ptr += curr_len - 1;
         }
     }
     return ESP_FAIL;
 }
 
 /*
  * @brief: Nhiệm vụ để chuyển tiếp dữ liệu cảm biến qua BLE
  */
 static void sensor_data_forwarding_task(void *pvParameters)
 {
     sensor_data_t sensor_data;
     
     while (1) {
         if (xQueueReceive(sensor_data_queue, &sensor_data, portMAX_DELAY) == pdTRUE) {
             // Ở đây bạn có thể thực hiện chuyển tiếp dữ liệu qua BLE
             // Ví dụ: đóng gói dữ liệu và gửi qua kênh BLE advertising hoặc GATT
             
             ESP_LOGI(TAG, "Forwarding data - Device: %s, Temp: %u°C, Pressure: %llu Pa", 
                      sensor_data.device_name, sensor_data.temperature, sensor_data.pressure);
             
             // Có thể thêm code để gửi dữ liệu qua BLE ở đây
             // Ví dụ: cập nhật advertising data với thông tin mới
         }
     }
 }
 
 /*
  * @brief: Nhiệm vụ xử lý dữ liệu cho từng thiết bị riêng biệt
  */
 static void device_data_processing_task(void *pvParameters)
 {
     int device_index = (int)pvParameters;
     const char *device_name = ai_devices[device_index].name;
     sensor_data_t sensor_data;
     
     ESP_LOGI(TAG, "Starting data processing task for %s (Queue %d)", device_name, device_index);
     
     while (1) {
         if (xQueueReceive(device_queues[device_index], &sensor_data, portMAX_DELAY) == pdTRUE) {
             // Xử lý dữ liệu cho thiết bị cụ thể
             ESP_LOGI(TAG, "[%s] Processing - Temp: %u°C, Pressure: %llu Pa", 
                      device_name, sensor_data.temperature, sensor_data.pressure);
             
             // Ở đây bạn có thể thêm xử lý cụ thể cho từng thiết bị
             // Ví dụ: áp dụng các bộ lọc khác nhau, ngưỡng cảnh báo khác nhau, etc.
             
             // Cập nhật thời gian cuối cùng
             last_update_time[device_index] = xTaskGetTickCount();
         }
     }
 }
 
 void hci_evt_process(void *pvParameters)
 {
     host_rcv_data_t *rcv_data = (host_rcv_data_t *)malloc(sizeof(host_rcv_data_t));
     if (rcv_data == NULL) {
         ESP_LOGE(TAG, "Malloc rcv_data failed!");
         return;
     }
     esp_err_t ret;
 
     while (1) {
         uint8_t sub_event, num_responses, total_data_len, data_msg_ptr, hci_event_opcode;
         uint8_t *queue_data = NULL, *event_type = NULL, *addr_type = NULL, *addr = NULL, *data_len = NULL, *data_msg = NULL;
         short int *rssi = NULL;
         uint16_t data_ptr;
         ble_scan_local_name_t *scanned_name = NULL;
         total_data_len = 0;
         data_msg_ptr = 0;
         if (xQueueReceive(adv_queue, rcv_data, portMAX_DELAY) != pdPASS) {
             ESP_LOGE(TAG, "Queue receive error");
         } else {
             /* `data_ptr' keeps track of current position in the received data. */
             data_ptr = 0;
             queue_data = rcv_data->q_data;
 
             /* Parsing `data' and copying in various fields. */
             hci_event_opcode = queue_data[++data_ptr];
             if (hci_event_opcode == LE_META_EVENTS) {
                 /* Set `data_ptr' to 4th entry, which will point to sub event. */
                 data_ptr += 2;
                 sub_event = queue_data[data_ptr++];
                 /* Check if sub event is LE advertising report event. */
                 if (sub_event == HCI_LE_ADV_REPORT) {
 
                     scanned_count += 1;
 
                     /* Get number of advertising reports. */
                     num_responses = queue_data[data_ptr++];
                     event_type = (uint8_t *)malloc(sizeof(uint8_t) * num_responses);
                     if (event_type == NULL) {
                         ESP_LOGE(TAG, "Malloc event_type failed!");
                         goto reset;
                     }
                     for (uint8_t i = 0; i < num_responses; i += 1) {
                         event_type[i] = queue_data[data_ptr++];
                     }
 
                     /* Get advertising type for every report. */
                     addr_type = (uint8_t *)malloc(sizeof(uint8_t) * num_responses);
                     if (addr_type == NULL) {
                         ESP_LOGE(TAG, "Malloc addr_type failed!");
                         goto reset;
                     }
                     for (uint8_t i = 0; i < num_responses; i += 1) {
                         addr_type[i] = queue_data[data_ptr++];
                     }
 
                     /* Get BD address in every advetising report and store in
                      * single array of length `6 * num_responses' as each address
                      * will take 6 spaces. */
                     addr = (uint8_t *)malloc(sizeof(uint8_t) * 6 * num_responses);
                     if (addr == NULL) {
                         ESP_LOGE(TAG, "Malloc addr failed!");
                         goto reset;
                     }
                     for (int i = 0; i < num_responses; i += 1) {
                         for (int j = 0; j < 6; j += 1) {
                             addr[(6 * i) + j] = queue_data[data_ptr++];
                         }
                     }
 
                     /* Get length of data for each advertising report. */
                     data_len = (uint8_t *)malloc(sizeof(uint8_t) * num_responses);
                     if (data_len == NULL) {
                         ESP_LOGE(TAG, "Malloc data_len failed!");
                         goto reset;
                     }
                     for (uint8_t i = 0; i < num_responses; i += 1) {
                         data_len[i] = queue_data[data_ptr];
                         total_data_len += queue_data[data_ptr++];
                     }
 
                     if (total_data_len != 0) {
                         /* Get all data packets. */
                         data_msg = (uint8_t *)malloc(sizeof(uint8_t) * total_data_len);
                         if (data_msg == NULL) {
                             ESP_LOGE(TAG, "Malloc data_msg failed!");
                             goto reset;
                         }
                         for (uint8_t i = 0; i < num_responses; i += 1) {
                             for (uint8_t j = 0; j < data_len[i]; j += 1) {
                                 data_msg[data_msg_ptr++] = queue_data[data_ptr++];
                             }
                         }
                     }
 
                     /* Counts of advertisements done. This count is set in advertising data every time before advertising. */
                     rssi = (short int *)malloc(sizeof(short int) * num_responses);
                     if (rssi == NULL) {
                         ESP_LOGE(TAG, "Malloc rssi failed!");
                         goto reset;
                     }
                     for (uint8_t i = 0; i < num_responses; i += 1) {
                         rssi[i] = -(0xFF - queue_data[data_ptr++]);
                     }
 
                     /* Extracting advertiser's name. */
                     data_msg_ptr = 0;
                     scanned_name = (ble_scan_local_name_t *)malloc(num_responses * sizeof(ble_scan_local_name_t));
                     if (scanned_name == NULL) {
                         ESP_LOGE(TAG, "Malloc scanned_name failed!");
                         goto reset;
                     }
                     
                     for (uint8_t i = 0; i < num_responses; i += 1) {
                         ret = get_local_name(&data_msg[data_msg_ptr], data_len[i], &scanned_name[i]);
 
                         /* Check if this is AI-8000 device and parse sensor data */
                         if (ret == ESP_OK) {
                             scanned_name[i].scan_local_name[scanned_name[i].name_len] = '\0';
                             if (strcmp(scanned_name[i].scan_local_name, "AI-8000") == 0) {
                                 printf("======== AI-8000 Device Found ========\n");
                                 printf("Address: ");
                                 for (int j = 5; j >= 0; j--) {
                                     printf("%02x", addr[(6 * i) + j]);
                                     if (j > 0) printf(":");
                                 }
                                 printf("\nRSSI: %ddB\n", rssi[i]);
 
                                 // Parse sensor data from the complete raw data (21 bytes cuối)
                                 sensor_data_t sensor_data = parse_sensor_data(queue_data, rcv_data->q_data_len, &addr[6 * i]);
                                 
                                 printf("Device: %s\n", sensor_data.device_name);
                                 printf("Temperature: %u°C\n", sensor_data.temperature);
                                 printf("Pressure: %llu Pa\n", sensor_data.pressure);
                                 
                                 // Gửi dữ liệu đến hàng đợi chính
                                 xQueueSend(sensor_data_queue, &sensor_data, portMAX_DELAY);
                                 
                                 // Gửi dữ liệu đến queue tương ứng của thiết bị
                                 const ai_device_t *device = get_device_by_address(&addr[6 * i]);
                                 if (device != NULL) {
                                     send_to_device_queue(&sensor_data, device->queue_index);
                                 } else {
                                     ESP_LOGW(TAG, "Unknown device, data not sent to device queue");
                                 }
                                 
                                 // Debug: Print the 21 bytes being parsed
                                 printf("Parsed bytes: ");
                                 uint8_t *sensor_bytes = &queue_data[rcv_data->q_data_len - 21];
                                 for (int k = 0; k < 21; k++) {
                                     printf("%02x ", sensor_bytes[k]);
                                 }
                                 printf("\n");
                                 printf("=====================================\n");
                             }
                         }
                         data_msg_ptr += data_len[i];
                     }
 
                     /* Freeing all spaces allocated. */
 reset:
                     free(scanned_name);
                     free(rssi);
                     free(data_msg);
                     free(data_len);
                     free(addr);
                     free(addr_type);
                     free(event_type);
                 }
             }
 #if (CONFIG_LOG_DEFAULT_LEVEL_DEBUG || CONFIG_LOG_DEFAULT_LEVEL_VERBOSE)
             printf("Raw Data:");
             for (uint8_t j = 0; j < rcv_data->q_data_len; j += 1) {
                 printf(" %02x", queue_data[j]);
             }
             printf("\nQueue free size: %d\n", uxQueueSpacesAvailable(adv_queue));
 #endif
             free(queue_data);
         }
         memset(rcv_data, 0, sizeof(host_rcv_data_t));
     }
 }
 
 void app_main(void)
 {
     bool continue_commands = 1;
     int cmd_cnt = 0;
 
     const esp_timer_create_args_t periodic_timer_args = {
         .callback = &periodic_timer_callback,
         .name = "periodic"
     };
 
     /* Create timer for logging scanned devices. */
     esp_timer_handle_t periodic_timer;
     ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
 
     /* Start periodic timer for 5 sec. */
     ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 5000000));
 
     // Tạo hàng đợi cho dữ liệu cảm biến chính
     sensor_data_queue = xQueueCreate(10, sizeof(sensor_data_t));
     if (sensor_data_queue == NULL) {
         ESP_LOGE(TAG, "Failed to create sensor data queue");
         return;
     }
 
     // Tạo hàng đợi riêng cho từng thiết bị
     for (int i = 0; i < 4; i++) {
         device_queues[i] = xQueueCreate(QUEUE_SIZE_PER_DEVICE, sizeof(sensor_data_t));
         if (device_queues[i] == NULL) {
             ESP_LOGE(TAG, "Failed to create queue for device %s", ai_devices[i].name);
             return;
         }
         ESP_LOGI(TAG, "Created queue for %s (size: %d)", ai_devices[i].name, QUEUE_SIZE_PER_DEVICE);
     }
 
     /* Initialize NVS — it is used to store PHY calibration data */
     esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         ESP_ERROR_CHECK(nvs_flash_erase());
         ret = nvs_flash_init();
     }
     ESP_ERROR_CHECK( ret );
     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
 
     ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
     if (ret) {
         ESP_LOGI(TAG, "Bluetooth controller release classic bt memory failed: %s", esp_err_to_name(ret));
         return;
     }
 
     if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
         ESP_LOGI(TAG, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
         return;
     }
 
     if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
         ESP_LOGI(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
         return;
     }
 
     /* A queue for storing received HCI packets. */
     adv_queue = xQueueCreate(15, sizeof(host_rcv_data_t));
     if (adv_queue == NULL) {
         ESP_LOGE(TAG, "Queue creation failed");
         return;
     }
 
     esp_vhci_host_register_callback(&vhci_host_cb);
     
     // Tạo task để xử lý chuyển tiếp dữ liệu chính
     xTaskCreatePinnedToCore(&sensor_data_forwarding_task, "sensor_data_forwarding", 2048, NULL, 5, NULL, 0);
     
     // Tạo task riêng cho từng thiết bị
     for (int i = 0; i < 4; i++) {
         char task_name[32];
         snprintf(task_name, sizeof(task_name), "device_task_%d", i);
         xTaskCreatePinnedToCore(&device_data_processing_task, task_name, 2048, (void *)i, 5, NULL, 0);
     }
     
     while (continue_commands) {
         if (continue_commands && esp_vhci_host_check_send_available()) {
             switch (cmd_cnt) {
             case 0: hci_cmd_send_reset(); ++cmd_cnt; break;
             case 1: hci_cmd_send_set_evt_mask(); ++cmd_cnt; break;
 
             /* Send advertising commands. */
             case 2: hci_cmd_send_ble_set_adv_param(); ++cmd_cnt; break;
             case 3: hci_cmd_send_ble_set_adv_data(); ++cmd_cnt; break;
             case 4: hci_cmd_send_ble_adv_start(); ++cmd_cnt; break;
 
             /* Send scan commands. */
             case 5: hci_cmd_send_ble_scan_params(); ++cmd_cnt; break;
             case 6: hci_cmd_send_ble_scan_start(); ++cmd_cnt; break;
             default: continue_commands = 0; break;
             }
             ESP_LOGI(TAG, "BLE Advertise, cmd_sent: %d", cmd_cnt);
         }
         vTaskDelay(1000 / portTICK_PERIOD_MS);
     }
     xTaskCreatePinnedToCore(&hci_evt_process, "hci_evt_process", 2048, NULL, 6, NULL, 0);
 }