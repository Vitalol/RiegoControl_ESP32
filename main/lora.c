

#include "lora.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "measurements.h"
#include "protocol.h"
#include "sx127x.h"

// Definitions
#define LORA_TSK_SIZE_TX  (10 * 1024)
#define LORA_TSK_SIZE_RX  (10 * 1024)
#define LORA_MAX_PKT_SIZE 255
#define LORA_HEADER_SIZE  4

#define LORA_SINK_ADDR 0

#define LORA_RECEIVER_INDX     0
#define LORA_SENDER_INDX       1
#define LORA_MSG_PROTOCOL_INDX 2
#define LORA_SIZE_INDX         3
#define LORA_DATA_INIT_INDX    4

#define TIME_KEEP_ALIVE (15)  // in seconds
// Typedef

typedef struct lora_task_handler {
    node_handler_t node_h;  // Node info, address and mode
    measure_handler_t
        measure_h;  // measure handler, contains the measures queue
    SemaphoreHandle_t mutex_lora;  // Mutex to access lora hardware
    SemaphoreHandle_t
           rx_task_semaphore;    // Semaphore to controll rx_task from tx_task
    time_t rx_receive_duration;  // time receiving after sending msg

} lora_task_handler;

// Prototypes
void task_lora_tx(void *pvParameters);
void task_lora_rx(void *pvParameters);
int  lora_rx_wait(lora_task_handler *handler);
int  lora_rx_notify(lora_task_handler *handler);

// PORT

void lora_send_packet(lora_task_handler *handler, uint8_t *buf, int size) {
    xSemaphoreTake(handler->mutex_lora, portMAX_DELAY);
    sx127x_send_packet(buf, size);
    xSemaphoreGive(handler->mutex_lora);
}

int lora_receive_packet(lora_task_handler *handler, uint8_t *buf, int size) {
    xSemaphoreTake(handler->mutex_lora, portMAX_DELAY);
    int ret = sx127x_receive_packet(buf, size);
    xSemaphoreGive(handler->mutex_lora);
    return ret;
}

int lora_received(lora_task_handler *handler) {
    xSemaphoreTake(handler->mutex_lora, portMAX_DELAY);
    int ret = sx127x_received();
    xSemaphoreGive(handler->mutex_lora);
    return ret;
}

void lora_receive(lora_task_handler *handler) {
    xSemaphoreTake(handler->mutex_lora, portMAX_DELAY);
    sx127x_receive();
    xSemaphoreGive(handler->mutex_lora);
}

//

void lora_init(sx127xConf_t sx127x_configuration) {
    // Recognize module
    do {
        ESP_LOGE(pcTaskGetName(NULL), "Does not recognize sx127x module");
        vTaskDelay(1000 / portTICK_RATE_MS);
    } while ((sx127x_init() == 0));
    ESP_LOGI(pcTaskGetName(NULL), "sx127x module recognized");

    // sx127x configuration
    sx127x_enable_crc();

    sx127x_configuration.frequency       = FREQ_915MHz;
    sx127x_configuration.codingRate      = 5;
    sx127x_configuration.bandwith        = 7;
    sx127x_configuration.spreadingFactor = 7;
    sx127x_config(sx127x_configuration);
}

void lora_init_task(node_handler_t node_h, measure_handler_t measure_h) {
    sx127xConf_t sx127x_configuration;
    lora_init(sx127x_configuration);

    static lora_task_handler lora_handler = {0};

    lora_handler.node_h              = node_h;
    lora_handler.measure_h           = measure_h;
    lora_handler.mutex_lora          = xSemaphoreCreateMutex();
    lora_handler.rx_task_semaphore   = xSemaphoreCreateBinary();
    lora_handler.rx_receive_duration = 10;  // in seconds

    ESP_LOGI(pcTaskGetName(NULL), "Coge semaforo de rx ?");
    xTaskCreate(&task_lora_tx, "task_lora_tx", LORA_TSK_SIZE_TX,
                (void *)&lora_handler, 5, NULL);

    xTaskCreate(&task_lora_rx, "task_lora_rx", LORA_TSK_SIZE_RX,
                (void *)&lora_handler, 5, NULL);
}

void task_lora_tx(void *pvParameters) {
    lora_task_handler lora_handler    = *(lora_task_handler *)pvParameters;
    node_handler_t    node            = lora_handler.node_h;
    measure_handler_t measure_handler = lora_handler.measure_h;
    int               print_offset    = 0;
    char              lora_pkt_string[1024] = {0};

    time_t time_last    = 0;
    time_t time_current = 0;

    int msg_to_send = PROTOCOL_NONE;
    while (1) {
        if (measurements_wait(&measure_handler,
                              (2 * 1000) / portTICK_RATE_MS) ==
            MEASURE_READY_FLAG) {
            msg_to_send = PROTOCOL_MSG_SEND_MEASURE;
            ESP_LOGI(pcTaskGetName(NULL), "Se manda medida");
        }

        time(&time_current);
        int diff_time = time_current - time_last;
        ESP_LOGI(pcTaskGetName(NULL), "Diff time_current time_last = %d",
                 (diff_time));
        if (time_current - time_last > TIME_KEEP_ALIVE) {
            time(&time_last);
            msg_to_send = PROTOCOL_MSG_SET_TIME;
            ESP_LOGI(pcTaskGetName(NULL), "Se pide hora");
        }
        switch (msg_to_send) {
            case PROTOCOL_MSG_SET_TIME:
                // Periodically we send our current time to the sinker
                // the sinker will respond with the correct current time
                msg_to_send                        = 0;
                protocol_set_hour_str msg_set_time = {0};
                msg_set_time.header.destination    = LORA_SINK_ADDR;
                msg_set_time.header.origin         = conf_get_node_addr(node);
                msg_set_time.header.type           = PROTOCOL_MSG_SET_TIME;
                msg_set_time.header.lenght = sizeof(protocol_set_hour_str);

                time_t rawtime;
                time(&rawtime);

                msg_set_time.time = rawtime;
                // Print msg
                print_offset = 0;

                for (int i = 0; i < (msg_set_time.header.lenght); i++) {
                    print_offset +=
                        sprintf(&lora_pkt_string[print_offset], "%02X ",
                                ((uint8_t *)&msg_set_time)[i]);
                }

                ESP_LOGI(pcTaskGetName(NULL), "lora_pkt(%d) = %s",
                         msg_set_time.header.lenght, lora_pkt_string);
                lora_send_packet(&lora_handler, (uint8_t *)&msg_set_time,
                                 (int)msg_set_time.header.lenght);

                lora_rx_notify(&lora_handler);  // we ex

                break;

            case PROTOCOL_MSG_SEND_MEASURE:
                msg_to_send                           = 0;
                protocol_send_measure_str msg_measure = {0};
                msg_measure.header.destination        = LORA_SINK_ADDR;
                msg_measure.header.origin = conf_get_node_addr(node);
                msg_measure.header.type   = PROTOCOL_MSG_SEND_MEASURE;
                // Get measures
                int       measures_count = 0;
                measure_t measure;
                while (measurements_get(&measure_handler, &measure) > 0) {
                    msg_measure.measures[measures_count++] = measure;
                }
                // Finish filling msg
                msg_measure.header.lenght = sizeof(protocol_header_str) +
                                            measures_count * sizeof(measure_t);

                // Print msg
                print_offset = 0;

                for (int i = 0; i < (msg_measure.header.lenght); i++) {
                    print_offset +=
                        sprintf(&lora_pkt_string[print_offset], "%02X ",
                                ((uint8_t *)&msg_measure)[i]);
                }

                ESP_LOGI(pcTaskGetName(NULL), "lora_pkt(%d) = %s",
                         msg_measure.header.lenght, lora_pkt_string);
                // Send by lora
                lora_send_packet(&lora_handler, (uint8_t *)&msg_measure,
                                 (int)msg_measure.header.lenght);
                break;

        }  // end switch

    }  // end while
}

int lora_rx_wait(lora_task_handler *handler) {
    return xSemaphoreTake(handler->rx_task_semaphore, portMAX_DELAY);
}

int lora_rx_notify(lora_task_handler *handler) {
    return xSemaphoreGive(handler->rx_task_semaphore);
}

void task_lora_rx(void *pvParameters) {
    lora_task_handler lora_handler = *(lora_task_handler *)pvParameters;
    node_handler_t    node         = lora_handler.node_h;
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    uint8_t buf[256];  // Maximum Payload size of SX1276/77/78/79 is 255
    int     task_lora_rx_running = 1;
    time_t  time_start           = 0;
    time_t  time_current         = 0;
    time_t  time_received        = 0;

    while (task_lora_rx_running) {
        lora_rx_wait(&lora_handler);
        ESP_LOGW(pcTaskGetName(NULL), "Receiving...");

        time(&time_start);
        time_current = time_start;
        lora_receive(&lora_handler);  // put into receive mode

        while ((time_current - time_start) < lora_handler.rx_receive_duration) {
            time(&time_current);
            if (lora_received(&lora_handler)) {
                int receive_len =
                    lora_receive_packet(&lora_handler, buf, sizeof(buf));
                for (int i = 0; i < receive_len; i++) {
                    printf("%.2X ", buf[i]);
                }
                printf("\n");
                ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s]",
                         receive_len, receive_len, buf);

                if (buf[LORA_RECEIVER_INDX] == conf_get_node_addr(node)) {
                    ESP_LOGI(pcTaskGetName(NULL), "MSG is for this node");

                    switch (buf[LORA_MSG_PROTOCOL_INDX]) {
                        case PROTOCOL_MSG_SET_TIME:
                            time_received = *(int *)(&buf[LORA_DATA_INIT_INDX]);
                            struct timeval tv = {0};
                            tv.tv_sec         = (time_t)time_received;
                            ESP_LOGI(pcTaskGetName(NULL), "Change time to %d",
                                     (int)time_received);
                            settimeofday(&tv, NULL);
                            break;

                        default:
                            break;
                    }
                }
            }
            vTaskDelay(2);
        }
        ESP_LOGW(pcTaskGetName(NULL), "Stop Receiving...");
    }
}
