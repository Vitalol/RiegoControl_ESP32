#include "lora.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "sx127x.h"

// Definitions

#define LORA_TSK_SIZE          (10 * 1024)
#define LORA_MAX_PKT_SIZE      255
#define LORA_HEADER_SIZE       4
#define LORA_SINK_ADDR         0
#define LORA_RECEIVER_INDX     0
#define LORA_SENDER_INDX       1
#define LORA_MSG_PROTOCOL_INDX 2
#define LORA_SIZE_INDX         3

// Typedef

typedef struct lora_task_parameters {
    node_handler_t    node_h;
    measure_handler_t measure_h;
} lora_task_parameters;

// Prototypes
void task_lora_comm(void *pvParameters);

// PORT

void lora_send_packet(uint8_t *buf, int size) { sx127x_send_packet(buf, size); }

int lora_receive_packet(uint8_t *buf, int size) {
    return sx127x_receive_packet(buf, size);
}

int lora_received(void) { return sx127x_received(); }

void lora_receive(void) { sx127x_receive(); }

//

void lora_init(void) {
    // Recognize module
    do {
        ESP_LOGE(pcTaskGetName(NULL), "Does not recognize sx127x module");
        vTaskDelay(1000 / portTICK_RATE_MS);
    } while ((sx127x_init() == 0));
    ESP_LOGI(pcTaskGetName(NULL), "sx127x module recognized");

    // sx127x configuration
    sx127x_enable_crc();
    sx127xConf_t sx127x_configuration;
    sx127x_configuration.frequency       = FREQ_915MHz;
    sx127x_configuration.codingRate      = 5;
    sx127x_configuration.bandwith        = 7;
    sx127x_configuration.spreadingFactor = 7;
    sx127x_config(sx127x_configuration);
}

void lora_init_task(node_handler_t node_h, measure_handler_t measure_h) {
    static lora_task_parameters parameters = {0};
    parameters.node_h                      = node_h;
    parameters.measure_h                   = measure_h;
    xTaskCreate(&task_lora_comm, "task_lora_comm", LORA_TSK_SIZE,
                (void *)&parameters, 5, NULL);
}

void task_lora_comm(void *pvParameters) {
    lora_task_parameters parameters = *(lora_task_parameters *)pvParameters;
    node_handler_t       node       = parameters.node_h;
    measure_handler_t    measure_handler = parameters.measure_h;
    measure_t            measure;
    uint8_t              lora_pkt[LORA_MAX_PKT_SIZE];
    int                  lora_pkt_indx         = 0;
    char                 lora_pkt_string[1024] = {0};

    msg_protocol_t msg_to_send = MSG_PROTOCOL_SEND_MEASURES;
    while (1) {
        lora_pkt_indx = 0;
        switch (msg_to_send) {
            case MSG_PROTOCOL_SEND_MEASURES:
                // send measurements
                measurements_wait(&measure_handler, portMAX_DELAY);
                int       measures_count = 0;
                measure_t measures_buff[MAX_MEASURES_NUM];
                // Header
                lora_pkt[LORA_RECEIVER_INDX] = LORA_SINK_ADDR;
                lora_pkt_indx++;
                lora_pkt[LORA_SENDER_INDX] = conf_get_node_addr(node);
                lora_pkt_indx++;
                // Measures

                while (measurements_get(&measure_handler, &measure) > 0) {
                    measures_buff[measures_count++] = measure;
                }
                int bytes_num            = measures_count * sizeof(measure_t);
                lora_pkt[LORA_SIZE_INDX] = bytes_num;
                lora_pkt_indx++;
                lora_pkt[LORA_MSG_PROTOCOL_INDX] = MSG_PROTOCOL_SEND_MEASURES;
                lora_pkt_indx++;
                int offset = 0;

                memcpy((void *)&lora_pkt[lora_pkt_indx], (void *)&measures_buff,
                       bytes_num);

                for (int i = 0; i < (bytes_num + LORA_HEADER_SIZE); i++) {
                    offset +=
                        sprintf(&lora_pkt_string[offset], "%02X ", lora_pkt[i]);
                }

                ESP_LOGI(pcTaskGetName(NULL), "lora_pkt(4+%d) = %s", bytes_num,
                         lora_pkt_string);
                lora_send_packet(lora_pkt, bytes_num + LORA_HEADER_SIZE);

                break;
        }

    }  // end while
}

void task_rx(void *pvParameters) {
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    uint8_t buf[256];  // Maximum Payload size of SX1276/77/78/79 is 255
    while (1) {
        lora_receive();  // put into receive mode
        if (lora_received()) {
            int receive_len = lora_receive_packet(buf, sizeof(buf));
            for (int i = 0; i < receive_len; i++) {
                printf("%.2X ", buf[i]);
            }
            printf("\n");
            ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s]",
                     receive_len, receive_len, buf);
        }               // end if
        vTaskDelay(2);  // Avoid WatchDog alerts
    }                   // end while
}