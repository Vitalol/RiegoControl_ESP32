#ifndef __SX127x_H__
#define __SX127x_H__


/* Library original */
void sx127x_reset(void);
void sx127x_explicit_header_mode(void);
void sx127x_implicit_header_mode(int size);
void sx127x_idle(void);
void sx127x_sleep(void); 
void sx127x_receive(void);
int sx127x_get_irq(void);
void sx127x_set_tx_power(int level);
void sx127x_set_frequency(long frequency);
void sx127x_set_spreading_factor(int sf);
int sx127x_get_spreading_factor(void);
void sx127x_set_dio_mapping(int dio, int mode);
int sx127x_get_dio_mapping(int dio);
void sx127x_set_bandwidth(int sbw);
int sx127x_get_bandwidth(void);
void sx127x_set_coding_rate(int denominator);
int sx127x_get_coding_rate(void);
void sx127x_set_preamble_length(long length);
long sx127x_get_preamble_length(void);
void sx127x_set_sync_word(int sw);
void sx127x_enable_crc(void);
void sx127x_disable_crc(void);
int sx127x_init(void);
void sx127x_send_packet(uint8_t *buf, int size);
int sx127x_receive_packet(uint8_t *buf, int size);
int sx127x_received(void);
int sx127x_packet_rssi(void);
float sx127x_packet_snr(void);
void sx127x_close(void);
int sx127x_initialized(void);
void sx127x_dump_registers(void);

/* Added functions, structs, defines, etc */

#define FREQ_169MHz 169e6 
#define FREQ_433MHz 433e6 
#define FREQ_470MHz 470e6 
#define FREQ_866MHz 866e6 
#define FREQ_915MHz 915e6 

/* structs */
typedef struct sx127xConf_t{
    int64_t frequency;
    uint8_t codingRate;
    uint8_t bandwith;
    uint8_t spreadingFactor;
}sx127xConf_t;

/* functions */
void sx127x_config(sx127xConf_t sx127xConf);

#endif
