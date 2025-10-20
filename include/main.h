#ifndef _MAIN_H
#define _MAIN_H

void applicationTask(void* pvParameters);
void CommTask(void* pvParameters);
void initcomm(void);
void displayline(uint8_t line, char *pucMsg, ...);
int send_beacon(void);
int send_data_request(uint8_t dstaddr, uint16_t value);
int send_data_response(void);


extern QueueHandle_t txQueue;    //App transmite para comunicacao
extern QueueHandle_t rxQueue;    //Comunicacao responde para App

typedef struct {
    uint8_t dst;
    uint8_t function;
    uint8_t payload[BUFFER_SIZE];
    uint8_t size;
} TxMessage_t;

typedef struct {
    uint8_t src;
    uint8_t function;
    uint8_t payload[BUFFER_SIZE];
    uint8_t size;
    int16_t rssi;
} RxMessage_t;

#endif