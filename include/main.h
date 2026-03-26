#ifndef _MAIN_H
#define _MAIN_H

#include "devconfig.h"


void applicationTask(void* pvParameters);
void CommTask(void* pvParameters);
void TCP_communicationTask(void* pvParameters);
void init_TCP_comm();
void initcomm(void);
void displayline(uint8_t line, char *pucMsg, ...);
int send_beacon(void);
int send_data_request(uint8_t dstaddr, uint16_t value);
int send_data_response(void);


extern QueueHandle_t txQueue;    //App transmite para comunicacao
extern QueueHandle_t rxQueue;    //Comunicacao responde para App
extern QueueHandle_t q_tcp2app;   //tcp tranmite para app
extern QueueHandle_t q_app2tcp;   //app transmite para tcp
extern QueueHandle_t q_app2comm;  //app trasmite para comm
extern QueueHandle_t q_comm2app;  //comm transmite para app



typedef struct {
    uint8_t src;
    uint8_t dst;
    uint8_t function;
    uint8_t start;
    uint8_t qtdParametros; //start e qtdParametros fazem parte do pacote de requisição 
    uint8_t payload[BUFFER_SIZE];
    uint8_t size;
    uint8_t value; //valor usado na escrita
} TxMessage_t;

typedef struct {
    uint8_t src;
    uint8_t dst;
    uint8_t function;
    uint8_t start;
    uint8_t qtdParametros;
    uint8_t value; //valor usado na escrita
    uint8_t payload[BUFFER_SIZE];
    uint8_t size;
    int16_t rssi;
} RxMessage_t;

//buffer para armazenar as mensagens da rxMSg

typedef struct {
    uint8_t src;
    uint8_t dst;
    uint8_t function;
    uint8_t start;
    uint8_t qtdParametros;
    uint8_t value;
    uint8_t payload[BUFFER_SIZE];
    uint8_t size;
    int16_t rssi;
    bool ocupado;
} SlotBuffer;

#endif