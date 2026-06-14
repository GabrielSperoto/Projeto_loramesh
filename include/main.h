#ifndef _MAIN_H
#define _MAIN_H

#include "devconfig.h"
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "devconfig.h"
#include "heltec.h"
#include "Lora/loramesh.h"
#include <Wire.h>
#include <RadioLib.h>
#include "esp_log.h"

#if DISPLAY_ENABLE
#include "OLED/SSD1306.h"
#endif

// Handles das tarefas
TaskHandle_t App_TaskHandle = nullptr;
TaskHandle_t Send_TaskHandle = nullptr;
TaskHandle_t Watchdog_TaskHandle = nullptr;
// TaskHandle_t LerPotenciometro_TaskHandle = nullptr;
TaskHandle_t TCP_Communication_TaskHandle = nullptr;

// Variáveis globais de estado
extern LoRaClass loramesh;
// extern volatile bool messageReceived;
// char rxpacket[BUFFER_SIZE];
#if DISPLAY_ENABLE  
char display_line1[20];
char display_line2[20];
char display_line3[20];
#endif

#define PinPot 37
uint16_t valorPot = 0;
float TensaoDeSaida = 0;

QueueHandle_t txQueue;    //App transmite para comunicacao
QueueHandle_t rxQueue;    //Comunicacao responde para App
QueueHandle_t q_tcp2app;   //Tcp transmite para app
QueueHandle_t q_app2tcp;   //app transmite para tcp
QueueHandle_t q_app2comm;  //app trasmite para comm
QueueHandle_t q_comm2app;  //comm transmite para app

uint16_t idx_response = 0;

uint32_t lastActivityMillis = 0;


void applicationTask(void* pvParameters);
void CommTask(void* pvParameters);
void TCP_communicationTask(void* pvParameters);
void init_TCP_comm();
void initcomm(void);
void displayline(uint8_t line, char *pucMsg, ...);
int send_beacon(void);
int send_data_request(uint8_t dstaddr, uint16_t value);
int send_data_response(void);
void ledblink(uint8_t ledpin);
void setindpolls();



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