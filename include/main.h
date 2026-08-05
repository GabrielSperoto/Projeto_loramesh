// Arquivo central de definições compartilhadas

#ifndef _MAIN_H
#define _MAIN_H

#include "devconfig.h"
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "devconfig.h"
#include "heltec.h"
#include "Lora/Loramesh.h"
#include <Wire.h>
#include <RadioLib.h>
#include "esp_log.h"

#if DISPLAY_ENABLE
#include "OLED/SSD1306.h"
#endif

// Handles das tarefas
extern TaskHandle_t App_TaskHandle;
extern TaskHandle_t Send_TaskHandle;
extern TaskHandle_t Watchdog_TaskHandle;
// TaskHandle_t LerPotenciometro_TaskHandle = nullptr;
extern TaskHandle_t TCP_Communication_TaskHandle;

// Variáveis globais de estado
extern LoRaClass loramesh;
// extern volatile bool messageReceived;
// char rxpacket[BUFFER_SIZE];
#if DISPLAY_ENABLE  
extern char display_line1[20];
extern char display_line2[20];
extern char display_line3[20];
#endif

#define PinPot 37
extern uint16_t valorPot;
extern float TensaoDeSaida;

extern QueueHandle_t txQueue;    //App transmite para comunicacao
extern QueueHandle_t rxQueue;    //Comunicacao responde para App
extern QueueHandle_t q_tcp2app;   //Tcp transmite para app
extern QueueHandle_t q_app2tcp;   //app transmite para tcp
extern QueueHandle_t q_app2comm;  //app trasmite para comm
extern QueueHandle_t q_comm2app;  //comm transmite para app

extern uint16_t idx_response;

extern uint32_t lastActivityMillis;


// Tasks que serão executadas pelo FreeRTOS
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



// Filas que serão usadas para enviar mensagens entre as tasks
extern QueueHandle_t txQueue;    //App transmite para comunicacao
extern QueueHandle_t rxQueue;    //Comunicacao responde para App
extern QueueHandle_t q_tcp2app;   //tcp tranmite para app
extern QueueHandle_t q_app2tcp;   //app transmite para tcp
extern QueueHandle_t q_app2comm;  //app trasmite para comm
extern QueueHandle_t q_comm2app;  //comm transmite para app


// Estrutura que evia a mensagem
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

//Estrutura que recebe/responde a mensagem
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

// Buffer para armazenar as mensagens da rxMSg
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