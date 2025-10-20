#include "Arduino.h"
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
TaskHandle_t LerPotenciometro_TaskHandle = nullptr;

// Variáveis globais de estado
extern LoRaClass loramesh;
extern volatile bool messageReceived;
char rxpacket[BUFFER_SIZE];

#if DISPLAY_ENABLE  
char display_line1[20];
char display_line2[20];
char display_line3[20];
#endif

uint16_t idx_response = 0;


uint32_t lastActivityMillis = 0;

void applicationTask(void* pvParameters);
void sendTask(void* pvParameters);
void initcomm(void);

void displayline(uint8_t line, char *pucMsg, ...) {

    #if DISPLAY_ENABLE  
    {
        if (line == 2){
            Heltec.DisplayShowAll(display_line1,pucMsg,display_line3);
        }
        else if (line == 3)
        {
           Heltec.DisplayShowAll(display_line1,display_line2,pucMsg);
        }
        
    }   
    #endif
}

void setindpolls() {
    uint16_t lastmyseqnum = loramesh.getLastSeqNum();
    uint16_t lastpacketseqnum = loramesh.getLastPctSeqNum();
    if (lastmyseqnum == lastpacketseqnum) {
        idx_response++;
        lastActivityMillis = millis();
        // log_i("Tx.sn=%d Rx.sn=%d Rx.cnt=%d ", lastmyseqnum, lastpacketseqnum, idx_response);

        #if DISPLAY_ENABLE  
        {
            sprintf(display_line3, "Tx=%d Rx=%d", lastmyseqnum, idx_response);
            Heltec.DisplayShowAll(display_line1, display_line2, display_line3);
        }   
        #endif
    }
}

// --- Definições das Tarefas com todas as correções ---

void watchdogTask(void* pvParameters) {
    while (true) {
        if ((millis() - lastActivityMillis) > 15000) {
            log_w("Watchdog: Sem atividade por 15 segundos. Reiniciando...");
            ESP.restart();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void setup() {
    Serial.begin(115200);
    delay(1000); 

    Heltec.begin();
    loramesh.begin();
    lastActivityMillis = millis();

    initcomm();

    #if DISPLAY_ENABLE  
        Heltec.DisplayClear();
        if (loramesh.mydd.devtype == DEV_TYPE_ROUTER) {
            //sprintf(display_line1, "RT=%x ", loramesh.mydd.devserialnumber);
            sprintf(display_line1, "RT.ADDR=%x ", loramesh.mydd.devaddr);
            Heltec.DisplayShow1(display_line1);
        } else {
            //sprintf(display_line1, "ED=%x ", loramesh.mydd.devserialnumber);
            sprintf(display_line1, "ED.ADDR=%x ", loramesh.mydd.devaddr);
            Heltec.DisplayShow1(display_line1);
        }
        sprintf(display_line2, "B=", loramesh.mydd.devaddr);
        Heltec.DisplayShow2(display_line2);
    #endif

    Serial.printf("\nMemoria livre antes de criar tarefas: %u bytes\n", ESP.getFreeHeap());
    

    // CORREÇÃO: Pilhas com tamanhos seguros para evitar crashes
    xTaskCreatePinnedToCore(applicationTask, "ApplicationTask", 4096, NULL, 3, &App_TaskHandle, 1);
    xTaskCreatePinnedToCore(sendTask, "SendTask", 3072, NULL, 3, &Send_TaskHandle, 1);
    xTaskCreatePinnedToCore(watchdogTask, "WatchdogTask", 2048, NULL, 1, &Watchdog_TaskHandle, 1);
    
    // if (loramesh.mydd.devtype == DEV_TYPE_ENDDEV) {
    //     xTaskCreatePinnedToCore(LerPotenciometro, "LerPotenciometroTask", 2048, NULL, 1, &LerPotenciometro_TaskHandle, 1);
    // }
    Serial.println("--- Criacao de tarefas finalizada ---\n");
}

void loop() {
    // Tudo é feito nas tasks do FreeRTOS
}