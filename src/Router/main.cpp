#include "main.h"


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

uint16_t idx_response = 0;

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

// void LerPotenciometro(void* pvParameters) {
//     log_i("LerPotenciometroTask iniciada.");
//     // NOTA: Esta tarefa agora serve apenas para log local no ED.
//     // O valor lido aqui não é mais enviado pela rede.
//     for (;;) {
//         uint16_t leitura_completa = analogRead(PinPot);
//         valorPot = leitura_completa;
//         TensaoDeSaida = (((float)valorPot / 4095.0) * 3.3);
        
//         log_d("ValorPot (0-4095) = %d | Tensao de Saida = %.2fV", valorPot, TensaoDeSaida);

//         #if DISPLAY_ENABLE
//         sprintf(display_line3, "Pot: %d  %.2fV", valorPot, TensaoDeSaida);
//         Heltec.DisplayShowAll(display_line1, display_line2, display_line3);
//         #endif
        
//         vTaskDelay(5000 / portTICK_PERIOD_MS);
//     }
// }

void setindpolls() {
    uint16_t lastmyseqnum = loramesh.getLastSeqNum();
    uint16_t lastpacketseqnum = loramesh.getLastPctSeqNum();
    if (lastmyseqnum == lastpacketseqnum) {
        idx_response++;
        lastActivityMillis = millis();
        // log_i("Tx.sn=%d Rx.sn=%d Rx.cnt=%d ", lastmyseqnum, lastpacketseqnum, idx_response);

    }
    #if DISPLAY_ENABLE  
    {
        sprintf(display_line3, "Tx=%d Rx=%d", lastmyseqnum, lastpacketseqnum);
        Heltec.DisplayShowAll(display_line1, display_line2, display_line3);
    }   
    #endif
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
    
    lastActivityMillis = millis();
    Serial.begin(115200);
    delay(1000); 

    Heltec.begin();
    loramesh.begin();


    q_app2comm = xQueueCreate(10, sizeof(msg_t));
    if (q_app2comm == NULL) 
        Serial.println("Falha ao criar fila q_app2comm!");
    q_comm2app = xQueueCreate(10, sizeof(msg_t));
    if (q_comm2app == NULL) 
        Serial.println("Falha ao criar fila q_comm2app!");
    q_app2tcp = xQueueCreate(10, sizeof(msg_t));
    if (q_app2tcp == NULL) 
        Serial.println("Falha ao criar fila q_app2tcp!");
    q_tcp2app = xQueueCreate(10, sizeof(msg_t));
    if (q_tcp2app == NULL) 
        Serial.println("Falha ao criar fila q_tcp2app!");    
   


    

    #if DISPLAY_ENABLE  
        Heltec.DisplayClear();

        sprintf(display_line1, "RT = %x", loramesh.mydd.devserialnumber);
        Heltec.DisplayShow1(display_line1);

        sprintf(display_line2, "ADD = %d", loramesh.mydd.devaddr);
        Heltec.DisplayShow2(display_line2);
    #endif

    Serial.printf("\nMemoria livre antes de criar tarefas: %u bytes\n", ESP.getFreeHeap());
    

    // CORREÇÃO: Pilhas com tamanhos seguros para evitar crashes
    xTaskCreatePinnedToCore(applicationTask, "ApplicationTask", 4096, NULL, 3, &App_TaskHandle, 1);
    xTaskCreatePinnedToCore(CommTask, "CommTask", 3072, NULL, 3, &Send_TaskHandle, 1);
    xTaskCreatePinnedToCore(watchdogTask, "WatchdogTask", 2048, NULL, 1, &Watchdog_TaskHandle, 1);

    #if 1 //tarefa é desativada para depuração
    xTaskCreatePinnedToCore(TCP_communicationTask, "TCP_CommunicationTask", 4096, NULL, 2, &TCP_Communication_TaskHandle, 1);
    init_TCP_comm();
    #endif

    initcomm();
    Serial.println("--- Criacao de tarefas finalizada ---\n");
}

void loop() {
    // Tudo é feito nas tasks do FreeRTOS
}