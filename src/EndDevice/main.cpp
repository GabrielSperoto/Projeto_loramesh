#include "main.h"


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

        sprintf(display_line1, "ED = %x", loramesh.mydd.devserialnumber);
        Heltec.DisplayShow1(display_line1);
    
        sprintf(display_line2, "ADD = %d", loramesh.mydd.devaddr);
        Heltec.DisplayShow2(display_line2);
    #endif

    Serial.printf("\nMemoria livre antes de criar tarefas: %u bytes\n", ESP.getFreeHeap());
    

    // CORREÇÃO: Pilhas com tamanhos seguros para evitar crashes
    xTaskCreatePinnedToCore(applicationTask, "ApplicationTask", 4096, NULL, 3, &App_TaskHandle, 1);
    xTaskCreatePinnedToCore(CommTask, "CommTask", 3072, NULL, 3, &Send_TaskHandle, 1);
    xTaskCreatePinnedToCore(watchdogTask, "WatchdogTask", 2048, NULL, 1, &Watchdog_TaskHandle, 1);


    //a leitura do potenciometro deve ser feita na aplication task
    // xTaskCreatePinnedToCore(LerPotenciometro, "LerPotenciometroTask", 2048, NULL, 1, &LerPotenciometro_TaskHandle, 1);
    // log_i("Tarefa de leitura do potenciômetro criada.");

    initcomm();
    Serial.println("--- Criacao de tarefas finalizada ---\n");
}

void loop() {
    // Tudo é feito nas tasks do FreeRTOS
}