#if defined(MOCK_TEST_MODE)
#include "MockLoRa.h"
#include <Arduino.h>
#include <main.h>


void vTaskSend2Tcp(void *pvParameters) {
    msg_t pacoteMock;
    payload_t payload;
    payload_t contador_simulado;
    contador_simulado.value = 0; 

    while (1) {
        pacoteMock.src = 2;
        pacoteMock.dst = 1;
        pacoteMock.function = 3;
        pacoteMock.start = 0;
        pacoteMock.payload = contador_simulado;
        
        if (xQueueSend(q_app2tcp, &pacoteMock, pdMS_TO_TICKS(100)) == pdPASS) {
            Serial.printf("SIMULADOR: Pacote mock (val=%d) enviado!\n", contador_simulado.value);
            contador_simulado.value++;
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void vTaskSend2Lora(void *pvParameters){
    extern tcpMsgs_t tcpMsgs[MAX_SLOTS]; // array usado para receber mensagens do TCP e armazenar no slot correspondente
    extern uint8_t actualslot;

    while(1){
        //tcpMsgs e preenchida manualmente para fins de teste 
        if(actualslot == 0){
            tcpMsgs[2].ocupado = true; // marca o slot como ocupado
            tcpMsgs[3].ocupado = true; // marca o slot como ocupado
            tcpMsgs[4].ocupado = true; // marca o slot como ocupado
            tcpMsgs[2].msg.payload.value = !tcpMsgs[2].msg.payload.value;
        }

        tcpMsgs[3].msg.dst = 3;
        tcpMsgs[3].msg.src = 1;
        tcpMsgs[3].msg.function = FCT_READING;
        tcpMsgs[3].msg.start = POT;
        tcpMsgs[3].msg.qtdParametros = 1;
        tcpMsgs[3].msg.payload.value = 0; //valor para acender o led
        tcpMsgs[3].ocupado = true; //marca o slot como ocupado

        tcpMsgs[2].msg.dst = 2;
        tcpMsgs[2].msg.src = 1;
        tcpMsgs[2].msg.function = FCT_WRITTING;
        tcpMsgs[2].msg.start = LEDP;
        tcpMsgs[2].msg.qtdParametros = 1;
        tcpMsgs[2].msg.payload.value = 1; //valor para apagar o led
        tcpMsgs[2].ocupado = true; //marca o slot como ocupado

        tcpMsgs[4].msg.dst = 4;
        tcpMsgs[4].msg.src = 1;
        tcpMsgs[4].msg.function = FCT_READING;
        tcpMsgs[4].msg.start = POT;
        tcpMsgs[4].msg.qtdParametros = 1;
        tcpMsgs[4].msg.payload.value = 0; //valor para acender o led
        tcpMsgs[4].ocupado = true; //marca o slot como ocupado
    }
}
#endif