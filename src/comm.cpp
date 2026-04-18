#include "Arduino.h"
#include "devconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "main.h"
#include "heltec.h"
#include "Lora/loramesh.h"
#include <Wire.h>
#include <RadioLib.h>
#include "esp_log.h"

uint8_t send_pct = 0;
extern LoRaClass loramesh;
extern void SendMessage(String src, String dst, String fct, String param, String val);
long lastabstime = 0;
long lastscantime_ms = 0;
uint8_t actualslot = 0;
long slot_period = 0;
bool syncronized = false;
float adjustedPeriod = EXPECTED_PERIOD_MS;
uint16_t value = 0; // exemplo de leitura de um potenciometro

#define END_DEVICE_1_SLOT 1
#define END_DEVICE_2_SLOT 2
#define END_DEVICE_3_SLOT 3

#if DISPLAY_ENABLE  
extern char display_line1[];
extern char display_line2[];
extern char display_line3[];
#endif

statemac nextstate;


uint32_t previous_FR = 0;
uint32_t current_FR = 0;
int32_t drift = 0;

void initcomm(void){

    if (loramesh.mydd.devtype == DEV_TYPE_ROUTER) {
        actualslot = 0;
        nextstate = ST_TXBEACON;
    } else {
        log_i("Dispositivo configurado como End Device");
        send_pct = 0;
        nextstate = ST_STARTRX;
    }
}

void node_init_sync(uint32_t new_FR) {
    current_FR = new_FR;

    if (previous_FR != 0) {
        drift = (current_FR - previous_FR) - SYNC_INTERVAL_MS;
        adjustedPeriod -= drift * ADJUSTMENT_FACTOR;
        adjustedPeriod = MAX(MAX_VAL, MIN(MIN_VAL, adjustedPeriod)); 
    }

    syncronized = true;
    previous_FR = current_FR;
}

void slottimecontrol() {
    long currscantime_ms = 0;

    if (lastabstime == 0) {
        lastabstime = millis();
    }
    
    if (syncronized == true) {
        syncronized = false;
        slot_period = (uint32_t) adjustedPeriod;
        actualslot = 0; //aqui eu considero se estiver muito defasado eu entro no slot do router
    } else {
        slot_period = SLOT_INTERVAL;
    }

    currscantime_ms = (millis() - lastabstime);
    if (currscantime_ms >= slot_period) {
        lastabstime = millis();
        lastscantime_ms += currscantime_ms;
        actualslot++;

        if (actualslot > MAX_SLOTS){
            actualslot = 0;
            if(loramesh.mydd.devtype == DEV_TYPE_ROUTER) 
                nextstate = ST_TXBEACON;
        }
   }
}

void CommTask(void* pvParameters) {
    uint8_t ret=0;
    // TxMessage_t txMsg;
    // RxMessage_t rxMsg;
    msg_t msgTx;
    msg_t msgRx;

    while (true) {

        // log_i("Comm task iniciada. Slot atual: %d", actualslot);

        //verifica se há mensagem do radio
        if (loramesh.receivePacket()) {
            //aqui talvez seria interessante descompactar a mensagem recebida em lastpkt para envia-la pelas tarefas atraves da estrtutura msg

            log_i("Pacote recebido no slot %d", actualslot);

            loramesh.decodeLoraPacket(&msgRx);

            
            // msg.rssi = loramesh.packetRssi(); //o que e esse rssi?
            if(xQueueSend(q_comm2app, &msgRx, 0) == pdTRUE){
                log_i("Mensagem da rede LoRa enviada para a aplicação. Src: %d, Function: %d, Start: %d, QtdParametros: %d, Data: %d", 
                    msgRx.src, msgRx.function, msgRx.start, msgRx.qtdParametros, msgRx.payload.value);
                }
            else{
                log_i("Erro ao enviar mensagem para a aplicação");
            }
            
        }

        //verifica se há mensagens da aplicação
    
       if (xQueueReceive(q_app2comm, &msgTx, 10 / portTICK_PERIOD_MS) == pdTRUE) {
            log_i("sendmsg devtype=%d slot=%d", loramesh.mydd.devtype, actualslot);
            //envia o pacote pela rede
            if(loramesh.encodeAndSendPacket(&msgTx)){
                //apos finalizar a transmissão, enviar um status para a aplicação indicando que a transmissão foi concluída
                msg_t txStatus;
                txStatus.function = FCT_TXDONE;
                log_i("Mensagem enviada! Dst: %d Function: %d Value: %d", msgTx.dst, msgTx.function, msgTx.payload.value); 
                xQueueSend(q_comm2app,&txStatus, 0);
            }
            else
                log_i("Erro no envio da mensagem");
            
        }
        

        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

