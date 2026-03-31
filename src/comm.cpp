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
        int res= loramesh.startReceiving(2000);
        if (res > 0){
            log_e("Error startReceiving=%d",res);
        }
        Serial.println("Dispositivo configurado como End Device");
        nextstate = ST_RXWAIT;
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

        if (actualslot > MAX_SLOTS) 
           actualslot = 0;
   }
}

void CommTask(void* pvParameters) {
    uint8_t ret=0;
    // TxMessage_t txMsg;
    // RxMessage_t rxMsg;
    msg_t msg;

    while (true) {

        log_i("Comm task iniciada. Slot atual: %d", actualslot);
    
       if (xQueueReceive(q_app2comm, &msg, 10 / portTICK_PERIOD_MS) == pdTRUE) {
            log_i("sendmsg devtype=%d slot=%d", loramesh.mydd.devtype, actualslot);
            //envia o pacote pela rede
            if(loramesh.encodeAndSendPacket(&msg))
                log_i("Mensagem enviada! Dst: %d Function: %d", msg.dst, msg.function); 
            else
                log_i("Erro no envio da mensagem");
            
            // switch (msg.function) {
            //     case FCT_BEACON:
            //         log_i("Seq.num: %d",loramesh.mydd.seqnum);
            //         #if DISPLAY_ENABLE
            //             sprintf(display_line3,"Seq. number: %d",loramesh.mydd.seqnum);
            //             Heltec.DisplayShowAll(display_line1,display_line2,display_line3);
            //         #endif
            //         loramesh.sendBeacon(millis());
            //         break;

            //     case FCT_READING:
            //         if(loramesh.mydd.devtype == DEV_TYPE_ROUTER){
            //             if (loramesh.sendReadingReq(msg.dst, msg.start,msg.qtdParametros))
            //                 log_i("msg.dst: %d msg.start: %d msg.qtdParametros: %d",msg.dst,msg.start,msg.qtdParametros);
            //             else
            //                 log_i("Erro no envio da requisição de leitura");
            //         }
                    
            //         else{ //end device
            //             //ed envia a resposta para o router
            //             //payload é um buffer para guardar o valor lido
            //             if(loramesh.sendReadingRes(msg.dst, msg.size, msg.payload.bytes))
            //                 log_i("Resposta enviada! msg.dst: %d msg.size: %d msg.data.bytes: %d",msg.dst,msg.size,msg.payload.bytes); 
            //             else                      
            //                 log_i("Erro no envio da resposta de leitura");
            //         }
            //         break;

            //     case FCT_WRITTING:
            //         if(loramesh.mydd.devtype == DEV_TYPE_ROUTER){
            //             if (loramesh.sendWrittingReq(msg.dst, msg.start,msg.qtdParametros,msg.payload.value))
            //                 log_i("msg.dst: %d msg.start: %d msg.qtdParametros: %d msg.value: %d",msg.dst,msg.start,msg.qtdParametros,msg.payload.value);
            //             else
            //                 log_i("Erro no envio da requisição de escrita");
            //         } 

            //         else{ //end device
            //             if (loramesh.sendWrittingRes(msg.dst, msg.payload.value)) 
            //                 log_i("msg.dst: %d status: %d",msg.dst, (msg.payload.value == 1) ? "Sucesso" : "Falha"); 
            //             else
            //                 log_i("Erro no envio da resposta de escrita");
            //         }
            //         break;
            //     default:
            //         log_w("Funcao nao suportada: %d", txMsg.function);
            // }
        }

        if (loramesh.receivePacket()) {
            //aqui talvez seria interessante descompactar a mensagem recebida em lastpkt para envia-la pelas tarefas atraves da estrtutura msg
            // msg.src = loramesh.getSrcAdress();
            // msg.function = loramesh.getFunctionCode();
            // msg.start = loramesh.getStart();
            // msg.qtdParametros = loramesh.getQtdParametros();
            // msg.payload.value = loramesh.getReadingDataAsUint32();
            // msg.size = loramesh.getSizeMsg();

            log_i("Pacote recebido! RSSI: %d dBm", loramesh.packetRssi());

            loramesh.decodeLoraPacket(&msg);

            
            // msg.rssi = loramesh.packetRssi(); //o que e esse rssi?
            xQueueSend(q_comm2app, &loramesh.msg, 0);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

