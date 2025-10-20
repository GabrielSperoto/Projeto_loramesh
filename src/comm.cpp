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
    TxMessage_t txMsg;
    RxMessage_t rxMsg;

    while (true) {
    
       if (xQueueReceive(txQueue, &txMsg, 10 / portTICK_PERIOD_MS) == pdTRUE) {
            log_i("sendmsg devtype=%d slot=%d", loramesh.mydd.devtype, actualslot);
            switch (txMsg.function) {
                case FCT_BEACON:
                    loramesh.sendBeacon(millis());
                    break;
                case FCT_READING:
                    if(loramesh.mydd.devtype == DEV_TYPE_ROUTER){
                        loramesh.sendData(txMsg.dst, loramesh.lastpkt.seqnum);
                    }
                    else{ //end device
                        loramesh.sendPacketResponse(txMsg.dst, txMsg.size, txMsg.payload);
                    }
                    break;
                default:
                    log_w("Funcao nao suportada: %d", txMsg.function);
            }
        }

        if (loramesh.receivePacket()) {
            rxMsg.src = loramesh.lastpkt.srcaddress;
            rxMsg.function = loramesh.lastpkt.fct;
            rxMsg.size = loramesh.lastpkt.packetSize;
            memcpy(rxMsg.payload, loramesh.lastpkt.rxpacket, rxMsg.size);
            rxMsg.rssi = loramesh.packetRssi();

            xQueueSend(rxQueue, &rxMsg, 0);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

