#include "Arduino.h"
#include "devconfig.h"
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
void displayline(uint8_t line, char *pucMsg, ...);

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

void sendTask(void* pvParameters) {
    uint8_t ret=0;
    log_i("SendTask iniciada.");

    while (true) {
        if (send_pct) {
            //log_i("send devtype=%d slot=%d", loramesh.mydd.devtype, actualslot);

            if (loramesh.mydd.devtype == DEV_TYPE_ROUTER){
                uint16_t lastmyseqnum = loramesh.getLastSeqNum();  
                if(actualslot == 0){
                    ret = loramesh.sendBeacon(lastscantime_ms);
                    if(ret > 0){
                        //log_i("Tx Beacon seqnum= %d", lastmyseqnum);
                        #if 0 //DISPLAY_ENABLE  
                            char display_line[20];
                            sprintf(display_line,"B = %d", lastmyseqnum+1);
                            displayline(2,display_line);
                        #endif                        
                    }
                    else{
                        log_i("Falha no envio do beacon !");
                    }
                }
                else if(actualslot == 2){
                    //uint8_t* pucaux = (uint8_t*) &lastmyseqnum;
                    //frame de requisição que o route vai enviar
                    //{SRC,DST,FCT,Seq numb,START,QTD PARAMETROS,CRC}
                    //uint8_t frame[] = {1,2,FCT_READING,*(pucaux+1),*(pucaux),1,1,BYTE_CRC};
                    //uint8_t frameSize = sizeof(frame);

                    ret = loramesh.sendData(2,lastmyseqnum);
                    if (ret > 0) {
                      //log_i("Tx Data Req= %d",lastmyseqnum);
                        #if DISPLAY_ENABLE  
                            char display_line[20];
                            sprintf(display_line,"TxD = %d", lastmyseqnum);
                            displayline(2,display_line);
                        #endif   
                    }
                    else 
                      log_i("Falha no envio do frame !");
                }
            } else {   //ed response
                uint8_t *msg;
                uint8_t msg_size;
                switch(loramesh.lastpkt.fct)
                {
                    //case FCT_BEACON:
                        //loramesh.sendPacketRes(1);
                        //nextstate = ST_RXWAIT;
                    //    break;
                    case FCT_READING:
                        //response frame
                        //{SRC,DST,FCT,SEQ NUMBER,SIZE VALUE,VALUE,CRC};
                        uint8_t sizeValue = sizeof(value);
                        value = loramesh.lastpkt.seqnum;
                        vTaskDelay(40 / portTICK_PERIOD_MS); 
                        if (loramesh.sendPacketResponse(1,sizeValue,value)) {
                           log_i("Resposta enviada!");
                            #if DISPLAY_ENABLE
                                sprintf(display_line3,"TxD = %d", value);
                                Heltec.DisplayShowAll(display_line1,display_line2,display_line3);
                            #endif
                        }
                        else 
                           log_e ("Falha no envio da resposta!");
                        //nextstate = ST_RXWAIT;
                        break;
                }
                

            }
        }
        send_pct = 0;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

