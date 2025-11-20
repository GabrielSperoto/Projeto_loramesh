#include "Arduino.h"
#include "devconfig.h"
#include "main.h"
#include "heltec.h"
#include "Lora/loramesh.h"
#include <Wire.h>
#include <RadioLib.h>
#include "esp_log.h"

#define PinPot 37
#define PinLed 36

extern statemac nextstate;
extern LoRaClass loramesh;
extern volatile bool messageReceived;
extern uint8_t actualslot;
extern uint8_t send_pct;
extern uint32_t lastActivityMillis;
uint8_t lastslot = MAX_SLOTS;

void setindpolls();
void slottimecontrol(void);
void node_init_sync(uint32_t new_FR);
void displayline(uint8_t line, char *pucMsg, ...);

extern float TensaoDeSaida;
extern uint16_t valorPot;

extern char display_line1[20];
extern char display_line2[20];
extern char display_line3[20];

bool ledtoogle = 0;

//variaveis usadas em ST_STARTRX
uint32_t rx_timeout;
int res;


void applicationTask(void* pvParameters) {
    TxMessage_t txmsg;
    RxMessage_t rxMsg;

    while (true) {
        slottimecontrol();
        
        if (actualslot != lastslot) {
            if(actualslot == 0){
                if(loramesh.mydd.devtype == DEV_TYPE_ROUTER) 
                    nextstate = ST_TXBEACON;
                else 
                    nextstate = ST_RXWAIT;
            }

            else if(actualslot == 2){
                if(loramesh.mydd.devtype == DEV_TYPE_ROUTER)
                    nextstate = ST_TXDATA;
            }
        }

        lastActivityMillis = millis();
        // if (lastslot != actualslot){
        //     log_i("app : slot=%d ns=%d ", actualslot, nextstate);
        // }

        switch (nextstate) {
            case ST_TXBEACON:
                if ((loramesh.mydd.devtype == DEV_TYPE_ROUTER) && (actualslot == 0)) {
                    txmsg.dst = BROADCAST_ADDR;
                    txmsg.function = FCT_BEACON;
                    txmsg.size = 0;
                    xQueueSend(txQueue, &txmsg, 0);

                    lastActivityMillis = millis();
                    nextstate = ST_STANDBY; 
                }
                break;
            case ST_RXWAIT:
                //manipula os pacotes recebidos pelo router e pelo ed
                if (xQueueReceive(rxQueue, &rxMsg, 0) == pdTRUE) {

                    log_i("App recebeu pacote no slot %d de %d func=%d size=%d RSSI=%d ",
                        actualslot, rxMsg.src, rxMsg.function, rxMsg.size, rxMsg.rssi);
                    // aqui você decide o que fazer com os dados recebidos
                    lastActivityMillis = millis();
                    if(loramesh.mydd.devtype == DEV_TYPE_ROUTER){
                        //verifica a função da mensagem recebida
                        switch (rxMsg.function){
                            case FCT_SYNC_SUCESS:{
                                setindpolls();
                                // log_i("Rx.seqnumb: %d",loramesh.lastpkt.seqnum);
                                break;
                            }
                            case FCT_WRITING: {
                                // uint8_t writtingCode = loramesh.getResponseStatus();
                                // if (writtingCode == 1){
                                //     log_i("Escrita realizada com sucesso no no %d",rxMsg.src);
                                // }
                                // else{
                                //     log_i("Falha na escrita no no %d",rxMsg.src);
                                // }
                                // break;
                            }
                            case FCT_READING: {
                                // a divisao por 100 é para converter o valor inteiro de volta para float
                                float value = loramesh.getReadingDataAsUint32()/100.0;
                                if(value > 0){
                                    log_i("Valor lido: %.2f",value);
                                }
                                break;
                            }
                            case FCT_DESCRIPTION: {
                                //mensagem de descrição
                                break;
                            }
                        }
                    }
                    else{ //end device
                        //verifica o destino da mensagem
                        switch (rxMsg.function){
                            case FCT_BEACON:{
                                uint8_t* rxpacket = rxMsg.payload;
                                uint8_t len =rxMsg.size;
                                uint32_t timestamp = loramesh.gettimestamp(rxpacket,len);
                                loramesh.mydd.seqnum = loramesh.getLastPctSeqNum();
                                node_init_sync(timestamp);
    
                                #if DISPLAY_ENABLE  
                                  char display_line[20];
                                  sprintf(display_line,"Seq. number: %d", loramesh.mydd.seqnum);
                                  displayline(3,display_line);
                                #endif
    
                                //log_i("Rx.seqnum: %d time=%d slot=%d", loramesh.lastpkt.seqnum,timestamp,actualslot);
                                nextstate = ST_STARTRX;
    
                                break;
                            }
                            case FCT_WRITING:{
                                //mensagem de escrita
                                if (loramesh.getWrittingCode() == 1){
                                    log_i("Led Aceso!");
                                    digitalWrite(PinLed, HIGH);
                                }
                                else{
                                    log_i("Led apagado!");
                                    digitalWrite(PinLed, LOW);

                                }
                                break;
                            }
                            case FCT_READING:{
                                // log_i("Pacote de leitura recebido! ");
                                uint16_t seqnumber = loramesh.lastpkt.seqnum;
                                log_i("Rx.seqnumber: %d",seqnumber);
                                #if DISPLAY_ENABLE  
                                  char display_line[20];
                                  sprintf(display_line,"Seq. number: %d", seqnumber);
                                  displayline(3,display_line);
                                #endif
                                nextstate = ST_TXDATA;
                                //mensagem de leitura
                                break;
                            }
                            case FCT_DESCRIPTION: {
                                //mensagem de descrição
                                break;
                            }
                        }
                    }
                }

                //a camada comm agora é a responsável por checar se há dados a serem lidos
                // if(loramesh.receivePacket()){
                // }
                break;
            case ST_TXDATA: 
                lastActivityMillis = millis();
                //achq que aqui eu deveria montar o frame especifico e colocar na fila...
                if (loramesh.mydd.devtype == DEV_TYPE_ROUTER){
                    txmsg.dst = 2;
                    txmsg.function = FCT_READING;
                    txmsg.size = 0;
                    txmsg.value = 0;
                    txmsg.start = 1;
                    txmsg.qtdParametros = 1;
                    xQueueSend(txQueue, &txmsg, 0);                    
                }
                else{ //end device

                    //resposta de leitura do ED
                    if(rxMsg.function == FCT_READING){
                        // o valor é multiplicado por 100 para enviar como inteiro
                        uint32_t value= TensaoDeSaida*100;
                        uint8_t *pucaux = (uint8_t *) &value;
                        // log_i("Value: %d",value);
                        txmsg.dst = 1;
                        txmsg.function = FCT_READING;
                        txmsg.size = sizeof(value);
                        txmsg.value = 0;
                        txmsg.payload[0] = *(pucaux+3);
                        txmsg.payload[1] = *(pucaux+2);
                        txmsg.payload[2] = *(pucaux + 1);
                        txmsg.payload[3] = *pucaux;
                        xQueueSend(txQueue, &txmsg, 0);

                }   else {
                    //reposta de escrita do ED
                    txmsg.dst = 1;
                    txmsg.function = FCT_WRITING;
                    txmsg.size = 0;
                    txmsg.start = 0;
                    txmsg.qtdParametros = 0;
                    txmsg.value = 1; //código de sucesso
                    
                    xQueueSend(txQueue, &txmsg, 0);
                }
                }
                send_pct = 1;
                nextstate = ST_STARTRX;
                break;

            case ST_STARTRX:
                if (send_pct == 0){
                    rx_timeout = (loramesh.mydd.devtype == DEV_TYPE_ROUTER) ? 1000 : 2000; 
                    log_i("Sent...startReceiving=%d",rx_timeout);
                    loramesh.ClearRadioIRQs();
                    res= loramesh.startReceiving(rx_timeout);
                    if (res > 0){
                        log_e("Error startReceiving=%d",res);
                    }
                }
                nextstate = ST_RXWAIT;
                break;

            case ST_STANDBY:
                //slepping mode
                if ((actualslot > lastslot) && ( send_pct == 0)) {
                    //estava em standby mas agora mudou o slottime
                   if ((actualslot == 2) && (loramesh.mydd.devtype == DEV_TYPE_ROUTER))
                        nextstate = ST_TXDATA;
                }
                lastActivityMillis = millis();
                break;
            default:
                break;
        }
        lastslot = actualslot;

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}


void ledblink(uint8_t ledpin) {
    digitalWrite(ledpin, HIGH);
    delay(100);
    digitalWrite(ledpin, LOW);
}