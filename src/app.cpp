#include "Arduino.h"
#include "devconfig.h"
#include "heltec.h"
#include "Lora/loramesh.h"
#include <Wire.h>
#include <RadioLib.h>
#include "esp_log.h"

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

#define PinPot 37
uint16_t valorPot = 0;
float TensaoDeSaida = 0;

bool ledtoogle = 0;


void applicationTask(void* pvParameters) {
    while (true) {
        slottimecontrol();
        
        if ((actualslot == 0) && (actualslot != lastslot)) {
            if(loramesh.mydd.devtype == DEV_TYPE_ROUTER) 
                nextstate = ST_TXBEACON;
            else 
                nextstate = ST_RXWAIT;
        }

        lastActivityMillis = millis();
        if (lastslot != actualslot){
            log_i("app : slot=%d ns=%d ", actualslot, nextstate);
        }

        switch (nextstate) {
            case ST_TXBEACON:
                if ((loramesh.mydd.devtype == DEV_TYPE_ROUTER) && (actualslot == 0)) {
                    send_pct = 1;
                    lastActivityMillis = millis();
                    nextstate = ST_STANDBY; 
                }
                break;
            case ST_RXWAIT:
                //manipula os pacotes recebidos pelo router e pelo ed
                log_i("Rxwait =%d",loramesh.lastpkt.packetSize);
                if(loramesh.receivePacket()){
                    lastActivityMillis = millis();
                    if(loramesh.mydd.devtype == DEV_TYPE_ROUTER){
                        //verifica a função da mensagem recebida
                        switch (loramesh.lastpkt.fct){
                            case FCT_DATA:{
                                setindpolls();
                                // log_i("Rx.seqnumb: %d",loramesh.lastpkt.seqnum);
                                break;
                            }
                            case FCT_WRITING: {
                                //mensagem de escrita
                                break;
                            }
                            case FCT_READING: {
                                uint8_t* rxPacket = loramesh.lastpkt.rxpacket;
                                uint8_t valueSize = rxPacket[5];
                                uint8_t* bufferValue = (uint8_t*) malloc(valueSize * sizeof(uint8_t)); //buffer para armazenamento do valor lido

                                //pega o o valor lido pelo ed

                                uint8_t size = loramesh.getResponseValue(rxPacket,bufferValue,valueSize);
                                #if DISPLAY_ENABLE  
                                  char display_line[20];
                                  sprintf(display_line,"B=  Rx=%d", loramesh.lastpkt.seqnum);
                                  displayline(2,display_line);
                                #endif

                                // log_i("Pacote recebido: %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X", rxpacket[0],rxpacket[1],rxpacket[2],rxpacket[3],rxpacket[4],rxpacket[5],rxpacket[6],rxpacket[7],rxpacket[8],rxpacket[9]);

                                // log_i("Size: %d",size);

                                if(size > 0){
                                    log_i("Pacote de leitura recebido. Valor lido (%d bytes): ",size); 

                                    uint32_t value;
                                    log_i("bufferValue: %2X %2X %2X %2X", bufferValue[0],bufferValue[1],bufferValue[2],bufferValue[3]);

                                    //copia os bytes do buffer para uma variavei valor;
                                    //a copia e feita invertida pois o processador da esp e little-endian
                                    int aux = 0;
                                    for(int i = 3; i >= 0; i--){
                                        *(&value + aux++) = bufferValue[i];
                                    }
                                    Serial.printf("Valor: %d\n",value);
                                }

                                else log_e("Erro na leitura do valor!");

                                free(bufferValue);
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
                        switch (loramesh.lastpkt.fct){
                            case FCT_BEACON:{
                                uint8_t* rxpacket = loramesh.lastpkt.rxpacket;
                                uint8_t len = loramesh.lastpkt.packetSize;
                                uint32_t timestamp = loramesh.gettimestamp(rxpacket,len);
                                node_init_sync(timestamp);

                                #if DISPLAY_ENABLE  
                                  char display_line[20];
                                  sprintf(display_line,"B= %d", loramesh.lastpkt.seqnum);
                                  displayline(2,display_line);
                                #endif

                                //log_i("Rx.seqnum: %d time=%d slot=%d", loramesh.lastpkt.seqnum,timestamp,actualslot);
                                nextstate = ST_STARTRX;

                                break;
                            }
                            case FCT_WRITING:{
                                //mensagem de escrita
                                break;
                            }
                            case FCT_READING:{
                                uint16_t seqnumber = loramesh.lastpkt.seqnum;
                                log_i("Rx.seqnumber: %d",seqnumber);
                                #if DISPLAY_ENABLE  
                                  char display_line[20];
                                  sprintf(display_line,"B=    Rx=%d", seqnumber);
                                  displayline(2,display_line);
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
                break;
            case ST_TXDATA: 
                lastActivityMillis = millis();
                //achq que aqui eu deveria montar o frame especifico e colocar na fila...
                send_pct = 1;
                nextstate = ST_STARTRX;
                break;

            case ST_STARTRX:
                if (send_pct == 0){
                    uint32_t rx_timeout = (loramesh.mydd.devtype == DEV_TYPE_ROUTER) ? 1000 : 2000; 
                    log_i("Sent...startReceiving=%d",rx_timeout);
                    loramesh.ClearRadioIRQs();
                    int res= loramesh.startReceiving(rx_timeout);
                    if (res > 0){
                        log_e("Error startReceiving=%d",res);
                    }
                    nextstate = ST_RXWAIT;
                }
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

void LerPotenciometro(void* pvParameters) {
    log_i("LerPotenciometroTask iniciada.");
    // NOTA: Esta tarefa agora serve apenas para log local no ED.
    // O valor lido aqui não é mais enviado pela rede.
    for (;;) {
        uint16_t leitura_completa = analogRead(PinPot);
        valorPot = leitura_completa;
        TensaoDeSaida = (((float)valorPot / 4095.0) * 3.3);
        
        log_d("ValorPot (0-4095) = %d | Tensao de Saida = %.2fV", valorPot, TensaoDeSaida);
        
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void ledblink(uint8_t ledpin) {
    digitalWrite(ledpin, HIGH);
    delay(100);
    digitalWrite(ledpin, LOW);
}