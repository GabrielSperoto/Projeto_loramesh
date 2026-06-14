#include "Arduino.h"
#include "devconfig.h"
#include "main.h"
#include "heltec.h"
#include "Lora/loramesh.h"
#include <Wire.h>
#include <RadioLib.h>
#include "esp_log.h"

#define PinPot 37
#define PinLed 25

extern statemac nextstate;
extern LoRaClass loramesh;
extern uint8_t actualslot;
extern uint8_t send_pct;
uint8_t lastslot = 0;



void slottimecontrol(void);
void node_init_sync(uint32_t new_FR);
void displayline(uint8_t line, char *pucMsg, ...);

bool ledtoogle = 0;


int res;


void applicationTask(void* pvParameters) {
    msg_t msgTx;
    msg_t msgRx;

    uint32_t last_time = 0;
    uint32_t counter = 0; //simula a leitura de um valor analogico

    while (true) {
        slottimecontrol();

        if(millis() - last_time >= 50){
            last_time = millis();
            counter += 10; //simula a leitura de um valor analogico
            if (counter > 1023) counter = 0; //reseta o contador para simular a leitura de um potenciometro
            TensaoDeSaida = (counter / 1023.0) * 3.3; //converte o valor do contador para uma tensão entre 0 e 3.3V
        }

        #if DISPLAY_ENABLE
            sprintf(display_line2, "Leitura: %.2f V", TensaoDeSaida);
            Heltec.DisplayShow2(display_line2);
        #endif

        lastActivityMillis = millis();

        switch (nextstate) {

            case ST_RXWAIT:

                lastActivityMillis = millis();
                
               
                if(xQueueReceive(q_comm2app,&msgRx,0) == pdTRUE){
                    
                    switch (msgRx.function){
                        case FCT_BEACON:
                            node_init_sync(msgRx.payload.value); 
                            //monta o frame de resposta do beacon

                            msgTx.dst = 1; //endereço do router
                            msgTx.src = loramesh.mydd.devaddr;
                            msgTx.seqnum = loramesh.mydd.seqnum; 
                            msgTx.function = FCT_BEACON;
                            msgTx.size = 4;
                            msgTx.payload.value = msgRx.payload.value;

                            #if DISPLAY_ENABLE
                                sprintf(display_line3,"Seqnum: %d",loramesh.mydd.seqnum);
                                Heltec.DisplayShowAll(display_line1,display_line2,display_line3);
                            #endif
                            break;
                        case FCT_WRITTING:
                            //verifica o paramtro da escrita
                            if(msgRx.start == LEDP){
                                if (msgRx.payload.value == 1){
                                    log_i("Led Aceso!");
                                    digitalWrite(PinLed, HIGH);

                                }
                                else{
                                    log_i("Led apagado!");
                                    digitalWrite(PinLed, LOW);
                                }

                                //retorna uma resposta ao router
                                msgTx.dst = 1; //endereço do router
                                msgTx.src = loramesh.mydd.devaddr;
                                msgTx.seqnum = loramesh.mydd.seqnum; //o seqnum da resposta é o mesmo da requisição   
                                
                                msgTx.function = FCT_WRITTING;
                                msgTx.size = 1;
                                msgTx.payload.value = 1; //status de sucesso
                                
                                
                            }
                            else if(msgRx.start == POT){
                                //como não há como escrever em um potenciometro, retorna um codigo de erro (0)
                                log_e("Tentativa de escrita em um parâmetro de leitura (POT)");
                                msgTx.src = loramesh.mydd.devaddr;
                                msgTx.dst = 1; //endereço do router
                                msgTx.seqnum = loramesh.mydd.seqnum; //o seqnum da resposta é o mesmo da requisição
                                msgTx.function = FCT_WRITTING;
                                msgTx.size = 1;
                                msgTx.payload.value = 0; //status de erro
    
                            }
                            break;
                        case FCT_READING:
                            //verifica o parametro da leitura
                            if(msgRx.start == POT){
                                //obtem o valor do potenciometro e retorna na resposta
                
                                msgTx.src = loramesh.mydd.devaddr;
                                msgTx.dst = 1; //endereço do router
                                msgTx.seqnum = loramesh.mydd.seqnum; //o seqnum da resposta é o mesmo da requisição
                                msgTx.function = FCT_READING;
                                msgTx.size = sizeof(counter);
                                msgTx.payload.value = counter;
                                
                            }
                            else if(msgRx.start == LEDP){
                                //aqui eu retorno o status do LED
                                log_i("Status do LED lido: %s", digitalRead(PinLed) == HIGH ? "ON" : "OFF");
                                msgTx.src = loramesh.mydd.devaddr;
                                msgTx.dst = 1; //endereço do router
                                msgTx.seqnum = loramesh.mydd.seqnum; //o seqnum da resposta é o mesmo da requisição
                                msgTx.function = FCT_READING;
                                msgTx.size = 1;
                                msgTx.payload.value = digitalRead(PinLed) == HIGH ? 1 : 0; //status do LED
                            
                            }
                            break;
                        
                        case FCT_DESCRIPTION: 
                            //mensagem de descrição
                            break;
                    }
                            
                    nextstate = ST_TXDATA; 
                    continue;
                }
                        
                

                break;

            case ST_TXDATA: 
                lastActivityMillis = millis();
              
                //messagem vinda da aplicação, deve ser enviada para a rede LoRa
                if(xQueueSend(q_app2comm, &msgTx, 0) != pdTRUE){
                    log_e("Falha ao enviar mensagem para a fila q_app2comm");
                };
                
                nextstate = ST_WAITTXDONE;
                break;

            case ST_WAITTXDONE:
                msg_t TxStatus;
                if(xQueueReceive(q_comm2app, &TxStatus, 0) == pdTRUE){
                    if(TxStatus.function == FCT_TXDONE){
                        send_pct = 0;
                        nextstate = ST_STARTRX;
                    }
                }
                vTaskDelay(20 / portTICK_PERIOD_MS);
                
                break;

            case ST_STARTRX:
                if (send_pct == 0){
                    send_pct = 1;
                    // log_i("Sent...startReceiving=%d",rx_timeout);
                    loramesh.ClearRadioIRQs();
                    res= loramesh.startReceiving(RX_ENDDEV_TIMEOUT);
                    if (res != RADIOLIB_ERR_NONE) {
                        log_e("Error startReceiving=%d",res);
                    }
                }
                nextstate = ST_RXWAIT;
                break;

            case ST_STANDBY:
                //estado a ser implementado
                lastActivityMillis = millis();
                break;
            
                default:
                break;
        }

        lastslot = actualslot;

        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}
