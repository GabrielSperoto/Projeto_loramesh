#include "Arduino.h"
#include "devconfig.h"
#include "main.h"
#include "heltec.h"
#include "Lora/Loramesh.h"
#include <Wire.h>
#include <RadioLib.h>
#include "esp_log.h"

#define PinPot 37
#define PinLed 25

extern statemac nextstate;
extern volatile bool messageReceived;
extern uint8_t actualslot;
extern uint8_t send_pct;
uint8_t lastslot = 0;



void setindpolls();
void slottimecontrol(void);
void node_init_sync(uint32_t new_FR);


extern float TensaoDeSaida;
extern uint16_t valorPot;

bool ledtoogle = 0;

tcpMsgs_t tcpMsgs[MAX_SLOTS]; // array usado para receber mensagens do TCP e armazenar no slot correspondente
task_t dstTask; //indica para qual fila a app deve enviar a mensagem

//variaveis usadas em ST_STARTRX
uint32_t rx_timeout;
int res;


void applicationTask(void* pvParameters) {
    msg_t msgTx;
    msg_t msgRx;

    // ledblink(PinLed);

    for(int i=0;i<MAX_SLOTS;i++){
        memset(&tcpMsgs[i], 0, sizeof(tcpMsgs_t)); // Limpa a estrutura de cada slot
        tcpMsgs[i].ocupado = false; // inicializa todos os slots como livres
    }

    

    while (true) {
        slottimecontrol();

        // if(actualslot == 0){
        //     //pisca o led somente para conferir a sincronização dos nos
        //     ledblink(PinLed);
        // }


        lastActivityMillis = millis();

        switch (nextstate) {
            case ST_TXBEACON:
                if (actualslot == 0) {

                
                    lastActivityMillis = millis();

                    memset(&msgTx, 0, sizeof(msg_t)); // Limpa a estrutura msg antes de usá-la

                    //montagem do pacote em msg
                    dstTask = COMM;
                    msgTx.dst = BROADCAST_ADDR;
                    msgTx.src = loramesh.mydd.devaddr;
                    msgTx.seqnum = loramesh.mydd.seqnum; //o seqnum é incrementado na função de envio do pacote
                    msgTx.function = FCT_BEACON;
                    msgTx.size = 4;
                    msgTx.payload.value = 0; // timestamp zerado para verificar uma condição de overflow

                    #if DISPLAY_ENABLE
                        // Heltec.DisplayClear();
                        // sprintf(display_line3,"seqnum: %d",loramesh.mydd.seqnum);
                        // Heltec.DisplayShowAll(display_line1,display_line2,display_line3);
                    #endif
                    nextstate = ST_TXDATA;
                    continue;

                    
                    // nextstate = ST_STANDBY; 
                }
                break;

            case ST_RXWAIT:

                lastActivityMillis = millis();
                
                //verifica se há msg para ser enviada
                if(tcpMsgs[actualslot].ocupado){
                    //monta o pacote
                    

                    msgTx = tcpMsgs[actualslot].msg;
                    dstTask = COMM;
            
                    tcpMsgs[actualslot].ocupado = false; 
                    nextstate = ST_TXDATA;
                    continue;
                }

                
                //verifica se há mensagens vindas da rede LoRa

                if(xQueueReceive(q_comm2app, &msgRx, 0) == pdTRUE){ 

                    // dstTask = TCP; não utilizada por enquanto

                    switch(msgRx.function){

                        case FCT_BEACON:
                            // setindpolls();
                            break;

                        case FCT_WRITTING:
                            if (msgRx.payload.value == 1){
                                log_i("Escrita realizada com sucesso no no %d",msgRx.src);
                            }
                            else{
                                log_i("Falha na escrita! Erro no nó ou parâmetro de escrita inválido%d",msgRx.src);
                            }

                            #if DISPLAY_ENABLE
                                Heltec.DisplayClear();
                                sprintf(display_line3,(msgRx.payload.value == 1) ? "Escrita: Sucesso" : "Escrita: Falha");
                                Heltec.DisplayShowAll(display_line1,display_line2,display_line3);
                            #endif

                            msgTx = msgRx;
                            dstTask = TCP;
                            nextstate = ST_TXDATA;
                            continue;

                        case FCT_READING:
                            #if DISPLAY_ENABLE
                                Heltec.DisplayClear();
                                if(msgRx.start == POT){
                                    float leitura = ((float)msgRx.payload.value * 3.3f) / 1000.0f;
                                    sprintf(display_line3,"Leitura: %.2f", leitura);
                                } else {
                                    sprintf(display_line3,"Leitura: %d", msgRx.payload.value);
                                }
                                Heltec.DisplayShowAll(display_line1,display_line2,display_line3);
                            #endif

                            msgTx = msgRx;
                            dstTask = TCP;
                            nextstate = ST_TXDATA;
                            continue;
                    }
                }

                if(xQueueReceive(q_tcp2app,&msgRx,0) == pdTRUE){
                    uint8_t slot = msgRx.dst;

                    if(msgRx.dst >= MAX_SLOTS){
                        log_e("Endereço de destino inválido recebido do TCP: %d", msgRx.dst);
                        continue;
                        //talvez seria interessante aqui retornar uma resposta de erro à aplicação
                    }
                    
                    else if(slot < MAX_SLOTS){
                        tcpMsgs[slot].msg = msgRx;
                        tcpMsgs[slot].ocupado = true;
                    }
                    else{
                        log_e("Slot inválido recebido do TCP: %d", slot);
                    }
                }
                

                break;

            case ST_TXDATA: 
                lastActivityMillis = millis();

                if(dstTask == COMM){
                    //mensagem vinda do TCP, deve ser enviada para a rede LoRa
                    xQueueSend(q_app2comm, &msgTx, 0);
                }

                else if(dstTask == TCP){
                    //mensagem vinda da rede LoRa, deve ser enviada para o TCP
                    //a mensagem já está pronta para ser enviada, basta colocá-la na fila q_app2tcp
                    xQueueSend(q_app2tcp, &msgTx, 0);
                }


                // send_pct = 0;
                nextstate = ST_WAITTXDONE;
                break;

            case ST_WAITTXDONE:

                msg_t TxStatus;
                if(xQueueReceive(q_comm2app, &TxStatus, 0) == pdTRUE){
                    if(TxStatus.function == FCT_TXDONE){
                        // log_i("Transmissão concluída. Voltando para modo de escuta.");
                        send_pct = 0;
                        nextstate = ST_STARTRX;
                    }
                }
                vTaskDelay(20 / portTICK_PERIOD_MS); // espera 100ms (ajustar conforme necessário)
                
                break;

            case ST_STARTRX:
                if (send_pct == 0){
                    send_pct = 1;

                    loramesh.ClearRadioIRQs();
                    res= loramesh.startReceiving(RX_ROUTER_TIMEOUT);
                    if (res != RADIOLIB_ERR_NONE) {
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

        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}


