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
extern volatile bool messageReceived;
extern uint8_t actualslot;
extern uint8_t send_pct;
extern uint32_t lastActivityMillis;
uint8_t lastslot = 0;



void setindpolls();
void slottimecontrol(void);
void node_init_sync(uint32_t new_FR);
void displayline(uint8_t line, char *pucMsg, ...);
void ledblink(uint8_t ledpin);

extern float TensaoDeSaida;
extern uint16_t valorPot;

extern char display_line1[20];
extern char display_line2[20];
extern char display_line3[20];

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

    //tcpMsgs e preenchida manualmente para fins de teste 

    // tcpMsgs[3].msg.dst = 4;
    // tcpMsgs[3].msg.src = 1;
    // tcpMsgs[3].msg.function = FCT_READING;
    // tcpMsgs[3].msg.start = LEDP;
    // tcpMsgs[3].msg.qtdParametros = 1;
    // tcpMsgs[3].msg.payload.value = 0; //valor para acender o led
    // tcpMsgs[3].ocupado = true; //marca o slot como ocupado

    tcpMsgs[2].msg.dst = 2;
    tcpMsgs[2].msg.src = 0;
    tcpMsgs[2].msg.function = FCT_WRITTING;
    tcpMsgs[2].msg.start = LEDP;
    tcpMsgs[2].msg.qtdParametros = 1;
    tcpMsgs[2].msg.payload.value = 1; //valor para apagar o led
    tcpMsgs[2].ocupado = true; //marca o slot como ocupado


    while (true) {
        slottimecontrol();
        // log_i("Aplication task iniciaida. Slot atual: %d lastslot=%d", actualslot, lastslot);
        
        // log_i("app : slot=%d ns=%d ", actualslot, nextstate);

        lastActivityMillis = millis();

        switch (nextstate) {
            case ST_TXBEACON:
                if ((loramesh.mydd.devtype == DEV_TYPE_ROUTER) && (actualslot == 0)) {
                    // txmsg.dst = BROADCAST_ADDR;
                    // txmsg.function = FCT_BEACON;
                    // txmsg.size = 0;
                    // xQueueSend(txQueue, &txmsg, 0);

                    tcpMsgs[2].ocupado = true; // marca o slot como ocupado

                    lastActivityMillis = millis();

                    memset(&msgTx, 0, sizeof(msg_t)); // Limpa a estrutura msg antes de usá-la

                    //montagem do pacote em msg
                    dstTask = COMM;
                    msgTx.dst = BROADCAST_ADDR;
                    msgTx.src = loramesh.mydd.devaddr;
                    msgTx.seqnum = loramesh.mydd.seqnum; //o seqnum é incrementado na função de envio do pacote
                    msgTx.function = FCT_BEACON;
                    msgTx.size = 4;
                    msgTx.payload.value = millis() & 0xFFFFFFFF; // exemplo de payload, pode ser o timestamp ou outro dado relevante

                    #if DISPLAY_ENABLE
                        sprintf(display_line3,"seqnum: %d",loramesh.mydd.seqnum);
                        Heltec.DisplayShowAll(display_line1,display_line2,display_line3);
                    #endif
                    nextstate = ST_TXDATA;

                    
                    // nextstate = ST_STANDBY; 
                }
                break;

            case ST_RXWAIT:

                lastActivityMillis = millis();
                //router
                if(loramesh.mydd.devtype == DEV_TYPE_ROUTER){
                    //verifica se há msg para ser enviada
                    if(tcpMsgs[actualslot].ocupado){
                        //monta o pacote
                        

                        msgTx = tcpMsgs[actualslot].msg;
                        dstTask = COMM;
                    
                        log_i("Mensagem pronta para envio no slot %d", actualslot);
                        log_i("Mensagem detalhes - Src: %d, Dst: %d, Function: %d, Start: %d, QtdParametros: %d, Data: %d", 
                            msgTx.src, msgTx.dst, msgTx.function, msgTx.start, msgTx.qtdParametros, msgTx.payload.value);

                        // tcpMsgs[actualslot].ocupado = false; // marca o slot como livre
                        tcpMsgs[actualslot].msg.payload.value = !tcpMsgs[actualslot].msg.payload.value; // inverte o valor do payload para teste (ex: se for 1, vira 0 e vice-versa)
                        tcpMsgs[actualslot].ocupado = false; // marca o slot como livre
                        nextstate = ST_TXDATA;
                        continue;
                    }

                   
                    //verifica se há mensagens vindas da rede LoRa
                    //seria necessario aqui tambem montar um pacote para enviar para a aplicação
                    if(xQueueReceive(q_comm2app, &msgRx, 0) == pdTRUE){ 
                        log_i("Resposta do ed %d recebida. FCT=%d value=%d", msgRx.src, msgRx.function, msgRx.payload.value);

                        dstTask = TCP;

                        //verifica o tipo de função
                        switch(msgRx.function){

                            case FCT_BEACON:
                                setindpolls();
                                break;

                            case FCT_WRITTING:
                                if (msgRx.payload.value == 1){
                                    log_i("Escrita realizada com sucesso no no %d",msgRx.src);
                                }
                                else{
                                    log_i("Falha na escrita! Erro no nó ou parâmetro de escrita inválido%d",msgRx.src);
                                }

                                #if DISPLAY_ENABLE
                                    sprintf(display_line3,(msgRx.payload.value == 1) ? "Escrita: Sucesso" : "Escrita: Falha");
                                    Heltec.DisplayShowAll(display_line1,display_line2,display_line3);
                                #endif
                                break;

                            case FCT_READING:
                                // a divisao por 100 é para converter o valor inteiro de volta para float
                                float value = msgRx.payload.value / 100.0;
                                log_i("Valor lido: %.2f",value);

                                #if DISPLAY_ENABLE
                                    sprintf(display_line3,"Leitura: %.2f",value);
                                    Heltec.DisplayShowAll(display_line1,display_line2,display_line3);
                                #endif
                                break;
                        }
                    }

                    //verifica se há mensagens recebidas para o slot atual. Se sim, guarda-as em um array
                    if(xQueueReceive(q_tcp2app,&msgRx,0) == pdTRUE){
                        uint8_t slot = msgRx.dst;

                        //verifica se o endereço da mensagem é valido
                        if(msgRx.dst >= MAX_SLOTS){
                            log_e("Endereço de destino inválido recebido do TCP: %d", msgRx.dst);
                            continue;
                            //talvez seria interessante aqui retornar uma resposta de erro à aplicação
                        }

                        //preciso verificar a validade dos paramatros
                        
                        else if(slot < MAX_SLOTS){
                            tcpMsgs[slot].msg = msgRx;
                            tcpMsgs[slot].ocupado = true;
                            log_i("Mensagem recebida do TCP para o slot %d", slot);
                        }
                        else{
                            log_e("Slot inválido recebido do TCP: %d", slot);
                        }
                    }
                }

                //end device
                else{
                    // log_i("Aguardando pacote LoRa... Slot atual: %d", actualslot);
                    if(xQueueReceive(q_comm2app,&msgRx,0) == pdTRUE){
                        log_i("Mensagem recebida! FCT=%d",msgRx.function);
                        //verifica o tipo de função
                        switch (msgRx.function){
                            case FCT_BEACON:
                                node_init_sync(msgRx.payload.value); //o payload do beacon é o timestamp do router
                                //monta o frame de resposta do beacon
                                msgTx.dst = 1; //endereço do router
                                msgTx.src = loramesh.mydd.devaddr;
                                msgTx.seqnum = loramesh.mydd.seqnum; //o seqnum da resposta é o mesmo da requisição
                                msgTx.function = FCT_BEACON;
                                msgTx.size = 4;
                                msgTx.payload.value = msgRx.payload.value; //o payload da resposta do beacon é o mesmo da requisição (timestamp do router)

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
                                    msgTx.dst = 0; //endereço do router
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
                                    uint16_t valorPot = analogRead(PinPot);
                                    log_i("Valor do potenciometro lido: %d", valorPot);

                                    msgTx.src = loramesh.mydd.devaddr;
                                    msgTx.dst = 0; //endereço do router
                                    msgTx.seqnum = loramesh.mydd.seqnum; //o seqnum da resposta é o mesmo da requisição
                                    msgTx.function = FCT_READING;
                                    msgTx.size = sizeof(valorPot);
                                    msgTx.payload.value = valorPot;
                                    
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
                                
                        //envia a resposta para o router
                        nextstate = ST_TXDATA; 
                        continue; //pula o delay para enviar a resposta imediatamente 
                    }
                        
                }

                break;

            case ST_TXDATA: 
                lastActivityMillis = millis();
                //achq que aqui eu deveria montar o frame especifico e colocar na fila...
                if (loramesh.mydd.devtype == DEV_TYPE_ROUTER){

                    if(dstTask == COMM){
                        //mensagem vinda do TCP, deve ser enviada para o ED
                        
                        // if(msgTx.function == FCT_BEACON){
                        //     log_i("Mensagem de beacon enviada para a rede LoRa");
                        // }
                        // else
                        //     log_i("Mensagem do TCP enviada no slot %d para o ed %d", actualslot, msgTx.dst);
                        
                        xQueueSend(q_app2comm, &msgTx, 0);
                        
                    }

                    else if(dstTask == TCP){
                        //mensagem vinda da rede LoRa, deve ser enviada para o TCP
                        //a mensagem já está pronta para ser enviada, basta colocá-la na fila q_app2tcp
                        xQueueSend(q_app2tcp, &msgTx, 0);

                        // log_i("Mensagem da rede LoRa enviada para o TCP. Src: %d, Dst: %d, Function: %d, Start: %d, QtdParametros: %d, Data: %d", 
                        //     msgTx.src, msgTx.dst, msgTx.function, msgTx.start, msgTx.qtdParametros, msgTx.payload.value);
                    }


                }
                else{//end device


                    //como o ed so possui um fluxo de comunicação (resposta ao router), ele simplesmente pega a mensagem da fila q_comm2app e envia para o router
                    xQueueSend(q_app2comm, &msgTx, 0);
                //     //resposta de leitura do ED
                //     if(rxMsg.function == FCT_READING){
                //         // o valor é multiplicado por 100 para enviar como inteiro
                //         uint32_t value= TensaoDeSaida*100;
                //         uint8_t *pucaux = (uint8_t *) &value;
                //         // log_i("Value: %d",value);
                //         txmsg.dst = 1;
                //         txmsg.function = FCT_READING;
                //         txmsg.size = sizeof(value);
                //         txmsg.value = 0;
                //         txmsg.payload[0] = *(pucaux+3);
                //         txmsg.payload[1] = *(pucaux+2);
                //         txmsg.payload[2] = *(pucaux + 1);
                //         txmsg.payload[3] = *pucaux;
                //         xQueueSend(txQueue, &txmsg, 0);

                // }   else {
                //         //reposta de escrita do ED
                //         txmsg.dst = 1;
                //         txmsg.function = FCT_WRITTING;
                //         txmsg.size = 0;
                //         txmsg.start = 0;
                //         txmsg.qtdParametros = 0;
                //         txmsg.value = 1; //código de sucesso
                        
                //         xQueueSend(txQueue, &txmsg, 0);
                // }
                }
                // send_pct = 0;
                nextstate = ST_WAITTXDONE;
                break;

            case ST_WAITTXDONE:
                //espera por uma reposta da comm de que a transmissão terminou antes de iniciiar o modo de escuta
                msg_t TxStatus;
                if(xQueueReceive(q_comm2app, &TxStatus, 0) == pdTRUE){
                    if(TxStatus.function == FCT_TXDONE){
                        log_i("Transmissão concluída. Voltando para modo de escuta.");
                        send_pct = 0;
                        nextstate = ST_STARTRX;
                    }
                }
                vTaskDelay(20 / portTICK_PERIOD_MS); // espera 100ms (ajustar conforme necessário)
                
                break;

            case ST_STARTRX:
                if (send_pct == 0){
                    send_pct = 1;
                    rx_timeout = (loramesh.mydd.devtype == DEV_TYPE_ROUTER) ? 0 : 0; 
                    log_i("Sent...startReceiving=%d",rx_timeout);
                    loramesh.ClearRadioIRQs();
                    res= loramesh.startReceiving(rx_timeout);
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


void ledblink(uint8_t ledpin) {
    digitalWrite(ledpin, HIGH);
    delay(100);
    digitalWrite(ledpin, LOW);
}