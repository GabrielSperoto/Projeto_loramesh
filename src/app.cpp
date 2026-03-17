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

msg_t tcpMsgs[MAX_SLOTS]; // array usado para receber mensagens do TCP e armazenar no slot correspondente

//variaveis usadas em ST_STARTRX
uint32_t rx_timeout;
int res;


void applicationTask(void* pvParameters) {
    TxMessage_t txmsg;
    RxMessage_t rxMsg;
    msg_t msg;

    for(int i=0;i<MAX_SLOTS;i++){
    tcpMsgs[i].ocupado = false; // inicializa todos os slots como livres
    }

    while (true) {
        slottimecontrol();
        
        if (actualslot != lastslot) {
            if(actualslot == 0){
                if(loramesh.mydd.devtype == DEV_TYPE_ROUTER) 
                    nextstate = ST_TXBEACON;
                else 
                    nextstate = ST_RXWAIT;
            }

            // else if(actualslot == 2){
            //     if(loramesh.mydd.devtype == DEV_TYPE_ROUTER)
            //         nextstate = ST_TXDATA;
            // }
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

                    //montagem do pacote em msg
                    msg.origem = APP;
                    msg.dst = BROADCAST_ADDR;
                    msg.function = FCT_BEACON;
                    msg.size = 0;

                    nextstate = ST_TXDATA;

                    lastActivityMillis = millis();
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
                        // msg.origem = tcpMsgs[actualslot].origem;
                        // msg.src = tcpMsgs[actualslot].src;
                        // msg.dst = tcpMsgs[actualslot].dst;
                        // msg.function = tcpMsgs[actualslot].function;
                        // msg.start = tcpMsgs[actualslot].start;
                        // msg.qtdParametros = tcpMsgs[actualslot].qtdParametros;
                        // msg.data = tcpMsgs[actualslot].data;

                        msg = tcpMsgs[actualslot];
                    
                        log_i("Mensagem pronta para envio no slot %d", actualslot);
                        log_i("Mensagem detalhes - Origem: %d, Src: %d, Dst: %d, Function: %d, Start: %d, QtdParametros: %d, Data: %d", 
                            msg.origem, msg.src, msg.dst, msg.function, msg.start, msg.qtdParametros, msg.data.value);

                        tcpMsgs[actualslot].ocupado = false; // marca o slot como livre
                        nextstate = ST_TXDATA;
                    }

                   
                    //verifica se há mensagens vindas da rede LoRa
                    else if(xQueueReceive(q_comm2app, &msg, 0) == pdTRUE){ 
                        log_i("Resposta do ed %d recebida. FCT=%d value=%d", msg.src, msg.function, msg.data.value);

                        //verifica o tipo de função
                        switch(msg.function){

                            case FCT_SYNC_SUCESS:
                                setindpolls();
                                break;

                            case FCT_WRITINGRES:
                                if (msg.data.value == 1){
                                    log_i("Escrita realizada com sucesso no no %d",msg.src);
                                }
                                else{
                                    log_i("Falha na escrita! Erro no nó ou parâmetro de escrita inválido%d",msg.src);
                                }

                                #if DISPLAY_ENABLE
                                    sprintf(display_line3,(msg.data.value == 1) ? "Escrita: Sucesso" : "Escrita: Falha");
                                    Heltec.DisplayShowAll(display_line1,display_line2,display_line3);
                                #endif
                                break;

                            case FCT_READINGRES:
                                // a divisao por 100 é para converter o valor inteiro de volta para float
                                float value = msg.data.value / 100.0;
                                log_i("Valor lido: %.2f",value);

                                #if DISPLAY_ENABLE
                                    sprintf(display_line3,"Leitura: %.2f",value);
                                    Heltec.DisplayShowAll(display_line1,display_line2,display_line3);
                                #endif
                                break;
                        }
                        
                        nextstate = ST_TXDATA;  
                    }

                    //verifica se há mensagens recebidas para o slot atual. Se sim, guarda-as em um array
                    else if(xQueueReceive(q_tcp2app,&msg,0) == pdTRUE){
                        uint8_t slot = msg.dst;

                        //verifica se o endereço da mensagem é valido
                        if(msg.dst >= MAX_SLOTS){
                            log_e("Endereço de destino inválido recebido do TCP: %d", msg.dst);
                            continue;
                            //talvez seria interessante aqui retornar uma resposta de erro à aplicação
                        }

                        //preciso verificar a validade dos paramatros
                        
                        else if(slot < MAX_SLOTS){
                            tcpMsgs[slot] = msg;
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
                    if(xQueueReceive(q_comm2app,&msg,0) == pdTRUE){
                        log_i("Mensagem recebida! FCT=%d",msg.function);
                        //verifica o tipo de função
                        switch (msg.function){
                            case FCT_WRITINGREQ:
                                //verifica o paramtro da escrita
                                if(msg.start == LEDP){
                                    if (msg.data.value == 1){
                                        log_i("Led Aceso!");
                                        digitalWrite(PinLed, HIGH);

                                    }
                                    else{
                                        log_i("Led apagado!");
                                        digitalWrite(PinLed, LOW);
                                    }

                                    //retorna uma resposta ao router
                                    msg.src = loramesh.mydd.devaddr;
                                    msg.dst = 0; //endereço do router
                                    msg.function = FCT_WRITINGRES;
                                    msg.size = 1;
                                    msg.data.value = 1; //status de sucesso
                                    
                                    
                                }
                                else if(msg.start == POT){
                                    //como não há como escrever em um potenciometro, retorna um codigo de erro (0)
                                    log_e("Tentativa de escrita em um parâmetro de leitura (POT)");
                                    msg.src = loramesh.mydd.devaddr;
                                    msg.dst = 0; //endereço do router
                                    msg.function = FCT_WRITINGRES;
                                    msg.size = 1;
                                    msg.data.value = 0; //status de erro
        
                                }
                                break;
                            case FCT_READINGREQ:
                                //verifica o parametro da leitura
                                if(msg.start == POT){
                                    //obtem o valor do potenciometro e retorna na resposta
                                    uint16_t valorPot = analogRead(PinPot);
                                    log_i("Valor do potenciometro lido: %d", valorPot);

                                    msg.src = loramesh.mydd.devaddr;
                                    msg.dst = 0; //endereço do router
                                    msg.function = FCT_READINGRES;
                                    msg.size = sizeof(valorPot);
                                    msg.data.value = valorPot;
                                    
                                }
                                else if(msg.start == LEDP){
                                    //aqui eu retorno o status do LED
                                    log_i("Status do LED lido: %s", digitalRead(PinLed) == HIGH ? "ON" : "OFF");
                                    msg.src = loramesh.mydd.devaddr;
                                    msg.dst = 0; //endereço do router
                                    msg.function = FCT_READINGRES;
                                    msg.size = 1;
                                    msg.data.value = digitalRead(PinLed) == HIGH ? 1 : 0; //status do LED
                                
                                }
                                break;
                            
                            case FCT_DESCRIPTION: 
                                //mensagem de descrição
                                break;
                        }
                                
                        //envia a resposta para o router
                        nextstate = ST_TXDATA;  
                    }
                        
                }
                //manipula os pacotes recebidos pelo router e pelo ed
                // if (xQueueReceive(rxQueue, &rxMsg, 0) == pdTRUE) {

                //     log_i("App recebeu pacote no slot %d de %d func=%d size=%d RSSI=%d ",
                //         actualslot, rxMsg.src, rxMsg.function, rxMsg.size, rxMsg.rssi);

                //     // guarda a mensagem recebida no buffer do slot correspondente
                    
                //     if(loramesh.mydd.devtype == DEV_TYPE_ROUTER){
    
                //         log_i("rxMsg.payload[0]: %d, rxMsg.payload[1]: %d", rxMsg.payload[0], rxMsg.payload[1]);

                //         //verifica a função da mensagem recebida
                //         switch (rxMsg.function){
                //             case FCT_SYNC_SUCESS:{
                //                 setindpolls();
                //                 // log_i("Rx.seqnumb: %d",loramesh.lastpkt.seqnum);
                //                 break;
                //             }
                //             case FCT_WRITINGRES: {
                //                 uint8_t writtingCode = loramesh.getResponseStatus();
                //                 txmsg.value = writtingCode;
                //                 if (writtingCode == 1){
                //                     log_i("Escrita realizada com sucesso no no %d",rxMsg.src);
                //                 }
                //                 else{
                //                     log_i("Falha na escrita no no %d",rxMsg.src);
                //                 }
                //                 nextstate = ST_TXDATA;
                //                 break;
                //             }
                //             case FCT_READINGRES: {
                //                 // a divisao por 100 é para converter o valor inteiro de volta para float
                //                 float value = loramesh.getReadingDataAsUint32();
                //                 txmsg.value = value;
                //                 if(value > 0){
                //                     log_i("Valor lido: %.2f",value);
                //                 }
                //                 nextstate = ST_TXDATA;
                //                 break;
                //             }
                //             case FCT_DESCRIPTION: {
                //                 //mensagem de descrição
                //                 break;
                //             }
                //         }
                //     }

                //     else{ //end device
                //         //verifica o destino da mensagem
                //         switch (rxMsg.function){
                //             case FCT_BEACON:{
                //                 uint8_t* rxpacket = rxMsg.payload;
                //                 uint8_t len =rxMsg.size;
                //                 uint32_t timestamp = loramesh.gettimestamp(rxpacket,len);
                //                 loramesh.mydd.seqnum = loramesh.getLastPctSeqNum();
                //                 node_init_sync(timestamp);
    
                //                 #if DISPLAY_ENABLE  
                //                   char display_line[20];
                //                   sprintf(display_line,"Seq. number: %d", loramesh.mydd.seqnum);
                //                   displayline(3,display_line);
                //                 #endif
    
                //                 //log_i("Rx.seqnum: %d time=%d slot=%d", loramesh.lastpkt.seqnum,timestamp,actualslot);
                //                 nextstate = ST_STARTRX;
    
                //                 break;
                //             }
                //             case FCT_WRITINGREQ:{
                //                 //mensagem de escrita
                //                 if (loramesh.getWrittingCode() == 1){
                //                     log_i("Led Aceso!");
                //                     digitalWrite(PinLed, HIGH);
                //                 }
                //                 else{
                //                     log_i("Led apagado!");
                //                     digitalWrite(PinLed, LOW);

                //                 }
                //                 nextstate = ST_TXDATA;
                //                 break;
                //             }
                //             case FCT_READINGREQ:{
                //                 // log_i("Pacote de leitura recebido! ");
                //                 uint16_t seqnumber = loramesh.lastpkt.seqnum;
                //                 log_i("Rx.seqnumber: %d",seqnumber);
                //                 #if DISPLAY_ENABLE  
                //                   char display_line[20];
                //                   sprintf(display_line,"Seq. number: %d", seqnumber);
                //                   displayline(3,display_line);
                //                 #endif
                //                 nextstate = ST_TXDATA;
                //                 //mensagem de leitura
                //                 break;
                //             }
                //             case FCT_DESCRIPTION: {
                //                 //mensagem de descrição
                //                 break;
                //             }
                //         }
                //     }
                // }

                break;

            case ST_TXDATA: 
                lastActivityMillis = millis();
                //achq que aqui eu deveria montar o frame especifico e colocar na fila...
                if (loramesh.mydd.devtype == DEV_TYPE_ROUTER){
                    // txmsg.src = loramesh.mydd.devaddr;
                    // txmsg.dst = BROADCAST_ADDR; //DST seria o enderco do ed que enviou a resposta ?
                    // txmsg.function = rxMsg.function;
                    // txmsg.start = 1;
                    // txmsg.qtdParametros = 1;
                    // xQueueSend(txQueue, &txmsg, 0);
                    
                    // else{
                    //     //mensagem vinda da rede LoRa, deve ser enviada para o TCP
                    //     //a mensagem já está pronta para ser enviada, basta colocá-la na fila q_app2tcp
                    //     xQueueSend(q_app2tcp, &msg, 0);

                    //     log_i("Mensagem da rede LoRa enviada para o TCP. Origem: %d, Src: %d, Dst: %d, Function: %d, Start: %d, QtdParametros: %d, Data: %d", 
                    //         msg.origem, msg.src, msg.dst, msg.function, msg.start, msg.qtdParametros, msg.data.value);
                    // }

                    if(msg.origem == TCP){
                        //mensagem vinda do TCP, deve ser enviada para o ED
                        xQueueSend(q_app2comm, &msg, 0);

                        log_i("Mensagem do TCP enviada no slot %d para o ed %d", actualslot, msg.dst);
                    }

                    else if(msg.origem == LORA){
                        //mensagem vinda da rede LoRa, deve ser enviada para o TCP
                        //a mensagem já está pronta para ser enviada, basta colocá-la na fila q_app2tcp
                        xQueueSend(q_app2tcp, &msg, 0);

                        log_i("Mensagem da rede LoRa enviada para o TCP. Origem: %d, Src: %d, Dst: %d, Function: %d, Start: %d, QtdParametros: %d, Data: %d", 
                            msg.origem, msg.src, msg.dst, msg.function, msg.start, msg.qtdParametros, msg.data.value);
                    }

                    else if(msg.origem == APP){
                        //mensagem gerada internamente na aplicação
                        //verifica o tipo dela
                        if(msg.function == FCT_BEACON && msg.dst == BROADCAST_ADDR){
                            xQueueSend(q_app2comm,&msg,0);
                            log_i("Mensagem de beacon enviada para a rede LoRa");
                        }
                        else{
                            log_i("Mensagem interna ignorada. Origem: %d, Src: %d, Dst: %d, Function: %d, Start: %d, QtdParametros: %d, Data: %d", 
                                msg.origem, msg.src, msg.dst, msg.function, msg.start, msg.qtdParametros, msg.data.value);

                        }
                    }
                }
                else{//end device


                    //como o ed so possui um fluxo de comunicação (resposta ao router), ele simplesmente pega a mensagem da fila q_comm2app e envia para o router
                    xQueueSend(q_app2comm, &msg, 0);
                //     //resposta de leitura do ED
                //     if(rxMsg.function == FCT_READINGREQ){
                //         // o valor é multiplicado por 100 para enviar como inteiro
                //         uint32_t value= TensaoDeSaida*100;
                //         uint8_t *pucaux = (uint8_t *) &value;
                //         // log_i("Value: %d",value);
                //         txmsg.dst = 1;
                //         txmsg.function = FCT_READINGRES;
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
                //         txmsg.function = FCT_WRITINGRES;
                //         txmsg.size = 0;
                //         txmsg.start = 0;
                //         txmsg.qtdParametros = 0;
                //         txmsg.value = 1; //código de sucesso
                        
                //         xQueueSend(txQueue, &txmsg, 0);
                // }
                }
                send_pct = 0;
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