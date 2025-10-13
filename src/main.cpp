#include "Arduino.h"
#include "devconfig.h"
#include "heltec.h"
#include "Lora/loramesh.h"
#include <Wire.h>
#include <RadioLib.h>
#include "esp_log.h"

// --- Variáveis Globais (Mantidas do seu código original) ---
#define PinPot 37
uint16_t valorPot = 0;
float TensaoDeSaida = 0;

#if DISPLAY_ENABLE
#include "OLED/SSD1306.h"
#endif

// Handles das tarefas
TaskHandle_t App_TaskHandle = nullptr;
TaskHandle_t Send_TaskHandle = nullptr;
TaskHandle_t Watchdog_TaskHandle = nullptr;
TaskHandle_t LerPotenciometro_TaskHandle = nullptr;

// Variáveis globais de estado
extern LoRaClass loramesh;
extern volatile bool messageReceived;
char rxpacket[BUFFER_SIZE];
#if DISPLAY_ENABLE  
char display_line1[20];
char display_line2[20];
char display_line3[20];
#endif

statemac nextstate;
uint16_t idx_response = 0;
uint8_t send_pct = 0;
bool ledtoogle = 0;

long lastabstime = 0;
long lastscantime_ms = 0;
uint8_t actualslot = 0;
long slot_period = 0;
bool syncronized = false;

#define END_DEVICE_1_SLOT 1
#define END_DEVICE_2_SLOT 2
#define END_DEVICE_3_SLOT 3

uint32_t previous_FR = 0;
uint32_t current_FR = 0;
int32_t drift = 0;
float adjustedPeriod = EXPECTED_PERIOD_MS;

uint32_t lastActivityMillis = 0;

// --- Funções Auxiliares (Mantidas do seu código original) ---
void ledblink(uint8_t ledpin) {
    digitalWrite(ledpin, HIGH);
    delay(100);
    digitalWrite(ledpin, LOW);
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
    } else {
        slot_period = SLOT_INTERVAL;
    }

    currscantime_ms = (millis() - lastabstime);
    if (currscantime_ms >= slot_period) {
        lastabstime = millis();
        lastscantime_ms += currscantime_ms;
        actualslot++;

        if (actualslot > MAX_SLOTS) actualslot = 0;

        //router envia sempre o mesmo frame no slot 2
        if(actualslot == 0){
            if(loramesh.mydd.devtype == DEV_TYPE_ROUTER) nextstate = ST_TXBEACON;
            else nextstate = ST_RXWAIT;
        }
        else if(actualslot == 2){
        if(loramesh.mydd.devtype == DEV_TYPE_ROUTER) nextstate = ST_TXDATA;
    }
    }

    
    
}

void setindpolls() {
    uint16_t lastmyseqnum = loramesh.getLastSeqNum();
    uint16_t lastpacketseqnum = loramesh.getLastPctSeqNum();
    if (lastmyseqnum == lastpacketseqnum) {
        idx_response++;
        lastActivityMillis = millis();
        // log_i("Tx.sn=%d Rx.sn=%d Rx.cnt=%d ", lastmyseqnum, lastpacketseqnum, idx_response);

        #if DISPLAY_ENABLE  
        {
            sprintf(display_line3, "Tx=%d Rx=%d", lastmyseqnum, idx_response);
            Heltec.DisplayShowAll(display_line1, display_line2, display_line3);
        }   
        #endif
    }
}

// --- Definições das Tarefas com todas as correções ---

void watchdogTask(void* pvParameters) {
    while (true) {
        if ((millis() - lastActivityMillis) > 15000) {
            log_w("Watchdog: Sem atividade por 15 segundos. Reiniciando...");
            ESP.restart();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void applicationTask(void* pvParameters) {
    log_i("ApplicationTask iniciada.");
    while (true) {
        slottimecontrol();

    switch (nextstate) {
        case ST_TXBEACON:
            if ((loramesh.mydd.devtype == DEV_TYPE_ROUTER) && (actualslot == 0)) {
                send_pct = 1;
                lastActivityMillis = millis();
                nextstate = ST_RXWAIT; 
            }
            break;
        case ST_RXWAIT:
            //manipula os pacotes recebidos pelo router e pelo ed
            // loramesh.startReceiving();
            // log_i("parsePacket(0): %d",loramesh.parsePacket(0));
            // log_i("receivePacket(): %d",loramesh.receivePacket());
            if(loramesh.receivePacket()){
                lastActivityMillis = millis();
                if(loramesh.mydd.devtype == DEV_TYPE_ROUTER){
                    log_i("Mensagem recebida com sucesso ! Função do pacote: %d",loramesh.lastpkt.fct);
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
                            uint8_t packetSize = sizeof(rxPacket);
                            uint8_t valueSize = rxPacket[5];
                            uint8_t* bufferValue = (uint8_t*) malloc(valueSize * sizeof(uint8_t));

                            //pega o o valor lido pelo ed

                            uint8_t size = loramesh.getResponseValue(rxPacket,packetSize,bufferValue,valueSize);

                            // log_i("Pacote recebido: %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X", rxpacket[0],rxpacket[1],rxpacket[2],rxpacket[3],rxpacket[4],rxpacket[5],rxpacket[6],rxpacket[7],rxpacket[8],rxpacket[9]);

                            log_i("Size: %d",size);

                            if(size > 0){
                                log_i("Pacote de leitura recebido. Valor lido (%d bytes): ",size); 

                                float value;
                                memcpy(&value,bufferValue,size);
                                Serial.printf("Valor float: %f\n",value);
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
                            uint8_t len = sizeof(rxpacket);
                            uint32_t timestamp = loramesh.gettimestamp(rxpacket,len);
                            node_init_sync(timestamp);
                            log_i("Rx.seqnum: %d", loramesh.lastpkt.seqnum);
                            nextstate = ST_TXDATA;
                            break;
                        }
                        case FCT_WRITING:{
                            //mensagem de escrita
                            break;
                        }
                        case FCT_READING:{
                            uint16_t seqnumber = loramesh.lastpkt.seqnum;
                            log_i("Requisição de leitura recebida! Rx.seqnumber: %d",seqnumber);
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
            send_pct = 1;
            nextstate = ST_RXWAIT;
            break;
        case ST_STANDBY:
            lastActivityMillis = millis();
            //slepping mode
            break;
        default:
            break;
    }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void sendTask(void* pvParameters) {
    log_i("SendTask iniciada.");
    while (true) {
        if (send_pct) {
            send_pct = 0;
            if (loramesh.mydd.devtype == DEV_TYPE_ROUTER){
                uint16_t lastmyseqnum = loramesh.getLastSeqNum();  
                if(actualslot == 0){
                    uint8_t ret = loramesh.sendPacketReq(lastscantime_ms);
                    if(ret > 0){
                        log_i("Tx.seqnum= %d", lastmyseqnum);
                        #if 1
                            sprintf(display_line3,"Tx.seqnum = %d", lastmyseqnum);
                            Heltec.DisplayShowAll(display_line1,display_line2,display_line3);
                        #endif
                    }
                    else{
                        log_i("Falha no envio do beacon !");
                    }
                    
                }
                else if(actualslot == 2){
                    uint8_t* pucaux = (uint8_t*) &lastmyseqnum;
                    //frame de requisição que o route vai enviar
                    //{SRC,DST,FCT,Seq numb,START,QTD PARAMETROS,CRC}
                    uint8_t frame[] = {1,2,FCT_READING,*(pucaux+1),*(pucaux),1,1,BYTE_CRC};
                    uint8_t frameSize = sizeof(frame);
                    if(loramesh.sendPacket(frame,frameSize)) log_i("Requisição de leitura enviada. Aguardando resposta ...");
                    else log_i("Falha no envio do frame !");
                    
                }
                nextstate = ST_RXWAIT;
            } else {
                //ed response
                uint8_t *msg;
                uint8_t msg_size;
                switch(loramesh.lastpkt.fct){
                    case FCT_BEACON:{
                        loramesh.sendPacketRes(1);
                        nextstate = ST_RXWAIT;
                        break;
                    case FCT_READING:{
                        //response frame
                        //{SRC,DST,FCT,SEQ NUMBER,SIZE VALUE,VALUE,CRC};
                        uint32_t value = 230; // exemplo de leitura de um potenciometro
                        uint8_t sizeValue = sizeof(value);

                        if (loramesh.sendPacketResponse(1,sizeValue,value)) log_i("Resposta enviada!");
                        else log_e ("Falha no envio da resposta!");
                        nextstate = ST_RXWAIT;
                        break;
                        
                    }
                    }
                }
                

            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
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


void setup() {
    Serial.begin(115200);
    delay(1000); 

    Heltec.begin();
    loramesh.begin();
    lastActivityMillis = millis();

    if (loramesh.mydd.devtype == DEV_TYPE_ROUTER) {
        actualslot = 0;
        nextstate = ST_TXBEACON;
    } else {
        loramesh.startReceiving();
        Serial.println("Dispositivo configurado como End Device. Aguardando beacon...");
        nextstate = ST_RXWAIT;
    }

    #if DISPLAY_ENABLE  
    Heltec.DisplayClear();
    if (loramesh.mydd.devtype == DEV_TYPE_ROUTER) {
        sprintf(display_line1, "RT=%x ", loramesh.mydd.devserialnumber);
        Heltec.DisplayShow1(display_line1);
    } else {
        sprintf(display_line1, "ED=%x ", loramesh.mydd.devserialnumber);
        Heltec.DisplayShow1(display_line1);
    }
    sprintf(display_line2, "Addr=%x ", loramesh.mydd.devaddr);
    Heltec.DisplayShow2(display_line2);
    #endif

    Serial.printf("\nMemoria livre antes de criar tarefas: %u bytes\n", ESP.getFreeHeap());
    

    // CORREÇÃO: Pilhas com tamanhos seguros para evitar crashes
    xTaskCreatePinnedToCore(applicationTask, "ApplicationTask", 4096, NULL, 3, &App_TaskHandle, 1);
    xTaskCreatePinnedToCore(sendTask, "SendTask", 3072, NULL, 3, &Send_TaskHandle, 1);
    xTaskCreatePinnedToCore(watchdogTask, "WatchdogTask", 2048, NULL, 1, &Watchdog_TaskHandle, 1);
    
    // if (loramesh.mydd.devtype == DEV_TYPE_ENDDEV) {
    //     xTaskCreatePinnedToCore(LerPotenciometro, "LerPotenciometroTask", 2048, NULL, 1, &LerPotenciometro_TaskHandle, 1);
    // }
    Serial.println("--- Criacao de tarefas finalizada ---\n");
}

void loop() {
    // Tudo é feito nas tasks do FreeRTOS
}