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

// Frames de dados que os End Devices devem enviar
uint8_t msg_device1[] = {10, 20, 30, 40};
uint8_t msg_device2[] = {50, 60, 70, 80};
uint8_t msg_device3[] = {20,70,40,10};

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

        if (actualslot > MAX_SLOTS) {
            actualslot = 0;
            if (loramesh.mydd.devtype == DEV_TYPE_ROUTER)
                nextstate = ST_TXBEACON;
            else
                nextstate = ST_RXWAIT;
        }
    }
}

void setindpolls() {
    uint16_t lastmyseqnum = loramesh.getLastSeqNum();
    uint16_t lastpacketseqnum = loramesh.getLastPctSeqNum();
    if (lastmyseqnum == lastpacketseqnum) {
        idx_response++;
        lastActivityMillis = millis();
        log_i("Tx.sn=%d Rx.sn=%d Rx.cnt=%d ", lastmyseqnum, lastpacketseqnum, idx_response);

        #if DISPLAY_ENABLE  
        {
            char display_line3[20];
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
        // Serial.print("Message received: ");
        // Serial.println(messageReceived);
            if (messageReceived){
                messageReceived = false;
                int len = 0;
                if (loramesh.mydd.devtype == DEV_TYPE_ROUTER) {
                    // Serial.println(loramesh.parsePacket(0));
                    // log_i("%d",loramesh.receivePacket());
                        if (loramesh.receivePacket()) {
                            uint8_t sender_address = loramesh.lastpkt.srcaddress;
                            uint8_t* rxpacket = loramesh.lastpkt.rxpacket; // Usa o buffer da própria loramesh

                            // Verifica qual dispositivo enviou a mensagem pelo endereço de origem
                            switch (sender_address) {
                                case 2: // Endereço do End Device 1, conforme devid[] em Loramesh.cpp
                                    if (actualslot != END_DEVICE_1_SLOT) {
                                        log_w("Alerta: Pacote do End Device 1 (Addr %d) recebido fora do slot esperado (Slot Atual: %d)", sender_address, actualslot);
                                    }
                                    log_i("Resposta do End Device 1 recebida no slot %d: %d", actualslot, rxpacket[0]);
                                    break;

                                case 3: // Endereço do End Device 2
                                    if (actualslot != END_DEVICE_2_SLOT) {
                                        log_w("Alerta: Pacote do End Device 2 (Addr %d) recebido fora do slot esperado (Slot Atual: %d)", sender_address, actualslot);
                                    }
                                    log_i("Resposta do End Device 2 recebida no slot %d: %d", actualslot, rxpacket[0]);
                                    break;

                                case 4: // Endereço do End Device 3
                                    if (actualslot != END_DEVICE_3_SLOT) {
                                        log_w("Alerta: Pacote do End Device 3 (Addr %d) recebido fora do slot esperado (Slot Atual: %d)", sender_address, actualslot);
                                    }
                                    log_i("Resposta do End Device 3 recebida no slot %d: %d", actualslot, rxpacket[0]);
                                    break;

                                default:
                                    log_w("Recebido pacote de um endereço desconhecido: %d", sender_address);
                                    break;
                        }
                        setindpolls();
                        lastActivityMillis = millis();
                        // nextstate = ST_STANDBY; que estado é esse??
                    }
                }else{
                    if (loramesh.receivePacket()) {
                        if (loramesh.lastpkt.fct == FCT_BEACON) {
                            node_init_sync(loramesh.lastpkt.timestamp);
                           //send_pct = 1;
                           lastActivityMillis = millis();

                           log_i("Rx.seqnum: %d",loramesh.lastpkt.seqnum);
                            
                           #if DISPLAY_ENABLE
                           char display_line4[20]; 
                           sprintf(display_line4,"Tx.seqnun: %d",loramesh.lastpkt.seqnum);
                           Heltec.DisplayShowAll(display_line1,display_line2,display_line4);
                           #endif

                        }
                        nextstate = ST_TXDATA; 
                    }
                }
                loramesh.startReceiving();
            }
            break;
        case ST_TXDATA: 
            if ((loramesh.mydd.devtype == DEV_TYPE_ENDDEV) && (actualslot == loramesh.mydd.dataslot)) {
                send_pct = 1;
                nextstate = ST_RXWAIT; 
            }
            loramesh.startReceiving();
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
                loramesh.sendPacketReq(lastscantime_ms);
                log_i("Tx.seqnum=%d",lastmyseqnum);
                lastActivityMillis = millis();
            } else {
                uint8_t *msg;
                uint8_t msg_size;
            
                if (loramesh.mydd.dataslot == END_DEVICE_1_SLOT) {
                    msg = msg_device1;
                    msg_size = sizeof(msg_device1);
                }else if (loramesh.mydd.dataslot == END_DEVICE_2_SLOT) {
                    msg = msg_device2;
                    msg_size = sizeof(msg_device2);
                }
                 else if(loramesh.mydd.dataslot == END_DEVICE_3_SLOT) {
                    msg = msg_device3;
                    msg_size = sizeof(msg_device3);
                }
            
                //envia o pacote para o router (dstaddress = 1)
                loramesh.sendPacketRes(1, msg[0]);
                lastActivityMillis = millis();
                Serial.println("Mensagem enviada");

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