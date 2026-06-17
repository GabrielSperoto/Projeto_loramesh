
#include <WiFi.h>
#include <lwip/sockets.h>
#include <WebSocketsClient.h>
#include <vars.h>
#include <ArduinoJson.h>
#include "main.h"
#include "Lora/loramesh.h"


int count = 0;

WebSocketsClient webSocket;
msg_t msg;

extern class LoRaClass loramesh;


unsigned long lastReconnectAttempt = 0;
static unsigned long lastSend = 0;

void wifiTick();
void connectToWiFi();
void reconnect();
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
void SendMessage(String src, String dst, String fct, String param, String val);



void init_TCP_comm() {
  connectToWiFi();
  // Conecta ao WebSocket Server
  webSocket.begin(server_ip, server_port, "/");
  webSocket.onEvent(webSocketEvent);

  log_i("Conectando ao servidor WebSocket...");
}


void TCP_communicationTask(void* pvParameters){

  uint8_t tentativasFalhas = 0;
  uint32_t tempoDeEsperaMs = 1000; // Começa esperando 1 segundo
  const uint32_t TEMPO_MAXIMO_MS = 30000; // Teto de 30 segundos

  init_TCP_comm();
  while(true){
    
    webSocket.loop();
    reconnect();

    if(WiFi.status() != WL_CONNECTED){
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    if (!webSocket.isConnected()) {
        tentativasFalhas++;
        log_i("Tentando reconectar ao WebSocket. Tentativa: %d", tentativasFalhas);
        
        // Tenta reconectar
        webSocket.begin(server_ip, server_port, "/");

        // Se falhou 3 vezes, começa a aumentar o tempo de espera
        if (tentativasFalhas >= 3) {
            tempoDeEsperaMs *= 2; // Dobra o tempo de espera (2s, 4s, 8s...)
            if (tempoDeEsperaMs > TEMPO_MAXIMO_MS) {
                tempoDeEsperaMs = TEMPO_MAXIMO_MS; // Trava no máximo de 30s
            }
            log_i("Muitas falhas. Aumentando o intervalo para %d ms", tempoDeEsperaMs);
        }

        // Dorme pelo tempo estipulado antes de tentar de novo
        vTaskDelay(tempoDeEsperaMs / portTICK_PERIOD_MS);
        continue;
    }

    // Reseta os contadores para o estado inicial, pois o link está saudável
    if (tentativasFalhas > 0) {
        log_i("Conexão reestabelecida! Resetando contadores de backoff.");
        tentativasFalhas = 0;
        tempoDeEsperaMs = 1000; 
    }


    if(xQueueReceive(q_app2tcp,&msg, 10/portTICK_PERIOD_MS) == pdTRUE){
      //descompacta a mensgaem recebida e envia para app
      String src = String(msg.src);
      String dst = String(msg.dst);
      String fct = String(msg.function);
      String param = String(msg.start);
      String val = String(msg.payload.value);

      SendMessage(src, dst, fct, param, val);
      
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


/*------------------ Conexão WiFi ------------------*/
void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.println("Conectando ao WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    uint32_t lastime = millis();
    //delay de 500ms
    vTaskDelay(500/portTICK_PERIOD_MS);
  }

  log_i("Conectado ao Wifi!");
  log_i("IP local: %s", WiFi.localIP().toString().c_str());
}


/*------------------ Reconexão WebSocket ------------------*/
void reconnect() {
  if (!webSocket.isConnected() && millis() - lastReconnectAttempt > 5000) {
    log_i("Tentando reconectar ao servidor WebSocket...");
    webSocket.begin(server_ip, server_port, "/");
    lastReconnectAttempt = millis();

  }

}


/*------------------ Evento de mensagem ------------------*/
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    log_i("Conectado ao servidor WebSocket!");

    // Assim que conectar, se identifica com o servidor
    StaticJsonDocument<128> identifyMsg;
    identifyMsg["src"] = std::to_string(loramesh.mydd.devserialnumber);
    identifyMsg["type"] = "login_router";
    identifyMsg["nodes"] = loramesh.getNodes().nodes;
    String json;
    serializeJson(identifyMsg, json);
    webSocket.sendTXT(json);
  }

  if (type == WStype_TEXT) {
    log_i("Mensagem recebida: %s\n", payload);

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      log_i("Erro ao ler JSON recebido");
      return;
    }

    const char* src = doc["dst"];
    const char* dst = doc["src"];
    const char* fct = doc["fct"]; //associar a função (read,writting) a um codigo numérico
    const char* val = doc["val"];
    const char* param = doc["param"];

    //converte para byte
    uint8_t src_addr = strtol(src, nullptr, 10);
    uint8_t dst_addr = strtol(dst, nullptr, 10);
    uint8_t function_num = strtol(fct, nullptr, 10);
    uint8_t val_num = strtol(val, nullptr, 10);
    uint8_t param_num = strtol(param, nullptr, 10);


    msg.src = src_addr;
    msg.dst = dst_addr;
    msg.function = function_num;
    msg.start = param_num;
    msg.qtdParametros = 1; //falta à aplicação definir a qtd de parametros
    msg.payload.value = val_num;
    msg.size = sizeof(val_num);


    // xQueueSend(rxQueue, &rxMsg, 0); a mensagem é enviada pela fila q_tcp_tx
    xQueueSend(q_tcp2app,&msg,0);
  
    log_i("Queue enviada. Src: %d, Dst: %d, Fct: %d, Param: %d, Val: %d", src_addr, dst_addr, function_num, param_num, msg.payload.value);

    
  }
}


/*----------------------- Envio de resposta -----------------------*/
void SendMessage(String src, String dst, String fct, String param, String val) {
  if (webSocket.isConnected()) {
    StaticJsonDocument<200> doc;
    doc["dst"] = dst;
    doc["src"] = src;
    doc["fct"] = fct;
    doc["param"] = param;
    doc["val"] = val;

    String jsonString;
    serializeJson(doc, jsonString);
    webSocket.sendTXT(jsonString);

    Serial.print("Mensagem enviada: ");
    Serial.println(jsonString);

    lastSend = millis();
  }
}
