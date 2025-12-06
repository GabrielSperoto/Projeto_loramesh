#include <WiFi.h>
#include <lwip/sockets.h>
#include <WebSocketsClient.h>
#include <vars.h>
#include <ArduinoJson.h>
#include "main.h"
#include "Lora/loramesh.h"


#define PinLED 25
#define PinPOT 2

int count = 0;

WebSocketsClient webSocket;
TxMessage_t txMsg;
RxMessage_t rxMsg;

extern class LoRaClass loramesh;


unsigned long lastReconnectAttempt = 0;
static unsigned long lastSend = 0;

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
  while(true){
    webSocket.loop();
    reconnect();
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
    while(millis() - lastime < 500){}
    // delay(500);
    log_i(".");
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
    identifyMsg["info"] = "ESP32 conectada";
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

    uint8_t src_addr = strtol(src, nullptr, 10);
    uint8_t dst_addr = strtol(dst, nullptr, 10);
    uint8_t function_num = strtol(fct, nullptr, 10);
    uint8_t val_num = strtol(val, nullptr, 10);
    uint8_t param_num = strtol(param, nullptr, 10);

    txMsg.dst = dst_addr;
    txMsg.function = function_num;
    txMsg.value = val_num;
    txMsg.start = param_num;

    xQueueSend(txQueue, &txMsg, 0);

    //a mensagem é agora enviada pela rede

    // Verifica se a mensagem é destinada a este dispositivo
    // if (src && strcmp(src, "0x907F") == 0) {
    //   Serial.println("Mensagem destinada a mim!");

    //   /*-------------------------------------------- LEITURA --------------------------------------------*/
    //   if (fct && strcmp(fct, "read") == 0) {

    //     /* ---------------------- Verifica se é no Pot ----------------------*/
    //     if(param && strcmp(param, "1") == 0) {
    //       std::string valorStr = std::to_string(count);
    //       SendMessage(src, dst, fct, param, valorStr.c_str());
    //     }

    //     /* ---------------------- Verifica se é no LED ----------------------*/  
    //     else if(param && strcmp(param, "2") == 0) {
    //       SendMessage(src, dst, fct, param, (digitalRead(PinLED) == HIGH ? "true" : "false"));
    //     }
    //   }



    //   /*-------------------------------------------- ESCRITA --------------------------------------------*/
    //   else if (fct && strcmp(fct, "write") == 0) {
    //     const char* val = doc["val"];

    //     /* ---------------------- Verifica se é no Pot ----------------------*/
    //     if(param && strcmp(param, "1") == 0) {
    //       if (val && strcmp(val, "true") == 0) {
    //         digitalWrite(PinLED, HIGH);
    //       } 
          
    //       else {
    //         digitalWrite(PinLED, LOW);
    //       }

    //       SendMessage(src, dst, fct, param, (digitalRead(PinLED) == HIGH ? "true" : "false")); // Envia confirmação do novo estado
    //     }

    //     /* ---------------------- Verifica se é no LED ----------------------*/  
    //     if(param && strcmp(param, "2") == 0 ) {
          
    //       if (val && strcmp(val, "true") == 0) {
    //         digitalWrite(PinLED, HIGH);
    //         count++;
    //       } 
          
    //       else {
    //         digitalWrite(PinLED, LOW);
    //       }

    //       SendMessage(src, dst, fct, param, (digitalRead(PinLED) == HIGH ? "true" : "false")); // Envia confirmação do novo estado
    //     }
        
    //   }
    // }
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
