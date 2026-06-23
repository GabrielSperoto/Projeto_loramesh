
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

void connectToWiFi();
void reconnect();
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
void SendMessage(String src, String dst, String fct, String param, String val);
void sendRouterID();
void receiveTextData(uint8_t * payload);



void init_TCP_comm() {
  connectToWiFi();
  // Conecta ao WebSocket Server
  webSocket.begin(server_ip, server_port, "/");
  webSocket.onEvent(webSocketEvent);

  log_i("Conectando ao servidor WebSocket...");
}


void TCP_communicationTask(void* pvParameters){

  init_TCP_comm();
  while(true){
    
    webSocket.loop();
  
    if(WiFi.status() != WL_CONNECTED){
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    if(WiFi.status() == WL_CONNECTED && webSocket.isConnected()){

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


/*------------------ Evento de mensagem ------------------*/
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
		case WStype_DISCONNECTED:
      reconnect();
    break;
		case WStype_CONNECTED: 
      sendRouterID();
    break;
		case WStype_TEXT:
      receiveTextData(payload);
    break;
		case WStype_BIN:
    break;
		case WStype_ERROR:			
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
    break;
	}
  
}

void reconnect() {
  if (!webSocket.isConnected() && millis() - lastReconnectAttempt > 5000) {
    log_i("Tentando reconectar ao servidor WebSocket...");
    webSocket.begin(server_ip, server_port, "/");
    vTaskDelay(2000/portTICK_PERIOD_MS);
    lastReconnectAttempt = millis();

  }

}

void sendRouterID(){
  log_i("Conectado ao servidor WebSocket!");
  
  StaticJsonDocument<512> identifyMsg;
    identifyMsg["type"] = "login_router";
    identifyMsg["src"] = String(loramesh.mydd.devserialnumber); 
    
    JsonArray nodesArray = identifyMsg.createNestedArray("nodes");
    strDevicedescription buffer[BUFFER_SIZE];

    uint8_t numNodes = loramesh.getNodes(buffer, BUFFER_SIZE);
    for(uint8_t i =0; i < numNodes; i++){
      nodesArray.add(buffer[i].devaddr);
    }

    String json;
    serializeJson(identifyMsg, json);
    
    // Opcional: Imprime no Serial para garantir que o JSON foi montado corretamente
    Serial.println("Enviando login: " + json);
    
    webSocket.sendTXT(json);
}


void receiveTextData(uint8_t * payload){
  // log_i("Mensagem recebida: %s\n", payload);

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      log_i("Erro ao ler JSON recebido");
      return;
    }

    const char* dst = doc["dst"];
    const char* fct = doc["fct"]; //associar a função (read,writting) a um codigo numérico
    const char* val = doc["val"];
    const char* param = doc["param"];

    //converte para byte
    uint8_t dst_addr = strtol(dst, nullptr, 10);
    uint8_t function_num = strtol(fct, nullptr, 10);
    uint8_t val_num = strtol(val, nullptr, 10);
    uint8_t param_num = strtol(param, nullptr, 10);


    msg.src = loramesh.mydd.devaddr;
    msg.dst = dst_addr;
    msg.function = function_num;
    msg.start = param_num;
    msg.qtdParametros = 1; //falta à aplicação definir a qtd de parametros
    msg.payload.value = val_num;
    msg.size = sizeof(val_num);


    // xQueueSend(rxQueue, &rxMsg, 0); a mensagem é enviada pela fila q_tcp_tx
    if(xQueueSend(q_tcp2app,&msg,0) == pdTRUE) 
      log_i("Queue enviada. Src: %d, Dst: %d, Fct: %d, Param: %d, Val: %d", msg.src, dst_addr, function_num, param_num, msg.payload.value);
  
    
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
