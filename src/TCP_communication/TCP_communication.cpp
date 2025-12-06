#include <WiFi.h>
#include <lwip/sockets.h>
#include <WebSocketsClient.h>
#include <vars.h>
#include <ArduinoJson.h>

#define PinLED 25
#define PinPOT 2

int count = 0;

WebSocketsClient webSocket;

unsigned long lastReconnectAttempt = 0;
static unsigned long lastSend = 0;

void connectToWiFi();
void reconnect();
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
void SendMessage(String src, String dst, String fct, String param, String val);


void setup() {
  Serial.begin(115200);
  pinMode(PinLED, OUTPUT);
  connectToWiFi();

  // Conecta ao WebSocket Server
  webSocket.begin(server_ip, server_port, "/");
  webSocket.onEvent(webSocketEvent);

  Serial.println("Conectando ao servidor WebSocket...");
}


void loop() {
  webSocket.loop();
  reconnect();
}


/*------------------ Conexão WiFi ------------------*/
void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.println("Conectando ao WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConectado ao WiFi!");
  Serial.print("IP local: ");
  Serial.println(WiFi.localIP());
}


/*------------------ Reconexão WebSocket ------------------*/
void reconnect() {
  if (!webSocket.isConnected() && millis() - lastReconnectAttempt > 5000) {
    Serial.println("Tentando reconectar ao servidor WebSocket...");
    webSocket.begin(server_ip, server_port, "/");
    lastReconnectAttempt = millis();
  }
}


/*------------------ Evento de mensagem ------------------*/
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.println("Conectado ao servidor WebSocket!");

    // Assim que conectar, se identifica com o servidor
    StaticJsonDocument<128> identifyMsg;
    identifyMsg["src"] = "0x907F";
    identifyMsg["info"] = "ESP32 conectada";
    String json;
    serializeJson(identifyMsg, json);
    webSocket.sendTXT(json);
  }

  if (type == WStype_TEXT) {
    Serial.printf("Mensagem recebida: %s\n", payload);

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.println("Erro ao ler JSON recebido");
      return;
    }

    const char* src = doc["dst"];
    const char* dst = doc["src"];
    const char* fct = doc["fct"];
    const char* param = doc["param"];

    // Verifica se a mensagem é destinada a este dispositivo
    if (src && strcmp(src, "0x907F") == 0) {
      Serial.println("Mensagem destinada a mim!");

      /*-------------------------------------------- LEITURA --------------------------------------------*/
      if (fct && strcmp(fct, "read") == 0) {

        /* ---------------------- Verifica se é no Pot ----------------------*/
        if(param && strcmp(param, "1") == 0) {
          std::string valorStr = std::to_string(count);
          SendMessage(src, dst, fct, param, valorStr.c_str());
        }

        /* ---------------------- Verifica se é no LED ----------------------*/  
        else if(param && strcmp(param, "2") == 0) {
          SendMessage(src, dst, fct, param, (digitalRead(PinLED) == HIGH ? "true" : "false"));
        }
      }



      /*-------------------------------------------- ESCRITA --------------------------------------------*/
      else if (fct && strcmp(fct, "write") == 0) {
        const char* val = doc["val"];

        /* ---------------------- Verifica se é no Pot ----------------------*/
        if(param && strcmp(param, "1") == 0) {
          if (val && strcmp(val, "true") == 0) {
            digitalWrite(PinLED, HIGH);
          } 
          
          else {
            digitalWrite(PinLED, LOW);
          }

          SendMessage(src, dst, fct, param, (digitalRead(PinLED) == HIGH ? "true" : "false")); // Envia confirmação do novo estado
        }

        /* ---------------------- Verifica se é no LED ----------------------*/  
        if(param && strcmp(param, "2") == 0 ) {
          
          if (val && strcmp(val, "true") == 0) {
            digitalWrite(PinLED, HIGH);
            count++;
          } 
          
          else {
            digitalWrite(PinLED, LOW);
          }

          SendMessage(src, dst, fct, param, (digitalRead(PinLED) == HIGH ? "true" : "false")); // Envia confirmação do novo estado
        }
        
      }
    }
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
