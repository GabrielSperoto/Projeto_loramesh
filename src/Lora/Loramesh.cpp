#include "loramesh.h"

#if defined ( WIFI_LoRa_32_V3 )
#include <modules/sx126x/sx1262.h>

SX1262  radio = new Module(SS,DIO0,RST_LoRa,BUSY_LoRa);

#endif

#if defined ( WIFI_LoRa_32_V2 )
#include <modules/SX127x/SX1276.h>

SX1276 radio = new Module(SS, DIO0, RST_LoRa, DIO1);
#endif

LoRaClass loramesh;

// flag to indicate that a preamble was detected
volatile bool detectedFlag = false;
// flag to indicate that a preamble was not detected
volatile bool timeoutFlag = false;

volatile bool rxFlag = false;
//char Readback[50];
char frame[50];
bool newvalue=0;
int packetSize = 0;

// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate transmission or reception state
bool transmitFlag = false;

// flag to indicate that a packet was sent or received
volatile bool operationDone = false;

//table of node devices...
//{DeviceID, DEV_TYPE, DeviceAddress, dataslot}
#if defined ( WIFI_LoRa_32_V2 )
strDevicedescription devid[]={
   {0xACFD,DEV_TYPE_ROUTER,1,0},
   {0xF482,DEV_TYPE_ENDDEV,2,2},
   {0xCC7F,DEV_TYPE_ENDDEV,3,3},
   {0X8096,DEV_TYPE_ENDDEV,4,4},
};
#else  //WIFI_LoRa_32_V3
strDevicedescription devid[]={
   {0x9C87,DEV_TYPE_ROUTER,1,0},
   {0xDC78,DEV_TYPE_ENDDEV,2,2},
};
#endif

// ===========================
LoRaClass::LoRaClass() :
  _spiSettings(8E6, MSBFIRST, SPI_MODE0),
  _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
  _frequency(0),
  _packetIndex(0),
  _implicitHeaderMode(0),
  _onReceive(NULL)
{
  // overide Stream timeout value
  setTimeout(0);
}

void setFlag(void) {
  // we sent or received  packet, set the flag
  operationDone = true;
}

#if ENABLE_RX_INTERRUPT
// Can't do Serial or display things here, takes too much time for the interrupt
void rx() {
  rxFlag = true;
}
#endif
void LoRaClass::VextON(void)
{
	pinMode(Vext,OUTPUT);
	digitalWrite(Vext, LOW);
}

void LoRaClass::VextOFF(void) //Vext default OFF
{
	pinMode(Vext,OUTPUT);
	digitalWrite(Vext, HIGH);
}

int16_t LoRaClass::standby(){
  return (radio.standby());
}

void setFlagTimeout(void) {
  // we timed out, set the flag
  timeoutFlag = true;
}

void setFlagDetected(void) {
  // we got a preamble, set the flag
  detectedFlag = true;
}

// ISR for handling LoRa reception interrupt
volatile bool messageReceived = false;

void LoRaClass::ClearRadioIRQs(){

  writeRegister(RADIOLIB_SX127X_REG_IRQ_FLAGS, 0xFF); // clear all IRQs
  writeRegister(RADIOLIB_SX127X_REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);  
} 

void onReceiveInterrupt() {
  messageReceived = true;
}

int LoRaClass::begin() 
{
  float freq = LORA_FREQUENCY; 
  float bw = LORA_BW; 
  uint8_t sf = LORA_SF; 
  uint8_t cr = LORA_CR; 
  uint8_t power = LORA_TRANSMIT_POWER; 
  uint16_t preambleLength = 12; 
  uint8_t gain = LORA_GAIN;
  uint8_t ret=0;
  uint8_t syncWord=0;
  
  #if defined( WIFI_LoRa_32_V3 ) 
    syncWord = RADIOLIB_SX126X_SYNC_WORD_PRIVATE; 

    SPI.begin(SCK,MISO,MOSI,SS);

    int state = radio.begin(freq,bw,sf,cr,syncWord,power, preambleLength, 1.6, gain);

  if (state == RADIOLIB_ERR_NONE) {
    log_i("Radio begin success! Freq=%4.2f Bw=%4.2f sf=%d cr=%d power=%d",freq, bw, sf, cr, power);
    
  } else {
    log_v("failed, code =%d",state);
    while (true) { delay(10); }
  }

    radio.setFrequency(freq);
    //radio.setDataRate(LORA_DATARATE);
    radio.setBandwidth(bw);
    radio.setSpreadingFactor(sf);
    radio.setOutputPower(power);

  //get the device type and device address based in the chip id
  ret = getdevicedescription(); 
  
  #if ENABLE_RX_INTERRUPT
  // Set the callback function for received packets
  radio.setDio1Action(onReceiveInterrupt);
  
  // este eh o continuos mode
  //state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);   
  // este eh o single mode
  state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);   
  #endif
  
  if (state == RADIOLIB_ERR_NONE) {
    log_i("Starting receiving!");
  } 
  else {
    log_e("Radio starting failed, code %d",state);
    while (true) { delay(100); }
  }



#else //( WIFI_LoRa_32_V2 ) 
  syncWord = RADIOLIB_SX127X_SYNC_WORD; 

  VextON();

  // Initialize the radio
  setPins(SS,RST_LoRa,DIO0);

  int state = radio.begin(freq,bw,sf,cr,syncWord, power, preambleLength, gain);

  //parece que aqui ele seta diferente do radio lib   
  //setTxPowerMax(20);

  if (state == RADIOLIB_ERR_NONE) {
    log_i("Radio begin success! Freq=%4.2f Bw=%4.2f sf=%d cr=%d power=%d gain=%d",freq, bw, sf, cr, power, gain);
    
  } else {
    log_v("failed, code =%d",state);
    while (true) { delay(10); }
  }

  // Attach the interrupt to DIO0 pin
  pinMode(DIO0, INPUT);
  attachInterrupt(digitalPinToInterrupt(DIO0), onReceiveInterrupt, RISING);

  
  //get the device type and device address based in the chip id
  ret = getdevicedescription();

  if ((ret) && (mydd.devtype == DEV_TYPE_ENDDEV)) {
    // start listening for LoRa packets on this node
    radio.setDio0Action(onReceiveInterrupt, RISING);

    uint32_t rx_timeout = 2000;
    int res = radio.startReceive(rx_timeout);
    state = radio.startReceive();

    if (state == RADIOLIB_ERR_NONE) {
      log_i("Starting receiving!");
    } 
    else {
      log_e("Radio starting failed, code %d",state);
      while (true) { delay(100); }
    }

  } 
#endif

  return 1;
}

void LoRaClass::end()
{
  // put in sleep mode
  sleep();
  // stop SPI
  SPI.end();
}

void LoRaClass::onReceive(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
}

//TODO: Retry start receiving if it fails
void LoRaClass::clearDioActions () {
  #if defined( WIFI_LoRa_32_V2 )   
    radio.clearDio0Action();
  #else  
    radio.clearDio1Action();

  #endif  
}

void LoRaClass::setDioActionsForReceivePacket() {
  clearDioActions();

  #if defined( WIFI_LoRa_32_V2 ) 
    radio.setDio0Action(onReceiveInterrupt, RISING);
  #else
    radio.setDio1Action(onReceiveInterrupt);
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);    
  #endif

}

void LoRaClass::restartRadio() {
    radio.reset();
    //initializeLoRa();
    log_e("Restarting radio DONE");
}

int LoRaClass::startReceiving(uint32_t timeout) {
    setDioActionsForReceivePacket();

    int res = radio.startReceive(timeout);
    if (res != RADIOLIB_ERR_NONE) {
        if (res == RADIOLIB_ERR_RX_TIMEOUT){
          log_e("Starting receiving timeout");
        }
        else{
          log_e("Starting receiving gave error: %d", res);
          restartRadio();
        }

    }
    return res;
}

#if defined( WIFI_LoRa_32_V2 )  
int LoRaClass::beginPacket(int implicitHeader)
{
  // put in standby mode
  idle();
  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }
  // reset FIFO address and paload length
  writeRegister(RADIOLIB_SX127X_REG_FIFO_ADDR_PTR, 0);
  writeRegister(RADIOLIB_SX127X_REG_PAYLOAD_LENGTH, 0);
  
  return 1;
}

int LoRaClass::endPacket(bool async)
{
  // put in TX mode
  writeRegister(RADIOLIB_SX127X_REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  if (async) {
    // grace time is required for the radio
    delayMicroseconds(150);
  } else {
    // wait for TX done
    //V3 -  
#if defined( WIFI_LoRa_32_V3 )    
    while ((readRegister(RADIOLIB_SX127X_REG_IRQ_FLAGS) & RADIOLIB_SX126X_IRQ_TX_DONE) == 0) {
      log_i("write3");
      yield();
    }
    // clear IRQ's
    log_i("write4");
    writeRegister(RADIOLIB_SX127X_REG_IRQ_FLAGS, RADIOLIB_SX126X_IRQ_TX_DONE);
#else
    while ((readRegister(RADIOLIB_SX127X_REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
      yield();
    }
    // clear IRQ's
    writeRegister(RADIOLIB_SX127X_REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
#endif

  }

  return 1;
}

#endif




int LoRaClass::parsePacket(int size)
{
  char *pframe=&frame[0];
  bool rxCRCOn=0;
  int packetLength = 0;

#if defined(WIFI_LoRa_32_V2)
    int irqFlags = readRegister(RADIOLIB_SX127X_REG_IRQ_FLAGS);

    // when use CRC enable
    // CrcOnPayload (bit 6 on RegHopChannel) 
    // CRC Information extracted from the received packet header (Explicit header mode only)
    // 0 Header indicates CRC off
    // 1 Header indicates CRC on
    int RegHopChannel = readRegister(RADIOLIB_SX127X_REG_HOP_CHANNEL);

    if (size > 0) {
      implicitHeaderMode();
      writeRegister(RADIOLIB_SX127X_REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
      explicitHeaderMode();

  #if defined(WIFI_LoRa_32_V2)
      if (RegHopChannel & CRC_ON_PAYLOAD)
        rxCRCOn = 1;
  #endif       
    }

    // clear IRQ's
    writeRegister(RADIOLIB_SX127X_REG_IRQ_FLAGS, irqFlags);
  
   if ((irqFlags & IRQ_RX_DONE_MASK) && (rxCRCOn) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;
    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(RADIOLIB_SX127X_REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(RADIOLIB_SX127X_REG_RX_NB_BYTES);
    }
    //log_i("irqflags1 =%2x packetsize=%d",irqFlags,packetLength);
    // set FIFO address to current RX address
    writeRegister(RADIOLIB_SX127X_REG_FIFO_ADDR_PTR, readRegister(RADIOLIB_SX127X_REG_FIFO_RX_CURRENT_ADDR));
    // put in standby mode
    idle();
  }
  else if (readRegister(RADIOLIB_SX127X_REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode
    // reset FIFO address
    writeRegister(RADIOLIB_SX127X_REG_FIFO_ADDR_PTR, 0);
    // put in single RX mode
    writeRegister(RADIOLIB_SX127X_REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

#endif

  return packetLength;
}



bool LoRaClass::getdevicedescription(){
   uint8_t ret;
   uint8_t mac[6];
   esp_chip_info_t chip_info;
   strDevicedescription *pdd = devid;

#if 0
  esp_read_mac(mac,ESP_MAC_WIFI_STA);
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) {
      Serial.print(":");
    }
  }

  // Get Chip Info
  esp_chip_info(&chip_info);

//  Serial.printf("Cores: %d\n", chip_info.cores);
//  Serial.printf("Chip Revision: %d\n", chip_info.revision);
#endif

  // Get the Unique Chip ID (Serial Number)
  uint64_t chipId = ESP.getEfuseMac(); // 48-bit unique identifier
  uint32_t chipidu = (uint32_t)(chipId >> 32);
  uint32_t chipidl = (uint32_t)(chipId);

  Serial.print("Hardware Serial Number (Chip ID): ");
  Serial.println(chipidu, HEX); // Upper 32 bits

  //Find the Device Address and device type
  for (int i = 0; i < sizeof(devid) / sizeof(devid[0]); i++)
  {
    if (pdd->devserialnumber == chipidu) {
        log_i("Device Description");
        log_i("ChipID = %4x DevAddress=%d",pdd->devserialnumber, pdd->devaddr);
        
        memcpy(&mydd,pdd,sizeof(mydd));

        if (pdd->devtype == DEV_TYPE_ROUTER)
            log_i("Device is a Router");
        else
            log_i("Device is a End Device");
        ret = 1;
        break;
    }
    pdd++;   
  }

  if (ret == 0){
    log_e("nao encontrei device description do device=%4x !!!!",chipidu);

  }

  //initialize invoke id 
   mydd.seqnum = 0;

   return ret; 
}

uint8_t LoRaClass::getrouteaddr(){
  
  uint8_t rtaddr=0;
  strDevicedescription *pdd = devid;

  for (int i=0;i<sizeof(pdd);i++)
  {
    if (pdd->devtype == DEV_TYPE_ROUTER) {
        rtaddr = pdd->devaddr;
        break;
    }
    pdd++;   
  }

   return rtaddr; 
}


Node_t LoRaClass::getNodes() {
    Node_t nodes;
    strDevicedescription* pdd = devid;
    
    for(int i=0; i<sizeof(devid)/sizeof(devid[0]); i++) {
        nodes.nodes[i] = pdd->devaddr;
        pdd++;
    }
    return nodes;
}

void LoRaClass::clearBuffer(uint8_t *buffer, int size)
{
    for (int i = 0; i < size; i++)
    {
        buffer[i] = '\0';
    }
}

uint16_t LoRaClass::getseqnum(uint8_t *packet,uint8_t len){
    if (len > 4){
        return ((packet[2] << 8) | packet[3]);
    }
    else
        return 0;

}


// uint32_t LoRaClass::gettimestamp(uint8_t *packet,uint8_t len){
//     uint32_t timestamp;
//     uint8_t *pucaux = (uint8_t *) &timestamp;

//     if (len > 5){
//         *pucaux++ = packet[8];
//         *pucaux++ = packet[7];
//         *pucaux++ = packet[6];
//         *pucaux = packet[5];
//         //log_i ("timestamp=%4x",timestamp);
//         return timestamp;
//     }
//     else
//         return 0;

// }


//Todo!!! implementar um CRC
//checa somente o ultimo byte do frame é igual ao definido
uint16_t LoRaClass::calculate_crc (uint8_t *packet, uint8_t len){
    uint16_t crc = 0xFFFF; // Valor inicial do CRC

    for (uint8_t i = 0; i < len; i++) {
        crc ^= (packet[i] << 8); // XOR do byte atual com o CRC

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) { // Se o bit mais significativo for 1
                crc = (crc << 1) ^ POLYNOMIAL_CRC; // Desloca e XOR com o polinômio
            } else {
                crc <<= 1; // Apenas desloca
            }
        }
    }

    return crc;
}

uint16_t LoRaClass::getLastSeqNum(){

    return (mydd.seqnum);
}

uint16_t LoRaClass::getLastPctSeqNum(){

    return ((lastpkt.payload[2] << 8) | lastpkt.payload[3]);
}

// uint8_t LoRaClass::getResponseStatus(){
//   uint8_t* rxPacket = lastpkt.payload;
//   uint8_t size = lastpkt.packetSize;
//   if(size > 5) return rxPacket[5];
//   return -1;
// }

uint32_t LoRaClass::getPayloadValue(uint8_t *packet, uint8_t len) {
  if (len > 0) {
    uint32_t value = 0;
    uint8_t* buffer = &packet[6];
    // log_i("getPayloadValue: len=%d, buffer[0..%d]=%02x %02x %02x %02x", len, len-1, buffer[0], buffer[1], buffer[2], buffer[3]);

    // Monta o valor dependendo de quantos bytes chegaram
    switch (len) {
        case 1:
            value = buffer[0];
            break;
        case 2:
            value = (buffer[0] << 8) | buffer[1];
            break;
        case 3:
            value = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
            break;
        case 4:
          value = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
        default: // Se vier mais de 4, ignora o excesso e pega os primeiros 4 bytes
            break;
    }

    return value;
  }
  
  log_i("getPayloadValue: len=%d, não há payload para extrair", len);
  return (uint32_t)-1; // Erro
}

uint8_t LoRaClass::getWrittingCode(){
  uint8_t* rxPacket = lastpkt.payload;
  uint8_t size = lastpkt.packetSize;
  if(size > 5 && rxPacket[4] == FCT_WRITTING) return rxPacket[8];
  return -1;
}


uint8_t LoRaClass::getSizeMsg(){
  return lastpkt.packetSize;
}




//estrutura do pacote de requisição [dst,src,seq number, fct, start, qtd parametros, crc]

/* Formato de um frame de escrita
   {Destination address,Source address, sequence number, function code, start, qtd. parametros,value,crc}
*/


void LoRaClass::decodeLoraPacket(msg_t *msg){
  uint8_t *rxPacket = lastpkt.payload;
  uint8_t size = lastpkt.packetSize;

  log_i("lastpkt.payload[6]: %02x", lastpkt.payload[6]);
  msg->dst = rxPacket[0];
  msg->src = rxPacket[1];
  msg->seqnum = (rxPacket[2] << 8) | rxPacket[3];
  msg->function = rxPacket[4];

  
  // msg.size = rxPacket[7];

  log_i("seqnum: %d",msg->seqnum);

  if(mydd.devtype == DEV_TYPE_ROUTER){
    //pacote de respsota não possui nem o campo start nem o qtd Parametros
    msg->size = rxPacket[5];
    // log_i("size: %d",msg->size);
    msg->payload.value = getPayloadValue(rxPacket,msg->size);
    
  }
  else{ //end device 
    switch(msg->function)
        {
            case FCT_BEACON:
                //aqui o end device recebe o beacon do router, entao ele pode atualizar a tabela de rotas e enviar mensagens para o router
                //log_i("Received BEACON from device %d", getSrcAdress());
                // msg.size = 4;
                msg->payload.value = getPayloadValue(rxPacket,msg->size);
                break;

            case FCT_READING:
                //aqui o router nao enviar nenhum valor para o payload, por isso ele fica vazio
                msg->start = rxPacket[5];
                msg->qtdParametros = rxPacket[6];
                msg->size = 0;
                msg->payload.value = 0;
                break;

            case FCT_WRITTING:
                //aqui o end device recebe a requisição de escrita do router, entao ele deve enviar a resposta com o status da escrita
                //log_i("Received WRITTING REQUEST from device %d", getSrcAdress());
                // msg.size = 1;
                msg->start = rxPacket[5];
                msg->qtdParametros = rxPacket[6];
                msg->size = rxPacket[7];
                msg->payload.value = getWrittingCode();
                log_i("Writting code: %d", msg->payload.value);
                break;
            case FCT_DESCRIPTION:
                //aqui o end device recebe a requisição de descrição do router, entao ele deve enviar a resposta com a descrição do dispositivo
                //log_i("Received DESCRIPTION REQUEST from device %d", getSrcAdress());
                break;
            default:
                log_w("Funcao nao suportada: %d", msg->function);
        }
  }

}

uint8_t LoRaClass::encodeAndSendPacket(msg_t* message) {
    uint8_t buffer[BUFFER_SIZE];
    uint8_t pos = 0;
    
    // 1. Cabeçalho padrão para todas as mensagens
    buffer[pos++] = message->dst;     // dst
    buffer[pos++] = mydd.devaddr;     // src
    
    // 2. Tratamento do Sequence Number
    // Se for requisição/beacon, incrementa o próprio seqnum. Se for resposta, usa o seqnum do pacote recebido
    uint16_t current_seq;
    if (mydd.devtype == DEV_TYPE_ROUTER && (message->function == FCT_BEACON)) { 
      current_seq = mydd.seqnum++; 
    } else {
      current_seq = message->seqnum; // Assumindo que é uma resposta
    }



    buffer[pos++] = (current_seq >> 8) & 0xFF; // MSB
    buffer[pos++] = current_seq & 0xFF;        // LSB
    
    buffer[pos++] = message->function;// fct
    // 3. Serialização do Payload baseada na Função
    switch (message->function) {
        case FCT_BEACON:
            // Serializa o timestamp no payload se necessário
              buffer[pos++] = message->size; // tamanho do payload (timestamp tem 4 bytes)
              // uint8_t* pucaux = (uint8_t*) &message->payload.value;
              buffer[pos++] = message->payload.bytes[3]; // MSB
              buffer[pos++] = message->payload.bytes[2];
              buffer[pos++] = message->payload.bytes[1];
              buffer[pos++] = message->payload.bytes[0]; // LSB
              log_i("Encoded BEACON payload: %02X %02X %02X %02X", message->payload.bytes[3], message->payload.bytes[2], message->payload.bytes[1], message->payload.bytes[0]);
            break;
            
        case FCT_WRITTING:
            // Diferencia se é requisição ou resposta pelo tamanho esperado ou origem
            if (mydd.devtype == DEV_TYPE_ROUTER) { // Requisição
                buffer[pos++] = message->start;
                buffer[pos++] = message->qtdParametros;
                buffer[pos++] = message->size; //coloquei o tamanho do payload so por convencão, ja que ele e sempre igual a 1 byte
                buffer[pos++] = message->payload.value & 0xFF; // valor
            } else { // Resposta do End Device
                buffer[pos++] = message->size;
                buffer[pos++] = message->payload.value & 0xFF; // status
            }
            break;
            
        case FCT_READING:
            if (mydd.devtype == DEV_TYPE_ROUTER) { // Requisição
                // Nenhum dado extra ou apenas start/qtdParametros
                buffer[pos++] = message->start;
                buffer[pos++] = message->qtdParametros;
            } else { // Resposta
                buffer[pos++] = message->size;
                for (int i = 0; i < message->size; i++) {
                    buffer[pos++] = message->payload.bytes[i];
                }
            }
            break;
    }

    // 4. Fechamento do frame
    uint16_t crc = calculate_crc(buffer, pos);
    buffer[pos++] = (crc >> 8) & 0xFF; // MSB
    buffer[pos++] = crc & 0xFF;        // LSB

    // log_i("crc: %02X %02X", buffer[pos-2], buffer[pos-1]);

    // 5. Envio físico
    if (sendPacket(buffer, pos)) {
        log_i("Pacote enviado com sucesso, tamanho: %d", pos);
        log_i("Buffer: %02X %02X %02X %02X %02X ...", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
        // Retorna a rádio para modo de recepção, se necessário
        return pos;
    }
    return 0;
}

/* Formato de uma resposta de escrita
  {src,dst,fct,seq number, status,crc}
*/




bool LoRaClass::receivePacket()
{

    // formato do pacote recebido [src, dst, fct, seq number, size, payload (data), crc]
    // log_i("Checking for received packets...");
    bool retcrc=0;
    uint8_t ret=0;
    uint8_t srcadress;
    uint8_t dstadress;
    uint8_t fct;
    uint16_t seqnum;
    int len = 0;


#if defined(WIFI_LoRa_32_V3)
  uint8_t offset = 0;
  int16_t state = 0;

  // get packet length and Rx buffer offset
  packetSize = radio.getPacketLength(true, &offset);
  if (packetSize) {
      state = radio.readData(lastpkt.payload,packetSize); 
      RADIOLIB_ASSERT(state);

#else // V2
    lastpkt.packetSize = loramesh.parsePacket(0);
    if (lastpkt.packetSize) {
        while (loramesh.available() && len < BUFFER_SIZE - 1) {
            lastpkt.payload[len++] = (char)loramesh.read(); // Lê o pacote byte a byte
        }
#endif

        // verifica o srcaddress e dstaddress do pacote
        // log_i("lastpkt.payload[6]: %02x", lastpkt.payload[6]);
        dstadress = lastpkt.payload[0];
        srcadress = lastpkt.payload[1];
        fct = lastpkt.payload[4];
        //big endian
        seqnum = getseqnum((uint8_t *)lastpkt.payload,lastpkt.packetSize);
        if(mydd.devtype == DEV_TYPE_ENDDEV)
          mydd.seqnum = seqnum; //atualiza o seqnum do dispositivo com o valor do pacote recebido, para que ele possa usar esse valor para enviar a resposta

        //verifica se o srcaddress e dstaddress do pacote sao validos
        // ret = getaddress((uint8_t *)lastpkt.payload,packetSize);
        if ((srcadress < MAX_ADDR) && ((dstadress < MAX_ADDR) || (dstadress == BROADCAST_ADDR))) 
          ret = 1;
        else
          ret = 0;

        // ret = getaddress((uint8_t *)buffer,packetSize);
        // log_i("Rx Pktsize: %d",packetSize);
        
        //verifica se o pacote recebido nao eh o mesmo que acabou de ser enviado
        if ((ret) && ((srcadress != mydd.devaddr))) {
            // lastpkt.fct       = getfunction((uint8_t *)lastpkt.payload,packetSize);
            // lastpkt.seqnum    = getseqnum((uint8_t *)lastpkt.payload,packetSize);
            // lastpkt.timestamp = gettimestamp((uint8_t *)lastpkt.payload,packetSize);
            retcrc = calculate_crc((uint8_t *)lastpkt.payload,lastpkt.packetSize);

            
            log_i("Rx[%d] = %02X %02X %02X %02X %02X %02X %02X",lastpkt.packetSize, dstadress, srcadress,lastpkt.payload[2],lastpkt.payload[3],fct, lastpkt.payload[lastpkt.packetSize-2], lastpkt.payload[lastpkt.packetSize-1]);
            // log_i("SeqNum: %d",seqnum);
            // log_i("crc: %02X", retcrc);

            if ((!retcrc) && ((dstadress == mydd.devaddr) || (dstadress == BROADCAST_ADDR))) {
              log_i("Packet received with valid CRC, destined for this device or broadcast. Processing...");
                return 1;
            }
            else
                return 0;
        }
        else
           return 0;

    }
    else
        return 0;
}

int LoRaClass::packetRssi()
{
	int8_t snr=0;
    int8_t SnrValue = readRegister( 0x19 );
    int16_t rssi = readRegister(RADIOLIB_SX127X_REG_PKT_RSSI_VALUE);

	if( SnrValue & 0x80 ) // The SNR sign bit is 1
	{
		// Invert and divide by 4
		snr = ( ( ~SnrValue + 1 ) & 0xFF ) >> 2;
		snr = -snr;
	}
	else
	{
		// Divide by 4
		snr = ( SnrValue & 0xFF ) >> 2;
	}
    if(snr<0)
    {
    	rssi = rssi - (_frequency < 525E6 ? 164 : 157) + ( rssi >> 4 ) + snr;
    }
    else
    {
    	rssi = rssi - (_frequency < 525E6 ? 164 : 157) + ( rssi >> 4 );
    }

  return ( rssi );
}


void LoRaClass::idle()
{
  writeRegister(RADIOLIB_SX127X_REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRaClass::sleep()
{
  writeRegister(RADIOLIB_SX127X_REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

int LoRaClass::available()
{
  return (readRegister(RADIOLIB_SX127X_REG_RX_NB_BYTES) - _packetIndex);
}

int LoRaClass::read()
{
  #if defined ( WIFI_LoRa_32_V2 )  
    if (!available()) {
      return -1; 
    }
    _packetIndex++;
    return readRegister(RADIOLIB_SX127X_REG_FIFO);
  #else
     return 0;
  #endif  
}

void LoRaClass::setTxPowerMax(int level)
{
	if (level < 5)		{
		level = 5;
	}
	else if(level > 20)	{
		level = 20;
	}
	writeRegister(REG_OCP,0x3f);
	writeRegister(REG_PADAC,0x87);//Open PA_BOOST
	writeRegister(RADIOLIB_SX127X_REG_PA_CONFIG, RF_PACONFIG_PASELECT_PABOOST | (level - 5));
}


void LoRaClass::setPins(int ss, int reset, int dio0)
{
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
}

void LoRaClass::enableCrc()
{
#if defined ( WIFI_LoRa_32_V2 ) 
  writeRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_2, readRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_2) | 0x04);
#else
  writeRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_2, readRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_2) | 0x04);
#endif

}

void LoRaClass::disableCrc()
{
  writeRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_2, readRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_2) & 0xfb);
}


void LoRaClass::explicitHeaderMode()
{
  _implicitHeaderMode = 0;
  writeRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_1, readRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRaClass::implicitHeaderMode()
{
  _implicitHeaderMode = 1;
  writeRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_1, readRegister(RADIOLIB_SX127X_REG_MODEM_CONFIG_1) | 0x01);
}

bool LoRaClass::sendPacket(uint8_t* p, uint8_t len) {
    
    // 1. FORÇA O MODO STANDBY ANTES DE TRANSMITIR
    // Isso "limpa" o estado do rádio e garante que ele saia do RX com segurança
    radio.standby(); 

    clearDioActions();
    enableCrc();

    // 2. REMOVA O PARÂMETRO '1' NO FINAL
    // Deixe apenas o buffer e o tamanho. 
#if defined ( WIFI_LoRa_32_V3 )
    int16_t transmissionState = radio.transmit(p, len); 
#else  //WIFI_LoRa_32_V2
    int transmissionState = radio.transmit(p, len);
#endif

    // 3. RETORNA PARA A ESCUTA SE NECESSÁRIO
    // (Apenas se o dispositivo precisar voltar a escutar imediatamente)
    // startReceiving(); // Cuidado ao deixar isso aqui se você for usar o Sleep no ED!

    if (transmissionState == RADIOLIB_ERR_NONE) {
        return true;
    } else {
        log_e("transmission failed, code=%d ", transmissionState);
        
        // Em caso de falha grave, reiniciar o rádio é uma boa tática de segurança
        if (transmissionState == RADIOLIB_ERR_TX_TIMEOUT) {
            restartRadio(); 
        }
        return false;
    }   
}


uint8_t LoRaClass::readRegister(uint8_t address)
{
  return singleTransfer(address & 0x7f, 0x00);
}

void LoRaClass::writeRegister(uint8_t address, uint8_t value)
{
  singleTransfer(address | 0x80, value);
}


uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value)
{
  uint8_t response;
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer(address);
  response = SPI.transfer(value);
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);
  return response;
}


uint32_t getRssi(void) {
  uint32_t retRssi=0;

 #if  defined ( WIFI_LoRa_32_V3 ) 
     retRssi = (uint32_t) radio.getRSSI(1);
 #else
     retRssi = (uint32_t) loramesh.packetRssi();
 #endif
 
 return retRssi;
}

/*
* Function to receive a frame
*/
uint8_t LoRaClass::ReceiveFrame(char *pframe) {
  uint8_t packetSize = 0;

 #if  defined ( WIFI_LoRa_32_V3 ) 
  String str;

#if ENABLE_RX_INTERRUPT
  //radio.clearDio1Action();

  if (messageReceived) {
    messageReceived = false;
    //log_i("msg received!!!");

  #if 0  
  int state = radio.receive(str);

  if (state == RADIOLIB_ERR_NONE) {
    // Packet received successfully
    packetSize = str.length();
    strcpy(pframe,str.c_str());
      log_i("Received packet [%d]",packetSize);
  }
 #else
 int state = radio.receive(str);

 #endif

    // Start Receiving
    startReceiving();
    //radio.setDio1Action(rx);
    //radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);    

  }
#else
  int state = radio.receive(str);

  if (state == RADIOLIB_ERR_NONE) {
    // Packet received successfully
    packetSize = str.length();
    strcpy(pframe,str.c_str());

    log_i("Received packet [%d] rssi=%d",packetSize,getRssi());
  }
#endif


#else // WIFI_LoRa_V2

  String str;

  enableCrc();
  
  int state = radio.readData(str);
  if (state == RADIOLIB_ERR_NONE) {
    // packet was successfully received
    log_i("Received packet len=%d",str.length());
    Serial.println(str);
  }
#endif

  return packetSize;
}


size_t LoRaClass::write(uint8_t byte)
{
  return write(&byte, sizeof(byte));
}

size_t LoRaClass::write(const uint8_t *buffer, size_t size)
{
  int currentLength = readRegister(RADIOLIB_SX127X_REG_PAYLOAD_LENGTH);
  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }
  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(RADIOLIB_SX127X_REG_FIFO, buffer[i]);
  }
  // update length
  writeRegister(RADIOLIB_SX127X_REG_PAYLOAD_LENGTH, currentLength + size);
  return size;
}


int LoRaClass::peek()
{
  if (!available()) {
  	return -1; 
	}
  // store current FIFO address
  int currentAddress = readRegister(RADIOLIB_SX127X_REG_FIFO_ADDR_PTR);
  // read
  uint8_t b = readRegister(RADIOLIB_SX127X_REG_FIFO);
  // restore FIFO address
  writeRegister(RADIOLIB_SX127X_REG_FIFO_ADDR_PTR, currentAddress);
  return b;
}

void LoRaClass::flush()
{
  ;
}

