#include "devconfig.h"
#include "loramesh.h"
#include <RadioLib.h>
#include "radio.h"

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
// V2
// F095
// 0x907F
// 0x5006
// V3
// 0x707D
// 0xDC78
// 0x1C65
#if defined ( WIFI_LoRa_32_V2 )
strDevicedescription devid[]={
   {0xACFD,DEV_TYPE_ROUTER,1,0},
   {0xF095,DEV_TYPE_ENDDEV,2,2},
   {0xCC7F,DEV_TYPE_ENDDEV,3,3},
   {0X8096,DEV_TYPE_ENDDEV,4,4}
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

// Função para limpar o buffer
void LoRaClass::clearBuffer(uint8_t *buffer, int size)
{
    for (int i = 0; i < size; i++)
    {
        buffer[i] = '\0';
    }
}

uint16_t LoRaClass::getseqnum(uint8_t *packet,uint8_t len){
    uint16_t aux;
    uint8_t *pucaux = (uint8_t *) &aux;
    if (len > 4){
        *pucaux++ = packet[4];
        *pucaux = packet[3];
        //log_i ("seq.num=%d",aux);
        return aux;
    }
    else
        return 0;

}

uint8_t LoRaClass::getfunction(uint8_t *packet,uint8_t len){
    uint8_t function;

    if (len > 3){
        function = packet[2];
        //log_i ("function=%d",function);
        return function;
    }
    else
        return 0;

}

uint32_t LoRaClass::gettimestamp(uint8_t *packet,uint8_t len){
    uint32_t timestamp;
    uint8_t *pucaux = (uint8_t *) &timestamp;

    if (len > 5){
        *pucaux++ = packet[8];
        *pucaux++ = packet[7];
        *pucaux++ = packet[6];
        *pucaux = packet[5];
        //log_i ("timestamp=%4x",timestamp);
        return timestamp;
    }
    else
        return 0;

}
uint8_t LoRaClass::getaddress(uint8_t *packet,uint8_t len){
  
  lastpkt.srcaddress = packet[0];
  lastpkt.dstaddress = packet[1];

  //log_i("src=%d dst=%d",lastpkt.srcaddress,lastpkt.dstaddress);

  if ((lastpkt.srcaddress < MAX_ADDR) && ((lastpkt.dstaddress < MAX_ADDR) || (lastpkt.dstaddress == BROADCAST_ADDR))) 
    return 1;
  else
    return 0;

}

//Todo!!! implementar um CRC
//checa somente o ultimo byte do frame é igual ao definido
uint8_t LoRaClass::checkcrc (uint8_t *packet, uint8_t len){
   
   if (packet[len-1] == BYTE_CRC)
    return 1;
  else
    return 0;
}

uint16_t LoRaClass::getLastSeqNum(){

    return (mydd.seqnum);
}

uint16_t LoRaClass::getLastPctSeqNum(){

    return (lastpkt.seqnum);
}

uint8_t LoRaClass::getResponseCode(uint8_t* packet){
  if(lastpkt.packetSize > 5) return packet[5];
  return -1;
}

uint8_t LoRaClass::getResponseValue(uint8_t* packet, uint8_t* responseBuffer, uint8_t buffersize){
  if(lastpkt.packetSize > 5 && packet[2] == FCT_READING){
    uint8_t valueSize = packet[5];
    if(valueSize <= buffersize){
      memcpy(responseBuffer,&packet[6],valueSize);
      return valueSize;
    }
  }

  return 0;
  

}


//estrutura do pacote de requisição [dst,src,seq number, fct, start, qtd parametros, crc]
//função não utilizad no código por enquanto
uint8_t LoRaClass::sendPacketReq(uint8_t dst, uint8_t fct, uint8_t start, uint8_t qtdParametros){
  
  uint8_t buf[BUFFER_SIZE];
  uint8_t pos = 0;
  uint8_t *pucaux = (uint8_t *) &mydd.seqnum;
  uint8_t ret;

  buf[pos++] = dst;
  buf[pos++] = mydd.devaddr;
  buf[pos++] = *(pucaux+1);
  buf[pos++] = *(pucaux+0);
  buf[pos++] = fct;
  buf[pos++] = start;
  buf[pos++] = qtdParametros;
  buf[pos++] = BYTE_CRC;

  ret = sendPacket(buf,pos);

  if (ret) 
    return pos;
  else
    return 0;
}

uint8_t LoRaClass::sendBeacon(long timestamp)
{
    uint8_t ret=0;
    uint8_t pos=0;
    uint8_t buf[BUFFER_SIZE];
    uint8_t *pucaux = (uint8_t *) &mydd.seqnum;

    mydd.seqnum++;

    buf[pos++] =  mydd.devaddr;
    buf[pos++] =  BROADCAST_ADDR;
    buf[pos++] =  FCT_BEACON;
    buf[pos++] =  *(pucaux+1);
    buf[pos++] =  *(pucaux+0);
    pucaux = (uint8_t *) &timestamp;
    buf[pos++] =  *(pucaux+3);
    buf[pos++] =  *(pucaux+2);
    buf[pos++] =  *(pucaux+1);
    buf[pos++] =  *(pucaux+0);
    buf[pos++] =  BYTE_CRC;

    ret = sendPacket(buf,pos);
    if (ret){
       //vTaskDelay(10 / portTICK_PERIOD_MS);
       //int res = startReceiving(0);
       //if (res > 0){
       //   log_e("Error startReceiving=%d",res);
       //   return 0;
       //}
       log_i("BEACON [%d] = %2x %2x %2x %2x %2x", pos, buf[0], buf[1],buf[2],buf[3], buf[4]);
       return pos;
    }
    else
       return 0;   
}


uint8_t LoRaClass::sendData(uint8_t dstaddr,uint16_t value)
{
    uint8_t ret=0;
    uint8_t pos=0;
    uint8_t buf[BUFFER_SIZE];
    uint8_t *pucaux = (uint8_t *) &value;

    buf[pos++] =  mydd.devaddr;
    buf[pos++] =  dstaddr;
    buf[pos++] =  FCT_READING;
    buf[pos++] =  *(pucaux+1);
    buf[pos++] =  *(pucaux+0);
    buf[pos++] =  BYTE_CRC;

    ret = sendPacket(buf,pos);
    if (ret){
       //vTaskDelay(20 / portTICK_PERIOD_MS);
       //uint32_t rx_timeout = (mydd.devtype == DEV_TYPE_ROUTER) ? 0 : 2000; 
       //int res= startReceiving(rx_timeout);
       //if (res > 0){
       //   log_e("Error startReceiving=%d",res);
       //   return 0;
       //}
       log_i("Data.REQ [%d] = %2x %2x %2x %2x %2x", pos, buf[0], buf[1],buf[2],buf[3], buf[4]);
       return pos;
    }
    else
       return 0;   
}

//função implementada para enviar valores inteiros (2 bytes)

uint8_t LoRaClass::sendPacketResponse(uint8_t dst, uint8_t size, uint8_t *buf){
  uint8_t buffer[BUFFER_SIZE];
  uint8_t* pucaux = buf;
  uint8_t aux = 0, i=0;

  buffer[aux++] = mydd.devaddr;
  buffer[aux++] = dst;
  buffer[aux++] = FCT_READING;
  
  for (i=0;i<size;i++){
      buffer[aux++] = *(pucaux++);
  }
  buffer[aux++] = size;
//  pucaux = (uint8_t*) &value;
//  buffer[aux++] = *(pucaux+3);
//  buffer[aux++] = *(pucaux+2);
//  buffer[aux++] = *(pucaux+1);
//  buffer[aux++] = *(pucaux);
  buffer[aux++] = BYTE_CRC;

  if(loramesh.sendPacket(buffer,aux)){
     vTaskDelay(20 / portTICK_PERIOD_MS);

     uint32_t rx_timeout = (mydd.devtype == DEV_TYPE_ROUTER) ? 0 : 2000; 
     int res= startReceiving(rx_timeout);
     if (res > 0){
       log_e("Error startReceiving=%d",res);
       return 0;
     }

     log_i("Data.RES [%d] = %2x %2x %2x %2x %2x", aux, buffer[0], buffer[1],buffer[2],buffer[3], buffer[4]);
     return 1;
  }
  else
    return 0;
}

uint8_t LoRaClass::sendPacketRes(uint8_t dstaddr)
{
    uint8_t ret=0;
    uint8_t pos=0;
    uint8_t buf[BUFFER_SIZE];
    uint8_t *pucaux = (uint8_t *) &lastpkt.seqnum; 

    buf[pos++] =  mydd.devaddr;
    buf[pos++] =  dstaddr;
    buf[pos++] =  FCT_DATA;
    buf[pos++] =  *(pucaux+1);
    buf[pos++] =  *(pucaux+0);
    buf[pos++] =  BYTE_CRC;

#if 1
    ret = sendPacket(buf,pos);
    if (ret){
        vTaskDelay(20 / portTICK_PERIOD_MS);

        uint32_t rx_timeout = (mydd.devtype == DEV_TYPE_ROUTER) ? 0 : 2000; 
        int res= startReceiving(rx_timeout);
        if (res > 0){
          log_e("Error startReceiving=%d",res);
          return 0;
        }

       log_i("RES[%d]=%2x %2x %2x %2x %2x %2x %2x %2x", pos, buf[0], buf[1],buf[2],buf[3], buf[4], buf[5],buf[6],buf[7]);
       return pos;
    }
    else
       return 0;   
#else
  loramesh.beginPacket();
  //print: adiciona os dados no pacote
  for (int i = 0; i < sizeof(frame1); i++) {
      loramesh.write((uint8_t)txpacket[i]);
  }
  loramesh.endPacket(); //retorno= 1:sucesso | 0: falha

#endif    
}


bool LoRaClass::receivePacket()
{
    bool retcrc=0;
    int packetSize = 0;
    uint8_t ret=0;
    int len = 0;


#if defined(WIFI_LoRa_32_V3)
  uint8_t offset = 0;
  int16_t state = 0;

  // get packet length and Rx buffer offset
  packetSize = radio.getPacketLength(true, &offset);
  if (packetSize) {
      state = radio.readData(lastpkt.rxpacket,packetSize); 
      RADIOLIB_ASSERT(state);

#else // V2
    packetSize = loramesh.parsePacket(0);
    lastpkt.packetSize = packetSize;
    if (packetSize) {
        while (loramesh.available() && len < BUFFER_SIZE - 1) {
            lastpkt.rxpacket[len++] = (char)loramesh.read(); // Lê o pacote byte a byte
        }
#endif

        // verifica o srcaddress e dstaddress do pacote
        ret = getaddress((uint8_t *)lastpkt.rxpacket,packetSize);
        log_i("Rx Pktsize: %d",lastpkt.packetSize);
        
        //verifica se o pacote recebido nao eh o mesmo que acabou de ser enviado
        if ((ret) && ((lastpkt.srcaddress != mydd.devaddr))) {
            lastpkt.fct       = getfunction((uint8_t *)lastpkt.rxpacket,packetSize);
            lastpkt.seqnum    = getseqnum((uint8_t *)lastpkt.rxpacket,packetSize);
            // lastpkt.timestamp = gettimestamp((uint8_t *)lastpkt.rxpacket,packetSize);
            retcrc = checkcrc((uint8_t *)lastpkt.rxpacket,packetSize);

            log_i("Rx[%d] = %d %d %d %d",lastpkt.packetSize, lastpkt.srcaddress, lastpkt.dstaddress, lastpkt.fct,lastpkt.seqnum);

            if ((retcrc == 1) && ((lastpkt.dstaddress == mydd.devaddr) || (lastpkt.dstaddress == BROADCAST_ADDR))) {
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

bool LoRaClass::sendPacket(uint8_t* p,uint8_t len) {
    
    //waitBeforeSend(1);

#if  defined ( WIFI_LoRa_32_V3 )

  clearDioActions();
  enableCrc();

  int16_t transmissionState = radio.transmit(p,len,1);

  //Start receiving again after sending a packet
  startReceiving();

  if (transmissionState == RADIOLIB_ERR_NONE) {
  return true;
} else {
  log_e("transmission failed, code=%d ",transmissionState);
  return false;
}   

#else  //WIFI_LoRa_32_V2

    clearDioActions();
    enableCrc();
    //Blocking transmit, it is necessary due to deleting the packet after sending it. 
    int transmissionState = radio.transmit(p, len,1);

    ///RFF
    //log_i("SEND [%d] = %2x %2x %2x %2x %2x", len, p[0], p[1],p[2],p[3], p[4]);
   
    //Start receiving again after sending a packet
    //startReceiving();

   if (transmissionState == RADIOLIB_ERR_NONE) {
    return true;
  } else {
    log_e("transmission failed, code=%d ",transmissionState);
    return false;
  }   

#endif

    return true;
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

