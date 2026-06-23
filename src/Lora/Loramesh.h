#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <SPI.h>
#include "devconfig.h"
#include <RadioLib.h>
#include "radio.h"


#if defined( WIFI_LoRa_32_V3 )
#define LORA_DEFAULT_SS_PIN     8
#define LORA_DEFAULT_RESET_PIN  12
#define LORA_DEFAULT_DIO0_PIN   14
#endif

#if defined( WIFI_LoRa_32_V2)
#define LORA_DEFAULT_SS_PIN     18
#define LORA_DEFAULT_RESET_PIN  14
#define LORA_DEFAULT_DIO0_PIN   26
#endif

#define PA_OUTPUT_PA_BOOST_PIN  1
#define PA_OUTPUT_RFO_PIN       0

#define LOCAL_ADDRESS 1
/*!
 * RegPaConfig
 */
#define RF_PACONFIG_PASELECT_MASK                   0x7F
#define RF_PACONFIG_PASELECT_PABOOST                0x80
#define RF_PACONFIG_PASELECT_RFO                    0x00 // Default

#define RF_PACONFIG_MAX_POWER_MASK                  0x8F

#define RF_PACONFIG_OUTPUTPOWER_MASK                0xF0

/*!
 * RegPaDac
 */
#define RF_PADAC_20DBM_MASK                         0xF8
#define RF_PADAC_20DBM_ON                           0x07
#define RF_PADAC_20DBM_OFF                          0x04  // Default




// O __attribute__((packed)) garante que a estrutura tenha o tamanho exato dos bytes somados
typedef struct __attribute__((packed)) {
    uint8_t  packetSize;
    uint8_t  payload[BUFFER_SIZE]; // Troquei de rxpacket para payload
} strPacket;

typedef enum {
    TCP,
    APP,
    COMM
} task_t;

typedef union {
    uint8_t bytes[4];
    uint32_t value;
} payload_t;

typedef struct{
    uint8_t dst;
    uint8_t src;
    uint16_t seqnum;
    uint8_t function;
    uint8_t start;
    uint8_t qtdParametros; //start e qtdParametros fazem parte do pacote de requisição 
    uint8_t size;
    payload_t payload; //valor usado na escrita ou leitura
} msg_t;

typedef struct {
    msg_t msg;
    uint8_t ocupado; //flag para indicar se a estrutura está ocupada ou não
} tcpMsgs_t;

typedef struct  {
    uint16_t devserialnumber;
    uint8_t  devtype;
    uint8_t  devaddr;
    uint8_t  dataslot;
    uint16_t seqnum;
} strDevicedescription;


typedef enum {
   DEV_TYPE_ROUTER=1,
   DEV_TYPE_ENDDEV
} devicetype;

typedef enum {
   FCT_BEACON=1,
   FCT_JOIN, 
   FCT_DESCRIPTION,
   FCT_READING,
   FCT_WRITTING,
   FCT_TXDONE
} functioncode;

typedef enum  {
    ST_TXBEACON,
    ST_STARTRX,
    ST_RXWAIT,
    ST_STARTTX,
    ST_TXDATA,
    ST_WAITTXDONE,
    ST_STANDBY
}statemac;

//paremtros disponiveis
typedef enum{
    LEDP = 1,
    POT = 2
}parametercode;

#if defined (__STM32F1__)
inline unsigned char  digitalPinToInterrupt(unsigned char Interrupt_pin) { return Interrupt_pin; } //This isn't included in the stm32duino libs (yet)
#define portOutputRegister(port) (volatile byte *)( &(port->regs->ODR) ) //These are defined in STM32F1/variants/generic_stm32f103c/variant.h but return a non byte* value
#define portInputRegister(port) (volatile byte *)( &(port->regs->IDR) ) //These are defined in STM32F1/variants/generic_stm32f103c/variant.h but return a non byte* value
#endif

enum LoraModules {
    SX1276_MOD,
    SX1262_MOD,
    SX1278_MOD,
    SX1268_MOD,
    SX1280_MOD,
};

class LoRaClass : public Stream {
public:
  strDevicedescription mydd;
  strPacket lastpkt;
  msg_t msg;

  LoRaClass();

  int16_t standby();
  int begin();
  void end();
  void initializeLoRa();
  uint8_t getNodes(strDevicedescription *Nodes, uint8_t numNodes);

  bool sendPacket(uint8_t *data, uint8_t len);
  
  void VextON(void);
  void VextOFF(void);
    
  void restartRadio(void);
  int startReceiving(uint32_t timeout);
  

  void decodeLoraPacket(msg_t *msg);
  uint8_t encodeAndSendPacket(msg_t *msg);


  void setDioActionsForReceivePacket(void);
  void clearDioActions(void);
  void onReceive(void);
  
  uint8_t getrouteaddr(void);
  uint16_t calculate_crc (uint8_t *packet, uint8_t len);
//   uint8_t getaddress(uint8_t *packet,uint8_t len);
//   uint32_t gettimestamp(uint8_t *packet,uint8_t len);
//   uint8_t getfunction(uint8_t *packet,uint8_t len);
  uint16_t getseqnum(uint8_t *packet,uint8_t len);
  uint16_t getLastSeqNum(void);
  uint16_t getLastPctSeqNum(void);
//   uint8_t getResponseStatus(void);
  uint32_t getPayloadValue(uint8_t *packet, uint8_t len); // obtem o valor de leitura
  uint8_t getWrittingCode(void);

//   uint8_t getSrcAdress();
//   uint8_t getFunctionCode();
  uint8_t getStart();
  uint8_t getQtdParametros();
  uint8_t getSizeMsg();


 

  void clearBuffer(uint8_t *buffer, int size);
  bool getdevicedescription(void);
  
  uint8_t ReceiveFrame(char *pframe); 

  int beginPacket(int implicitHeader = false);
  int endPacket(bool async = false);

  int parsePacket(int size = 0);

  uint32_t getRssi(void);
  int packetRssi();

  virtual size_t write(uint8_t byte);
  virtual size_t write(const uint8_t *buffer, size_t size);
  virtual int peek();

  virtual int available();
  virtual int read();
  virtual void flush();

  void onReceive(void(*callback)(int));
  bool receivePacket(void);
  
  void ClearRadioIRQs();
  void receive(int size = 0);
  void idle();
  void sleep();

  void enableCrc();
  void disableCrc();
  
  // deprecated
  void crc() { enableCrc(); }
  void noCrc() { disableCrc(); }

  void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
  
  void createPacketAndSend(uint16_t dst, uint8_t* payload, uint8_t payloadSize);
  
  static LoRaClass& getInstance() {
        static LoRaClass instance;
        return instance;
  };  

    template <typename T>
    void createPacketAndSend(uint16_t dst, T* payload, uint8_t payloadSize) {
        //Cannot send an empty packet
        log_i("createPacketAndSend");  
        if (payloadSize == 0)
            return;

        //Get the size of the payload in bytes
        size_t payloadSizeInBytes = payloadSize * sizeof(T);


    }

  void setTxPowerMax(int level);

  #if 0
  static void onDio0Rise();
 
  float packetSnr();
  void handleDio0Rise();
  void dumpRegisters(Stream& out);
  byte random();
  void setSPIFrequency(uint32_t frequency);
  void enableTxInvertIQ();
  void enableRxInvertIQ();
  void enableInvertIQ();
  void disableInvertIQ();
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void setTxPower(int8_t power, int8_t outputPin);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void sendPackets();
  
 #endif
  

private:
  void explicitHeaderMode();
  void implicitHeaderMode();
  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

  SPISettings _spiSettings;
  int _ss;
  int _reset;
  int _dio0;
  int _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  void (*_onReceive)(int);
};

//extern LoRaClass loramesh;

#endif
