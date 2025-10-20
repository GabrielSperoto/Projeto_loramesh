#ifndef _DEVCONFIG_H_
#define _DEVCONFIG_H_

//Suport Heltec V2 and V3 
// heltec_wifi_lora_32_V2 -  radio chipset SX1276 OLED chip SSD1306)
// heltec_wifi_lora_32_V3 -  radio chipset SX1262 OLED chip SSD1306)

//define se vai usar ou nao o Display Oled da placa 
#define DISPLAY_ENABLE 1

#define ENABLE_RX_INTERRUPT 1

#define SYNC_INTERVAL_MS  1000   // Synchronization interval
#define EXPECTED_PERIOD_MS 1000  // Desired period for ESP32 A
#define ADJUSTMENT_FACTOR  0.1   // Proportional correction factor
#define MIN_VAL 2000.0
#define MAX_VAL 500.0

#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif // min
         
#ifndef MAX
#define MAX(a,b) ((a)>(b)?(a):(b))
#endif // max


// Packet types
#define NEED_ACK_P 0b00000011
#define DATA_P     0b00000010
#define HELLO_P    0b00000100
#define ACK_P      0b00001010
#define XL_DATA_P  0b00010010
#define LOST_P     0b00100010
#define SYNC_P     0b01000010
#define LOG_LOCAL_LEVEL CONFIG_LOG_MAXIMUM_LEVEL
 
//#define RX_TIMEOUT_VALUE   1000

//#define MINIMUM_DELAY 900 

#define MAX_ADDR 5
#define BROADCAST_ADDR 0
#define BYTE_CRC 0x66

#define MAX_PACKET_SIZE  30

#define MAX_SLOTS   3
#define BEACON_SLOT 0 

//Intervalo entre os envios
#define SLOT_INTERVAL 1000

#define BUFFER_SIZE           50 // Define the payload size here

/*  LoRa spreading factor. Defaults to 9.
   goes from 7 to 12 where SF7 is the shortest and SF12 the longest */
// Number from 5 to 12. Higher means slower but higher "processor gain",

#define LORA_SF 7

/*Frequency for lora
   434.0 MHz (default) 
   470E6
   868E6 
   915E6 */
#define LORA_FREQUENCY 868.0 

/* LoRa bandwidth in kHz.  Defaults to 125.0 kHz.
125.0
250.0 
500.0
1625.0  (para freq 2.4Ghz)
*/
// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.

#define LORA_BW 125.0


/* Output power in dBm. 
  V3 - 
 Defaults to 10 dBm. range from 10 to 20 dBm
// 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
V2 -  
 Allowed values range from -3 to 15 dBm (RFO pin) or +2 to +17 dBm (PA_BOOST pin).
*/
#define LORA_TRANSMIT_POWER 2  //valor minimo

/* Gain for lora. Defaults to 1.
   goes from 0 to 1 where 0 is the lowest gain and 1 is the highest gain 
   nao sei o que eh este ganho...sera que eh o #define PABOOST TRUE ?*/
 
#define LORA_GAIN 1

/* cr */
#define LORA_CR 5



#endif
