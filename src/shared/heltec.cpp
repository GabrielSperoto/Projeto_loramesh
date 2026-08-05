// Copyright (c) Heltec Automation. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#include "devconfig.h"
#include "heltec.h"

#define LOG_LOCAL_LEVEL CONFIG_LOG_MAXIMUM_LEVEL

Heltec_ESP32::Heltec_ESP32()
{

#if defined( WIFI_Kit_32 ) || defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined ( WIFI_LoRa_32_V3 )
      display = new SSD1306Wire(0x3c, SDA_OLED, SCL_OLED, RST_OLED, GEOMETRY_128_64);
#elif defined( Wireless_Stick )
	  display = new SSD1306Wire(0x3c, SDA_OLED, SCL_OLED, RST_OLED, GEOMETRY_64_32);
#endif
 
}


Heltec_ESP32::~Heltec_ESP32()
{
#if defined( WIFI_Kit_32 ) || defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick ) || defined( WIFI_LoRa_32_V3 )
	delete display;
#endif
}


void Heltec_ESP32::begin() {

#if DISPLAY_ENABLE 
	{
#if defined( Wireless_Stick_Lite ) || defined( Wireless_Bridge )
		if(SerialEnable)		{
			Serial.print("Wireless Stick Lite and Wireless Bridge don't have an on board display, Display option must be FALSE!!!\r\n");
		}
#endif

#if defined( WIFI_LoRa_32_V3 ) 
/* O estado do GPIO36 é utilizado para controlar o display OLED. 
   Para o OLED permanecer ligado, o GPIO36 deve permanecer HIGH e deve estar neste estado quando chamar display Init.
   NAO INVERTER esta ORDEM.
*/
	pinMode(36,OUTPUT);
	digitalWrite(36,LOW);
	delay(50);
	digitalWrite(36,HIGH);
#endif

#if defined( WIFI_Kit_32 ) || defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick ) || defined( WIFI_LoRa_32_V3 ) 
	display->init();
	display->flipScreenVertically();
	display->setFont(ArialMT_Plain_10);
	display->drawString(0, 0, "OLED initial done!");
	display->display();
	log_i("OLED initial done!\r\n");
#endif
	}
#endif

#if defined( WIFI_Kit_32 ) || defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick ) 	
	pinMode(LED_BUILTIN,OUTPUT);
#endif

#if DISPLAY_ENABLE
	display->display();
	delay(300);
#endif

}


void Heltec_ESP32::DisplayClear() 
{
	Heltec.display->clear();
	Heltec.display->setFont(ArialMT_Plain_16);
}


void Heltec_ESP32::DisplayShow1(char *pframe) 
{
	char buf[20];
	uint32_t rssi=0;

	memset(buf,0x20,sizeof(buf));

	sprintf(buf, "%s", pframe);  
	Heltec.display->drawString(0, 0, buf);
	//Heltec.display->display();
}

void Heltec_ESP32::DisplayShow2(char *pframe) 
{
	char buf[20];
	uint32_t rssi=0;

	//Heltec.display->clear();
	//Heltec.display->setFont(ArialMT_Plain_16);

	memset(buf,0x20,sizeof(buf));
	sprintf(buf, "%s", pframe);  
	Heltec.display->drawString(0, 20, buf);
	Heltec.display->display();
	
}

void Heltec_ESP32::DisplayShow3(char *pframe) 
{
	char buf[20];
	uint32_t rssi=0;

	//Heltec.display->clear();
	Heltec.display->setFont(ArialMT_Plain_16);

	memset(buf,0x20,sizeof(buf));
	sprintf(buf, "%s", pframe);  
	Heltec.display->drawString(0, 40, buf);
	Heltec.display->display();

}

void Heltec_ESP32::DisplayShowAll(char *line1, char *line2, char *line3) 
{
	char buf[20];

	Heltec.display->clear();
	Heltec.display->setFont(ArialMT_Plain_16);

	memset(buf,0x20,sizeof(buf));
	sprintf(buf, "%s", line1);  
	Heltec.display->drawString(0, 0, buf);

	memset(buf,0x20,sizeof(buf));
	sprintf(buf, "%s", line2);  
	Heltec.display->drawString(0, 20, buf);

	memset(buf,0x20,sizeof(buf));
	sprintf(buf, "%s", line3);  
	Heltec.display->drawString(0, 40, buf);

	Heltec.display->display();
}


Heltec_ESP32 Heltec;
