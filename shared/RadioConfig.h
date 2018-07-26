#pragma once

#define MY_RADIO_NRF24

#define DEFAULT_RF24_CHANNEL		(90)
#define DEFAULT_RF24_PA_LEVEL_NODE	(RF24_PA_LOW)
#define DEFAULT_RF24_PA_LEVEL_GW	(RF24_PA_LOW)
#define DEFAULT_RF24_DATARATE		(RF24_250KBPS)

// Tell MySensors where to find the radio parameters
extern uint8_t iRf24Channel;
extern uint8_t iRf24DataRate;
extern uint8_t iRf24PaLevel;
#define MY_RF24_CHANNEL			(iRf24Channel)
#define MY_RF24_DATARATE		(iRf24DataRate)
#define MY_RF24_PA_LEVEL		(iRf24PaLevel)
#define MY_RF24_BASE_RADIO_ID 0x00,0xCA,0xFE,0xBA,0xBE
