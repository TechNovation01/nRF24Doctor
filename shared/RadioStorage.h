#pragma once

#include <stdint.h>

#define DEFAULT_DESTINATION_NODE	(0)				// Default 0 = gateway, Settable in Menu
#define DEFAULT_PAYLOAD_SIZE		(2)				// 2 Bytes is the minimum for the Counter data
#define DEFAULT_MESSAGE_RATE 		(10)

//**** EEPROM STORAGE LOCATIONS *****
#define EEPROM_FLAG_MAGIC		0xA5u	// Indication contents are valid. Empty eeprom will contain 0xFF
#define EEPROM_FLAG				0
#define EEPROM_CHANNEL			1
#define EEPROM_PA_LEVEL			2
#define EEPROM_PA_LEVEL_GW		3
#define EEPROM_DATARATE			4
#define EEPROM_DESTINATION_NODE	5
#define EEPROM_PAYLOAD_SIZE		6
#define EEPROM_MESSAGE_RATE		7

#ifndef MY_GATEWAY_FEATURE
// Node-only parameters
uint8_t iRf24PaLevelGw;		//PA Level for the Gateway
uint8_t iPayloadSize;
uint8_t iSetMsgRate;
uint8_t iDestinationNode;
#endif

void saveEepromAndReset();
void reset();

void logRadioSettings()
{
	Sprint(F("Channel:"));		Sprint(iRf24Channel);
	Sprint(F("\tPaLevel:"));	Sprint(rf24PaLevelToString(iRf24PaLevel));
#ifndef MY_GATEWAY_FEATURE
	Sprint(F("\tPaLevelGw:"));	Sprint(rf24PaLevelToString(iRf24PaLevelGw));
#endif
	Sprint(F("\tDataRate:"));	Sprint(rf24DataRateToString(iRf24DataRate));
#ifndef MY_GATEWAY_FEATURE
	Sprint(F("\tDest:"));		Sprint(iDestinationNode);
	Sprint(F("\tPayload:"));	Sprint(iPayloadSize);
	Sprint(F("\tRate:"));		Sprint(iSetMsgRate);
#endif
	Sprintln();
}

void loadDefaults()
{
	Sprintln(F("Load defaults"));

	iRf24Channel		= DEFAULT_RF24_CHANNEL;
#ifdef MY_GATEWAY_FEATURE
	iRf24PaLevel		= DEFAULT_RF24_PA_LEVEL_GW;
#else
	iRf24PaLevel		= DEFAULT_RF24_PA_LEVEL_NODE;
#endif
	iRf24DataRate		= DEFAULT_RF24_DATARATE;
#ifndef MY_GATEWAY_FEATURE
	iRf24PaLevelGw		= DEFAULT_RF24_PA_LEVEL_GW;
	iDestinationNode	= DEFAULT_DESTINATION_NODE;
	iPayloadSize		= DEFAULT_PAYLOAD_SIZE;
	iSetMsgRate			= DEFAULT_MESSAGE_RATE;
#endif
}

void loadEeprom()
{
	if (loadState(EEPROM_FLAG) == EEPROM_FLAG_MAGIC)
	{
		// Eeprom contents are valid
    	Sprintln(F("Read eeprom"));

		iRf24Channel 		= loadState(EEPROM_CHANNEL);
		iRf24PaLevel 		= loadState(EEPROM_PA_LEVEL);
		iRf24DataRate 		= loadState(EEPROM_DATARATE);
#ifndef MY_GATEWAY_FEATURE
		iRf24PaLevelGw 		= loadState(EEPROM_PA_LEVEL_GW);
		iDestinationNode	= loadState(EEPROM_DESTINATION_NODE);
		iPayloadSize		= loadState(EEPROM_PAYLOAD_SIZE);
		iSetMsgRate			= loadState(EEPROM_MESSAGE_RATE);	 
#endif
	}
	else
	{
		// Eeprom contents are invalid: Load defaults & save to eeprom
		loadDefaults();
		saveEepromAndReset();
		// Never return here...
	}
}

void saveEeprom()
{
	Sprintln(F("Save eeprom"));

	saveState(EEPROM_CHANNEL, iRf24Channel);
	saveState(EEPROM_PA_LEVEL, iRf24PaLevel);
	saveState(EEPROM_DATARATE, iRf24DataRate);
#ifndef MY_GATEWAY_FEATURE
	saveState(EEPROM_PA_LEVEL_GW, iRf24PaLevelGw);
	saveState(EEPROM_DESTINATION_NODE, iDestinationNode);
	saveState(EEPROM_PAYLOAD_SIZE, iPayloadSize);
	saveState(EEPROM_MESSAGE_RATE, iSetMsgRate);
#endif

	// Mark eeprom contents valid
	saveState(EEPROM_FLAG, EEPROM_FLAG_MAGIC);
}

void invalidateEeprom()
{
	Sprintln(F("Invalidate eeprom"));
	// Mark eeprom contents invalid
	saveState(EEPROM_FLAG, uint8_t(~ EEPROM_FLAG_MAGIC));
}

void saveEepromAndReset()
{
	saveEeprom();
	// Do a Soft Reset - This allows for the radio to correctly reload with the new settings from EEPROM					
	reset();
	// Never return here...
}

void reset() __attribute__((noreturn));
void reset()
{
	Sprintln(F("Reset"));
	Sflush();
	asm volatile ("  jmp 0");
	__builtin_unreachable();
}

#ifndef MY_GATEWAY_FEATURE
void serializeGwSettings( MyMessage& msg )
{
	const uint16_t packed = iRf24Channel*100 + iRf24PaLevelGw*10 + iRf24DataRate;
	msg.set( packed );
}
#endif

#ifdef MY_GATEWAY_FEATURE
void deserializeGwSettings(const MyMessage& msg )
{
    const uint16_t packed = msg.getUInt();

    // Extract the new Gateway settings
    iRf24Channel  = packed / 100U;
    iRf24PaLevel  = (packed / 10U) % 10;	// yes iRf24PaLevel and not iRf24PaLevelGw, as iRf24PaLevel is sent to nRF24
    iRf24DataRate = packed % 10;
}
#endif
