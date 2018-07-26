/**
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
* Copyright (C) 2013-2015 Sensnology AB
* Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
*
* Documentation: http://www.mysensors.org
* Support Forum: http://forum.mysensors.org
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
*******************************
*
* DESCRIPTION
* The ArduinoGateway prints data received from sensors on the serial link.
* The gateway accepts input on seral which will be sent out on radio network.
*
* The GW code is designed for Arduino Nano 328p / 16MHz
*
* Wire connections (OPTIONAL):
* - Inclusion button should be connected between digital pin 3 and GND
* - RX/TX/ERR leds need to be connected between +5V (anode) and digital pin 6/5/4 with resistor 270-330R in a series
*
* LEDs (OPTIONAL):
* - To use the feature, uncomment any of the MY_DEFAULT_xx_LED_PINs
* - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
* - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
* - ERR (red) - fast blink on error during transmission error or recieve crc error
*
*/

// Enable debug prints to serial monitor
#define MY_DEBUG
//#define MY_DEBUG_VERBOSE_RF24

//**** DEBUG *****
#define LOCAL_DEBUG

#define BUTTON_RESTORE_DEFAULTS_PIN 			4    	// physical pin , use internal pullup
#define BUTTON_RESTORE_DEFAULTS_PRESS_TIME_MS 	(1000) 	// time the button to restore defaults has to be pressed to become active

// Tell MySensors where to find the radio parameters
extern uint8_t iRf24Channel;
extern uint8_t iRf24DataRate;
extern uint8_t iRf24PaLevel;
#define MY_RF24_CHANNEL			(iRf24Channel)
#define MY_RF24_DATARATE		(iRf24DataRate)
#define MY_RF24_PA_LEVEL		(iRf24PaLevel)

//extern uint8_t Rf24BaseId[];
//#define MY_RF24_BASE_RADIO_ID	Rf24BaseId[0],Rf24BaseId[1],Rf24BaseId[2],Rf24BaseId[3],Rf24BaseId[4]

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95
#define MY_RF24_IRQ_PIN               (2)
#define MY_RX_MESSAGE_BUFFER_FEATURE
#define MY_RX_MESSAGE_BUFFER_SIZE     (15)    // Default size of 20 is rather large and leads to memory warnings

// Set LOW transmit power level as default, if you have an amplified NRF-module and
// power your radio separately with a good regulator you can turn up PA level.
//#define MY_RF24_PA_LEVEL              RF24_PA_LOW

// Enable serial gateway
#define MY_GATEWAY_SERIAL

// Define a lower baud rate for Arduino's running on 8 MHz (Arduino Pro Mini 3.3V & SenseBender)
#if F_CPU == 8000000L
#define MY_BAUD_RATE 38400
#endif

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
//#define MY_INCLUSION_BUTTON_FEATURE

// Inverses behavior of inclusion button (if using external pullup)
//#define MY_INCLUSION_BUTTON_EXTERNAL_PULLUP

// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN  3

// Set blinking period
#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Inverses the behavior of leds
//#define MY_WITH_LEDS_BLINKING_INVERSE

// Flash leds on rx/tx/err
// Uncomment to override default HW configurations
//#define MY_DEFAULT_ERR_LED_PIN 4  // Error led pin
//#define MY_DEFAULT_RX_LED_PIN  6  // Receive led pin
//#define MY_DEFAULT_TX_LED_PIN  5  // the PCB, on board LED

//**** MySensors Messages -- ADDED FOR nRF24DoctorGateway Configuration ****
#define DELAY_BETWEEN_RADIO_SETTINGS_PRINT_MS (20000)           // Print Radio Settings to Serial Monitor every x[ms]
#define MY_TRANSPORT_SANITY_CHECK_INTERVAL_MS (3000000000)      // To Prevent the MySensors library from resetting back to default radio settings
#include <MySensors.h>
#include "shared/nRF24DoctorShared.h"
#include <Bounce2.h>	// button debounce  Download: https://github.com/thomasfredericks/Bounce2

static Bounce buttonRestoreDefaults = Bounce(); 

#define CHILD_ID_UPDATE_GATEWAY 250

void before()
{
	// Load radio settings from eeprom
	loadEeprom();
	logRadioSettings();
}

void setup()
{
	pinMode(BUTTON_RESTORE_DEFAULTS_PIN, INPUT_PULLUP); 
	buttonRestoreDefaults.attach(BUTTON_RESTORE_DEFAULTS_PIN);
	buttonRestoreDefaults.interval(BUTTON_RESTORE_DEFAULTS_PRESS_TIME_MS);
}

void loop()
{
	// Regularly print the Radio Settings we are using
	static unsigned long lastLogTimeMs = millis();
	const unsigned long nowMs = millis();
	if ((nowMs - lastLogTimeMs) > DELAY_BETWEEN_RADIO_SETTINGS_PRINT_MS)
	{
		lastLogTimeMs = nowMs;
		logRadioSettings();
	}

	buttonRestoreDefaults.update();
	if (buttonRestoreDefaults.read() == LOW)
	{
		// Button to restore default settings was pressed.
		Sprintln(F("Restore default radio settings"));
		loadDefaults();
		saveEeprom();
		Sprintln(F("Resetting..."));
		// Delay to prevent restoring defaults again, immediately after reset.
		delay(1000);
		reset();
	}
}

void receive( const MyMessage &message )
{
	if (message.type == V_CUSTOM && message.sensor == CHILD_ID_UPDATE_GATEWAY)
	{
		Sprintln(F("Received new Radio settings"));
		deserializeGwSettings( message );
		saveEeprom();
		Sprintln(F("Resetting..."));
		reset();
	}
}
