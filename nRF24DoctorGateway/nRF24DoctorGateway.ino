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


// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95
#define MY_RF24_IRQ_PIN (2)
#define MY_RX_MESSAGE_BUFFER_FEATURE

// Set LOW transmit power level as default, if you have an amplified NRF-module and
// power your radio separately with a good regulator you can turn up PA level.
#define MY_RF24_PA_LEVEL RF24_PA_LOW

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
#define DELAY_BETWEEN_RADIO_SETTINGS_PRINT 20000              //Print Radio Settings to Serial Monitor every x[ms]
#define MY_TRANSPORT_SANITY_CHECK_INTERVAL_MS 3000000000      //To Prevent the MySensors library from resetting back to default radio settings
#include <MySensors.h>

#define CHILD_ID_UPDATE_GATEWAY 250
MyMessage MsgUpdateGateway(CHILD_ID_UPDATE_GATEWAY, V_CUSTOM);    //Send value for Gateway settings: xxxyz (xxx = Channel, y = PaLevel, z = DataRate)
uint8_t iRf24Channel    = MY_RF24_CHANNEL;
uint8_t iRf24PaLevelGw    = MY_RF24_PA_LEVEL;
uint8_t iRf24DataRate     = MY_RF24_DATARATE;
bool bLoadNewRadioSettings  = 0;
const uint8_t iNrPaLevels = 4;
const char *pcPaLevelNames[iNrPaLevels]     = { "MIN", "LOW", "HIGH", "MAX" };
const uint8_t iNrDataRates = 3;
const char *pcDataRateNames[iNrDataRates]     = { "1MBPS", "2MBPS" , "250KBPS"};

//**** DEBUG *****
#define LOCAL_DEBUG

#ifdef LOCAL_DEBUG
#define Sprint(a) (Serial.print(a))             // macro as substitute for print, enable if no print wanted
#define Sprintln(a) (Serial.println(a))         
#else                     // macro for "no" debug print
#define Sprint(a)                         
#define Sprintln(a)                         
#endif

void setup()
{
	// Setup locally attached sensors
}

void presentation()
{
	// Present locally attached sensors
}

void loop()
{
	// Send locally attached sensor data here
  static unsigned long lLastRadioSettingsPrint = millis();
  wait(1);  
  //Set new Radio settings by nRF24 Doctor Node
  if (bLoadNewRadioSettings){
    bLoadNewRadioSettings = 0;
    loadNewRadioSettings();
  }
  //Regularly print the Radio Settings we are using
  if ((millis()- lLastRadioSettingsPrint) > DELAY_BETWEEN_RADIO_SETTINGS_PRINT){// wait for things to settle and ack's to arrive
    lLastRadioSettingsPrint = millis();
    PrintRadioSettings();
  }
 
}


/*****************************************************************************/
/************************ RECEIVE & TRANSMIT FUNCTIONS ***********************/
/*****************************************************************************/
void receive(const MyMessage &message) {
  if (message.type == V_CUSTOM && message.sensor==CHILD_ID_UPDATE_GATEWAY){ //Acknowledge message & of correct type & Sensor
    uint16_t iNewMessage = message.getUInt();           // get received value
    //Extract the new Gateway settings
    iRf24Channel  = (uint16_t)(iNewMessage/100U);
    iRf24PaLevelGw  = (uint16_t)((iNewMessage/10U)%10);
    iRf24DataRate   = (uint16_t)(iNewMessage%10);
    bLoadNewRadioSettings = 1;
    Sprintln("Received new Radio settings.");
  }
}

/********************************************************************************/
/************************ CONFIGURE nRF24 RADIO FUNCTIONS ***********************/
/********************************************************************************/
void loadNewRadioSettings() {
  uint8_t rfsetup = ( ((iRf24DataRate & 0b10 ) << 4) | ((iRf24DataRate & 0b01 ) << 3) | (iRf24PaLevelGw << 1) ) + 1;    //!< RF24_RF_SETUP, +1 for Si24R1 and LNA
  RF24_setChannel(iRf24Channel);
  RF24_setRFSetup(rfsetup);
  RF24_enableFeatures();
  Sprint("Updated Radio Settings!\t");
  PrintRadioSettings();
}

/********************************************************************************/
/******************************** SERIAL OUTPUT *********************************/
/********************************************************************************/
void PrintRadioSettings() {
  Sprint("Channel:");Sprint(iRf24Channel);Sprint("\t");
  Sprint("PaLevel:");Sprint(pcPaLevelNames[iRf24PaLevelGw]);Sprint("\t");
  Sprint("DataRate:");Sprint(pcDataRateNames[iRf24DataRate]);Sprintln("\t");
}

