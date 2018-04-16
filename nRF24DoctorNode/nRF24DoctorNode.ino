/*
  PROJECT: 		MySensors nRF24 Doctor
  PROGRAMMER: 	Technovation (based on the work of AWI and Heizelmann)
  DATE: 		2018March27
  FILE: 		nRF24DoctorNode.ino

  Hardware: 	ATMega328p board
  MySensorsAPI: 2.2.0

  Summary:  	a portable nRF24 Radio Doctor to diagnose module performance

  Change log:
	2018/03/27	New Release, based on: https://forum.mysensors.org/topic/3984/nrf24l01-connection-quality-meter
	2018/04/13	Fixed some bugs in Current Measurement & display of currents
*/

//**** CONNECTIONS *****
#define BUTTON1_PIN A0
#define BUTTON2_PIN A1
#define MOSFET_2P2OHM_PIN A2
#define MOSFET_100OHM_PIN A3
#define CURRENT_PIN A5
#define TRIGGER_PIN A4			//Debugging purposes with scope

//**** DEBUG *****
#define LOCAL_DEBUG

#ifdef LOCAL_DEBUG
#define Sprint(a) (Serial.print(a))           	// macro as substitute for print, enable if no print wanted
#define Sprintln(a) (Serial.println(a))         
#else											// macro for "no" debug print
#define Sprint(a)                   			
#define Sprintln(a)                   			
#endif

//**** MySensors *****
//#define MY_DEBUG							// Enable debug prints to serial monitor
//#define MY_DEBUG_VERBOSE_RF24				// debug nrf24
#define MY_TRANSPORT_WAIT_READY_MS 10 		// [ms] Init timeout for gateway not reachable
#define MY_NODE_ID 250						// Set a high node ID, which typically will not yet be used in the network
#define MY_PARENT_NODE_IS_STATIC			// Fixed parent Node ID, else MySensors Transport will attempt automatic fix after successive failures...but we don't want that while diagnosing our connection
#define MY_PARENT_NODE_ID 0              	// Typically 0 for Gateway

#define DESTINATION_NODE 0                	// Default 0 = gateway, Settable in Menu
#define MY_BAUD_RATE 115200

//**** MySensors - Radio *****
#define MY_RADIO_NRF24                  	// Enable and select radio type attached

#define CHARACTER_DISPLAY

#include <SPI.h>
#include <MySensors.h>

#ifdef CHARACTER_DISPLAY
	//#include <LiquidCrystal_I2C.h>                // LCD display with I2C interface
	#include <LiquidCrystal.h>                      // LCD display with parallel interface
#else
	#include <Adafruit_GFX.h>
	#include <TFT_ILI9163C.h>
#endif

#include <OneButton.h> 							//from https://github.com/mathertel/OneButton/blob/master/examples/TwoButtons/TwoButtons.ino

//**** LCD *****
#ifdef CHARACTER_DISPLAY
	#define LCD_COLS 16
	#define LCD_ROWS 2
	//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  	// Set the LCD I2C address
	//LiquidCrystal_I2C lcd(0x27, 16, 2);  								// Set the LCD I2C address
	LiquidCrystal lcd(8, 7, 6, 5, 4, 3); 								// LCD with parallel interface
#else
	// Color definitions. BLACK & WHITE are defined in library.
	#define BLUE            (0x001F)
	#define RED             (0xF800)
	#define GREEN           (0x07E0)
	#define CYAN            (0x07FF)
	#define MAGENTA         (0xF81F)
	#define YELLOW          (0xFFE0) 

	#define COLOR_BG        (BLACK)
	#define COLOR_TEXT      (WHITE) 
	#define COLOR_UNIT      (CYAN) 
	#define COLOR_SETACT    (RED) 

	#define TFT_WIDTH       (128)
	#define TFT_HEIGHT      (128)

	#define TEXT_SIZE       (1)
	#define CHAR_HEIGHT     (8)
	#define CHAR_WIDTH      (8)

	#define LCD_COLS        ((TFT_WIDTH)/(CHAR_WIDTH))
	#define LCD_ROWS        (2)	// for now

	#define TEXT_HEIGHT(c)  ((c)*CHAR_HEIGHT)
	#define TEXT_WIDTH(c)   ((c)*CHAR_WIDTH)
	  
	#define TFT_PIN_CS      (10)
	#define TFT_PIN_DC      (4)
	static TFT_ILI9163C tft(TFT_PIN_CS, TFT_PIN_DC);
#endif

//**** BUTTONS *****
OneButton button1(BUTTON1_PIN, true); //PullUp, Activelow
OneButton button2(BUTTON2_PIN, true); //PullUp, Activelow

//**** EEPROM STORAGE LOCATIONS *****
#define EEPROM_FLAG 0
#define EEPROM_CHANNEL 1
#define EEPROM_PA_LEVEL 2
#define EEPROM_PA_LEVEL_GW 3
#define EEPROM_DATARATE 4
#define EEPROM_BASE_RADIO_ID 5
#define EEPROM_DESTINATION_NODE 6
#define EEPROM_SEND_REPEATS 7

//**** MySensors Messages ****
#define CHILD_ID_COUNTER 0
#define CHILD_ID_UPDATE_GATEWAY 0
MyMessage MsgCounter(CHILD_ID_COUNTER, V_CUSTOM);   				//Send Message Counter value
MyMessage MsgUpdateGateway(CHILD_ID_UPDATE_GATEWAY, V_CUSTOM);   	//Send value for Gateway settings: xxxyz (xxx = Channel, y = PaLevel, z = DataRate)
#define DELAY_BETWEEN_MESSAGES_MICROS 500000						//Normal Interval between messages

//**** Monitoring Constants&Variables ****
const int iMaxNumberOfMessages = 100 ;           					// Number of Messages Used for MA calculation
boolean bArrayFailedMessages[iMaxNumberOfMessages] = { 0 };     	// Array for moving average storage
boolean bArrayNAckMessages[iMaxNumberOfMessages] = { 0 };			// Array for moving average storage
uint16_t iNrFailedMessages = 0;            							// total of Failed Messages
uint16_t iNrNAckMessages = 0;              							// total of Not Acknowledged Messages
uint16_t iMessageCounter = 0;

//**** LCD MENU Constants&Variables ****
enum mode {STATE_RUN, STATE_RUN2, STATE_RUN3, STATE_CURLEVEL, STATE_SET_RESET, STATE_SET_DESTINATION_NODE, STATE_SET_CHANNEL, STATE_SET_BASE_RADIO_ID, STATE_SET_PALEVEL, STATE_SET_PALEVEL_GW, STATE_SET_DATARATE, STATE_ASK_GATEWAY, STATE_UPDATE_GATEWAY };
mode opState = STATE_RUN;											// Default Mode
boolean bDspRefresh = true;

//nRF24 Settings
const uint8_t iNrPaLevels = 4;
const char *pcPaLevelNames[iNrPaLevels] 		= { "MIN", "LOW", "HIGH", "MAX" };
const uint8_t iNrDataRates = 3;
const char *pcDataRateNames[iNrDataRates] 		= { "1MBPS", "2MBPS" , "250KBPS"};

//Define your available RF24_BASE_ID 
const uint8_t iNrBaseRadioIds = 4;
uint8_t RF24_BASE_ID_VAR[MY_RF24_ADDR_WIDTH]				= { 0x00,0xFC,0xE1,0xA8,0xA8 };		//Used to Store the active BASE_ID
const uint8_t RF24_BASE_ID_VAR1[MY_RF24_ADDR_WIDTH] 		= { 0x00,0xFC,0xE1,0xA8,0xA8 };
const uint8_t RF24_BASE_ID_VAR2[MY_RF24_ADDR_WIDTH] 		= { 0x00,0xA1,0xF3,0x09,0xB6 };
const uint8_t RF24_BASE_ID_VAR3[MY_RF24_ADDR_WIDTH] 		= { 0x00,0xAA,0xA5,0xC4,0xD9 };
const uint8_t RF24_BASE_ID_VAR4[MY_RF24_ADDR_WIDTH] 		= { 0x00,0xB1,0x47,0xEE,0x82 };

//Load Default Radio values
uint8_t iRf24Channel 		= MY_RF24_CHANNEL;
uint8_t iRf24PaLevel 		= MY_RF24_PA_LEVEL;		//PA Level for the Node
uint8_t iRf24PaLevelGw 		= MY_RF24_PA_LEVEL;		//PA Level for the Gateway
uint8_t iRf24DataRate 		= MY_RF24_DATARATE;
uint8_t iRf24BaseRadioId 	= 1;
uint8_t iDestinationNode	= DESTINATION_NODE;

//**** Timing ****
const uint8_t iNrTimeDelays = 10;
uint16_t iMessageIndexBuffer[iNrTimeDelays]={0};
unsigned long lTimeOfTransmit_us[iNrTimeDelays]={0};
unsigned long lTimeDelayBuffer_Destination_us[iNrTimeDelays]={0};
unsigned long lTimeDelayBuffer_FirstHop_us[iNrTimeDelays]={0};
uint16_t iMeanDelayFirstHop_ms = 0;
uint16_t iMaxDelayFirstHop_ms = 0;
uint16_t iMeanDelayDestination_ms = 0;
uint16_t iMaxDelayDestination_ms = 0;

//**** Current Measurement ****
const uint8_t iNrCurrentMeasurements 	= 255;	//Nr of measurements for averaging
uint8_t iPowerMode 		= 0;
const uint8_t iNrPowerModes = 2;
const char *pcPowerModeNames[iNrPowerModes] = { "SLEEP", "STANDBY" };

const float uAperBit1 	= 473.1889;	//uAperBit = ((Vref/1024)/R1)*1e6 = ((1.1/1024)/2.2)*1e6				
const float uAperBit2	= 10.41016;	//uAperBit = ((Vref/1024)/R2)*1e6 = ((1.1/1024)/100)*1e6
const float uAperBit3 	= 0.105047;	//uAperBit = ((Vref/1024)/R3)*1e6 = ((1.1/1024)/10000)*1e6
float SleepCurrent_uA 	= 0;
float StandbyCurrent_uA = 0;

//**** Remote Gateway Update ****
uint8_t iRetryGateway 	= 0;
bool bUpdateGateway 	= 0;
bool bAckGatewayUpdate 	= 0;
const uint8_t iNrGatwayRetryOptions = 3;
const char *pcGatewayRetryNames[iNrGatwayRetryOptions] 		= { "SKIP GATEWAY", "RETRY GATEWAY" , "CANCEL ALL"};


/*****************************************************************************/
/************************************ STARTUP ********************************/
/*****************************************************************************/
void before() {						//Initialization before the MySensors library starts up
	pinMode(CURRENT_PIN, INPUT);	//Analog Input for Current Usage Pin
	pinMode(TRIGGER_PIN, OUTPUT);
	pinMode(MOSFET_2P2OHM_PIN,OUTPUT);
	pinMode(MOSFET_100OHM_PIN,OUTPUT);
	digitalWrite(MOSFET_2P2OHM_PIN,HIGH);
	digitalWrite(TRIGGER_PIN,LOW);
	analogReference(INTERNAL);

	//****  LCD *****
	#ifdef CHARACTER_DISPLAY
		//  Wire.begin();  // I2C
		lcd.begin(LCD_COLS, LCD_ROWS);
		//lcd.setBacklight(HIGH);
	#else
		tft.begin();
		tft.setRotation(1);
      	tft.setTextColor(COLOR_TEXT, COLOR_BG);
      	tft.setTextSize(TEXT_SIZE);
	#endif

	//****  BUTTON STATES *****
	button1.attachClick(onButton1Pressed);
	button1.attachLongPressStart(onButton1LongPressed);
	button2.attachClick(onButton2Pressed);
	button2.attachDuringLongPress(onButton2Hold);

	//****  RELOAD SETTINGS FROM EEPROM *****
	if (loadState(EEPROM_FLAG) == 0xFF) {
		//RELOAD SETTINGS 
		LoadStatesFromEEPROM();
		switch (iRf24BaseRadioId) {
			case 1:
				memcpy(RF24_BASE_ID_VAR,RF24_BASE_ID_VAR1,sizeof(RF24_BASE_ID_VAR));
				break;
			case 2:
				memcpy(RF24_BASE_ID_VAR,RF24_BASE_ID_VAR2,sizeof(RF24_BASE_ID_VAR));
				break;				
			case 3:
				memcpy(RF24_BASE_ID_VAR,RF24_BASE_ID_VAR3,sizeof(RF24_BASE_ID_VAR));
				break;
			case 4:
				memcpy(RF24_BASE_ID_VAR,RF24_BASE_ID_VAR4,sizeof(RF24_BASE_ID_VAR));
				break;
		}		
	} else {
		//LOAD DEFAULT IN CASE OF CLEAN EEPROM
		saveState(EEPROM_FLAG, 0xFF);
		SaveStatesToEEPROM();
	}
}

void setup() {
	loadNewRadioSettings();	//Load the Radio Settings as they are stored in EEPROM
}

void presentation() {
	sendSketchInfo(F("nRF24_Doctor_N250"), F("1.1"));
	present(CHILD_ID_COUNTER, S_CUSTOM) ;  // "CUSTOM" counter
}

/*****************************************************************************/
/*********************************** MAIN LOOP *******************************/
/*****************************************************************************/
void loop() {
	
	/************ Update Button State ************/
	ButtonTick();

	/************ Update LCD ************/
	if (bDspRefresh) LCD_local_display();	
	
	/************ PROCESS DIFFERENT OPERATIONAL STATES ************/
	// RUN, RUN2,RUN3: Are all similar with respect to processing, but show different results on the LCD
	static unsigned long lLastTransmit = 0;
	switch (opState) {
		case STATE_RUN: case STATE_RUN2: case STATE_RUN3:{
			// Transmit Message
			transmit();	
			lLastTransmit = micros();			

			getMeanAndMaxFromArray(&iMeanDelayFirstHop_ms,&iMaxDelayFirstHop_ms,lTimeDelayBuffer_FirstHop_us,iNrTimeDelays);
			getMeanAndMaxFromArray(&iMeanDelayDestination_ms,&iMaxDelayDestination_ms,lTimeDelayBuffer_Destination_us,iNrTimeDelays);

			bDspRefresh = true;
			break;
		}
		case STATE_CURLEVEL:{
			//First round: Measure "iNrCurrentMeasurements" the Current and average in "Standby Mode", then do the same for "Sleep mode"			
			iPowerMode = !iPowerMode;			//Toggle between modes: "Standby" & "Sleep"
			if (iPowerMode == 1){				//Mode: Standby
				transmit();
				lLastTransmit = micros();
				wait(50);
				digitalWrite(TRIGGER_PIN,HIGH);		//Debugging purposes with scope
				StandbyCurrent_uA = uAperBit1*GetAvgADCBits(iNrCurrentMeasurements);
				digitalWrite(TRIGGER_PIN,LOW);		//Debugging purposes with scope
			}
			else{							//Mode: Sleep
				transportDisable();
				SleepCurrent_uA = uAperBit1*GetAvgADCBits(iNrCurrentMeasurements);
				Sprint(F("uAperBit1:"));Sprintln(SleepCurrent_uA);
				if (SleepCurrent_uA < 10000){
					//Set Higher Sensitivity: uAperBit2
					digitalWrite(MOSFET_2P2OHM_PIN,LOW);
					digitalWrite(MOSFET_100OHM_PIN,HIGH);
					delay(10);		//Gate charge time and settle time, don't use wait as it will prevent the radio from sleep
					SleepCurrent_uA = uAperBit2*GetAvgADCBits(iNrCurrentMeasurements);
					Sprint(F("uAperBit2:"));Sprintln(SleepCurrent_uA);
				}
				if (SleepCurrent_uA < 100){
					//Set Higher Sensitivity: uAperBit3
					digitalWrite(MOSFET_2P2OHM_PIN,LOW);
					digitalWrite(MOSFET_100OHM_PIN,LOW);
					delay(10);		//Gate charge time and settle time, don't use wait as it will prevent the radio from sleep
					SleepCurrent_uA = uAperBit3*GetAvgADCBits(iNrCurrentMeasurements);
					Sprint(F("uAperBit3:"));Sprintln(SleepCurrent_uA);
				}
				//Restore standby power state
				digitalWrite(MOSFET_2P2OHM_PIN,HIGH);	//Enable 2.2Ohm
				digitalWrite(MOSFET_100OHM_PIN,LOW);
				transportStandBy();transmit();
				lLastTransmit = micros();
			}
			bDspRefresh = true;
			break;
		}		
		default:{
			lLastTransmit = micros()-DELAY_BETWEEN_MESSAGES_MICROS;
			wait(25); // To limit LCD update rate
			break;
		}	
	}
	
	//Wait before sending next message
	while ((micros()- lLastTransmit) < DELAY_BETWEEN_MESSAGES_MICROS){// wait for things to settle and ack's to arrive
		wait(1);
		ButtonTick();	//Check for button changes....we must keep responsive
	}
}

/*****************************************************************************/
/************************ RECEIVE & TRANSMIT FUNCTIONS ***********************/
/*****************************************************************************/
void receive(const MyMessage &message) {
	if (message.isAck() == 1 && message.type == V_CUSTOM && message.sensor==CHILD_ID_COUNTER){	//Acknowledge message & of correct type
		uint16_t iNewMessage = message.getUInt();           // get received value
		uint16_t iIndexInArray = iNewMessage % iMaxNumberOfMessages;
		bArrayNAckMessages[iIndexInArray] = 0; 			// set corresponding flag to received.
		
		//Check Message (Round trip) Delay
		uint8_t iIndexInTimeArray = IndexOfValueInArray(iNewMessage, iMessageIndexBuffer, iNrTimeDelays); //Look-up if message is present in MessageIndexBuffer for delay calculation
		if ((iIndexInTimeArray != 255) && iIndexInTimeArray <=iNrTimeDelays){
			lTimeDelayBuffer_Destination_us[iIndexInTimeArray] = micros()-lTimeOfTransmit_us[iIndexInTimeArray];
		}
		iNrNAckMessages--;	//Received an Acknowledge Message (so one less No Ack)
	}
	
	//Gateway Update Acknowledge - if we have received this we can "safely" apply the new settings to this node
	if (message.isAck() == 1 && message.type == V_CUSTOM && message.sensor==CHILD_ID_UPDATE_GATEWAY){	//Acknowledge message & of correct type & Sensor
		bAckGatewayUpdate = 1;
	}
}

void transmit() {
	static int iIndexInArrayFailedMessages  = 0 ;
	static int iIndexInArrayTimeMessages  = 0 ;	

	iMessageCounter++;
	//Cyclic Index counters of arrays
	iIndexInArrayFailedMessages = iMessageCounter % iMaxNumberOfMessages;
	iIndexInArrayTimeMessages 	= iMessageCounter % iNrTimeDelays;

	bArrayNAckMessages[iIndexInArrayFailedMessages] = 1; 			// set corresponding flag to "Not Received Yet"

	//Prepare time stamp logging for transmit
	lTimeDelayBuffer_Destination_us[iIndexInArrayTimeMessages] = 0; 		// Clear Buffer value, new value will be written when message is received
	iMessageIndexBuffer[iIndexInArrayTimeMessages]=iMessageCounter;		// To link the Time Stamp to the correct message when we receive the acknowledge
	iNrNAckMessages++;													// Add one to the Not Acknowledged Message counter and remove it again if/when it is received.
	lTimeOfTransmit_us[iIndexInArrayTimeMessages] = micros();
	
	//Transmit message with software ack request (returned in "receive function"), the boolean returned here is a Hardware hop-to-hop Ack
	boolean success = send(MsgCounter.setDestination(iDestinationNode).set(iMessageCounter), true);
	if (!success) {
		lTimeDelayBuffer_FirstHop_us[iIndexInArrayTimeMessages] = 0;	//It failed, so I can't use it to determine a First Hop Delay (i.e. it is "infinite" delay as it failed)
		bArrayFailedMessages[iIndexInArrayFailedMessages] = true;	//Log it as a failed message (for rolling average)
		iNrFailedMessages++ ;
	}
	else{
		lTimeDelayBuffer_FirstHop_us[iIndexInArrayTimeMessages] = micros() - lTimeOfTransmit_us[iIndexInArrayTimeMessages];	//Log First Hop Delay in buffer
		bArrayFailedMessages[iIndexInArrayFailedMessages] = false;	//Log it as a not-failed = succesful message (for rolling average)
	}
}

/********************************************************************************/
/************************ CONFIGURE nRF24 RADIO FUNCTIONS ***********************/
/********************************************************************************/
void loadNewRadioSettings() {
	ClearStorageAndCounters();
	uint8_t iTempVar0 = RF24_BASE_ID_VAR[0];
	uint8_t rfsetup = ( ((iRf24DataRate & 0b10 ) << 4) | ((iRf24DataRate & 0b01 ) << 3) | (iRf24PaLevel << 1) ) + 1;		//!< RF24_RF_SETUP, +1 for Si24R1 and LNA

	
	/*
	///ADDED
	// CRC and power up
	RF24_setRFConfiguration(RF24_CONFIGURATION | _BV(RF24_PWR_UP)) ;
	// settle >2ms
	delay(5);
	// set address width
	RF24_setAddressWidth(MY_RF24_ADDR_WIDTH);
	// auto retransmit delay 1500us, auto retransmit count 15
	RF24_setRetries(RF24_SET_ARD, RF24_SET_ARC);
	///ADD STOP
	*/	
	RF24_setChannel(iRf24Channel);
	RF24_setRFSetup(rfsetup);
	RF24_enableFeatures();
	/*
	//ADDED
		// enable ACK payload and dynamic payload
	RF24_setFeature(RF24_FEATURE);
	// sanity check (this function is P/non-P independent)
	if (!RF24_sanityCheck()) {
		RF24_DEBUG(PSTR("!RF24:INIT:SANCHK FAIL\n")); // sanity check failed, check wiring or replace module
		return false;
	}
	// enable broadcasting pipe
	RF24_setPipe(_BV(RF24_ERX_P0 + RF24_BROADCAST_PIPE));
	// disable AA on all pipes, activate when node pipe set
	RF24_setAutoACK(0x00);
	// enable dynamic payloads on used pipes
	RF24_setDynamicPayload(_BV(RF24_DPL_P0 + RF24_BROADCAST_PIPE) | _BV(RF24_DPL_P0));
	// listen to broadcast pipe
	//ADDED STOP
	*/
	
	RF24_BASE_ID_VAR[0] = RF24_BROADCAST_ADDRESS;
	RF24_setPipeAddress(RF24_REG_RX_ADDR_P0 + RF24_BROADCAST_PIPE, (uint8_t*)&RF24_BASE_ID_VAR,
	                    RF24_BROADCAST_PIPE > 1 ? 1 : MY_RF24_ADDR_WIDTH);
	RF24_setPipeAddress(RF24_REG_RX_ADDR_P0, (uint8_t*)&RF24_BASE_ID_VAR, MY_RF24_ADDR_WIDTH);
	RF24_setPipeAddress(RF24_REG_TX_ADDR, (uint8_t*)&RF24_BASE_ID_VAR, MY_RF24_ADDR_WIDTH);
	
	/*
	//ADDED
	// reset FIFO
	RF24_flushRX();
	RF24_flushTX();
	// reset interrupts
	RF24_setStatus(_BV(RF24_TX_DS) | _BV(RF24_MAX_RT) | _BV(RF24_RX_DR));
	//ADDED STOP
	*/
	
	RF24_BASE_ID_VAR[0] = iTempVar0;
	
	print_LCD_line("nRF24 DOCTOR",  1, 1);
	print_LCD_line("Connecting...", 2, 1);

	transportWaitUntilReady(10000);		// Give it 10[s] to connect, else continue to allow user to set new connection settings
}

void loadNewRadioSettingsGateway() {
	switch (iRetryGateway){
		case 0:	//Skip Gateway update (= only update this Node)
			//Store new values to EEPROM & Restart the program
			SaveStatesToEEPROM();
			asm volatile ("  jmp 0");	//Do a Soft Reset - This allows for the radio to correctly reload with the new settings from EEPROM
			break;				
		case 1:	{
			//(Re-)try Gateway update
			uint16_t iMessageToGateway = iRf24Channel*100+iRf24PaLevelGw*10+iRf24DataRate;
			boolean success = send(MsgUpdateGateway.setDestination(0).setSensor(250).set(iMessageToGateway), true);		//Transmit message with software ack request (returned in "receive function")
			int nRepeats = 0;
			while (!success && nRepeats<10) {	//Re-try
				nRepeats++;
				success = send(MsgUpdateGateway.setDestination(0).set(iMessageToGateway), true);
			}
			wait(2000);	//wait for ACK from Gateway
			if (bAckGatewayUpdate){
				//Store new values to EEPROM & Restart the program
				SaveStatesToEEPROM();
				asm volatile ("  jmp 0");	//Do a Soft Reset - This allows for the radio to correctly reload with the new settings from EEPROM					
			}
			break;
		}					
		case 2:	//Cancel All
			opState = STATE_RUN;
			LoadStatesFromEEPROM();
			bDspRefresh = true;					
			break;
	}
}
/*****************************************************************/
/************************ EEPROM FUNCTIONS ***********************/
/*****************************************************************/
void LoadStatesFromEEPROM() {
	iRf24Channel 		= loadState(EEPROM_CHANNEL);
	iRf24PaLevel 		= loadState(EEPROM_PA_LEVEL);
	iRf24PaLevelGw 		= loadState(EEPROM_PA_LEVEL_GW);
	iRf24DataRate 		= loadState(EEPROM_DATARATE);
	iRf24BaseRadioId	= loadState(EEPROM_BASE_RADIO_ID);
	iDestinationNode 	= loadState(EEPROM_DESTINATION_NODE);
}

void SaveStatesToEEPROM() {
	saveState(EEPROM_CHANNEL, iRf24Channel);
	saveState(EEPROM_PA_LEVEL, iRf24PaLevel);
	saveState(EEPROM_PA_LEVEL_GW, iRf24PaLevelGw);
	saveState(EEPROM_DATARATE, iRf24DataRate);
	saveState(EEPROM_BASE_RADIO_ID, iRf24BaseRadioId);
	saveState(EEPROM_DESTINATION_NODE, iDestinationNode);
}

/*****************************************************************/
/************************ BUTTON FUNCTIONS ***********************/
/*****************************************************************/
void ButtonTick() {
	//Check Button States (should be done Frequently
	button1.tick();
	button2.tick();
}

void onButton1Pressed() {			//Scroll through the menu items
	LCD_clear();
	bDspRefresh = true;
	switch (opState) {
		case STATE_RUN:
			opState = STATE_RUN2;
			break;
		case STATE_RUN2:
			opState = STATE_RUN3;
			break;			
		case STATE_RUN3:
			opState = STATE_CURLEVEL;
			break;			
		case STATE_CURLEVEL:
			opState = STATE_SET_RESET;
			break;
		case STATE_SET_RESET:
			opState = STATE_SET_DESTINATION_NODE;
			break;
		case STATE_SET_DESTINATION_NODE:
			opState = STATE_SET_CHANNEL;
			break;
		case STATE_SET_CHANNEL:
			opState = STATE_SET_PALEVEL;
			break;
		case STATE_SET_PALEVEL:
			opState = STATE_SET_PALEVEL_GW;
			break;
		case STATE_SET_PALEVEL_GW:
			opState = STATE_SET_DATARATE;
			break;		
		case STATE_SET_DATARATE:
			opState = STATE_SET_BASE_RADIO_ID;
			break;
		case STATE_SET_BASE_RADIO_ID:
			opState = STATE_ASK_GATEWAY;
			break;
		case STATE_ASK_GATEWAY:
			opState = STATE_UPDATE_GATEWAY;
			loadNewRadioSettingsGateway();
			break;
		case STATE_UPDATE_GATEWAY:
			loadNewRadioSettingsGateway();
			break;		
	}
}

void onButton2Pressed() {	//Apply/Select change (if available). In Viewing windows: toggle (reversed)
	bDspRefresh = true;
	switch (opState) {
		case STATE_RUN:
			//Do nothing
			break;
		case STATE_RUN2:
			//Do nothing
			break;
		case STATE_RUN3:
			//Do nothing
			break;			
		case STATE_CURLEVEL:
			//Do nothing
			break;
		case STATE_SET_RESET:
			ClearStorageAndCounters();
			opState = STATE_RUN;
			break;
		//Select Radio changes:
		case STATE_SET_DESTINATION_NODE:
			iDestinationNode++;
			break;				
		case STATE_SET_CHANNEL:
			iRf24Channel++; if (iRf24Channel > 125) iRf24Channel = 0;
			break;
		case STATE_SET_PALEVEL:
			iRf24PaLevel++; if (iRf24PaLevel > (iNrPaLevels-1)) iRf24PaLevel = 0;
			break;
		case STATE_SET_PALEVEL_GW:
			iRf24PaLevelGw++; if (iRf24PaLevelGw > (iNrPaLevels-1)) iRf24PaLevelGw = 0;
			break;			
		case STATE_SET_DATARATE:
			iRf24DataRate++; if (iRf24DataRate > (iNrDataRates-1)) iRf24DataRate = 0;
			break;			
		case STATE_SET_BASE_RADIO_ID:
			iRf24BaseRadioId++; if (iRf24BaseRadioId > iNrBaseRadioIds) iRf24BaseRadioId = 1;
			switch (iRf24BaseRadioId) {
				case 1:
					memcpy(RF24_BASE_ID_VAR,RF24_BASE_ID_VAR1,sizeof(RF24_BASE_ID_VAR));
					break;
				case 2:
					memcpy(RF24_BASE_ID_VAR,RF24_BASE_ID_VAR2,sizeof(RF24_BASE_ID_VAR));
					break;				
				case 3:
					memcpy(RF24_BASE_ID_VAR,RF24_BASE_ID_VAR3,sizeof(RF24_BASE_ID_VAR));
					break;
				case 4:
					memcpy(RF24_BASE_ID_VAR,RF24_BASE_ID_VAR4,sizeof(RF24_BASE_ID_VAR));
					break;
			}
			break;			
		case STATE_ASK_GATEWAY:
			bUpdateGateway= !bUpdateGateway;
			if (bUpdateGateway){
				iRetryGateway = 1;
			}
			else{
				iRetryGateway = 0;
			}
			break;
		case STATE_UPDATE_GATEWAY:
			iRetryGateway++; if (iRetryGateway > (iNrGatwayRetryOptions-1)) iRetryGateway = 0;
/* 			switch(iRetryGateway){
				case 0: 
					bUpdateGateway = 0;
					break;
				case 1:
					bUpdateGateway = 1;
					break;
				case 2:
					bUpdateGateway = 0;
					break;				
			} */
			break;
	}
}

void onButton1LongPressed() {
	// Return to normal RUN state without changing any settings (recall last set from EEPROM)
	opState = STATE_RUN;
	LoadStatesFromEEPROM();
	bDspRefresh = true;
}

void onButton2Hold() {	//Scroll through numbers quickly
	switch (opState) {
		case STATE_SET_CHANNEL:
			iRf24Channel++; if (iRf24Channel > 125) iRf24Channel = 0;
			bDspRefresh = true;
			break;
		case STATE_SET_DESTINATION_NODE:
			iDestinationNode++;
			bDspRefresh = true;
			break;
		default:
			break;
	}		
}

/*****************************************************************/
/**************** ARRAY PROCESSING FUNCTIONS *********************/
/*****************************************************************/

uint8_t IndexOfValueInArray(uint16_t val, uint16_t *array, uint8_t size){
	// Find the (first) array element which equals val and return the index.
	// If val not found in the array return 255
    for (int i=0; i < size; i++) {
        if (array[i] == val){
			return i;
		}
    }
    return 255;	//Not Found
}

int GetNrOfTrueValuesInArray(boolean countArray[], int size) {
	// Calculate number of TRUE values in array
	int Counter = 0 ;
	for (int i = 0 ; i < size ; i++) {Counter += countArray[i];}
	return Counter;
}

void ClearStorageAndCounters() {
	for (int n = 0; n < iMaxNumberOfMessages; n++){
		bArrayFailedMessages[n] = 0;
		bArrayNAckMessages[n] = 0;
	}
	iNrNAckMessages = iMessageCounter = iNrFailedMessages = 0;
	bAckGatewayUpdate = 0;
}

void getMeanAndMaxFromArray(uint16_t *mean_value, uint16_t *max_value, unsigned long *buffer, uint8_t size) {
	//Note: excluding 0 values from mean calculation
	uint8_t iNrOfSamples=0;		//max equals size
	unsigned long lMaxValue=0;	//max Array value
	float sum=0;
	for (int i=0; i < size; i++)
	{
		if (buffer[i] != 0){
			sum 		= sum + (float)buffer[i];	
			lMaxValue	= max(lMaxValue,buffer[i]);
			iNrOfSamples++;
		}
	}
	if (iNrOfSamples !=0){
		*mean_value 	= (uint16_t) ((sum / (float)iNrOfSamples)/1000);
		*max_value		= (uint16_t) (lMaxValue/1000L);
	}
	else {
		*mean_value = 65535;	//INF identifier
		*max_value 	= 65535;	//INF identifier
	}	
}

float GetAvgADCBits(int iNrSamples) {
	float sum = 0;
	for (int i=0; i < iNrSamples; i++) {
		sum = sum + analogRead(CURRENT_PIN);
	}
	return ((float)sum/(float)iNrSamples);
}


/*****************************************************************/
/************************* LCD FUNCTIONS *************************/
/*****************************************************************/

void print_LCD_line(const char *string, int row, int col) {
	col--;
	row--;
#ifdef CHARACTER_DISPLAY
	lcd.setCursor(col,row);
	lcd.print(string);
#else
	tft.setCursor(TEXT_WIDTH(col), TEXT_HEIGHT(row)); 
	tft.print(string);
#endif
}

void LCD_clear() {
#ifdef CHARACTER_DISPLAY
	lcd.clear();
#else
	tft.fillScreen(COLOR_BG);
#endif
}

void LCD_local_display(void) {
	static mode prevOpState = STATE_RUN;	//Remember previous state to allow for partial LCD updates
	char buf[LCD_COLS+1];
	bDspRefresh = false;
	
	switch (opState) {
		case STATE_RUN:
		{
			snprintf_P(buf, sizeof buf, PSTR("P%-3dFAIL%4d%3d%%"), MY_PARENT_NODE_ID, iNrFailedMessages, GetNrOfTrueValuesInArray(bArrayFailedMessages, iMaxNumberOfMessages));
			print_LCD_line(buf,1, 1);
			snprintf_P(buf, sizeof buf, PSTR("D%-3dNACK%4d%3d%%"), iDestinationNode , iNrNAckMessages, GetNrOfTrueValuesInArray(bArrayNAckMessages, iMaxNumberOfMessages));
			print_LCD_line(buf,2, 1);
			break;
		}
		case STATE_RUN2:
		{
			if (iMaxDelayFirstHop_ms>9999){
				snprintf_P(buf, sizeof buf, PSTR("HOP1 dTmax   INF"));
			}
			else{
				snprintf_P(buf, sizeof buf, PSTR("HOP1 dTmax%4dms"),iMaxDelayFirstHop_ms);
			}
			print_LCD_line(buf,1, 1);
			if (iMaxDelayDestination_ms>9999){
				snprintf_P(buf, sizeof buf, PSTR("D%-3d dTmax   INF"),iDestinationNode,iMaxDelayDestination_ms);
			}
			else{
				snprintf_P(buf, sizeof buf, PSTR("D%-3d dTmax%4dms"),iDestinationNode,iMaxDelayDestination_ms);
			}
			print_LCD_line(buf,2, 1);
			break;
		}
		case STATE_RUN3:
		{
			snprintf_P(buf, sizeof buf, PSTR("MESSAGE COUNT:  "));
			print_LCD_line(buf,1, 1);
			snprintf_P(buf, sizeof buf, PSTR("           %5d"),iMessageCounter);
			print_LCD_line(buf,2, 1);
			break;
		}		
		case STATE_CURLEVEL:
		{
			float fCurrent_uA = 0;
			if (iPowerMode == 1){
				fCurrent_uA = StandbyCurrent_uA;
			}
			else{
				fCurrent_uA = SleepCurrent_uA;	
			}
			Sprint(F("iPowerMode:"));Sprintln(iPowerMode);
			Sprint(F("fCurrent_uA:"));Sprintln(fCurrent_uA);
			
			if (fCurrent_uA > 1000){
				int Current_mA = (int)(fCurrent_uA/1000);
				if (Current_mA>=300){
					snprintf_P(buf, sizeof buf, PSTR("%s[mA]= ERR"),pcPowerModeNames[iPowerMode],Current_mA);
				}
				else if (Current_mA>=100){
					snprintf_P(buf, sizeof buf, PSTR("%s[mA]=%4d"),pcPowerModeNames[iPowerMode],Current_mA);
				}
				else if (Current_mA>=10){
					int iDecCurrent = (int)(((fCurrent_uA/1000)-(float)Current_mA)*10+0.5);if (iDecCurrent>=10){iDecCurrent=iDecCurrent-10;Current_mA +=1;}
					snprintf_P(buf, sizeof buf, PSTR("%s[mA]=%2d.%1d"),pcPowerModeNames[iPowerMode],Current_mA,iDecCurrent);
				}
				else {				
					int iDecCurrent = (int)(((fCurrent_uA/1000)-(float)Current_mA)*100+0.5);if (iDecCurrent >=100){iDecCurrent=iDecCurrent-100;Current_mA +=1;}
					snprintf_P(buf, sizeof buf, PSTR("%s[mA]=%1d.%2d"),pcPowerModeNames[iPowerMode],Current_mA,iDecCurrent);
				}
			}
			else if (fCurrent_uA < 100){		
				int iCurrent_uA = (int)fCurrent_uA;
				int iDecCurrent_uA = (int)((fCurrent_uA-(float)iCurrent_uA)*10+0.5); if (iDecCurrent_uA >= 10){iDecCurrent_uA=iDecCurrent_uA-10;iCurrent_uA +=1;}
					snprintf_P(buf, sizeof buf, PSTR("%s[uA]=%2d.%1d"),pcPowerModeNames[iPowerMode],iCurrent_uA,iDecCurrent_uA);
				}
			else{
				snprintf_P(buf, sizeof buf, PSTR("%s[uA]=%4d"),pcPowerModeNames[iPowerMode],(int)fCurrent_uA);
			}
			if (iPowerMode == 1){
				print_LCD_line(buf,1, 1);				
			}
			else{
				print_LCD_line(buf,2, 1);
			}
			break;
		}
		case STATE_SET_RESET:
		{
			snprintf_P(buf, sizeof buf, PSTR("RESET BUFFERS?"));
			print_LCD_line(buf,1, 1);
			break;
		}	
		case STATE_SET_DESTINATION_NODE:
		{
			if(opState==prevOpState){
				char buf2[4];		// why not re-use buf?
				snprintf(buf2, sizeof buf2, "%3d", iDestinationNode);
				print_LCD_line(buf2, 1, 14);
			}
			else{
				snprintf_P(buf, sizeof buf, PSTR("DEST. NODE = %3d"), iDestinationNode);
				print_LCD_line(buf, 1, 1);
			}
			break;
		}
		case STATE_SET_CHANNEL:
		{
			if(opState==prevOpState){
				char buf2[4];		// why not re-use buf?
				snprintf(buf2, sizeof buf2, "%3d", iRf24Channel);
				print_LCD_line(buf2, 1, 14);
			}
			else{
				snprintf_P(buf, sizeof buf, PSTR("CHANNEL NR = %3d"), iRf24Channel);
				print_LCD_line(buf, 1, 1);
			}
			break;
		}
		case STATE_SET_PALEVEL:
		{
			LCD_clear();
			snprintf_P(buf, sizeof buf, PSTR("NODE"));
			print_LCD_line(buf, 1, 1);
			snprintf_P(buf, sizeof buf, PSTR("PA Level = %s"), pcPaLevelNames[iRf24PaLevel]);
			print_LCD_line(buf, 2, 1);
			break;
		}
		case STATE_SET_PALEVEL_GW:
		{
			LCD_clear();
			snprintf_P(buf, sizeof buf, PSTR("GATEWAY"));
			print_LCD_line(buf, 1, 1);
			snprintf_P(buf, sizeof buf, PSTR("PA Level = %s"), pcPaLevelNames[iRf24PaLevelGw]);
			print_LCD_line(buf, 2, 1);
			break;
		}
		case STATE_SET_DATARATE:
		{
			LCD_clear();
			snprintf_P(buf, sizeof buf, PSTR("DataRate=%s"), pcDataRateNames[iRf24DataRate]);
			print_LCD_line(buf, 1, 1);
			break;
		}
		case STATE_SET_BASE_RADIO_ID:
		{
			LCD_clear();
			snprintf_P(buf, sizeof buf, PSTR("Base Radio ID=  "));
			print_LCD_line(buf, 1, 1);
			snprintf_P(buf, sizeof buf, PSTR("%02X:%0.2X:%0.2X:%0.2X:%0.2X"),RF24_BASE_ID_VAR[0],RF24_BASE_ID_VAR[1],RF24_BASE_ID_VAR[2],RF24_BASE_ID_VAR[3],RF24_BASE_ID_VAR[4]);
			print_LCD_line(buf, 2, 1);
			break;
		}		
		case STATE_ASK_GATEWAY:
		{
			LCD_clear();
			snprintf_P(buf, sizeof buf, PSTR("UPDATE GATEWAY?:"));
			print_LCD_line(buf, 1, 1);
			if (bUpdateGateway){
				snprintf_P(buf, sizeof buf, PSTR("YES"));
				print_LCD_line(buf, 2, 14);
			}
			else{
				snprintf_P(buf, sizeof buf, PSTR("NO"));
				print_LCD_line(buf, 2, 15);
			}
			break;
		}			
		case STATE_UPDATE_GATEWAY:
		{
			LCD_clear();
			snprintf_P(buf, sizeof buf, PSTR("FAILED GATEWAY!"));
			print_LCD_line(buf, 1, 1);
			snprintf_P(buf, sizeof buf, PSTR("%s"),pcGatewayRetryNames[iRetryGateway]);
			print_LCD_line(buf, 2, 1);
			break;								
		}	
	}
	prevOpState = opState;
}
