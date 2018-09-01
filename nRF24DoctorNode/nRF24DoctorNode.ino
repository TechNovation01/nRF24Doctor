/*
PROJECT: 		MySensors nRF24 Doctor
PROGRAMMER: 	Technovation & Yveaux
DATE: 		2018April18
FILE: 		nRF24DoctorNode.ino

Hardware: 	ATMega328p board
MySensorsAPI: 2.2.0

Summary:  	a portable nRF24 Radio Doctor to diagnose module performance

Change log:
	2018/03/27	New Release, based on: https://forum.mysensors.org/topic/3984/nrf24l01-connection-quality-meter
	2018/04/13	Fixed some bugs in Current Measurement & display of currents
	2018/04/17	Added support for TFT_ILI9163C display
*/

#define SKETCH_NAME_STRING    "nRF24_Doctor_N250"
#define SKETCH_VERSION_STRING "1.1"

//**** DEBUG *****
#define LOCAL_DEBUG
//#define MY_DEBUG							// Enable debug prints to serial monitor
//#define MY_DEBUG_VERBOSE_RF24

//**** CONNECTIONS *****
#define ENCODER_A_PIN       2		//Interrupt pin required for Encoder for optimal response
#define ENCODER_B_PIN       3		//Interrupt pin required for Encoder for optimal response
// Define either TRIGGER_PIN or LED_PIN, as they share a pin
//#define TRIGGER_PIN         4    	//Debugging purposes with scope
#define LED_PIN 	        4    	//LED to indicate status.
#define LCD_D7         		5    	
#define LCD_D6         		6
#define LCD_D5         		7
#define LCD_D4         		8
//PIN 9~13: NRF24 RADIO
#define LCD_ENABLE   		A0
#define LCD_RS   			A1
#define MOSFET_2P2OHM_PIN   A2
#define MOSFET_100OHM_PIN   A3
#define BUTTON_PIN          A4    	// physical pin , use internal pullup
#define CURRENT_PIN         A5
#define ADC_PIN_NR           5     	// A5, Match to CURRENT_PIN for configuring registers ADC


//**** MySensors *****
#include "shared/RadioConfig.h"

#define MY_SPLASH_SCREEN_DISABLED			// Disable splash screen (saves some flash)
#define MY_TRANSPORT_WAIT_READY_MS	(10)	// [ms] Init timeout for gateway not reachable
#define MY_NODE_ID					(250)	// Set a high node ID, which typically will not yet be used in the network
#define MY_PARENT_NODE_IS_STATIC			// Fixed parent Node ID, else MySensors Transport will attempt automatic fix after successive failures...but we don't want that while diagnosing our connection
#define MY_PARENT_NODE_ID			(0)		// Typically 0 for Gateway

#define MY_BAUD_RATE 115200
#define MY_INDICATION_HANDLER

#include <SPI.h>
#include <MySensors.h>
#include "shared/Generic.h"
#include "shared/RadioStorage.h"
#include <inttypes.h>

//**** LCD *****
#include <LiquidCrystal.h>                      // LCD display with parallel interface

#define LCD_COLS 16
#define LCD_ROWS 2
LiquidCrystal lcd(LCD_RS,LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7); // LCD with parallel interface


//**** LCD Menu *****
#include <LCDMenuLib2.h>	// Download 1.2.7: https://github.com/Jomelo/LCDMenuLib2

#define _LCDML_DISP_cols             LCD_COLS
#define _LCDML_DISP_rows             LCD_ROWS
#define _LCDML_DISP_cfg_scrollbar    1      // enable a scrollbar

const uint8_t scroll_bar[][8] = {
	{B10001, B10001, B10001, B10001, B10001, B10001, B10001, B10001}, // scrollbar top
	{B11111, B11111, B10001, B10001, B10001, B10001, B10001, B10001}, // scroll state 1
	{B10001, B10001, B11111, B11111, B10001, B10001, B10001, B10001}, // scroll state 2
	{B10001, B10001, B10001, B10001, B11111, B11111, B10001, B10001}, // scroll state 3
	{B10001, B10001, B10001, B10001, B10001, B10001, B11111, B11111}  // scrollbar bottom
}; 

void lcdml_menu_display();
void lcdml_menu_clear();
void lcdml_menu_control();

static LCDMenuLib2_menu LCDML_0 (255, 0, 0, NULL, NULL); // root menu element (do not change)
static LCDMenuLib2 LCDML(LCDML_0, LCD_ROWS, LCD_COLS, lcdml_menu_display, lcdml_menu_clear, lcdml_menu_control);

enum page { PAGE_STATISTICS, PAGE_TIMING, PAGE_MSGRATE, PAGE_COUNTERS, PAGE_TXRXPOWER, PAGE_SLEEPPOWER, PAGE_SCANNER };

// add            (id   prev_layer      new_num                      lang_char_array     callback_function)  
// addAdvanced    (id   prev_layer      new_num  condition           lang_char_array     callback_function    parameter (0-255)  menu function type )
//                                                                   "01234567890123"
LCDML_addAdvanced (0  , LCDML_0         , 1    , NULL              , "Statistics   >"  , menuPage           , PAGE_STATISTICS  , _LCDML_TYPE_default);
LCDML_addAdvanced (1  , LCDML_0         , 2    , NULL              , "Timing       >"  , menuPage           , PAGE_TIMING      , _LCDML_TYPE_default);
LCDML_addAdvanced (2  , LCDML_0         , 3    , NULL              , "Msg Rate     >"  , menuPage           , PAGE_MSGRATE     , _LCDML_TYPE_default);
LCDML_addAdvanced (3  , LCDML_0         , 4    , NULL              , "Counters     >"  , menuPage           , PAGE_COUNTERS    , _LCDML_TYPE_default);
LCDML_addAdvanced (4  , LCDML_0         , 5    , NULL              , "TxRx Power   >"  , menuPage           , PAGE_TXRXPOWER   , _LCDML_TYPE_default);
LCDML_addAdvanced (5  , LCDML_0         , 6    , NULL              , "Sleep Power  >"  , menuPage           , PAGE_SLEEPPOWER  , _LCDML_TYPE_default);
LCDML_add         (6  , LCDML_0         , 7                        , "Channel Scan >"  , NULL);
LCDML_addAdvanced (7  , LCDML_0_7     	, 1    , NULL              , ""                , menuCfgScanChStart	, 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (8  , LCDML_0_7     	, 2    , NULL              , ""                , menuCfgScanChStop	, 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (9  , LCDML_0_7     	, 3    , NULL              , "Run Scan     >"  , menuPage           , PAGE_SCANNER     , _LCDML_TYPE_default);
LCDML_add 		  (10 , LCDML_0_7       , 4                        , "Back         <"  , menuBack);
LCDML_add         (11 , LCDML_0         , 8                        , "Settings     >"  , NULL);
LCDML_add         (12 , LCDML_0_8       , 1                        , "Radio        >"  , NULL);
LCDML_addAdvanced (13 , LCDML_0_8_1     , 1    , NULL              , ""                , menuCfgChannel     , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (14 , LCDML_0_8_1     , 2    , NULL              , ""                , menuCfgGwNode      , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (15 , LCDML_0_8_1     , 3    , NULL              , ""                , menuCfgGwPa        , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (16 , LCDML_0_8_1     , 4    , NULL              , ""                , menuCfgNodePa      , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (17 , LCDML_0_8_1     , 5    , NULL              , ""                , menuCfgRate        , 0                , _LCDML_TYPE_dynParam);
LCDML_add         (18 , LCDML_0_8_1     , 6                        , "Back         <"  , menuBack);
LCDML_add         (19 , LCDML_0_8       , 2                        , "Doctor       >"  , NULL);
LCDML_addAdvanced (20 , LCDML_0_8_2     , 1    , NULL              , ""  			   , menuCfgPayload     , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (21 , LCDML_0_8_2     , 2    , NULL              , ""  			   , menuCfgMsgRate     , 0                , _LCDML_TYPE_dynParam);
LCDML_add         (22 , LCDML_0_8_2     , 3                        , "Reset buff   x"  , menuResetBuf);
LCDML_add         (23 , LCDML_0_8_2     , 4                        , "Back         <"  , menuBack);
LCDML_add         (24 , LCDML_0_8       , 3                        , "Eeprom       >"  , NULL);
LCDML_add         (25 , LCDML_0_8_3     , 1                        , "Save node    x"  , menuSaveNodeEeprom);
LCDML_add         (26 , LCDML_0_8_3     , 2                        , "Save node&gw x"  , menuSaveNodeAndGwEeprom);
LCDML_add         (27 , LCDML_0_8_3     , 3                        , "Defaults nodex"  , menuDefaultNodeEeprom);
LCDML_add         (28 , LCDML_0_8_3     , 4                        , "Back         <"  , menuBack);
LCDML_add         (29 , LCDML_0_8       , 4                        , "Reset node   x"  , menuResetNode);
LCDML_add         (30 , LCDML_0_8       , 5                        , "Back         <"  , menuBack);
#define _LCDML_DISP_cnt    30   // Should equal last id in menu



LCDML_createMenu(_LCDML_DISP_cnt);

# if(_LCDML_DISP_rows > _LCDML_DISP_cfg_max_rows)
# error change value of _LCDML_DISP_cfg_max_rows in LCDMenuLib2.h
# endif

//**** ENCODER & BUTTON  *****
//#define ENCODER_OPTIMIZE_INTERRUPTS //Only when using pin2/3 (or 20/21 on mega) - using this will screw up the other interrupt routine and we actually don't need it.
#include <Encoder.h>    // for Encoder      Download: https://github.com/PaulStoffregen/Encoder
#include <Bounce2.h>	// button debounce  Download: https://github.com/thomasfredericks/Bounce2

static Encoder encoder(ENCODER_A_PIN, ENCODER_B_PIN);
static Bounce button = Bounce(); 

//**** MySensors Messages ****
#define CHILD_ID_COUNTER 0
#define CHILD_ID_UPDATE_GATEWAY 0
MyMessage MsgCounter(CHILD_ID_COUNTER, V_CUSTOM);   				//Send Message Counter value

// Actual data exchanged in a message 
#define PAYLOAD_LENGTH_MIN         (2)				//The counter is always transmitted and is 2 bytes in size
#define PAYLOAD_LENGTH_MAX (MAX_PAYLOAD)
#pragma pack(push, 1)								// exact fit - no padding (to save space)
union t_MessageData {
  uint8_t m_dynMessage[MAX_PAYLOAD];
};
#pragma pack(pop)									//back to the previous packing mode
//Message Rate 
uint8_t iGetMsgRate = 0;

const int iNrArcCnt = 15;
uint8_t iArrayArcCnt[iNrArcCnt] = {0};
uint8_t iArcCntAvg = 0;
uint8_t iArcCntMax = 0;

//**** Monitoring Constants&Variables ****
const int iMaxNumberOfMessages = 100 ;           					// Number of Messages Used for MA calculation
boolean bArrayFailedMessages[iMaxNumberOfMessages] = { 0 };     	// Array for moving average storage
boolean bArrayNAckMessages[iMaxNumberOfMessages] = { 0 };			// Array for moving average storage
uint16_t iNrFailedMessages = 0;            							// total of Failed Messages
uint16_t iNrNAckMessages = 0;              							// total of Not Acknowledged Messages
uint16_t iMessageCounter = 0;

//**** Timing ****
const uint8_t iNrTimeDelays = 10;
uint16_t iMessageIndexBuffer[iNrTimeDelays];
unsigned long lTimeOfTransmit_us[iNrTimeDelays];
unsigned long lTimeDelayBuffer_Destination_us[iNrTimeDelays];
unsigned long lTimeDelayBuffer_FirstHop_us[iNrTimeDelays];
uint16_t iMeanDelayFirstHop_ms = 0;
uint16_t iMaxDelayFirstHop_ms = 0;
uint16_t iMeanDelayDestination_ms = 0;
uint16_t iMaxDelayDestination_ms = 0;

//**** Current Measurement ****
#include <PinChangeInterrupt.h>					// for Pin Change Interrupt      Download: https://github.com/NicoHood/PinChangeInterrupt
const uint8_t iNrCurrentMeasurements 	= 60;	//Nr of measurements for averaging current. <64 to prevent risk of overflow of iAdcSum
const float r1_ohm    = 2.2;
const float r2_ohm    = 100.0;
const float r3_ohm    = 10000.0;
const float Vref_volt = 1.1;
const float uAperBit1 = ((Vref_volt/1024.0)/r1_ohm)*1.0e6;
const float uAperBit2 = ((Vref_volt/1024.0)/r2_ohm)*1.0e6;
const float uAperBit3 = ((Vref_volt/1024.0)/r3_ohm)*1.0e6;

const float CurrentValueErrCap = -2.0;					// Will show 'Err cap' on display
const float CurrentValueWait   = -1.0;					// Will show 'WAIT' on display
const float CurrentValueErr    = 300000.0;				// Will show 'Err' on display

float SleepCurrent_uA 	 	= CurrentValueWait;
float TransmitCurrent_uA 	= 0;
float ReceiveCurrent_uA  	= 0;

//**** Configure ADC ****
volatile uint8_t iStartStorageAfterNrAdcSamples  = 7; 	//Note this depends on the set ADC prescaler (currently: 16x prescaler)
volatile uint8_t iStopStorageAfterNrAdcSamples 	= 28; 	//Note this depends on the set ADC prescaler (currently: 16x prescaler)
volatile uint16_t iAdcSum;								//Limit the number of samples to < 2^6 = 64
volatile uint8_t iNrAdcSamplesElapsed;
volatile boolean bAdcDone;

//**** Remote Gateway Update ****
//uint8_t iRetryGateway 	= 0;
bool bUpdateGateway = false;
uint8_t updateGatewayAttemptsRemaining;
const uint8_t updateGatewayNumAttempts = 10;

const uint8_t iNrGatwayRetryOptions = 3;
const char *pcGatewayRetryNames[iNrGatwayRetryOptions] = { "SKIP GATEWAY", "RETRY GATEWAY" , "CANCEL ALL"};

const uint16_t restartDelayMs = 3000u;

bool transportHwError = false;

//**** RPD Channel Scanner ****
#define LCD_NUM_SPECIAL_CHARS    (8)
#define LCD_WIDTH_SPECIAL_CHARS  (5)
#define LCD_HEIGHT_SPECIAL_CHARS (8)
#define CHANNEL_SCAN_NUM_BUCKETS (LCD_WIDTH_SPECIAL_CHARS*LCD_NUM_SPECIAL_CHARS)
static uint8_t iRf24ChannelScanStart = 0;
static uint8_t iRf24ChannelScanStop  = NRF24_MAX_CHANNEL;
static uint8_t iRf24ChannelScanCurrent = 0;
static uint8_t iRf24ChannelScanColDisplayed = LCD_WIDTH_SPECIAL_CHARS*LCD_NUM_SPECIAL_CHARS/2;
static uint8_t channelScanBuckets[CHANNEL_SCAN_NUM_BUCKETS];
static bool bChannelScanner = false;
#define SCANNEL_SCAN_MEASURE_TIME_US (5000)

/*****************************************************************************/
/******************************* ENCODER & BUTTON ****************************/
/*****************************************************************************/
void lcdml_menu_control(void)
{
	if (LCDML.BT_setup()) 
	{
		// run once; init pins & debouncer
		pinMode(ENCODER_A_PIN      , INPUT_PULLUP);
		pinMode(ENCODER_B_PIN      , INPUT_PULLUP);
		pinMode(BUTTON_PIN , INPUT_PULLUP); 
		button.attach(BUTTON_PIN);
		button.interval(5); // interval in ms
	}

	// We're interested in relative encoder moves only, so 8bits position suffices.
	const int8_t enc = int8_t(encoder.read());
	button.update();
	const bool pressed = button.read() == LOW;
	static bool prevPressed = false;

	// Mechanical encoder generates 4 increments in 1 mechanical 'step'.
	const int8_t encStep = 4;
	for (;;)
	{
		static int8_t encPrev = 0;
		int8_t delta = enc - encPrev;
		if (delta <= -encStep)
		{
			LCDML.BT_down();
			encPrev -= encStep;
		}
		else if (delta >= encStep)
		{
			LCDML.BT_up();
			encPrev += encStep;
		}
		else
		{
			break;
		}
	}

	if (pressed and (not prevPressed))
	{
		// Pressed and previously not pressed
		LCDML.BT_enter();  
	}
	prevPressed = pressed;
}

void delay_with_update(unsigned long delay_ms)
{
	unsigned long dTstart = millis();
	while ((millis()-dTstart)< delay_ms){
		LCDML.loop();
	}
}
/*****************************************************************************/
/******************************** ADC INTERRUPT ******************************/
/*****************************************************************************/
// ADC complete ISR
ISR (ADC_vect){
	//Continuous sampling of ADC
	iNrAdcSamplesElapsed++;	
	if (iNrAdcSamplesElapsed >= iStartStorageAfterNrAdcSamples){	//Skip first 130us for TX settling according to datasheet
#ifdef TRIGGER_PIN
		digitalWrite(TRIGGER_PIN,HIGH);				//Debugging purposes with scope
#endif
		iAdcSum = iAdcSum + ADC;
		if (iNrAdcSamplesElapsed < iStopStorageAfterNrAdcSamples){
			ADCSRA |= bit (ADSC) | bit (ADIE);	  	// start new conversion and enable interrupt flag on completion
		}
		else{
			bAdcDone = true;
#ifdef TRIGGER_PIN
			digitalWrite(TRIGGER_PIN,LOW);			//Debugging purposes with scope
#endif
		}
	}
	else{
		ADCSRA |= bit (ADSC) | bit (ADIE);	  		// start new conversion and enable interrupt flag on completion
	}
}

void ISR_TransmitTriggerADC(){
	detachPCINT(digitalPinToPinChangeInterrupt(MY_RF24_CE_PIN));
	//Settings for TX - Transmit measurement
	iStartStorageAfterNrAdcSamples  = 7; 	//Note this depends on the set ADC prescaler (currently: 16x prescaler) + Matched to TX timing
	switch (iRf24DataRate){
		case 0:
			iStopStorageAfterNrAdcSamples 	= 12 + uint8_t(iPayloadSize*0.25); 	//Note this depends on the set ADC prescaler (currently: 16x prescaler)	+ Matched to TX timing		
			break;
		case 1:
			iStopStorageAfterNrAdcSamples 	= 8 + uint8_t(iPayloadSize*0.125); 	//Note this depends on the set ADC prescaler (currently: 16x prescaler)	+ Matched to TX timing
			break;
		case 2:
			iStopStorageAfterNrAdcSamples 	= 25 + uint8_t(iPayloadSize*1.4); 	//Note this depends on the set ADC prescaler (currently: 16x prescaler)	+ Matched to TX timing		
			break;
	}
	
	iNrAdcSamplesElapsed	= 0;
	iAdcSum 				= 0;
	bAdcDone 				= false;
	ADCSRA |= bit (ADSC) | bit (ADIE);	  	//start new ADC conversion
}

/*****************************************************************************/
/************************************ STARTUP ********************************/
/*****************************************************************************/
void before() {						//Initialization before the MySensors library starts up
	pinMode(CURRENT_PIN, INPUT);	//Analog Input for Current Usage Pin
#ifdef TRIGGER_PIN
	pinMode(TRIGGER_PIN, OUTPUT);
	digitalWrite(TRIGGER_PIN,LOW);
#endif
#ifdef LED_PIN
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN,LOW);
#endif
	pinMode(MOSFET_2P2OHM_PIN,OUTPUT);
	pinMode(MOSFET_100OHM_PIN,OUTPUT);
	digitalWrite(MOSFET_2P2OHM_PIN,HIGH);

	//**** ADC SETUP ****
	ADCSRA =  bit (ADEN);                      				// turn ADC on
	ADCSRA |= bit (ADPS2);                               	// Prescaler of 16: To get sufficient samples in Tx Current Measurement 
	ADMUX  =  bit (REFS0) | bit (REFS1) | (ADC_PIN_NR & 0x07);  // ARef internal and select input port

	//****  LCD *****
	//  Wire.begin();  // I2C
	lcd.clear();
	lcd.begin(LCD_COLS, LCD_ROWS);
	LCD_SetScrollbarChars();
	//lcd.setBacklight(HIGH);

	// Load radio settings from eeprom
	loadEeprom();
	logRadioSettings();
}

void setup() {
	ClearStorageAndCounters();
//	activateRadioSettings();
//	logRadioSettings();
	Sprintln(F("Connecting..."));

	// Splash screen
	LCD_clear();
	print_LCD_line("nRF24 DOCTOR " SKETCH_VERSION_STRING,  0, 0);
	print_LCD_line("Connecting...", 1, 0);
	// Show splash screen for a short while, trying to connect to GW.
	// If GW connection has not been established after this delay the
	// node continues trying to connect.
	transportWaitUntilReady(2000);

	//**** MENU *****
	LCDML_setup(_LCDML_DISP_cnt);
	LCDML.SCREEN_disable();
}

void presentation() {
	sendSketchInfo(F(SKETCH_NAME_STRING), F(SKETCH_VERSION_STRING));
	present(CHILD_ID_COUNTER, S_CUSTOM) ;  // "CUSTOM" counter
}

/*****************************************************************************/
/*********************************** MAIN LOOP *******************************/
/*****************************************************************************/
void loop()
{
	LCDML.loop();  
	statemachine();
}

/*****************************************************************************/
/********************* TRANSMIT & MEASUREMENT STATEMACHINE *******************/
/*****************************************************************************/

enum state {	STATE_IDLE,
				// Regular measurement states
				STATE_TX, STATE_RX, STATE_PROCESS_DATA, STATE_SLEEP,								
				// Channel scanning Mode state
				STATE_CH_SCAN, STATE_CH_SCAN_RESTART, STATE_CH_SCAN_MEASURE, STATE_CH_SCAN_WAIT,
				// Gateway update states
				STATE_START_GW_UPDATE, STATE_TX_GW_UPDATE, STATE_FAILED_GW_UPDATE,
};
static state currState = STATE_IDLE;

void statemachine()
{
	static unsigned long timestamp = 0;	// reused inbetween some states

	unsigned long iSetMsgDelay = (1000000L/iSetMsgRate);

	switch (currState)
	{
		case STATE_IDLE:
			if (bUpdateGateway)
			{
				// Start of gateway update
				currState = STATE_START_GW_UPDATE;
			}
			else if (bChannelScanner)
			{
				// Start of channel scanner
				currState = STATE_CH_SCAN;
			}
			else if (isTransportReady())
			{
				// Start of next measurement round
				if ((micros() - timestamp) >= iSetMsgDelay){currState = STATE_TX;}	//Message Rate limiter
			}
			break;

		case STATE_TX:
			{
				// Transmit Current Measurement - Trigger measurement on interrupt
				EIFR |= 0x01;					//Clear interrupt flag to prevent an immediate trigger
				attachPCINT(digitalPinToPinChangeInterrupt(MY_RF24_CE_PIN), ISR_TransmitTriggerADC,RISING);
				unsigned long lTcurTransmit = transmit(iPayloadSize);
				
				//Time rate of transmissions
				iGetMsgRate = static_cast<uint8_t>((1e6/(lTcurTransmit-timestamp))+0.5);
				timestamp = lTcurTransmit;

				if (bAdcDone) {				//Get TX Current Measurement Data...it should already have finished
					TransmitCurrent_uA 	= uAperBit1*((float)iAdcSum/(float)(iStopStorageAfterNrAdcSamples-iStartStorageAfterNrAdcSamples+1));
					bAdcDone = false;
				}				
				//else{Sprintln(F("BAD ADC TIMING:"));} //Will happen if the node can not find the gateway @ startup - but no problem.
				store_ArcCnt_in_array();	//Store the number of auto re-transmits by the radio in the array

				currState = STATE_RX;
			}
			break;
      
		case STATE_RX:
				MY_RF24_startListening();	//Make sure I'm in RX mode
				ReceiveCurrent_uA 	= uAperBit1*GetAvgADCBits(iNrCurrentMeasurements);
				//MY_RF24_stopListening();	//I will automatically get out of RX mode when applicable
				//	Sprint(F("TransmitCurrent_uA:"));Sprintln(TransmitCurrent_uA);
				//	Sprint(F("ReceiveCurrent_uA:"));Sprintln(ReceiveCurrent_uA);
				currState = STATE_PROCESS_DATA;
			break;

		case STATE_PROCESS_DATA:
			//Calculate Mean & Max Values for display purposes
			getMeanAndMaxFromIntArray(&iArcCntAvg, &iArcCntMax, iArrayArcCnt, iNrArcCnt);
			getMeanAndMaxFromArray(&iMeanDelayFirstHop_ms,&iMaxDelayFirstHop_ms,lTimeDelayBuffer_FirstHop_us,iNrTimeDelays);
			getMeanAndMaxFromArray(&iMeanDelayDestination_ms,&iMaxDelayDestination_ms,lTimeDelayBuffer_Destination_us,iNrTimeDelays);
			
			currState = STATE_IDLE;
			break;

		case STATE_SLEEP:
		{
			//Sleep Current Measurement
			transportDisable();
			delay_with_update(20);									//Gate charge time and settle time, don't use wait as it will prevent the radio from sleep
			float SleepCurrent_uA_intermediate = uAperBit1*GetAvgADCBits(iNrCurrentMeasurements);
			if (SleepCurrent_uA_intermediate < 1500){
				//Set Higher Sensitivity: uAperBit2
				digitalWrite(MOSFET_2P2OHM_PIN, LOW);
				digitalWrite(MOSFET_100OHM_PIN, HIGH);
				delay_with_update(400);								//worst case settle time to charge through higher impedance
				SleepCurrent_uA_intermediate = uAperBit2*GetAvgADCBits(iNrCurrentMeasurements);
			}
			else {SleepCurrent_uA = SleepCurrent_uA_intermediate;}

			if (SleepCurrent_uA_intermediate < 15){
				//Set Higher Sensitivity: uAperBit3
				digitalWrite(MOSFET_2P2OHM_PIN, LOW);
				digitalWrite(MOSFET_100OHM_PIN, LOW);
				const float Init_Meas_SleepCurrent_uA = 0.4;
				const unsigned long lTimeOut_InitCurrent_ms = 6000;
				const unsigned long lTimeOut_Settled_SleepCurrent_ms = 30000;
				unsigned long ldT  = Time_to_reach_InitCurrent_uA(Init_Meas_SleepCurrent_uA, lTimeOut_InitCurrent_ms);
				if (ldT < lTimeOut_InitCurrent_ms){
					float fTarget_uA_per_sec = (0.4/(float(ldT)/1000))/20;	//Slew rate to which it should be reduced before we consider it settled
					SettledSleepCurrent_uA_reached(fTarget_uA_per_sec, lTimeOut_Settled_SleepCurrent_ms);
					SleepCurrent_uA = uAperBit3*GetAvgADCBits(iNrCurrentMeasurements);	//Even if SettledSleepCurrent_uA_reached has timed out it should have settled by now
				}
				else{		//Radio Cap > 1000uF - this will take too long....
					SleepCurrent_uA = CurrentValueErrCap;
				}
			}
			else {SleepCurrent_uA = SleepCurrent_uA_intermediate;}
//			Sprint(F("SleepCurrent_uA:"));Sprintln(SleepCurrent_uA);
			
			//Restore standby power state
			digitalWrite(MOSFET_2P2OHM_PIN, HIGH);	//Enable 2.2Ohm
			digitalWrite(MOSFET_100OHM_PIN, LOW);
			transportStandBy();

			currState = STATE_IDLE;
			break;		
		}

		case STATE_CH_SCAN:
			// Clear all buckets to start all over
			(void)memset(channelScanBuckets, 0, COUNT_OF(channelScanBuckets));
			currState = STATE_CH_SCAN_RESTART;
			break;

		case STATE_CH_SCAN_RESTART:
			iRf24ChannelScanCurrent = iRf24ChannelScanStart;
			currState = STATE_CH_SCAN_MEASURE;
			break;

		case STATE_CH_SCAN_MEASURE:
			// http://forum.diyembedded.com/viewtopic.php?f=4&t=809#p1047
			RF24_ce(LOW);
			RF24_setChannel(iRf24ChannelScanCurrent);
			RF24_flushRX();
			RF24_ce(HIGH);
			delayMicroseconds(130+40);
			timestamp = micros();
			currState = STATE_CH_SCAN_WAIT;
			break;

		case STATE_CH_SCAN_WAIT:
			if (not bChannelScanner)
			{
				// Requested to stop scanner
				currState = STATE_IDLE;
				break;
			}
			if ((micros() - timestamp) < SCANNEL_SCAN_MEASURE_TIME_US)
			{
				break;
			}
			if (RF24_getReceivedPowerDetector())
			{
				// Determine bucket and increase vote
				uint8_t bucket = (iRf24ChannelScanCurrent-iRf24ChannelScanStart) * CHANNEL_SCAN_NUM_BUCKETS / (iRf24ChannelScanStop-iRf24ChannelScanStart+1);
				bucket = CONSTRAIN_HI(bucket, COUNT_OF(channelScanBuckets)-1);	// just to be sure...
				if (channelScanBuckets[bucket] < 255)
				{
					++channelScanBuckets[bucket];
				}
			}
			if (iRf24ChannelScanCurrent >= iRf24ChannelScanStop)
			{
				for (size_t i = 0; i < COUNT_OF(channelScanBuckets); ++i)
				{
					Sprint(channelScanBuckets[i]);
					Sprint('\t');
				}
				Sprintln();
				currState = STATE_CH_SCAN_RESTART;
				break;
			}
			++iRf24ChannelScanCurrent;
			currState = STATE_CH_SCAN_MEASURE;
			break;

		case STATE_START_GW_UPDATE:
			updateGatewayAttemptsRemaining = updateGatewayNumAttempts;
			currState = STATE_TX_GW_UPDATE;
			break;

		case STATE_TX_GW_UPDATE:
			if (updateGatewayAttemptsRemaining)
			{
				--updateGatewayAttemptsRemaining;
				MyMessage MsgUpdateGateway(CHILD_ID_UPDATE_GATEWAY, V_CUSTOM);
				MsgUpdateGateway.setDestination(0);
				MsgUpdateGateway.setSensor(250);
				serializeGwSettings( MsgUpdateGateway );

				// Transmit message with software ack request (returned in "receive function")
				if ( send(MsgUpdateGateway, true) )
				{
					// Got a reply from gateway that message was received correctly.
					// Gateway will change to new settings, so the node can also activate the settings.
					saveEepromAndReset();
					// Never return here...
				}
			}
			else
			{
				// Retry attempts exhausted. Give up.
				currState = STATE_FAILED_GW_UPDATE;
			}
			break;

		case STATE_FAILED_GW_UPDATE:
			// Signals the UI that GW update failed. On next button/encoder change => return to prev menu
			bUpdateGateway = false;
			currState = STATE_IDLE;
			break;

		default:
			break;
	}
}

/*****************************************************************************/
/************************ RECEIVE & TRANSMIT FUNCTIONS ***********************/
/*****************************************************************************/
void receive(const MyMessage &message) {
	if (message.isAck() == 1 && message.type == V_CUSTOM && message.sensor==CHILD_ID_COUNTER){	//Acknowledge message & of correct type
		const t_MessageData& ReceivedData = *(static_cast<t_MessageData*>(message.getCustom()));
		uint16_t iNewMessage = ((uint16_t)ReceivedData.m_dynMessage[0] << 8)|((uint16_t)ReceivedData.m_dynMessage[1]); //2 Byte Counter

		uint16_t iIndexInArray = iNewMessage % iMaxNumberOfMessages;
		bArrayNAckMessages[iIndexInArray] = 0; 			// set corresponding flag to received.
		
		// Check Message (Round trip) Delay
		uint8_t iIndexInTimeArray = IndexOfValueInArray(iNewMessage, iMessageIndexBuffer, iNrTimeDelays); //Look-up if message is present in MessageIndexBuffer for delay calculation
		if ((iIndexInTimeArray != 255) && iIndexInTimeArray <=iNrTimeDelays){
			lTimeDelayBuffer_Destination_us[iIndexInTimeArray] = micros()-lTimeOfTransmit_us[iIndexInTimeArray];
		}
		iNrNAckMessages--;	//Received an Acknowledge Message (so one less No Ack)
	}
}

unsigned long transmit(size_t iPayloadLength) {
	static int iIndexInArrayFailedMessages  = 0 ;
	static int iIndexInArrayTimeMessages  = 0 ;	
	static t_MessageData MessageData;

	iPayloadLength = constrain(iPayloadLength,PAYLOAD_LENGTH_MIN,PAYLOAD_LENGTH_MAX);	
	iMessageCounter++;
	MessageData.m_dynMessage[0] = (uint8_t)((iMessageCounter & 0xFF00) >> 8);
	MessageData.m_dynMessage[1] = (uint8_t)(iMessageCounter & 0x00FF);
	//All other MessageData is just left undefined

	// Cyclic Index counters of arrays
	iIndexInArrayFailedMessages = iMessageCounter % iMaxNumberOfMessages;
	iIndexInArrayTimeMessages 	= iMessageCounter % iNrTimeDelays;

	bArrayNAckMessages[iIndexInArrayFailedMessages] = 1; 			// set corresponding flag to "Not Received Yet"

	// Prepare time stamp logging for transmit
	lTimeDelayBuffer_Destination_us[iIndexInArrayTimeMessages] = 0; 		// Clear Buffer value, new value will be written when message is received
	iMessageIndexBuffer[iIndexInArrayTimeMessages]=iMessageCounter;		// To link the Time Stamp to the correct message when we receive the acknowledge
	iNrNAckMessages++;													// Add one to the Not Acknowledged Message counter and remove it again if/when it is received.
	lTimeOfTransmit_us[iIndexInArrayTimeMessages] = micros();
	
#ifdef LED_PIN
	// Light LED
	digitalWrite(LED_PIN, HIGH);
#endif

	// Transmit message with software ack request (returned in "receive function"),
	// the boolean returned here is a Hardware hop-to-hop Ack
	boolean success = send(MsgCounter.setDestination(iDestinationNode).set(&MessageData,iPayloadLength), true);
	if (!success) {
		// Keep LED on to indicate failure
		lTimeDelayBuffer_FirstHop_us[iIndexInArrayTimeMessages] = 0;	//It failed, so I can't use it to determine a First Hop Delay (i.e. it is "infinite" delay as it failed)
		bArrayFailedMessages[iIndexInArrayFailedMessages] = true;	//Log it as a failed message (for rolling average)
		iNrFailedMessages++ ;
	}
	else{
		lTimeDelayBuffer_FirstHop_us[iIndexInArrayTimeMessages] = micros() - lTimeOfTransmit_us[iIndexInArrayTimeMessages];	//Log First Hop Delay in buffer
//		unsigned long temptime = lTimeDelayBuffer_FirstHop_us[iIndexInArrayTimeMessages];
		bArrayFailedMessages[iIndexInArrayFailedMessages] = false;	//Log it as a not-failed = succesful message (for rolling average)
#ifdef LED_PIN
		// LED off to indicate success
		digitalWrite(LED_PIN, LOW);
#endif
	}
	return lTimeOfTransmit_us[iIndexInArrayTimeMessages];
}

// nRF24 register: packet loss counter
// uint8_t get_rf24_register_plos_cnt(){
// 	return static_cast<uint8_t>((RF24_getObserveTX() & 0xF0)>>4);
// }

// nRF24 register: AcknowledgeRequestCount Counter. Counts the number of (hardware) re-transmissions for the current transaction
uint8_t get_rf24_register_arc_cnt(){
	return static_cast<uint8_t>(RF24_getObserveTX() & 0x0F);
}

void store_ArcCnt_in_array(){
	static size_t iIndexInArray=0;
	iIndexInArray++;
	iIndexInArray = iIndexInArray % iNrArcCnt;
	iArrayArcCnt[iIndexInArray] = get_rf24_register_arc_cnt();
}

void MY_RF24_startListening()
{
	// toggle PRX
	RF24_setRFConfiguration(RF24_CONFIGURATION | _BV(RF24_PWR_UP) | _BV(RF24_PRIM_RX) );
	// all RX pipe addresses must be unique, therefore skip if node ID is RF24_BROADCAST_ADDRESS
	if(RF24_NODE_ADDRESS!= RF24_BROADCAST_ADDRESS) {
		RF24_setPipeLSB(RF24_REG_RX_ADDR_P0, RF24_NODE_ADDRESS);
	}
	// start listening
	RF24_ce(HIGH);
}

// void MY_RF24_stopListening()
// {
// 	RF24_DEBUG(PSTR("RF24:SPL\n"));	// stop listening
// 	RF24_ce(LOW);
// 	// timing
// 	delayMicroseconds(130);
// 	RF24_setRFConfiguration(RF24_CONFIGURATION | _BV(RF24_PWR_UP) );
// 	// timing
// 	delayMicroseconds(100);
// }

/********************************************************************************/
/************************* MYSENSORS INDICATION CALLBACK ************************/
/********************************************************************************/
void indication( const indication_t ind )
{
	switch(ind)
	{
#ifdef LED_PIN
		// If transport is not ready, flash the LED to indicate something is happening
		case INDICATION_TX:
			if (not isTransportReady())
			{
				// Blink LED
				digitalWrite(LED_PIN, HIGH);
				delay_with_update(20);
				digitalWrite(LED_PIN, LOW);
			}
			break;
#endif
		case INDICATION_ERR_INIT_TRANSPORT:			// MySensors transport hardware (radio) init failure.
			transportHwError = true;
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
}

void getMeanAndMaxFromIntArray(uint8_t *mean_value, uint8_t *max_value, uint8_t *buffer, uint8_t size) {
	//Note: excluding 0 values from mean calculation
	boolean bNotZero = false;
	uint8_t iMaxValue=0;	//max Array value
	uint16_t sum=0;
	for (int i=0; i < size; i++)
	{
		if (buffer[i] != 0){
			sum 		= sum + static_cast<uint16_t>(buffer[i]);	
			iMaxValue	= max(iMaxValue,buffer[i]);
			bNotZero=true;
		}
	}
	*max_value		= iMaxValue;
	if (bNotZero){
		*mean_value 	= static_cast<uint8_t>((sum / size)+0.5);
	}
	else {
		*mean_value = 0;
	}	
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
		*mean_value 	= (uint16_t) (((sum / (float)iNrOfSamples)+500)/1000);
		*max_value		= (uint16_t) ((lMaxValue+500)/1000L);
	}
	else {
		*mean_value = 65535;	//INF identifier
		*max_value 	= 65535;	//INF identifier
	}	
}

float GetAvgADCBits(int iNrSamples) {
	//iNrSamples < 64, else risk of overflowing iAdcSum
	iStartStorageAfterNrAdcSamples  = 0;
	iStopStorageAfterNrAdcSamples 	= iNrSamples;

	iNrAdcSamplesElapsed	= 0;
	iAdcSum 				= 0;
	bAdcDone 				= false;
	ADCSRA |= bit (ADSC) | bit (ADIE);	  	//start new ADC conversion
	while (!bAdcDone){delay_with_update(1);};			//Wait until all ADC conversions have completed
	bAdcDone 				= false;
	return ((float)iAdcSum/(float)(iNrSamples));
}

unsigned long Time_to_reach_InitCurrent_uA(float Threshold_current_uA, unsigned long lTimeOut){
	float Current_uA = 0;
	unsigned long lTstart = millis();
	unsigned long ldT = 0;
	while ((Current_uA < Threshold_current_uA) & (ldT<lTimeOut)){
		delay_with_update(50);	//don't measure to often as it will load the sleep current too much.
		Current_uA = uAperBit3*GetAvgADCBits(iNrCurrentMeasurements);
		ldT = (millis()-lTstart);
	}
	return ldT;
}

bool SettledSleepCurrent_uA_reached(float Threshold_current_uA_per_sec, unsigned long lTimeOut){
	bool bReached = false;
	float Current_uA_new = 0;
	float Current_uA_prev = uAperBit3*GetAvgADCBits(iNrCurrentMeasurements);
	float Current_uA_per_sec = 0;
	unsigned long lTstart = millis();
	int n=0;
	unsigned long lTimeScaler = constrain(100/Threshold_current_uA_per_sec,100,15000);//don't measure to often as it will load the sleep current too much.
	while ((n<2) & ((millis()-lTstart)<lTimeOut)){
		delay_with_update(lTimeScaler);
		Current_uA_new 		= uAperBit3*GetAvgADCBits(iNrCurrentMeasurements);
		Current_uA_per_sec 	= (Current_uA_new - Current_uA_prev)/(float(lTimeScaler)/1000);
		Current_uA_prev = Current_uA_new;
		if (Current_uA_per_sec < Threshold_current_uA_per_sec){n++;}
		else{n=0;}
	}
	if (Current_uA_per_sec < Threshold_current_uA_per_sec){bReached = true;}
	return bReached;
}

/*****************************************************************/
/************************* LCD FUNCTIONS *************************/
/*****************************************************************/

void print_LCD_line(const char *string, int row, int col) {
	lcd.setCursor(col,row);
	lcd.print(string);
}

void print_LCD_line(const __FlashStringHelper *string, int row, int col) {
	lcd.setCursor(col,row);
	lcd.print(string);
}

void LCD_clear() {
	lcd.clear();
}

void LCD_SetScrollbarChars()
{
	// set special chars for scrollbar
	lcd.createChar(0, (uint8_t*)scroll_bar[0]);
	lcd.createChar(1, (uint8_t*)scroll_bar[1]);
	lcd.createChar(2, (uint8_t*)scroll_bar[2]);
	lcd.createChar(3, (uint8_t*)scroll_bar[3]);
	lcd.createChar(4, (uint8_t*)scroll_bar[4]);  
}

/*****************************************************************/
/************************* MENU HANDLERS *************************/
/*****************************************************************/
char* printBufCurrent(char *buf, int size, float curr)
{
	if (CurrentValueErrCap == curr)
	{
		snprintf_P(buf, size, PSTR("Err cap"));
	}
	else if (CurrentValueWait == curr)
	{
		snprintf_P(buf, size, PSTR("Wait"));
	}
	else if (curr >= CurrentValueErr)
	{
		snprintf_P(buf, size, PSTR("Err"));
	}
	else
	{
		char scalePrefix = 'u';
		if (curr > 1000)
		{
			scalePrefix = 'm';
			curr /= 1000.0;
		}
		const uint16_t currInt  = curr;
		const uint8_t  currFrac = (curr - float(currInt)) * 100.0 + 0.5;
		snprintf_P(buf, size, PSTR("%" PRIu16 ".%02" PRIu8 " %cA"), currInt, currFrac, scalePrefix);
	}
	return buf;
}

void menuPage(uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		LCDML.FUNC_setLoopInterval(200);  // starts a trigger event for the loop function every 200 millisecounds
	}

	if (LCDML.FUNC_loop())
	{
		LCD_clear();
		char buf[LCD_COLS+1];
		bool exit = LCDML.BT_checkAny(); // check if any button is pressed (enter, up, down, left, right)

		if (transportHwError)
		{
			print_LCD_line("Radio init error",  0, 0);
			print_LCD_line("Replace radio",     1, 0);
		}
		else if (not isTransportReady())
		{
			print_LCD_line("Search Gateway..",  0, 0);
		}
		else
		{
			switch(page(param))
			{
				case PAGE_STATISTICS:
					snprintf_P(buf, sizeof(buf), PSTR("P%-3dFAIL%4d%3d%%"), MY_PARENT_NODE_ID, iNrFailedMessages, GetNrOfTrueValuesInArray(bArrayFailedMessages, iMaxNumberOfMessages));
					print_LCD_line(buf, 0, 0);
					snprintf_P(buf, sizeof(buf), PSTR("D%-3dNACK%4d%3d%%"), iDestinationNode , iNrNAckMessages, GetNrOfTrueValuesInArray(bArrayNAckMessages, iMaxNumberOfMessages));
					print_LCD_line(buf, 1, 0);
					break;

				case PAGE_TIMING:
					if (iMaxDelayFirstHop_ms>9999){
						snprintf_P(buf, sizeof(buf), PSTR("HOP1 dTmax   INF"));
					} else {
						snprintf_P(buf, sizeof(buf), PSTR("HOP1 dTmax%4dms"),iMaxDelayFirstHop_ms);
					}
					print_LCD_line(buf, 0, 0);
					if (iMaxDelayDestination_ms>9999){
						snprintf_P(buf, sizeof(buf), PSTR("D%-3d dTmax   INF"),iDestinationNode,iMaxDelayDestination_ms);
					} else {
						snprintf_P(buf, sizeof(buf), PSTR("D%-3d dTmax%4dms"),iDestinationNode,iMaxDelayDestination_ms);
					}
					print_LCD_line(buf, 1, 0);
					break;

				case PAGE_MSGRATE:
					snprintf_P(buf, sizeof(buf), PSTR("MSG/SEC     %3d"), iGetMsgRate);
					print_LCD_line(buf, 0, 0);
					snprintf_P(buf, sizeof(buf), PSTR("ARC Avg%2d Max%2d"), iArcCntAvg,iArcCntMax);
					print_LCD_line(buf, 1, 0);
					break;

				case PAGE_COUNTERS:
					snprintf_P(buf, sizeof(buf), PSTR("MESSAGE COUNT:  "));
					print_LCD_line(buf, 0, 0);
					snprintf_P(buf, sizeof(buf), PSTR("           %5d"),iMessageCounter);
					print_LCD_line(buf, 1, 0);
					break;

				case PAGE_TXRXPOWER:
					{
						char buf1[LCD_COLS+1];
						snprintf_P(buf, sizeof(buf), PSTR("Tx %s"), printBufCurrent(buf1,sizeof(buf1), TransmitCurrent_uA) );
						print_LCD_line(buf, 0, 0);
						snprintf_P(buf, sizeof(buf), PSTR("Rx %s"), printBufCurrent(buf1,sizeof(buf1), ReceiveCurrent_uA) );
						print_LCD_line(buf, 1, 0);
					}
					break;

				case PAGE_SLEEPPOWER:
					{
						currState = STATE_SLEEP;
						char buf1[LCD_COLS+1];
						snprintf_P(buf, sizeof(buf), PSTR("Sleep %s"), printBufCurrent(buf1,sizeof(buf1), SleepCurrent_uA) );
						print_LCD_line(buf, 0, 0);
					}
					break;

				case PAGE_SCANNER:
					{
						// SCanner only exits on button press, as rotation is used to navigate channels.
						exit = LCDML.BT_checkEnter();

						bChannelScanner = not exit;

						iRf24ChannelScanColDisplayed = constrain(iRf24ChannelScanColDisplayed, 0, (LCD_WIDTH_SPECIAL_CHARS*LCD_NUM_SPECIAL_CHARS-1));

						snprintf_P(buf, sizeof(buf), PSTR("%3" PRIu8 "["), iRf24ChannelScanStart );
						print_LCD_line(buf, 0, 0);

						for (uint8_t i = 0; i < LCD_NUM_SPECIAL_CHARS; ++i)
						{
							lcd.write(i);
						}
						snprintf_P(buf, sizeof(buf), PSTR("]%-3" PRIu8), iRf24ChannelScanStop );
						print_LCD_line(buf, 0, 4+LCD_NUM_SPECIAL_CHARS);

						// Calculate nRF channel indicated by pointer
						const uint8_t chan = iRf24ChannelScanStart+((iRf24ChannelScanStop-iRf24ChannelScanStart)*iRf24ChannelScanColDisplayed+(LCD_WIDTH_SPECIAL_CHARS*LCD_NUM_SPECIAL_CHARS/2))/(LCD_WIDTH_SPECIAL_CHARS*LCD_NUM_SPECIAL_CHARS-1);
						snprintf_P(buf, sizeof(buf), PSTR("CH%-03" PRIu8 " 2.%03" PRIu16 "G"), chan, 400+chan );
						print_LCD_line(buf, 1, 0);
						// Show wifi channel
						const uint8_t wifi = (chan-5)/5;
						if ((wifi > 0) and (wifi <= 13))
						{
							snprintf_P(buf, sizeof(buf), PSTR("W%-2" PRIu8), wifi );
							print_LCD_line(buf, 1, LCD_COLS-3);
						}

						uint8_t b = 0;
						for (uint8_t i = 0; i < LCD_NUM_SPECIAL_CHARS; ++i)
						{
							uint8_t ch[LCD_HEIGHT_SPECIAL_CHARS];
							(void)memset(ch, 0, COUNT_OF(ch));
							for (uint8_t mask = 1 << (LCD_WIDTH_SPECIAL_CHARS-1); mask > 0; mask >>= 1)
							{
								uint8_t v = channelScanBuckets[b];
//								uint8_t lvl = 256-(256/LCD_HEIGHT_SPECIAL_CHARS);
								const uint8_t heightPix = LCD_HEIGHT_SPECIAL_CHARS-1;
								uint8_t lvl = 1<<heightPix;
								for (uint8_t h = 0; h < heightPix; ++h)
								{
									if (v > lvl)
									{
										ch[h] |= mask;
									}
//									lvl -= 256/LCD_HEIGHT_SPECIAL_CHARS;
									lvl >>= 1;

									// Draw XOR'ed pointer at the top of the chart to indicate column
									if ((0 == h) and (b == iRf24ChannelScanColDisplayed))
									{
										ch[h] ^= mask;
									}
								}
								++b;
							}
							ch[LCD_HEIGHT_SPECIAL_CHARS-1] = 0xFF;
							lcd.createChar(i, ch);
						}
						// Check range of pointer
						if (LCDML.BT_checkUp() and (iRf24ChannelScanColDisplayed < (LCD_WIDTH_SPECIAL_CHARS*LCD_NUM_SPECIAL_CHARS-1)))
						{
							++iRf24ChannelScanColDisplayed;
						}
						if (LCDML.BT_checkDown() and (iRf24ChannelScanColDisplayed > 0))
						{
							--iRf24ChannelScanColDisplayed;
						}
					}
					break;

				default:
					break;
			}
		}

		if (exit)
		{      
			LCDML.FUNC_goBackToMenu();  // leave this function
			// Restore special characters if they got overwritten
			LCD_SetScrollbarChars();
		}
	} 
}


void menuCfgEntry( uint8_t &value )
{
	// make only an action when the cursor stands on this menuitem
	//check Button
	if (LCDML.BT_checkAny()) 
	{
		if (LCDML.BT_checkEnter()) 
		{
			// this function checks returns the scroll disable status (0 = menu scrolling enabled, 1 = menu scrolling disabled)
			if (LCDML.MENU_getScrollDisableStatus() == 0)
			{
				// disable the menu scroll function to catch the cursor on this point
				// now it is possible to work with BT_checkUp and BT_checkDown in this function
				// this function can only be called in a menu, not in a menu function
				LCDML.MENU_disScroll();
			}
			else
			{
				// enable the normal menu scroll function
				LCDML.MENU_enScroll();
			}
			// dosomething for example save the data or something else
			LCDML.BT_resetEnter();
		}
	}
	if ((value < 255) and LCDML.BT_checkUp())   value++;
	if ((value > 0)   and LCDML.BT_checkDown()) value--;
}

void menuCfgScanChStart(uint8_t line)
{ 
	if (line == LCDML.MENU_getCursorPos()) 
	{
		menuCfgEntry( iRf24ChannelScanStart );
		iRf24ChannelScanStart = CONSTRAIN_HI( iRf24ChannelScanStart, iRf24ChannelScanStop );
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("Start Ch. %3d"), iRf24ChannelScanStart);

	// use the line from function parameters
	lcd.setCursor(1, line);
	lcd.print(buf); 
}

void menuCfgScanChStop(uint8_t line)
{ 
	if (line == LCDML.MENU_getCursorPos()) 
	{
		menuCfgEntry( iRf24ChannelScanStop );
		iRf24ChannelScanStop = constrain( iRf24ChannelScanStop, iRf24ChannelScanStart, NRF24_MAX_CHANNEL );
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("Stop  Ch. %3d"), iRf24ChannelScanStop);

	// use the line from function parameters
	lcd.setCursor(1, line);
	lcd.print(buf); 
}

void menuCfgPayload(uint8_t line)
{ 
	if (line == LCDML.MENU_getCursorPos()) 
	{
		menuCfgEntry( iPayloadSize );
		iPayloadSize = constrain(iPayloadSize, PAYLOAD_LENGTH_MIN, PAYLOAD_LENGTH_MAX );
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("Payload    %2d"), iPayloadSize);

	// use the line from function parameters
	lcd.setCursor(1, line);
	lcd.print(buf); 
}

void menuCfgMsgRate(uint8_t line)
{ 
	if (line == LCDML.MENU_getCursorPos()) 
	{
		menuCfgEntry( iSetMsgRate );
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("Msg Rate  %3d"), iSetMsgRate);

	// use the line from function parameters
	lcd.setCursor(1, line);
	lcd.print(buf); 
}

void menuCfgChannel(uint8_t line)
{ 
	if (line == LCDML.MENU_getCursorPos()) 
	{
		menuCfgEntry( iRf24Channel );
		iRf24Channel = CONSTRAIN_HI( iRf24Channel, NRF24_MAX_CHANNEL );
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("Channel   %3d"), iRf24Channel);

	// use the line from function parameters
	lcd.setCursor(1, line);
	lcd.print(buf); 
}

void menuCfgGwNode(uint8_t line)
{ 
	if (line == LCDML.MENU_getCursorPos()) 
	{
		menuCfgEntry( iDestinationNode );
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("GW Node   %3d"), iDestinationNode);

	// use the line from function parameters
	lcd.setCursor(1, line);
	lcd.print(buf); 
}

void menuCfgNodePa(uint8_t line)
{ 
	if (line == LCDML.MENU_getCursorPos()) 
	{
		menuCfgEntry( iRf24PaLevel );
		iRf24PaLevel = rf24PaLevelConstrain( iRf24PaLevel );
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("Node PA  %-4s"), rf24PaLevelToString(iRf24PaLevel));

	// use the line from function parameters
	lcd.setCursor(1, line);
	lcd.print(buf); 
}

void menuCfgGwPa(uint8_t line)
{ 
	if (line == LCDML.MENU_getCursorPos()) 
	{
		menuCfgEntry( iRf24PaLevelGw );
		iRf24PaLevelGw = rf24PaLevelConstrain( iRf24PaLevelGw );
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("GW PA    %-4s"), rf24PaLevelToString(iRf24PaLevelGw));

	// use the line from function parameters
	lcd.setCursor(1, line);
	lcd.print(buf); 
}

void menuCfgRate(uint8_t line)
{ 
	if (line == LCDML.MENU_getCursorPos()) 
	{
		menuCfgEntry( iRf24DataRate );
		iRf24DataRate = rf24DataRateConstrain( iRf24DataRate );
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("Datarate %-4s"), rf24DataRateToString(iRf24DataRate));

	// use the line from function parameters
	lcd.setCursor(1, line);
	lcd.print(buf);
}

void menuSaveNodeEeprom(__attribute__((unused)) uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		LCD_clear();
		print_LCD_line(F("Eeprom saved"), 0, 0);
		print_LCD_line(F("Restarting..."), 1, 0);
		delay(restartDelayMs);
		saveEepromAndReset();
		// Never return here...
	} 
}

void menuDefaultNodeEeprom(__attribute__((unused)) uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		LCD_clear();
		print_LCD_line(F("Defaults saved"), 0, 0);
		print_LCD_line(F("Restarting..."), 1, 0);
		delay(restartDelayMs);
		loadDefaults();
		saveEepromAndReset();
		// Never return here...
	} 
}

void menuSaveNodeAndGwEeprom(__attribute__((unused)) uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		// Trigger the gateway update sequence
		bUpdateGateway = true;
		LCD_clear();
		LCDML.FUNC_setLoopInterval(100);
	}

	if (LCDML.FUNC_loop())
	{
		static bool prevUpdateGateway = true;
		if (not bUpdateGateway)
		{
			// Gateway update finished with error
			if (prevUpdateGateway != bUpdateGateway)
			{
				// Print message only once
				print_LCD_line(F("Failed"), 0, 0);
			}
			if (LCDML.BT_checkAny()) // check if any button is pressed (enter, up, down, left, right)
			{      
				LCDML.FUNC_goBackToMenu();  // leave this function
			}
		}
		prevUpdateGateway = bUpdateGateway;
	} 
}

void menuResetBuf(__attribute__((unused)) uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		ClearStorageAndCounters();
		LCD_clear();
		print_LCD_line(F("Cleared"), 0, 0);
	} 
	if (LCDML.FUNC_loop())
	{
		if (LCDML.BT_checkAny()) // check if any button is pressed (enter, up, down, left, right)
		{      
			LCDML.FUNC_goBackToMenu();  // leave this function
		}
	}
}

void menuResetNode(__attribute__((unused)) uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		LCD_clear();
		print_LCD_line(F("Restarting..."), 0, 0);
		delay(restartDelayMs);
		reset();
		// Never return here...
	} 
}

void menuBack(__attribute__((unused)) uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		// Go one level up
		LCDML.FUNC_goBackToMenu(1);
	} 
}
