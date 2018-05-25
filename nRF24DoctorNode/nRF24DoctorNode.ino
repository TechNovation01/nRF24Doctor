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

//**** CONNECTIONS *****
#define ENCODER_A_PIN       2		//Interrupt pin required for Encoder for optimal response
#define ENCODER_B_PIN       3		//Interrupt pin required for Encoder for optimal response
#define TRIGGER_PIN         4    	//Debugging purposes with scope
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
#define MY_SPLASH_SCREEN_DISABLED			// Disable splash screen (saves some flash)
#define MY_TRANSPORT_WAIT_READY_MS 10 		// [ms] Init timeout for gateway not reachable
#define MY_NODE_ID 250						// Set a high node ID, which typically will not yet be used in the network
#define MY_PARENT_NODE_IS_STATIC			// Fixed parent Node ID, else MySensors Transport will attempt automatic fix after successive failures...but we don't want that while diagnosing our connection
#define MY_PARENT_NODE_ID 0              	// Typically 0 for Gateway

#define MY_BAUD_RATE 115200

//**** MySensors - Radio *****
#define MY_RADIO_NRF24                  	// Enable and select radio type attached

#define CHARACTER_DISPLAY

#include <SPI.h>
#include <MySensors.h>

//**** LCD *****
#ifdef CHARACTER_DISPLAY
	//#include <LiquidCrystal_I2C.h>                // LCD display with I2C interface
	#include <LiquidCrystal.h>                      // LCD display with parallel interface

	#define LCD_COLS 16
	#define LCD_ROWS 2
	//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  	// Set the LCD I2C address
	//LiquidCrystal_I2C lcd(0x27, 16, 2);  								// Set the LCD I2C address
	LiquidCrystal lcd(LCD_RS,LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7); // LCD with parallel interface
#else
	#include <Adafruit_GFX.h>
	#include <TFT_ILI9163C.h>

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

	#define TFT_PIN_CS      (8)
	#define TFT_PIN_DC      (7)
	static TFT_ILI9163C tft(TFT_PIN_CS, TFT_PIN_DC);
#endif


//**** LCD Menu *****
#include <LCDMenuLib2.h>	// Download: https://github.com/Jomelo/LCDMenuLib2

#define _LCDML_DISP_cols             LCD_COLS
#define _LCDML_DISP_rows             LCD_ROWS
#define _LCDML_DISP_cfg_scrollbar    1      // enable a scrollbar

#ifdef CHARACTER_DISPLAY
const uint8_t scroll_bar[][8] = {
	{B10001, B10001, B10001, B10001, B10001, B10001, B10001, B10001}, // scrollbar top
	{B11111, B11111, B10001, B10001, B10001, B10001, B10001, B10001}, // scroll state 1
	{B10001, B10001, B11111, B11111, B10001, B10001, B10001, B10001}, // scroll state 2
	{B10001, B10001, B10001, B10001, B11111, B11111, B10001, B10001}, // scroll state 3
	{B10001, B10001, B10001, B10001, B10001, B10001, B11111, B11111}  // scrollbar bottom
}; 
#endif

void lcdml_menu_display();
void lcdml_menu_clear();
void lcdml_menu_control();

static LCDMenuLib2_menu LCDML_0 (255, 0, 0, NULL, NULL); // root menu element (do not change)
static LCDMenuLib2 LCDML(LCDML_0, LCD_ROWS, LCD_COLS, lcdml_menu_display, lcdml_menu_clear, lcdml_menu_control);

enum page { PAGE_STATISTICS, PAGE_TIMING, PAGE_MSGRATE, PAGE_COUNTERS, PAGE_TXRXPOWER, PAGE_SLEEPPOWER };

// add            (id   prev_layer      new_num                      lang_char_array     callback_function)  
// addAdvanced    (id   prev_layer      new_num  condition           lang_char_array     callback_function  parameter (0-255)  menu function type )
//                                                                   "01234567890123"
LCDML_addAdvanced (0  , LCDML_0         , 1    , NULL              , "Statistics   >"  , menuPage         , PAGE_STATISTICS  , _LCDML_TYPE_default);
LCDML_addAdvanced (1  , LCDML_0         , 2    , NULL              , "Timing       >"  , menuPage         , PAGE_TIMING      , _LCDML_TYPE_default);
LCDML_addAdvanced (2  , LCDML_0         , 3    , NULL              , "Msg Rate     >"  , menuPage         , PAGE_MSGRATE     , _LCDML_TYPE_default);
LCDML_addAdvanced (3  , LCDML_0         , 4    , NULL              , "Counters     >"  , menuPage         , PAGE_COUNTERS    , _LCDML_TYPE_default);
LCDML_addAdvanced (4  , LCDML_0         , 5    , NULL              , "TxRx Power   >"  , menuPage         , PAGE_TXRXPOWER   , _LCDML_TYPE_default);
LCDML_addAdvanced (5  , LCDML_0         , 6    , NULL              , "Sleep Power  >"  , menuPage         , PAGE_SLEEPPOWER  , _LCDML_TYPE_default);
LCDML_add         (6  , LCDML_0         , 7                        , "Settings     >"  , NULL);
LCDML_addAdvanced (7  , LCDML_0_7       , 1    , NULL              , ""  			   , menuCfgPayload   , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (8  , LCDML_0_7       , 2    , NULL              , ""  			   , menuCfgMsgRate   , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (9  , LCDML_0_7       , 3    , NULL              , ""                , menuCfgChannel   , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (10 , LCDML_0_7       , 4    , NULL              , ""                , menuCfgGwNode    , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (11 , LCDML_0_7       , 5    , NULL              , ""                , menuCfgGwPa      , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (12 , LCDML_0_7       , 6    , NULL              , ""                , menuCfgNodePa    , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (13 , LCDML_0_7       , 7    , NULL              , ""                , menuCfgRate      , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (14 , LCDML_0_7       , 8    , NULL              , ""                , menuCfgRadioId   , 0                , _LCDML_TYPE_dynParam);
LCDML_add         (15 , LCDML_0_7       , 9                        , "Reset buff   x"  , menuResetBuf);
LCDML_add         (16 , LCDML_0_7       , 10                       , "Eeprom       >"  , NULL);
LCDML_add         (17 , LCDML_0_7_10    , 1                        , "Save node    x"  , menuSaveNodeEeprom);
LCDML_add         (18 , LCDML_0_7_10    , 2                        , "Save node&gw x"  , menuSaveNodeAndGwEeprom);
LCDML_add         (19 , LCDML_0_7_10    , 3                        , "Load node    x"  , menuLoadNodeEeprom);
LCDML_add         (20 , LCDML_0_7_10    , 4                        , "Defaults     x"  , menuDefaultNodeEeprom);
LCDML_add         (21 , LCDML_0_7_10    , 5                        , "Back         <"  , menuBack);
LCDML_add         (22 , LCDML_0_7       , 11                       , "Back         <"  , menuBack);
#define _LCDML_DISP_cnt    22   // Should equal last id in menu



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

//**** EEPROM STORAGE LOCATIONS *****
#define EEPROM_FLAG_MAGIC		0xA5	// Indication contents are valid. Empty eeprom will contain 0xFF
#define EEPROM_FLAG				0
#define EEPROM_CHANNEL			1
#define EEPROM_PA_LEVEL			2
#define EEPROM_PA_LEVEL_GW		3
#define EEPROM_DATARATE			4
#define EEPROM_BASE_RADIO_ID	5
#define EEPROM_DESTINATION_NODE	6
#define EEPROM_PAYLOAD_SIZE		7
#define EEPROM_MESSAGE_RATE		8

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
uint8_t iSetMsgRate = 10;
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

//nRF24 Settings
const char *pcPaLevelNames[]  = { "MIN", "LOW", "HIGH", "MAX" };
const char *pcDataRateNames[] = { "1MBPS", "2MBPS" , "250KBPS"};

//Define your available RF24_BASE_ID 
uint8_t RF24_BASE_ID_VAR[MY_RF24_ADDR_WIDTH] = { 0x00,0xFC,0xE1,0xA8,0xA8 };		//Used to Store the active BASE_ID
const uint8_t RF24_BASE_ID_VARS[][MY_RF24_ADDR_WIDTH] = {
	{ 0x00,0xFC,0xE1,0xA8,0xA8 }
	, { 0x00,0xA1,0xF3,0x09,0xB6 }
	, { 0x00,0xAA,0xA5,0xC4,0xD9 }
	, { 0x00,0xB1,0x47,0xEE,0x82 }
};

// Radio values
uint8_t iRf24Channel;
uint8_t iRf24PaLevel;		//PA Level for the Node
uint8_t iRf24PaLevelGw;		//PA Level for the Gateway
uint8_t iRf24DataRate;
uint8_t iRf24BaseRadioId;
uint8_t iDestinationNode;
uint8_t iPayloadSize;
#define DEFAULT_RF24_CHANNEL		(MY_RF24_CHANNEL)
#define DEFAULT_RF24_PA_LEVEL_NODE	(MY_RF24_PA_LEVEL)
#define DEFAULT_RF24_PA_LEVEL_GW	(MY_RF24_PA_LEVEL)
#define DEFAULT_RF24_DATARATE		(MY_RF24_DATARATE)
#define DEFAULT_RF24_BASE_ID_IDX	(0)
#define DEFAULT_DESTINATION_NODE	(0)				// Default 0 = gateway, Settable in Menu
#define DEFAULT_PAYLOAD_SIZE		(2)				// 2 Bytes is the minimum for the Counter data
#define DEFAULT_MESSAGE_RATE 		(10)

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

float SleepCurrent_uA 	 	= 20000000;			//Will show WAIT on display
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

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))
#define CONSTRAIN_HI(amt,high) ((amt)>(high)?(high):(amt))

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
		digitalWrite(TRIGGER_PIN,HIGH);				//Debugging purposes with scope
		iAdcSum = iAdcSum + ADC;
		if (iNrAdcSamplesElapsed < iStopStorageAfterNrAdcSamples){
			ADCSRA |= bit (ADSC) | bit (ADIE);	  	// start new conversion and enable interrupt flag on completion
		}
		else{
			bAdcDone = true;
			digitalWrite(TRIGGER_PIN,LOW);			//Debugging purposes with scope
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
	pinMode(TRIGGER_PIN, OUTPUT);
	pinMode(MOSFET_2P2OHM_PIN,OUTPUT);
	pinMode(MOSFET_100OHM_PIN,OUTPUT);
	digitalWrite(MOSFET_2P2OHM_PIN,HIGH);
	digitalWrite(TRIGGER_PIN,LOW);

	//**** ADC SETUP ****
	ADCSRA =  bit (ADEN);                      				// turn ADC on
	ADCSRA |= bit (ADPS2);                               	// Prescaler of 16: To get sufficient samples in Tx Current Measurement 
	ADMUX  =  bit (REFS0) | bit (REFS1) | (ADC_PIN_NR & 0x07);  // ARef internal and select input port

	//****  LCD *****
	#ifdef CHARACTER_DISPLAY
		//  Wire.begin();  // I2C
		lcd.clear();
		lcd.begin(LCD_COLS, LCD_ROWS);
		// set special chars for scrollbar
		lcd.createChar(0, (uint8_t*)scroll_bar[0]);
		lcd.createChar(1, (uint8_t*)scroll_bar[1]);
		lcd.createChar(2, (uint8_t*)scroll_bar[2]);
		lcd.createChar(3, (uint8_t*)scroll_bar[3]);
		lcd.createChar(4, (uint8_t*)scroll_bar[4]);  
		//lcd.setBacklight(HIGH);
	#else
		tft.begin();
		tft.setRotation(1);
		tft.setTextColor(COLOR_TEXT, COLOR_BG);
		tft.setTextSize(TEXT_SIZE);
	#endif


	//****  RELOAD SETTINGS FROM EEPROM *****
	LoadStatesFromEEPROM();
}

void setup() {
	loadNewRadioSettings();	//Load the Radio Settings as they are stored in EEPROM

	//**** MENU *****
	LCDML_setup(_LCDML_DISP_cnt);
	LCDML.SCREEN_disable();
}

void presentation() {
	sendSketchInfo(F("nRF24_Doctor_N250"), F("1.1"));
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
				// Gateway update states
				STATE_START_GW_UPDATE, STATE_TX_GW_UPDATE, STATE_FAILED_GW_UPDATE,
};
static state currState = STATE_IDLE;

void statemachine()
{
	static unsigned long lTprevTransmit = 0;
	unsigned long iSetMsgDelay = (1000000L/iSetMsgRate);

	switch (currState)
	{
		case STATE_IDLE:
			if (bUpdateGateway)
			{
				// Start of gateway update
				currState = STATE_START_GW_UPDATE;
			}
			else
			{
				// Start of next measurement round
				if ((micros() - lTprevTransmit) >= iSetMsgDelay){currState = STATE_TX;}	//Message Rate limiter
			}
			break;

		case STATE_TX:
			{
				// Transmit Current Measurement - Trigger measurement on interrupt
				EIFR |= 0x01;					//Clear interrupt flag to prevent an immediate trigger
				attachPCINT(digitalPinToPinChangeInterrupt(MY_RF24_CE_PIN), ISR_TransmitTriggerADC,RISING);
				unsigned long lTcurTransmit = transmit(iPayloadSize);
				
				//Time rate of transmissions
				iGetMsgRate = static_cast<uint8_t>((1e6/(lTcurTransmit-lTprevTransmit))+0.5);
				lTprevTransmit = lTcurTransmit;

				if (bAdcDone) {				//Get TX Current Measurement Data...it should already have finished
					TransmitCurrent_uA 	= uAperBit1*((float)iAdcSum/(float)(iStopStorageAfterNrAdcSamples-iStartStorageAfterNrAdcSamples+1));
					bAdcDone = false;
				}				
				else{Sprintln(F("BAD ADC TIMING:"));}
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
					SleepCurrent_uA = 20000001;			//Will show ERR CAP on display
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
				// Send value for Gateway settings: xxxyz (xxx = Channel, y = PaLevel, z = DataRate)
				const uint16_t iMessageToGateway = iRf24Channel*100 + iRf24PaLevelGw*10 + iRf24DataRate;
				MsgUpdateGateway.set(iMessageToGateway);

				// Transmit message with software ack request (returned in "receive function")
				if ( send(MsgUpdateGateway, true) )
				{
					// Got a reply from gateway that message was received correctly.
					// Gateway will change to new settings, so the node can also activate the settings.
					SaveStatesToEepromAndReset();
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
			// TODO: Signal the UI that GW update failed
			// ??? LoadStatesFromEEPROM(); ???
			// Signal update sequence finished (and failed, as it didn't reset afterwards)
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
	
	// Transmit message with software ack request (returned in "receive function"),
	// the boolean returned here is a Hardware hop-to-hop Ack
	boolean success = send(MsgCounter.setDestination(iDestinationNode).set(&MessageData,iPayloadLength), true);
	if (!success) {
		lTimeDelayBuffer_FirstHop_us[iIndexInArrayTimeMessages] = 0;	//It failed, so I can't use it to determine a First Hop Delay (i.e. it is "infinite" delay as it failed)
		bArrayFailedMessages[iIndexInArrayFailedMessages] = true;	//Log it as a failed message (for rolling average)
		iNrFailedMessages++ ;
	}
	else{
		lTimeDelayBuffer_FirstHop_us[iIndexInArrayTimeMessages] = micros() - lTimeOfTransmit_us[iIndexInArrayTimeMessages];	//Log First Hop Delay in buffer
//		unsigned long temptime = lTimeDelayBuffer_FirstHop_us[iIndexInArrayTimeMessages];
		bArrayFailedMessages[iIndexInArrayFailedMessages] = false;	//Log it as a not-failed = succesful message (for rolling average)
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
/************************ CONFIGURE nRF24 RADIO FUNCTIONS ***********************/
/********************************************************************************/
void loadNewRadioSettings() {
	ClearStorageAndCounters();
	uint8_t iTempVar0 = RF24_BASE_ID_VAR[0];
	uint8_t rfsetup = ( ((iRf24DataRate & 0b10 ) << 4) | ((iRf24DataRate & 0b01 ) << 3) | (iRf24PaLevel << 1) ) + 1;		//!< RF24_RF_SETUP, +1 for Si24R1 and LNA

	RF24_setChannel(iRf24Channel);
	RF24_setRFSetup(rfsetup);
	RF24_enableFeatures();
	
	RF24_BASE_ID_VAR[0] = RF24_BROADCAST_ADDRESS;
	RF24_setPipeAddress(RF24_REG_RX_ADDR_P0 + RF24_BROADCAST_PIPE, (uint8_t*)&RF24_BASE_ID_VAR,
						RF24_BROADCAST_PIPE > 1 ? 1 : MY_RF24_ADDR_WIDTH);
	RF24_setPipeAddress(RF24_REG_RX_ADDR_P0, (uint8_t*)&RF24_BASE_ID_VAR, MY_RF24_ADDR_WIDTH);
	RF24_setPipeAddress(RF24_REG_TX_ADDR, (uint8_t*)&RF24_BASE_ID_VAR, MY_RF24_ADDR_WIDTH);
	
	RF24_BASE_ID_VAR[0] = iTempVar0;
	
	LCD_clear();
	print_LCD_line("nRF24 DOCTOR",  0, 0);
	print_LCD_line("Connecting...", 1, 0);
	Sprintln(F("Connecting..."));
	transportWaitUntilReady(10000);		// Give it 10[s] to connect, else continue to allow user to set new connection settings
	Sprintln(F("Done"));
}

/*****************************************************************/
/************************ EEPROM FUNCTIONS ***********************/
/*****************************************************************/
void loadDefaults()
{
	iRf24Channel		= DEFAULT_RF24_CHANNEL;
	iRf24PaLevel		= DEFAULT_RF24_PA_LEVEL_NODE;
	iRf24PaLevelGw		= DEFAULT_RF24_PA_LEVEL_GW;
	iRf24DataRate		= DEFAULT_RF24_DATARATE;
	iRf24BaseRadioId	= DEFAULT_RF24_BASE_ID_IDX;
	iDestinationNode	= DEFAULT_DESTINATION_NODE;
	iPayloadSize		= DEFAULT_PAYLOAD_SIZE;
	iSetMsgRate			= DEFAULT_MESSAGE_RATE;
	if (iRf24BaseRadioId < COUNT_OF(RF24_BASE_ID_VARS))
	{
		memcpy(RF24_BASE_ID_VAR, RF24_BASE_ID_VARS[iRf24BaseRadioId], sizeof(RF24_BASE_ID_VAR));
	}
}

void LoadStatesFromEEPROM()
{
	if (loadState(EEPROM_FLAG) == EEPROM_FLAG_MAGIC)
	{
		// Eeprom contents are valid
		iRf24Channel 		= loadState(EEPROM_CHANNEL);
		iRf24PaLevel 		= loadState(EEPROM_PA_LEVEL);
		iRf24PaLevelGw 		= loadState(EEPROM_PA_LEVEL_GW);
		iRf24DataRate 		= loadState(EEPROM_DATARATE);
		iRf24BaseRadioId	= loadState(EEPROM_BASE_RADIO_ID);
		iDestinationNode	= loadState(EEPROM_DESTINATION_NODE);
		iPayloadSize		= loadState(EEPROM_PAYLOAD_SIZE);
		iSetMsgRate			= loadState(EEPROM_MESSAGE_RATE);	 
		if (iRf24BaseRadioId < COUNT_OF(RF24_BASE_ID_VARS))
		{
			memcpy(RF24_BASE_ID_VAR, RF24_BASE_ID_VARS[iRf24BaseRadioId], sizeof(RF24_BASE_ID_VAR));
		}
	}
	else
	{
		// Load defaults & save default to eeprom
		loadDefaults();
		SaveStatesToEepromAndReset();
		// Never return here...
	}
}

void SaveStatesToEepromAndReset()
{
	saveState(EEPROM_CHANNEL, iRf24Channel);
	saveState(EEPROM_PA_LEVEL, iRf24PaLevel);
	saveState(EEPROM_PA_LEVEL_GW, iRf24PaLevelGw);
	saveState(EEPROM_DATARATE, iRf24DataRate);
	saveState(EEPROM_BASE_RADIO_ID, iRf24BaseRadioId);
	saveState(EEPROM_DESTINATION_NODE, iDestinationNode);
	saveState(EEPROM_PAYLOAD_SIZE, iPayloadSize);
	saveState(EEPROM_MESSAGE_RATE, iSetMsgRate);
	// Mark eeprom contents valid
	saveState(EEPROM_FLAG, EEPROM_FLAG_MAGIC);

	// Do a Soft Reset - This allows for the radio to correctly reload with the new settings from EEPROM					
	asm volatile ("  jmp 0");
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
	unsigned long lT_last_meas = lTstart;
	int n=0;
	unsigned long lTimeScaler = constrain(100/Threshold_current_uA_per_sec,100,15000);//don't measure to often as it will load the sleep current too much.
	while ((n<2) & ((millis()-lTstart)<lTimeOut)){
		delay_with_update(lTimeScaler);
		Current_uA_new 		= uAperBit3*GetAvgADCBits(iNrCurrentMeasurements);
		lT_last_meas 		= millis();
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
#ifdef CHARACTER_DISPLAY
	lcd.setCursor(col,row);
	lcd.print(string);
#else
	tft.setCursor(TEXT_WIDTH(col), TEXT_HEIGHT(row)); 
	tft.print(string);
#endif
}

void print_LCD_line(const __FlashStringHelper *string, int row, int col) {
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

/*****************************************************************/
/************************* MENU HANDLERS *************************/
/*****************************************************************/
void printBufCurrent(char *buf, int iBufSize, float fCurrent_uA, const char* label) {
	//Check range for proper displaying	
	if (fCurrent_uA==20000000){
		snprintf_P(buf, iBufSize, PSTR("%s[mA]=WAIT"), label);
	}
	else if (fCurrent_uA==20000001){
		snprintf_P(buf, iBufSize, PSTR("%s[mA]=Err CAP"), label);
	}	
	else if (fCurrent_uA > 1000){
		int Current_mA = (int)(fCurrent_uA/1000);
		if (Current_mA>=300){
			snprintf_P(buf, iBufSize, PSTR("%s[mA]= ERR"), label);
		}
		else if (Current_mA>=100){
			snprintf_P(buf, iBufSize, PSTR("%s[mA]=%4d"), label, Current_mA);
		}
		else if (Current_mA>=10){
			int iDecCurrent = (int)(((fCurrent_uA/1000)-(float)Current_mA)*10+0.5);if (iDecCurrent>=10){iDecCurrent=iDecCurrent-10;Current_mA +=1;}
			snprintf_P(buf, iBufSize, PSTR("%s[mA]=%2d.%1d"), label, Current_mA, iDecCurrent);
		}
		else {				
			int iDecCurrent = (int)(((fCurrent_uA/1000)-(float)Current_mA)*100+0.5);if (iDecCurrent >=100){iDecCurrent=iDecCurrent-100;Current_mA +=1;}
			snprintf_P(buf, iBufSize, PSTR("%s[mA]=%1d.%02d"), label, Current_mA, iDecCurrent);
		}
	}
	else if (fCurrent_uA < 100){		
		int iCurrent_uA = (int)fCurrent_uA;
		int iDecCurrent_uA = (int)((fCurrent_uA-(float)iCurrent_uA)*10+0.5); if (iDecCurrent_uA >= 10){iDecCurrent_uA=iDecCurrent_uA-10;iCurrent_uA +=1;}
			snprintf_P(buf, iBufSize, PSTR("%s[uA]=%2d.%1d"), label, iCurrent_uA, iDecCurrent_uA);
		}
	else{
		snprintf_P(buf, iBufSize, PSTR("%s[uA]=%4d"), label, (int)fCurrent_uA);
	}
}

void menuPage(uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		LCDML.FUNC_setLoopInterval(100);  // starts a trigger event for the loop function every 100 millisecounds
	}

	if (LCDML.FUNC_loop())
	{
		LCD_clear();
		char buf[LCD_COLS+1];

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
			printBufCurrent(buf,sizeof(buf), TransmitCurrent_uA, "TX");
			print_LCD_line(buf, 0, 0);
			printBufCurrent(buf,sizeof(buf), ReceiveCurrent_uA, "RX");
			print_LCD_line(buf, 1, 0);
			break;

		case PAGE_SLEEPPOWER:
			currState = STATE_SLEEP;
			printBufCurrent(buf,sizeof(buf), SleepCurrent_uA, "SLEEP");
			print_LCD_line(buf, 0, 0);
			break;

		default:
			break;
		}

		if (LCDML.BT_checkAny()) // check if any button is pressed (enter, up, down, left, right)
		{      
			LCDML.FUNC_goBackToMenu();  // leave this function
		}
	} 

	// if(LCDML.FUNC_close())
	// {    
	// }
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
		iRf24Channel = CONSTRAIN_HI( iRf24Channel, 125 );
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
		iRf24PaLevel = CONSTRAIN_HI( iRf24PaLevel, COUNT_OF(pcPaLevelNames)-1 );
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("NODE PA  %-4s"), pcPaLevelNames[iRf24PaLevel]);

	// use the line from function parameters
	lcd.setCursor(1, line);
	lcd.print(buf); 
}

void menuCfgGwPa(uint8_t line)
{ 
	if (line == LCDML.MENU_getCursorPos()) 
	{
		menuCfgEntry( iRf24PaLevelGw );
		iRf24PaLevelGw = CONSTRAIN_HI( iRf24PaLevelGw, COUNT_OF(pcPaLevelNames)-1 );
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("GW PA    %-4s"), pcPaLevelNames[iRf24PaLevelGw]);

	// use the line from function parameters
	lcd.setCursor(1, line);
	lcd.print(buf); 
}

void menuCfgRate(uint8_t line)
{ 
	if (line == LCDML.MENU_getCursorPos()) 
	{
		menuCfgEntry( iRf24DataRate );
		iRf24DataRate = CONSTRAIN_HI( iRf24DataRate, COUNT_OF(pcDataRateNames)-1 );
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("DATARATE %-4s"), pcDataRateNames[iRf24DataRate]);

	// use the line from function parameters
	lcd.setCursor(1, line);
	lcd.print(buf);
}

void menuCfgRadioId(uint8_t line)
{ 
	if (line == LCDML.MENU_getCursorPos()) 
	{
		menuCfgEntry( iRf24BaseRadioId );
		iRf24BaseRadioId = CONSTRAIN_HI( iRf24BaseRadioId, COUNT_OF(RF24_BASE_ID_VARS)-1 );
		memcpy(RF24_BASE_ID_VAR, RF24_BASE_ID_VARS[iRf24BaseRadioId], sizeof(RF24_BASE_ID_VAR));
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("%02X:%0.2X:%0.2X:%0.2X:%0.2X"), RF24_BASE_ID_VAR[0],RF24_BASE_ID_VAR[1],RF24_BASE_ID_VAR[2],RF24_BASE_ID_VAR[3],RF24_BASE_ID_VAR[4]);

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
		SaveStatesToEepromAndReset();
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
		SaveStatesToEepromAndReset();
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
	}

	if (LCDML.FUNC_loop())
	{
		static bool prevUpdateGateway = false;
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

void menuLoadNodeEeprom(__attribute__((unused)) uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		LoadStatesFromEEPROM();
		LCD_clear();
		print_LCD_line(F("Eeprom loaded"), 0, 0);
	} 
	if (LCDML.FUNC_loop())
	{
		if (LCDML.BT_checkAny()) // check if any button is pressed (enter, up, down, left, right)
		{      
			LCDML.FUNC_goBackToMenu();  // leave this function
		}
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

void menuBack(__attribute__((unused)) uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		// Go one level up
		LCDML.FUNC_goBackToMenu(1);
	} 
}
