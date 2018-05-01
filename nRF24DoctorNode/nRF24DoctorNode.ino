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
#define ENCODER_A_PIN       A0
#define ENCODER_B_PIN       A1
#define MOSFET_2P2OHM_PIN   A2
#define MOSFET_100OHM_PIN   A3
#define BUTTON_PIN          A4    // physical pin , use internal pullup
#define CURRENT_PIN         A5
#define adcPin              5     // A5, Match to CURRENT_PIN for configuring registers ADC
#define TRIGGER_PIN         A6    // Debugging purposes with scope

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
	LiquidCrystal lcd(8, 7, 6, 5, 4, 3); 								// LCD with parallel interface
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

enum page { PAGE_STATISTICS, PAGE_TIMING, PAGE_COUNTERS, PAGE_TXRXPOWER, PAGE_SLEEPPOWER };

// add            (id   prev_layer      new_num                      lang_char_array     callback_function)  
// addAdvanced    (id   prev_layer      new_num  condition           lang_char_array     callback_function  parameter (0-255)  menu function type )
//                                                                   "01234567890123"
LCDML_addAdvanced (0  , LCDML_0         , 1    , NULL              , "Statistics   >"  , menuPage         , PAGE_STATISTICS  , _LCDML_TYPE_default);
LCDML_addAdvanced (1  , LCDML_0         , 2    , NULL              , "Timing       >"  , menuPage         , PAGE_TIMING      , _LCDML_TYPE_default);
LCDML_addAdvanced (2  , LCDML_0         , 3    , NULL              , "Counters     >"  , menuPage         , PAGE_COUNTERS    , _LCDML_TYPE_default);
LCDML_addAdvanced (3  , LCDML_0         , 4    , NULL              , "TxRx Power   >"  , menuPage         , PAGE_TXRXPOWER   , _LCDML_TYPE_default);
LCDML_addAdvanced (4  , LCDML_0         , 5    , NULL              , "Sleep Power  >"  , menuPage         , PAGE_SLEEPPOWER  , _LCDML_TYPE_default);
LCDML_add         (5  , LCDML_0         , 6                        , "Settings     >"  , NULL);
LCDML_addAdvanced (6  , LCDML_0_6       , 1    , NULL              , ""                , menuCfgChannel   , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (7  , LCDML_0_6       , 2    , NULL              , ""                , menuCfgGwNode    , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (8  , LCDML_0_6       , 3    , NULL              , ""                , menuCfgGwPa      , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (9  , LCDML_0_6       , 4    , NULL              , ""                , menuCfgNodePa    , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (10 , LCDML_0_6       , 5    , NULL              , ""                , menuCfgRate      , 0                , _LCDML_TYPE_dynParam);
LCDML_addAdvanced (11 , LCDML_0_6       , 6    , NULL              , ""                , menuCfgRadioId   , 0                , _LCDML_TYPE_dynParam);
LCDML_add         (12 , LCDML_0_6       , 7                        , "Reset buff   x"  , menuResetBuf);
LCDML_add         (13 , LCDML_0_6       , 8                        , "Eeprom       >"  , NULL);
LCDML_add         (14 , LCDML_0_6_8     , 1                        , "Save         x"  , menuSaveEeprom);
LCDML_add         (15 , LCDML_0_6_8     , 2                        , "Load         x"  , menuLoadEeprom);
LCDML_add         (16 , LCDML_0_6_8     , 3                        , "Defaults     x"  , menuDefaultEeprom);
LCDML_add         (17 , LCDML_0_6_8     , 4                        , "Back         <"  , menuBack);
LCDML_add         (18 , LCDML_0_6       , 9                        , "Back         <"  , menuBack);
#define _LCDML_DISP_cnt    18   // Should equal last id in menu



LCDML_createMenu(_LCDML_DISP_cnt);

# if(_LCDML_DISP_rows > _LCDML_DISP_cfg_max_rows)
# error change value of _LCDML_DISP_cfg_max_rows in LCDMenuLib2.h
# endif

//**** ENCODER & BUTTON  *****
//#define ENCODER_OPTIMIZE_INTERRUPTS //Only when using pin2/3 (or 20/21 on mega)
#define ENCODER_DO_NOT_USE_INTERRUPTS
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
#define EEPROM_SEND_REPEATS		7

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
#define DEFAULT_RF24_CHANNEL		(MY_RF24_CHANNEL)
#define DEFAULT_RF24_PA_LEVEL_NODE	(MY_RF24_PA_LEVEL)
#define DEFAULT_RF24_PA_LEVEL_GW	(MY_RF24_PA_LEVEL)
#define DEFAULT_RF24_DATARATE		(MY_RF24_DATARATE)
#define DEFAULT_RF24_BASE_ID_IDX	(0)
#define DEFAULT_DESTINATION_NODE	(0)				// Default 0 = gateway, Settable in Menu

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

float SleepCurrent_uA 	 	= 0;
float TransmitCurrent_uA 	= 0;
float ReceiveCurrent_uA  	= 0;

//**** Configure ADC ****
volatile uint8_t iStartStorageAfterNrAdcSamples  = 7; 	//Note this depends on the set ADC prescaler (currently: 16x prescaler)
volatile uint8_t iStopStorageAfterNrAdcSamples 	= 28; 	//Note this depends on the set ADC prescaler (currently: 16x prescaler)
volatile uint16_t iAdcSum;								//Limit the number of samples to < 2^6 = 64
volatile uint8_t iNrAdcSamplesElapsed;
volatile boolean bAdcDone;

//**** Remote Gateway Update ****
uint8_t iRetryGateway 	= 0;
bool bUpdateGateway 	= 0;
bool bAckGatewayUpdate 	= 0;
const uint8_t iNrGatwayRetryOptions = 3;
const char *pcGatewayRetryNames[iNrGatwayRetryOptions] = { "SKIP GATEWAY", "RETRY GATEWAY" , "CANCEL ALL"};

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
			iStopStorageAfterNrAdcSamples 	= 12; 	//Note this depends on the set ADC prescaler (currently: 16x prescaler)	+ Matched to TX timing		
			break;
		case 1:
			iStopStorageAfterNrAdcSamples 	= 10; 	//Note this depends on the set ADC prescaler (currently: 16x prescaler)	+ Matched to TX timing
			break;
		case 2:
			iStopStorageAfterNrAdcSamples 	= 28; 	//Note this depends on the set ADC prescaler (currently: 16x prescaler)	+ Matched to TX timing		
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
	ADMUX  =  bit (REFS0) | bit (REFS1) | (adcPin & 0x07);  // ARef internal and select input port

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
enum state { STATE_IDLE, STATE_TX, STATE_TX_WAIT, STATE_SLEEP };
static state currState = STATE_IDLE;
static bool transmitAcive = true;

void statemachine()
{
	static unsigned long lLastTransmit = 0;

//	state prevState = currState;

	switch (currState)
	{
		case STATE_IDLE:
			// Do nothing, wait for start of next measurement
			if (transmitAcive)
			{
				currState = STATE_TX;
			}
			break;

		case STATE_TX:
			//Transmit Current Measurement
			EIFR |= 0x01;					//Clear interrupt flag to prevent an immediate trigger
			attachPCINT(digitalPinToPinChangeInterrupt(MY_RF24_CE_PIN), ISR_TransmitTriggerADC,RISING);
			transmit();
			lLastTransmit = micros();
			currState = STATE_TX_WAIT;
			break;			

		case STATE_TX_WAIT:
			if (micros() - lLastTransmit >= 100000)
			{
				//Calculate Mean and Max Delays
				getMeanAndMaxFromArray(&iMeanDelayFirstHop_ms,&iMaxDelayFirstHop_ms,lTimeDelayBuffer_FirstHop_us,iNrTimeDelays);
				getMeanAndMaxFromArray(&iMeanDelayDestination_ms,&iMaxDelayDestination_ms,lTimeDelayBuffer_Destination_us,iNrTimeDelays);

				//Did we have a correct trigger of the ADC (i.e. a correct transmit), then it should have completed already...
				if (bAdcDone) {
					TransmitCurrent_uA 	= uAperBit1*((float)iAdcSum/(float)(iStopStorageAfterNrAdcSamples-iStartStorageAfterNrAdcSamples+1));
					ReceiveCurrent_uA 	= uAperBit1*GetAvgADCBits(iNrCurrentMeasurements);
				} else {
					// Current Measurement could not be completed...probably because the transmit failed
					TransmitCurrent_uA 	= 10000000;	//Will show ERR on display
					ReceiveCurrent_uA 	= 10000000;	//Will show ERR on display
				}
//				Sprint(F("TransmitCurrent_uA:"));Sprintln(TransmitCurrent_uA);
//				Sprint(F("ReceiveCurrent_uA:"));Sprintln(ReceiveCurrent_uA);
				currState = STATE_SLEEP;
			}
			break;

		case STATE_SLEEP:
			//Sleep Current Measurement
			transportDisable();
			delay(10);									//Gate charge time and settle time, don't use wait as it will prevent the radio from sleep
			SleepCurrent_uA = uAperBit1*GetAvgADCBits(iNrCurrentMeasurements);
			if (SleepCurrent_uA < 10000){
				//Set Higher Sensitivity: uAperBit2
				digitalWrite(MOSFET_2P2OHM_PIN, LOW);
				digitalWrite(MOSFET_100OHM_PIN, HIGH);
				delay(10);								//settle time
				SleepCurrent_uA = uAperBit2*GetAvgADCBits(iNrCurrentMeasurements);
			}
			if (SleepCurrent_uA < 100){
				//Set Higher Sensitivity: uAperBit3
				digitalWrite(MOSFET_2P2OHM_PIN, LOW);
				digitalWrite(MOSFET_100OHM_PIN, LOW);
				delay(10);								//settle time
				SleepCurrent_uA = uAperBit3*GetAvgADCBits(iNrCurrentMeasurements);
			}
//			Sprint(F("SleepCurrent_uA:"));Sprintln(SleepCurrent_uA);
			
			//Restore standby power state
			digitalWrite(MOSFET_2P2OHM_PIN, HIGH);	//Enable 2.2Ohm
			digitalWrite(MOSFET_100OHM_PIN, LOW);
			transportStandBy();

			currState = STATE_IDLE;
			break;			

		default:
			break;
	}
	// if (currState != prevState)
	// {
	// 	Sprint(prevState); Sprint(F(" -> ")); Sprintln(currState);
	// }
}

/*****************************************************************************/
/************************ RECEIVE & TRANSMIT FUNCTIONS ***********************/
/*****************************************************************************/
void receive(const MyMessage &message) {
	if (message.isAck() == 1 && message.type == V_CUSTOM && message.sensor==CHILD_ID_COUNTER){	//Acknowledge message & of correct type
		uint16_t iNewMessage = message.getUInt();           // get received value
		uint16_t iIndexInArray = iNewMessage % iMaxNumberOfMessages;
		bArrayNAckMessages[iIndexInArray] = 0; 			// set corresponding flag to received.
		
		// Check Message (Round trip) Delay
		uint8_t iIndexInTimeArray = IndexOfValueInArray(iNewMessage, iMessageIndexBuffer, iNrTimeDelays); //Look-up if message is present in MessageIndexBuffer for delay calculation
		if ((iIndexInTimeArray != 255) && iIndexInTimeArray <=iNrTimeDelays){
			lTimeDelayBuffer_Destination_us[iIndexInTimeArray] = micros()-lTimeOfTransmit_us[iIndexInTimeArray];
		}
		iNrNAckMessages--;	//Received an Acknowledge Message (so one less No Ack)
	}
	
	// Gateway Update Acknowledge - if we have received this we can "safely" apply the new settings to this node
	if (message.isAck() == 1 && message.type == V_CUSTOM && message.sensor==CHILD_ID_UPDATE_GATEWAY){	//Acknowledge message & of correct type & Sensor
		bAckGatewayUpdate = 1;
	}
}

void transmit() {
	static int iIndexInArrayFailedMessages  = 0 ;
	static int iIndexInArrayTimeMessages  = 0 ;	

	iMessageCounter++;
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
	boolean success = send(MsgCounter.setDestination(iDestinationNode).set(iMessageCounter), true);
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
}

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

void loadNewRadioSettingsGateway() {
	// Yveaux: This needs to land in a statemachine!

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
// TODO			opState = STATE_RUN;
			LoadStatesFromEEPROM();
//			bDspRefresh = true;
			break;
	}
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
		if (iRf24BaseRadioId < COUNT_OF(RF24_BASE_ID_VARS))
		{
			memcpy(RF24_BASE_ID_VAR, RF24_BASE_ID_VARS[iRf24BaseRadioId], sizeof(RF24_BASE_ID_VAR));
		}
	}
	else
	{
		// Load defaults & save default to eeprom
		loadDefaults();
		SaveStatesToEEPROM();
	}
}

void SaveStatesToEEPROM()
{
	saveState(EEPROM_CHANNEL, iRf24Channel);
	saveState(EEPROM_PA_LEVEL, iRf24PaLevel);
	saveState(EEPROM_PA_LEVEL_GW, iRf24PaLevelGw);
	saveState(EEPROM_DATARATE, iRf24DataRate);
	saveState(EEPROM_BASE_RADIO_ID, iRf24BaseRadioId);
	saveState(EEPROM_DESTINATION_NODE, iDestinationNode);
	// Mark eeprom contents valid
	saveState(EEPROM_FLAG, EEPROM_FLAG_MAGIC);
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
	while (!bAdcDone){delay(1);};			//Wait until all ADC conversions have completed
	bAdcDone 				= false;
	return ((float)iAdcSum/(float)(iNrSamples));
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
	if (fCurrent_uA > 1000){
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

	if(LCDML.FUNC_close())
	{    
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

void menuCfgChannel(uint8_t line)
{ 
	if (line == LCDML.MENU_getCursorPos()) 
	{
		menuCfgEntry( iRf24Channel );
		iRf24Channel = CONSTRAIN_HI( iRf24Channel, 125 );
	} 

	char buf[LCD_COLS+1];
	snprintf_P(buf, sizeof(buf), PSTR("Channel   %d"), iRf24Channel);

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
	snprintf_P(buf, sizeof(buf), PSTR("GW node   %d"), iDestinationNode);

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
	snprintf_P(buf, sizeof(buf), PSTR("Node pa   %s"), pcPaLevelNames[iRf24PaLevel]);

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
	snprintf_P(buf, sizeof(buf), PSTR("GW pa     %s"), pcPaLevelNames[iRf24PaLevelGw]);

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
	snprintf_P(buf, sizeof(buf), PSTR("Datarate  %s"), pcDataRateNames[iRf24DataRate]);

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

void menuSaveEeprom(__attribute__((unused)) uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		// TODO: Start the GW update sequence too!
		// loadNewRadioSettingsGateway();
		SaveStatesToEEPROM();
		LCDML.FUNC_goBackToMenu();
	} 
}

void menuLoadEeprom(__attribute__((unused)) uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		LoadStatesFromEEPROM();
		LCDML.FUNC_goBackToMenu();
	} 
}

void menuDefaultEeprom(__attribute__((unused)) uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		// TODO: Start the GW update sequence too!
		// loadNewRadioSettingsGateway();
		loadDefaults();
		SaveStatesToEEPROM();
		LCDML.FUNC_goBackToMenu();
	} 
}

void menuResetBuf(__attribute__((unused)) uint8_t param)
{
	if (LCDML.FUNC_setup())
	{
		ClearStorageAndCounters();
//  ... Maybe some done message ...
//	LCD_clear();
//	char buf[LCD_COLS+1];
//	print_LCD_line("Done", 0, 0);
//  wait some time...

		LCDML.FUNC_goBackToMenu();
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

/*
void onButton2Pressed() {	//Apply/Select change (if available). In Viewing windows: toggle (reversed)
	bDspRefresh = true;
	switch (opState) {
		
		//Select Radio changes:
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
			break;
	}
}

void LCD_local_display(void) {
	static mode prevOpState = STATE_RUN;	//Remember previous state to allow for partial LCD updates
	char buf[LCD_COLS+1];
	bDspRefresh = false;
	
	switch (opState) {
		case STATE_ASK_GATEWAY:
		{
			LCD_clear();
			snprintf_P(buf, sizeof(buf), PSTR("UPDATE GATEWAY?:"));
			print_LCD_line(buf, 0, 0);
			if (bUpdateGateway){
				snprintf_P(buf, sizeof(buf), PSTR("YES"));
				print_LCD_line(buf, 1, 13);
			}
			else{
				snprintf_P(buf, sizeof(buf), PSTR("NO"));
				print_LCD_line(buf, 1, 14);
			}
			break;
		}			
		case STATE_UPDATE_GATEWAY:
		{
			LCD_clear();
			snprintf_P(buf, sizeof(buf), PSTR("FAILED GATEWAY!"));
			print_LCD_line(buf, 0, 0);
			snprintf_P(buf, sizeof(buf), PSTR("%s"),pcGatewayRetryNames[iRetryGateway]);
			print_LCD_line(buf, 1, 0);
			break;								
		}	
	}
	prevOpState = opState;
}
*/