// Do not remove the include below
#include "BeerLoggerEc.h"

#include <LiquidCrystal.h>
#include <Wire.h>
#include "RTClib.h"
#include <PinChangeInt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "SD.h"
#include <SPI.h>

/// Liquid sensor
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 22
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

/// Air sensor
//TMP36 Pin Variables
int sensorPin = A8; //the analog pin the TMP36's Vout (sense) pin is connected to
int refPin = A7;

/// Rotary encoder
enum PinAssignments {
  encoderPinA = A11,   // right (labeled DT on our decoder, yellow wire)
  encoderPinB = A10,   // left (labeled CLK on our decoder, green wire)
  encoderSW = A12,    // switch (labeled SW on our decoder, orange wire)
// connect the +5v and gnd appropriately
  /// Write mode switch
  clearButton = A13,
};


enum UiActions {
	UI_DISPLAY,
	UI_ENC_UP,
	UI_ENC_DOWN,
	UI_ENC_SW,
	UI_CLEAR,
	UI_ENTER,
	UI_LEAVE
};
enum UiResults {
	RET_STAY,
	RET_HOME,
	RET_CONTINUE
};

// I'd like to have a better way to define this. Right now it's a bit murky
#define UI_TARGET_NUM 3
enum UiTargets {
	UIT_TEMP_DISPLAY = 0,
	UIT_LOGGER_SETTINGS = 1,
	UIT_MESSAGE = 2,
	UIT_DUMMY = -1
};

/// UI handler
// Returns:
// true if the UI leaves, false if it stays
// takes as parameter:
// int: the UiAction
// void *: a pointer to a custom "static structure" containing the UI state
typedef int (* UiTarget) (int);
volatile UiTarget uiTargets[UI_TARGET_NUM] = {
		&uiTempDisplay,
		&uiLoggerSettings,
		&uiMessage,
};
int uiTargetContinueMap[UI_TARGET_NUM] = {
		UIT_LOGGER_SETTINGS,
		UIT_TEMP_DISPLAY,
		UIT_DUMMY
};

volatile int uiTarget;

/// Rotary encoder
static boolean rotating=false;      // debounce management
// Rotary encoder: interrupt service routine vars
boolean A_set = false;
boolean B_set = false;

/// LCD
LiquidCrystal lcd(35,34, 30,31,32,33);

/// RTC
RTC_DS1307 RTC;


/// Software
String message = "";

volatile int logInterval = 10;

typedef void (* ScheduleFP)(void);

#define SCHEDULE_EVENTS_NO 5

enum scheduleEvents {
  updateScreen = 0,
  cycle = 1,
  readAirTemp = 2,
  clearDebounce = 3,
  manageSD = 4
  };

volatile long scheduleTime[SCHEDULE_EVENTS_NO] =
{
  2000,
  1000 * logInterval,
  0,
  0,
  -1
};

boolean scheduleActive[SCHEDULE_EVENTS_NO] =
{
   true,
   true,
   false,
   false,
   true
};

// 0: trigger on start
// 1+: trigger after n ms
// -1: do not trigger in start schedule
unsigned long scheduleTarget[SCHEDULE_EVENTS_NO] =
{
  0,
  0,
  -1,
  -1,
  0 // init SD on startup
  };

volatile long scheduleCommand[SCHEDULE_EVENTS_NO] =
{
  -1,-1,-1,-1,-1
  };

ScheduleFP scheduleFunc[SCHEDULE_EVENTS_NO] =
{
  &fpUpdateScreen,
  &fpCycle,
  &fpReadAirTemp,
  &fpClearDebounce,
  &fpManageSD
  };

volatile boolean eventsExecuted[SCHEDULE_EVENTS_NO] =
{
  false,false,false,false,false
  };



#define AIR_TEMP_BUF_SIZE 10
float airTempBuffer[AIR_TEMP_BUF_SIZE];
int airTempBufferPos = 0;
int airTempBufferPosMax = 0;

volatile byte screenPos = 0;
byte lastWrite = 0;
byte bufferPos = 0;
boolean liveWrite = true;
File logfile;

#define SENSOR_COUNT 2
float dataBuffer[256][SENSOR_COUNT];
DateTime tsBuffer[256];

#define DEBOUNCE_DELAY 400
volatile boolean debouncing = false;

#define LOOP_DELAY 40
const boolean delayLoop = true;

void setup() {
  // put your setup code here, to run once:
  /// Rotary encoder
  pinMode(encoderPinA, INPUT_PULLUP); // new method of enabling pullups
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderSW, INPUT_PULLUP);
  pinMode(clearButton, INPUT_PULLUP);
  // encoder pin on PCE (pin a)
  attachPinChangeInterrupt(encoderPinA, doEncoderA, CHANGE);
  // encoder pin on PCE (pin b)
  attachPinChangeInterrupt(encoderPinB, doEncoderB, CHANGE);

  attachPinChangeInterrupt(encoderSW, doEncSw, RISING);
  attachPinChangeInterrupt(clearButton, doClearButton, RISING);


  /// RTC
  Wire.begin();
  RTC.begin();

  /// LCD
  lcd.begin(16, 2);  // set up the LCD's number of columns and rows:

  /// Liquid sensor
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement

  /// Software: Init buffer to 0
  DateTime ts = RTC.now();
  for(int n1=0; n1<256; n1++){
    tsBuffer[n1] = ts;
    for(int n2=0; n2<SENSOR_COUNT; n2++)
      dataBuffer[n1][n2] = 0;
  }

}

void loop() {
  /// Rotary encoder
  rotating = true;
  // put your main code here, to run repeatedly:


  // Scheduler: execute pending events
  unsigned long ms = millis();
  for(int n=0; n<SCHEDULE_EVENTS_NO; n++)
   {
     if((scheduleTarget[n] < ms) && (scheduleActive[n]))
     {
       (scheduleFunc[n])();
       scheduleActive[n] = false;
       scheduleEvent(n, scheduleTime[n]);
     }
     //else
     //  eventsExecuted[n] = false;
   }

  // Scheduler: add new events
  ms = millis();
  for(int n=0; n<SCHEDULE_EVENTS_NO; n++)
   {
     if(eventsExecuted[n])
     {
       if(scheduleCommand[n] != -1)
       {
         scheduleTarget[n] = ms + scheduleCommand[n];
         scheduleActive[n] = true;
       }
       eventsExecuted[n] = false;
     }
   }

  if(delayLoop)
    delay(LOOP_DELAY);
}

/// Software: Scheduler
void scheduleEvent(int eventId, long tDelay)
{
  scheduleCommand[eventId] = tDelay;
  eventsExecuted[eventId] = true;
}



/// Software: LCD screen
void fpUpdateScreen()
{
  lcd.clear();

  handleUi(UI_DISPLAY);

}
void fpCycle()
{
  bufferPos++;
  // Keep the screen at the old position if it was not on liveshow (pos 0)
  if(screenPos != 0) screenPos++;
  // read date/time and temperatures into current buffer
  tsBuffer[bufferPos] = RTC.now();
  dataBuffer[bufferPos][0] = getAirTemp();
  dataBuffer[bufferPos][1] = getLiquidTemp();

  if(liveWrite)
  {
    while(lastWrite != bufferPos)
    {
      writeLog(lastWrite);
      lastWrite++;
    }
    logfile.flush();
  }
  //if(!messageState)
  //scheduleEvent(updateScreen,1);

}

float getAirTemp()
{
   // Read multiple times to get a halfway decent result
   long reading = analogRead(sensorPin);
   long ref = analogRead(refPin);
   reading = 0;
   ref = 0;
   for(int n=0;n<8;n++)
   {
	   delay(20);
	   reading += analogRead(sensorPin);
	   ref += analogRead(refPin);
   }

  float voltage = 3.300 * reading / ref;

  float temperatureC = (voltage - 0.5) * 100 ;
  return(temperatureC);
}

//float getAirTemp()
//{
//  float tempTot = 0;
//  int tempCnt = 0;
//  int pos = 0;
//  while(pos < airTempBufferPosMax)
//  {
//    tempTot += airTempBuffer[pos];
//    pos++;
//  }
//  tempTot = tempTot / (airTempBufferPosMax + 1);
//
//  airTempBufferPos = 0;
//  airTempBufferPosMax = 0;
//
//  return tempTot;
//
//}

float getLiquidTemp()
{
  sensors.requestTemperatures(); // Send the command to get temperatures
  return(sensors.getTempCByIndex(0)); // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
}

void fpReadAirTemp()
{

// Deactivated until I fix the problem. I don't get it
//  // Wraparound
//  if(airTempBufferPos >= AIR_TEMP_BUF_SIZE)
//    airTempBufferPos = 0;
//  // Adjust "divisor"
//  if(airTempBufferPos >= airTempBufferPosMax)
//    airTempBufferPosMax = airTempBufferPos;
//
//  // Read temperature and convert to C
//  int reading = analogRead(sensorPin);
//  float voltage = reading * 4.84;
//  voltage /= 1024.0;
//  float temperatureC = (voltage - 0.5) * 100 ;
//  airTempBuffer[airTempBufferPos] = temperatureC;
//
//  airTempBufferPos++;
}


void writeLog(int index)
{
  DateTime ts = tsBuffer[index];
  float tAir = dataBuffer[index][0];
  float tLiquid = dataBuffer[index][1];
  logfile.print(ts.unixtime());
  logfile.print(";");
  logfile.print(tAir);
  logfile.print(";");
  logfile.print(tLiquid);
  logfile.println();
}

void fpClearDebounce(){
  debouncing = false;
}

void fpManageSD(){
  // This function is called after the toggle was pressed.
  // Set the SD to the correct state now
  if(liveWrite)
  {
    if (!SD.begin(10,11,12,13)) {
      setMessage("SD init failed");
      liveWrite = false;
    }
    else
      logfile = SD.open("log.txt", FILE_WRITE);
  }
  else
  {
    logfile.close();
    SD.end();
  }
}

/// Software: helper to get index position


/// Software: Screen messages on errors etc
void setMessage(String msg){
  message = msg;
  handleUi(UI_LEAVE);
  uiTarget = UIT_MESSAGE;
  scheduleEvent(updateScreen, 1);
}

/// Encoder: rotator handling

// Interrupt on A changing state
void doEncoderA(){
  // debounce
  if ( rotating ) delay (10);  // wait a little until the bouncing is done

  // Test transition, did things really change?
  if( digitalRead(encoderPinA) != A_set ) {  // debounce once more
    A_set = !A_set;

    // adjust counter + if A leads B
    if ( A_set && !B_set )
    {
    	handleUi(UI_ENC_UP);
    }

    rotating = false;  // no more debouncing until loop() hits again
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB(){
  if ( rotating ) delay (10);
  if( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if( B_set && !A_set )
    {
    	handleUi(UI_ENC_DOWN);
    }

    rotating = false;
  }
}

void doEncSw(){
	if(debouncing) return;
	handleUi(UI_ENC_SW);

	debouncing = true;
	scheduleEvent(clearDebounce, DEBOUNCE_DELAY);
}

void doClearButton()
{
	if(debouncing) return;
	handleUi(UI_CLEAR);
	// debounce
	debouncing = true;
	scheduleEvent(clearDebounce, DEBOUNCE_DELAY);
}


void handleUi(int action)
{
	int ret;

	ret = (uiTargets[uiTarget])(action);
	switch(ret)
	{
	case RET_HOME:
		uiTarget = UIT_TEMP_DISPLAY;
	    scheduleEvent(updateScreen, 1);
		break;
	case RET_CONTINUE:
		uiTarget = uiTargetContinueMap[uiTarget];
	    scheduleEvent(updateScreen, 1);
		break;
	case RET_STAY:
		break;
	}
}

int uiTempDisplay(int action)
{
	int ret = 0;
	int sp = screenPos;
	switch(action)
	{
	case UI_LEAVE:
		// in this case, do nothing because the difference is handled
		// by UI_ENC_SW vs UI_CLEAR
		break;
	case UI_ENTER:
		break;
	case UI_ENC_UP:
		sp -= 1;
		if(sp < 0) sp = 0;
		screenPos = sp;
	    scheduleEvent(updateScreen, 1);
		break;
	case UI_ENC_DOWN:
		sp += 1;
		screenPos = sp;
	    scheduleEvent(updateScreen, 1);
		break;
	case UI_ENC_SW:
		ret = RET_CONTINUE;
		break;
	case UI_CLEAR:
		toggleWriteMode();
		break;
	case UI_DISPLAY:
		mainDisplay();
		break;
	}
	return(ret);
}

int uiLoggerSettings(int action)
{
	//static int intvSet;
	int ret = 0;
	long li = logInterval;
	switch(action)
	{
	case UI_LEAVE:
		// in this case, do nothing because the difference is handled
		// by UI_ENC_SW vs UI_CLEAR
		break;
	case UI_ENTER:
		break;
	case UI_ENC_UP:
		li += 1;
		if(li > 1000) li = 1000;
		logInterval = li;
	    scheduleEvent(updateScreen, 1);
		break;
	case UI_ENC_DOWN:
		li -= 1;
		if(li < 5) li = 5;
		logInterval = li;
	    scheduleEvent(updateScreen, 1);
		break;
	case UI_ENC_SW:
		scheduleTime[cycle] = 1000 * li;
		scheduleEvent(cycle, scheduleTime[cycle]);
		ret = RET_CONTINUE;
		break;
	case UI_CLEAR:
		ret = RET_HOME;
		break;
	case UI_DISPLAY:
		lcd.setCursor(0,0);
		lcd.print("Interval:");
		lcd.setCursor(0,1);
		lcd.print(logInterval);
		lcd.print(" ");
		lcd.print(scheduleTime[cycle]);
		break;
	}
	return(ret);
}

int uiMessage(int action)
{
	int ret = RET_STAY;
	long li = logInterval;
	switch(action)
	{
	case UI_LEAVE:
		break;
	case UI_ENTER:
		break;
	case UI_ENC_UP:
		break;
	case UI_ENC_DOWN:
		break;
	case UI_ENC_SW:
		ret = RET_HOME;
		break;
	case UI_CLEAR:
		ret = RET_HOME;
		break;
	case UI_DISPLAY:
	    lcd.print(message);
	    break;
	}
	return(ret);
}


void mainDisplay()
{
    char outString[16];

    byte screenBufPos = bufferPos - screenPos;
    DateTime ts = tsBuffer[screenBufPos];
    float tAir = dataBuffer[screenBufPos][0];
    float tLiquid = dataBuffer[screenBufPos][1];
     lcd.setCursor(0,0);

     lcd.print(screenBufPos);

     lcd.setCursor(4,0);
     //sprintf(outString, "%4.1f", tAir);
     lcd.print(tAir,1);
     lcd.print(" ");
     //sprintf(outString, "%4.1f", tLiquid);
     //lcd.print(outString);
     lcd.print(tLiquid,1);



    lcd.setCursor(0,1);
    sprintf(outString, "%02d/%02d %02d:%02d:%02d",
    		ts.month(), ts.day(), ts.hour(), ts.minute(), ts.second());
    lcd.print(outString);

    lcd.setCursor(15,1);
    if(liveWrite)
      lcd.print('W');
    else
      lcd.print('B');

    if(screenPos == 0)
    {
    	lcd.setCursor(0,0);
    	lcd.cursor();
    }
    else
    {
    	lcd.noCursor();
    }
}

void toggleWriteMode(){
  liveWrite = !liveWrite;
  scheduleEvent(manageSD, 1);

  scheduleEvent(updateScreen, 1);
}
