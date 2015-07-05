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
#define UI_TARGET_NUM 6
enum UiTargets {
	UIT_TEMP_DISPLAY = 0,
	UIT_LOGGER_SETTINGS = 1,
	UIT_MESSAGE = 2,
	UIT_THERMOSTAT_SETTINGS = 3,
	UIT_THERMOSTAT_MODE = 4,
	UIT_LOAD_STORE_SETTINGS = 5,
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
		&uiThermostatSettings,
		&uiThermostatMode,
		&uiLoadStoreSettings,
};
int uiTargetContinueMap[UI_TARGET_NUM] = {
		UIT_LOGGER_SETTINGS, // from UIT_TEMP_DISPLAY
		UIT_THERMOSTAT_SETTINGS, // from UIT_LOGGER_SETTINGS
		UIT_DUMMY, // from UIT_MESSAGE (because it always returns RET_HOME)
		UIT_THERMOSTAT_MODE, // from UIT_THERMOSTAT_SETTINGS
		UIT_LOAD_STORE_SETTINGS, // from UIT_THERMOSTAT_MODE
		UIT_TEMP_DISPLAY, // from UIT_LOAD_STORE_SETTINGS

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

#define SCHEDULE_EVENTS_NO 7

enum scheduleEvents {
  updateScreen = 0,
  cycle = 1,
  readAirTemp = 2,
  clearDebounce = 3,
  manageSD = 4,
  settingsLoad = 5,
  settingsStore = 6,
  };

volatile long scheduleTime[SCHEDULE_EVENTS_NO] =
{
  2000,
  1000 * logInterval,
  0,
  0,
  -1,
  -1,
  -1,
};

boolean scheduleActive[SCHEDULE_EVENTS_NO] =
{
   true,
   true,
   false,
   false,
   true,
   false,
   false,
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
  0, // init SD on startup
  -1,
  -1,
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
  &fpManageSD,
  &fpSettingsLoad,
  &fpSettingsStore,
  };

volatile boolean eventsExecuted[SCHEDULE_EVENTS_NO] =
{
  false,false,false,false,false,false,false
  };



#define AIR_TEMP_BUF_SIZE 10
float airTempBuffer[AIR_TEMP_BUF_SIZE];
int airTempBufferPos = 0;
int airTempBufferPosMax = 0;

volatile byte screenPos = 0;
byte lastWrite = 0;
byte bufferPos = 0;
boolean liveWrite = true;
boolean startupSettingsLoaded = false;
File logfile;

#define SENSOR_COUNT 2
float dataBuffer[256][SENSOR_COUNT];
DateTime tsBuffer[256];

float thermostatSettings[4] = {
		0, // target temperature
		1, // target temperature window
		0, // temperature undershoot
		0, // temperature overshoot
};
enum thermostatModes {
	THERMOSTAT_OFF = 0,
	THERMOSTAT_HEAT = 1,
	THERMOSTAT_COOL = 2,
};
volatile int thermostatMode = THERMOSTAT_OFF;

#define RELAY_PIN 40
bool relayState = false;
int relayStartupDelay = 3;


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
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

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

/// Software: Load/store settings
void fpSettingsLoad()
{
	File settingsFile;
	char character;
	String settingName;
	String settingValue;
	if(liveWrite)
	{
		// Close logfile and open settings file
		logfile.close();
		settingsFile = SD.open("settings.txt");

		char character;
		String settingName;
		String settingValue;

		if (settingsFile) {
			while (settingsFile.available()) {
				character = settingsFile.read();
				while((settingsFile.available()) && (character != '[')){
					character = settingsFile.read();
				}
				character = settingsFile.read();
				while((settingsFile.available()) && (character != '=')){
					settingName = settingName + character;
					character = settingsFile.read();
				}
				character = settingsFile.read();
				while((settingsFile.available()) && (character != ']')){
					settingValue = settingValue + character;
					character = settingsFile.read();
				}
				if(character == ']'){

					// Apply the value to the parameter
					settingApply(settingName,settingValue);
					// Reset Strings
					settingName = "";
					settingValue = "";
				}
			}
			settingsFile.close();
		}
		else
		{

			// if the file didn't open, print an error:
			setMessage("error loading");
		}

		// Reopen the logfile at the end
		logfile = SD.open("log.txt", FILE_WRITE);
	}
	else
	{
		setMessage("SD inactive");
	}

}

void fpSettingsStore()
{

	File settingsFile;
	char character;
	String settingName;
	String settingValue;
	if(liveWrite)
	{
		logfile.close();
		 SD.remove("settings.txt");
		 // Create new one
		 settingsFile = SD.open("settings.txt", FILE_WRITE);
	}
	// Delete the old One
	 SD.remove("settings.txt");
	 // Create new one
	 settingsFile = SD.open("settings.txt", FILE_WRITE);
	 // writing in the file works just like regular print()/println() function

	 settingsFile.println(settingPrint("logInterval", String(logInterval) ));
	 settingsFile.println(settingPrint("tempTarget", String(thermostatSettings[0],1) ));
	 settingsFile.println(settingPrint("tempRange", String(thermostatSettings[1],1) ));
	 settingsFile.println(settingPrint("tempUndershoot", String(thermostatSettings[2],1) ));
	 settingsFile.println(settingPrint("tempOvershoot", String(thermostatSettings[3],1) ));

	 String tsMode = "";
	 switch(thermostatMode)
	 {
	 case THERMOSTAT_HEAT:
		 tsMode = "H";
		 break;
	 case THERMOSTAT_COOL:
		 tsMode = "C";
		 break;
	 case THERMOSTAT_OFF:
		 tsMode = "X";
		 break;
	 }
	 settingsFile.println(settingPrint("thermostatMode", tsMode ));

	 // close the file:
	 settingsFile.close();
	 //Serial.println("Writing done.");

	 logfile = SD.open("log.txt");

}

void fpCycle()
{
  bufferPos++;
  // Keep the screen at the old position if it was not on liveshow (pos 0)
  if(screenPos != 0) screenPos++;
  // read date/time and temperatures into current buffer

  float liquidTemp = getLiquidTemp();

  tsBuffer[bufferPos] = RTC.now();
  dataBuffer[bufferPos][0] = getAirTemp();
  dataBuffer[bufferPos][1] = liquidTemp;

  controlRelay(liquidTemp);

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
  logfile.print(";");
  logfile.print(relayState);
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
    {
      logfile = SD.open("log.txt", FILE_WRITE);
      if(!startupSettingsLoaded)
    	  scheduleEvent(settingsLoad, 1);
    }
  }
  else
  {
    logfile.close();
    SD.end();
  }

  // Regardless whether or not it worked:
  startupSettingsLoaded = true;
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
		(uiTargets[uiTarget])(UI_LEAVE);
		uiTarget = UIT_TEMP_DISPLAY;
		(uiTargets[uiTarget])(UI_ENTER);
	    scheduleEvent(updateScreen, 1);
		break;
	case RET_CONTINUE:
		(uiTargets[uiTarget])(UI_LEAVE);
		uiTarget = uiTargetContinueMap[uiTarget];
		(uiTargets[uiTarget])(UI_ENTER);
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

int uiThermostatSettings(int action)
{
	int ret = RET_STAY;
	static float s[4];
	static int sPos = 0;

	switch(action)
	{
	case UI_LEAVE:
		break;
	case UI_ENTER:
		for(int c = 0; c < 4; c++)
		{
			s[c] = thermostatSettings[c];
		}
		sPos = 0;
		break;
	case UI_ENC_UP:
		s[sPos] += 0.1;
	    scheduleEvent(updateScreen, 1);
		break;
	case UI_ENC_DOWN:
		s[sPos] -= 0.1;
	    scheduleEvent(updateScreen, 1);
		break;
	case UI_ENC_SW:
		sPos++;
		for(int c = 0; c < 4; c++)
		{
			thermostatSettings[c] = s[c];
		}
		if(sPos == 4)
			ret = RET_CONTINUE;
		else
		    scheduleEvent(updateScreen, 1);
		break;
	case UI_CLEAR:
		ret = RET_HOME;
		break;
	case UI_DISPLAY:
	    thermostatSettingsDisplay(s, sPos, -1);
	    break;
	}
	return(ret);
}

int uiLoadStoreSettings(int action)
{
	int ret = RET_STAY;
	static int option = 0;

	switch(action)
	{
	case UI_LEAVE:
		break;
	case UI_ENTER:
		option = 0;
		break;
	case UI_ENC_UP:
		option++;
		if(option > 2) option = 0;
		scheduleEvent(updateScreen, 1);
		break;
	case UI_ENC_DOWN:
		option--;
		if(option < 0) option = 2;
		scheduleEvent(updateScreen, 1);
		break;
	case UI_ENC_SW:
		switch(option)
		{
		case 0:
			break;
		case 1:
			scheduleEvent(settingsStore, 1);
			break;
		case 2:
			scheduleEvent(settingsLoad, 1);
			break;
		}
		ret = RET_CONTINUE;
		break;
	case UI_CLEAR:
		ret = RET_HOME;
		break;
	case UI_DISPLAY:
		lcd.setCursor(0,0);
		lcd.print("Load/store:");
		lcd.setCursor(0,1);
		switch(option)
		{
		case 0:
			lcd.print("Cancel");
			break;
		case 1:
			lcd.print("Store");
			break;
		case 2:
			lcd.print("Load");
			break;
		}
		ret = RET_STAY;
		break;
	}
	return(ret);
}

int uiThermostatMode(int action)
{
	int ret = RET_STAY;
	static int thMode = 0;

	switch(action)
	{
	case UI_LEAVE:
		break;
	case UI_ENTER:
		thMode = thermostatMode;
		break;
	case UI_ENC_UP:
		thMode++;
		if(thMode > 2)
			thMode = 0;
	    scheduleEvent(updateScreen, 1);
		break;
	case UI_ENC_DOWN:
		thMode--;
		if(thMode < 0)
			thMode = 2;
		scheduleEvent(updateScreen, 1);
		break;
	case UI_ENC_SW:
		thermostatMode = thMode;
		ret = RET_CONTINUE;
		break;
	case UI_CLEAR:
		ret = RET_HOME;
		break;
	case UI_DISPLAY:
	    thermostatSettingsDisplay(NULL, -1, thMode);
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

void thermostatSettingsDisplay(float * s, int sPos, int thMode)
{
	char outString[25]; // just to be safe that we will never write into strange memory

	if(s == NULL)
	{
		s = thermostatSettings;
		sPos = -1;
	}
	else
		thMode = thermostatMode;

	// row 1: Set: 12.1 +- 0.3
//	lcd.setCursor(0,0);
//	sprintf(outString, "Set:%3.1f+-%1.1f", s[0], s[1]);
//	lcd.print(outString);
	lcd.setCursor(0,0);
	lcd.print("Set:");
	lcd.print(s[0], 1);
	lcd.setCursor(9,0);
	lcd.print("+-");
	lcd.print(s[1],1);

	// row 2: OS:0.1 US:0.1 H

//	sprintf(outString, "OS:%1.1f US:%1.1f", s[2], s[3]);
//	lcd.setCursor(0,1);
//	lcd.print(outString);
	lcd.setCursor(0,1);
	lcd.print("OS:");
	lcd.print(s[2],1);
	lcd.setCursor(7,1);
	lcd.print("US:");
	lcd.print(s[3],1);

	lcd.setCursor(15,1);
	// (last char "H": H for Heat, C for Cool, - for Off)
	switch(thMode)
	{
	case THERMOSTAT_OFF:
		lcd.print("-");
		break;
	case THERMOSTAT_HEAT:
		lcd.print("H");
		break;
	case THERMOSTAT_COOL:
		lcd.print("C");
		break;
	}
	// Set the cursor if appropriate
	switch(sPos)
	{
	case 0:
		lcd.setCursor(4,0);
		break;
	case 1:
		lcd.setCursor(11,0);
		break;
	case 2:
		lcd.setCursor(3,1);
		break;
	case 3:
		lcd.setCursor(10,1);
		break;
	case -1:
		lcd.setCursor(15,1);
		break;
	}
	lcd.cursor();

}

void toggleWriteMode(){
  liveWrite = !liveWrite;
  scheduleEvent(manageSD, 1);

  scheduleEvent(updateScreen, 1);
}


void settingApply(String name, String value)
{

	// For info:
	//	float thermostatSettings[4] = {
	//			0, // target temperature
	//			1, // target temperature window
	//			0, // temperature undershoot
	//			0, // temperature overshoot
	//	};

	if(name.equals("logInterval"))
	{
		int li = value.toInt();
		logInterval = li;
		scheduleTime[cycle] = 1000 * li;
		scheduleEvent(cycle, scheduleTime[cycle]);
	}
	else if(name.equals("tempTarget"))
	{
		thermostatSettings[0] = value.toFloat();
	}
	else if(name.equals("tempRange"))
	{
		thermostatSettings[1] = value.toFloat();
	}
	else if(name.equals("tempUndershoot"))
	{
		thermostatSettings[2] = value.toFloat();
	}
	else if(name.equals("tempOvershoot"))
	{
		thermostatSettings[3] = value.toFloat();

	}
	else if(name.equals("thermostatMode"))
	{
		if(value.equals("H"))
			thermostatMode = THERMOSTAT_HEAT;
		else if(value.equals("C"))
			thermostatMode = THERMOSTAT_COOL;
		else if(value.equals("X"))
			thermostatMode = THERMOSTAT_OFF;
	}
}


String settingPrint(String name, String value)
{
	String out;
	out = out + "[";
	out = out + name;
	out = out + "=";
	out = out + value;
	out = out + "]";
	return(out);
}


void controlRelay(float controlTemp)
{
	// When heating, we expect the temperature to go tOvershoot over its actual value.
	// When in heating mode but not heating, we expect the temperature to go
	// 	tUndershoot under its actual value.
	// Cooling: umgekehrt

	float switchTemp = 0;
	// for info:
	//	float thermostatSettings[4] = {
	//			0, // target temperature
	//			1, // target temperature window
	//			0, // temperature undershoot
	//			0, // temperature overshoot
	//	};

	switch(thermostatMode)
	{
	case THERMOSTAT_HEAT:
		if(!relayState)
		{
			switchTemp = controlTemp - thermostatSettings[2];
			if(switchTemp <= thermostatSettings[0] - thermostatSettings[1])
				relayState = true;
		}
		else
		{
			switchTemp = controlTemp + thermostatSettings[3];
			if(switchTemp >= thermostatSettings[0] + thermostatSettings[1])
				relayState = false;
		}
		break;
	case THERMOSTAT_COOL:
		if(!relayState)
		{
			switchTemp = controlTemp + thermostatSettings[3];
			if(switchTemp >= thermostatSettings[0] + thermostatSettings[1])
				relayState = true;
		}
		else
		{
			switchTemp = controlTemp - thermostatSettings[2];
			if(switchTemp <= thermostatSettings[0] - thermostatSettings[1])
				relayState = false;
		}
		break;
	case THERMOSTAT_OFF:
		break;
	}

	if(thermostatMode != THERMOSTAT_OFF)
	{
		if(relayState)
			digitalWrite(RELAY_PIN, LOW);
		else
			digitalWrite(RELAY_PIN, HIGH);
	}
}
