// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _BeerLoggerEc_H_
#define _BeerLoggerEc_H_
#include "Arduino.h"
//add your includes for the project BeerLoggerEc here


//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project BeerLoggerEc here

void scheduleEvent(int eventId, long tDelay);

float getAirTemp();
float getLiquidTemp();

// "Schedule function pointer" functions
void fpReadAirTemp();
void fpClearDebounce();
void fpManageSD();
void fpUpdateScreen();
void fpCycle();
void fpSettingsLoad();
void fpSettingsStore();

// Actor functions (that do actual stuff)
void writeLog(int index);
void control();

void setMessage(String msg);

// Encoder and clear button interrupt handlers
void doEncoderA();
void doEncoderB();
void doEncSw();
void doClearButton();


void settingApply(String name, String value);
String settingPrint(String name, String value);


void mainDisplay();
void thermostatSettingsDisplay(float *, int, int);
void toggleWriteMode();
void controlRelay(float);

int uiLoggerSettings(int);
int uiTempDisplay(int);
int uiMessage(int);
int uiThermostatSettings(int);
int uiThermostatMode(int);
int uiLoadStoreSettings(int);

void handleUi(int);



//Do not add code below this line
#endif /* _BeerLoggerEc_H_ */

