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
void doEncoderA();
void doEncoderB();
float getAirTemp();
float getLiquidTemp();

// "Schedule function pointer" functions
void fpReadAirTemp();
void fpClearDebounce();
void fpManageSD();
void fpUpdateScreen();
void fpCycle();

// Encoder target functions
void encScreenPos(int);
void encLogInterval(int);


void writeLog(int index);

void setMessage(String msg);
void doEncSw();
void doClearButton();


void settingsApply();

void mainDisplay();
void toggleWriteMode();

int uiSettings(int);
int uiTempDisplay(int);
int uiMessage(int);

void handleUi(int);



//Do not add code below this line
#endif /* _BeerLoggerEc_H_ */
