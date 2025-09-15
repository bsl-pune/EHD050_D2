#ifndef Ext_Var_h
#define Ext_Var_h
//---------------------------------------------------------------------------------------------------------------------
#include "Enco.h"

// --------------------------------------------------------------------------------------------------------------------
#include <EEPROM.h>
#include <OneButton.h>
#include "Ticker.h"

//----------------------------------------------------------------------------------------------------------------------
extern volatile byte aFlag, bFlag, reading;
extern const int maxEntries;  // Max total number of titration events
extern uint8_t currentPageStart;  // Start index for scrolling through the displayed titration events
extern int totalTitrations, CalVal;
// extern const int usersPerPage;
extern volatile uint16_t encoderPos, oldEncPos, encoder0Pos;
extern bool once, dispense_start,enter_dispense;
extern long dispense_val;
extern bool selecting_volume, selecting_volumes, selecting_steps;  // New variable to track if we are selecting time
extern uint8_t dispense_time,repeats,cal_flag, cal_mode, priming, pos1,checkflag, menus, previousPos,scrollData,Save_flag,singleClickCount, sub_menus, menu_flag;
// ---------------------------------------------------------------------------------------------------------------------
class Vclass
{
  public :
  Vclass();
};

extern Vclass var;
extern OneButton button0;
//extern LiquidCrystal lcd;

#endif
