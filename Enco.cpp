#include "Arduino.h"
#include "Ext_Var.h"

//-----------------------------------------------Encoder Variable -----------------------------------------------//
// static int pinA = 8;
// static int pinB = 9; 

volatile byte aFlag = 0;
volatile byte bFlag = 0;
volatile uint16_t encoderPos = 0; 
volatile uint16_t oldEncPos = 0;
volatile byte reading = 0; 
volatile uint16_t encoder0Pos = 0;
uint8_t checkflag = 0;

Eclass::Eclass()
{}

//-------------------------------------------------Encode Function---------------------------------------------------//
void Eclass :: SETUP()
{
  // pinMode(pinA, INPUT_PULLUP); 
  // pinMode(pinB, INPUT_PULLUP);
  // pinMode(pinA, INPUT); 
  // pinMode(pinB, INPUT);
 
}

//------------------------------ENCODER POSITION CHANGE ------------------------------//
void Eclass :: encode_increment()
{
  if(oldEncPos != encoderPos) 
  {
    oldEncPos = encoderPos;
    
  switch (menus) {
        case 0: // Main menu
            if (checkflag) {
                encoder0Pos = (encoder0Pos >= 3) ? 3 : encoder0Pos + 1;
            } else {
                encoder0Pos = (encoder0Pos <= 0) ? 0 : encoder0Pos - 1;
            }
            break;

        // case 1: // Titrate Mode
        // if(menu_flag == 1)
        // {
        //    if (checkflag) {
        //     // Scroll forward
        //     if (sub_menus >= 3) {
        //         sub_menus = 1;  // Loop back to first menu
        //     } else {
        //         sub_menus++;
        //     }
        // } else {
        //     // Scroll backward
        //     if (sub_menus <= 1) {
        //         sub_menus = 3;  // Loop back to last menu
        //     } else {
        //         sub_menus--;
        //     }
            
        // }
        // }
        //   if(menu_flag==0 && singleClickCount==1){
        //     if (checkflag) {
        //         encoder0Pos = (encoder0Pos >= 3) ? 3 : encoder0Pos + 1;
        //     } else {
        //         encoder0Pos = (encoder0Pos <= 3) ? 0 : encoder0Pos - 1;
        //     }
        //   }
        //     break;

        case 4: // Calibration Mode
            if (cal_flag == 0) {
                if (checkflag) {
                    cal_mode = (cal_mode >= 1) ? 1 : cal_mode + 1;
                } else {
                    cal_mode = (cal_mode <= 0) ? 0 : cal_mode - 1;
                }
            } else if (cal_flag == 1 || cal_flag == 4 || cal_flag == 7) {
                if (cal_mode == 1) {
                    if (checkflag) {
                        CalVal = (CalVal > 500) ? 1 : CalVal + 1;
                        
                    } else {
                        CalVal = (CalVal < 1) ? 500 : CalVal - 1;
                    }
                } else {
                    if (checkflag) {
                        CalVal = (CalVal > 110) ? 0 : CalVal + 1;
                    } else {
                        CalVal = (CalVal < 1) ? 110 : CalVal - 1;
                    }
                }
            }
            break;

        case 6: // User Select mode
              // checkflag ? (encoder0Pos >= 9) ? encoder0Pos = 0 : encoder0Pos++ : (encoder0Pos < 1) ? encoder0Pos = 9 : encoder0Pos--;
            if (checkflag) {
                encoder0Pos = (encoder0Pos >= 9) ? 9 : encoder0Pos + 1;
            } else {
                encoder0Pos = (encoder0Pos <= 0) ? 0 : encoder0Pos - 1;
            }
            break;
                case 12: // User Select mode
              // checkflag ? (encoder0Pos >= 9) ? encoder0Pos = 0 : encoder0Pos++ : (encoder0Pos < 1) ? encoder0Pos = 9 : encoder0Pos--;
            if (checkflag) {
                encoder0Pos = (encoder0Pos >= 3) ? 0 : encoder0Pos + 1;
            } else {
                encoder0Pos = (encoder0Pos <= 0) ? 3 : encoder0Pos - 1;
            }
            break;

        case 5: // Setting mode
            checkflag ? (currentPageStart >= 9) ? currentPageStart = 0 : currentPageStart++ : (currentPageStart <= 0) ? currentPageStart = 0 : currentPageStart--;
            break;

        // case 1: // Priming mode (clockwise and anticlockwise)
        //     if (priming == 1) {
        //         // Add your logic here if needed
        //     }
        //     break;

        case 1: // Dispense mode
            if(menu_flag && (!selecting_volume || !selecting_steps || singleClickCount != 0 || singleClickCount != 1 || singleClickCount != 2) )
              {
                if (checkflag) {
            // Scroll forward
            if (sub_menus >= 3) {
                sub_menus = 1;  // Loop back to first menu
            } else {
                sub_menus++;
            }
        } else {
            // Scroll backward
            if (sub_menus <= 1) {
                sub_menus = 3;  // Loop back to last menu
            } else {
                sub_menus--;
            }
        }
              }
           if(menu_flag == 0){
            if (!dispense_start) 
            {
              
                //  Serial.println("enterd menu 1");
                if (selecting_volume && singleClickCount == 1) 
                {
                    if (checkflag) 
                    {
                        dispense_val = (dispense_val >= 10000) ? 10000 : dispense_val + 2;
                    } else 
                    {
                        dispense_val = (dispense_val <= 10) ? 0 : dispense_val - 2;
                    }
                } 
                else if(selecting_steps)
                {
                    if (checkflag) 
                    {
                        repeats = (repeats >= 256) ? 256 : repeats + 1;
                    } 
                    else 
                    {
                        repeats = (repeats <= 1) ? 1 : repeats - 1;
                    }
                }
              }
            }
            break;

        case 2: // Dispense Time mode
             if(menu_flag && (!selecting_volumes || !selecting_steps || singleClickCount != 0 || singleClickCount != 1 || singleClickCount != 2) )
              {
                if (checkflag) {
            // Scroll forward
            if (sub_menus >= 3) {
                sub_menus = 1;  // Loop back to first menu
            } else {
                sub_menus++;
            }
        } else {
            // Scroll backward
            if (sub_menus <= 1) {
                sub_menus = 3;  // Loop back to last menu
            } else {
                sub_menus--;
            }
        }
              }
             if(menu_flag == 0){ 
            if (!dispense_start) {
                if (selecting_volumes && singleClickCount==1) {
                    if (checkflag) {
                        dispense_val = (dispense_val >= 10000) ? 10000 : dispense_val + 2;
                    } else {
                        dispense_val = (dispense_val <= 10) ? 10 : dispense_val - 2;
                    }
                } else if(selecting_steps)
                   {
                    if (checkflag) {
                        dispense_time = (dispense_time >= 360) ? 360 : dispense_time + 1;
                    } else {
                        dispense_time = (dispense_time <= 1) ? 1 : dispense_time - 1;
                    }
                   }
            }
             }
            break;
    }
   }
}  


Eclass Enco = Eclass();
