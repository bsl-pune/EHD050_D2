//-----------ADB code--------//
//18-July-2024 to 13-September-2024
//titrate mode - 3 speed of stepper motor, dispense mode -only 1 speed, user selct mode, setting mode, priming mode.
//Priming One Click Start
//Time based dispensing accurate calculation for time and volume
//Dispense calibration added
//Record correctly saving titrate colume and User ID
//Memory optimization from 90 % to 86%
//Dispensing sub menu is added such as parking & forced priming mode
//Shivani Testing bugs all solved
//------------------Date 10 Dec 2024--------------------------------//
//motor noise in dispense mode solved 
//---------------------declare the header files----------------------------//

//PLM Training 
//17092025
//Authgor - Kishanjee Shukla
//Third revision

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <U8g2lib.h>
#include <math.h>
// #include <Adafruit_SSD1306.h>
//#include <Fonts/FreeSerifBold9pt7b.h>
#include "Ext_Var.h"
// #include "Adafruit_Thermal.h"  //printer
// #include <SoftwareSerial.h>
// #define TX 4    //printer  Serial2  24
// #define RX 5    //25
// #define RTCAddress 0x68   //RTC address
//--------------animation----------------------------------//
uint8_t angle = 0;      // Initial angle for the handle
static bool parking_once = false;
//-----------------------Dispense Mode Variables----------------------------//
// const float stepperrev = 235.984;
// uint8_t cylinderCapacity = 35; // Piston cylinder capacity in ml
uint8_t repeats = 0;
// bool select_repeats = false;
bool exitDispenseMode = false;
bool dispensing_complete = false; 
bool selecting_volume = true;  // Start by selecting volume
bool selecting_volumes = true;
uint8_t dispense_time = 1;
long dispense_val = 0;
// uint8_t e_flag = 0;
bool selecting_steps = false; //for selecting number of steps
uint8_t singleClickCount = 0;
uint8_t menu_flag = 1;
// bool disable_sensor = false;// to disable sensor when in parking mode
bool dispensing = false;
bool retracting = false;

bool draw_flag = false;
//--------------------------------EEPROM--------------------------------------//
const int maxEntries = 10;  // Max total number of titration events
uint8_t currentPageStart = 0;  // Start index for scrolling through the displayed titration events
int totalTitrations = 0;   // Total number of titration events
int nextEntryIndex = 0; 
//------------------------Calibration variables--------------------------------//
double stepc;
double voln = 1.0;
float E_LOW;
float E_HIGH;
float MC;
float V_h = 1.0;
double stepd; 

uint8_t cal_flag = 0;
uint8_t cal_mode = 0;
int CalVal = 0;
bool change_flag = true;
bool parking_motor_flag = false;
bool parking_display_flag = false;
// int dispense_calb = 0;
float rawLow, rawHigh = 0.0;
// long calbSteps = 0;
//----------------------------Titration Flags---------------------------------\-//
  static bool processRunning = false;  // State variable to track if the process is ongoing
  static bool initialPositioning = true;  // State variable to track if initial positioning is needed
  static bool initialDetectionDone = false;  // State variable to track if initial sensor detection has occurred
  const int sensorPin = 36 ;   // 22 old//36 new; 
  volatile bool objectDetected = false;
bool processStarted = false; 
//-------------------------------------for OLED display------------------------------------------------------------------------------------------//
U8G2_SSD1309_128X64_NONAME2_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=13*/ 35, /* data=11*/ 30, /* cs=10*/ 33, /* dc=9*/ 31, /* reset=8*/ 32);  

//----------------------printer -----------------------------------------------------------------------//
// SoftwareSerial mySerial(TX, RX);   //SOFTWARE SERIAL DECLARE FIRST        //printer
// Adafruit_Thermal printer(&mySerial);  //PASS ADDRESS OF THERMAL PRINTER

//-----------------------------------------------Encoder-----------------------------------------------//
static int enSW = 28;/*27 old; //new //28 new pcb */
static int pinA = 29;
static int pinB = 27;/*28 old; //new //27 new pcb*/
OneButton button0(enSW, true);
//------------------------------stepper motor----------------------------------------------------------//  
#define dirPin 13
#define stepPin 11
#define stepsPerRevolution 200
#define M0 10
#define M1 9
#define M2 8
#define enablem 12

//---------------------------------buzzer----------------------------------//
#define buzzer 26           //buzzer pin 

//---------------------------------Menus Variable--------------------------//
bool once = false;
uint8_t menus = 0;
uint8_t sub_menus = 0;
uint8_t previousPos = 0;
unsigned long pressStartTime = 0; 
// int cnt = 0;//,revers = 0
//---------------------------------------------Show menu flag---------------------------------------------//


//---------------------------------------user select mode variables----------------------------------------------------------------//
const char* menu_option1[] = {
  //  "Priming",
  //  "Titrate",
   "Dispense",
   "Time Dispense",
   "Delay Dispense",
   "Calibration",
   " ",
  //  "User Select",  
  //  "Calibration",  
  //  "Record",
   //""
   //""
}; 

const char* menu_option2[] = {
     "Priming",
     "User Select",  
    //  "Calibration",  
     "Record",
     " ",
     " ",
  //  "User Select",  
  //  "Calibration",  
  //  "Record",
   //""
   //""
}; 


const char* User_name[13]= {"NO USER","USER1 ","USER2","USER3","USER4","USER5","USER6","USER7","USER8","USER9","  ", " "," "};
uint8_t Position[12] = {38,38,42,40,31,30,40,32,30,34}; 
uint8_t once_update = 1;
uint8_t  User_set_flag = 0,User=0;
//-----------------------------------setting mode variables------------------------------------------------------------------------//
// uint8_t Save_flag=0,Print_flag=0;//previous int
// int disp_count=0;
// int y_pos1[3]={39,50,61};
//----------------------------------------Scroll logic---------------------------------------//
uint8_t scrollOffset = 0;
uint8_t scrollData = 0;
//----------------------------------------Functions Declaration------------------------------//
void stepperPulse();
void buzzer_stop();
// void writeRecord();
// void display_dispense();
// void display_titrate();
// void buzzer_stop();
void stopDispense();
void startDispense();
// void titrateModeSpeed(int modeSpeed);
// void display_priming();
// void display_record();
// void display_calibrate();
// void ReadTime();//------------------------------//
// void rev1();
// void Data_Print();

//---------------------------------ticker function-----------------------------------------//
// Ticker animate(arrow_animate, 500, 0, MILLIS);
Ticker motor(stepperPulse, 1000, 0, MICROS_MICROS); // once, immediately
// Ticker motor1(stepperPulse1, 1000, 0, MICROS_MICROS); // once, immediately
Ticker buzz(buzzer_stop, 500, 0, MILLIS);  


// Ticker Time_update(ReadTime, 0, 0);
// Ticker PrintData( Data_Print, 0, 0);  //data print ticker function

//-------------------------------------Animation--------------------------------------------//
// Define the rectangle dimensions and arrow parameters

// Array of starting y-coordinates for arrowheads within the rectangle

// Array to hold current y-positions of arrows
// int yPos[3];
uint8_t yPos = 5;
// int ani_start = 1;
// uint8_t titra_step[4] = {0, 19, 12, 5};
//----------------------------------------Motor Variable-----------------------------------//
unsigned long stepperCount = 0;
unsigned long prev_stepperCount = 0;
unsigned int totalDispense = 0;
double tempVal = 0;
uint8_t titrateMode = 0;
int titrationSteps = 800;
int previous_main_menu = 0;
//--------------------------------------Checkflags----------------------------------------//
bool dispense_start = false;
bool entry_chk = true;
// int currData = 0;
uint8_t priming = 0;

//---------------------------------------for RTC------------------------------------------//

// int SECOND, MINUTE, DAT, MONT, YER;
String daY;
int houR;

//---------------------previous optic flag variables----------------------------------------//
// #define pos_reach 12
uint8_t pos1=0;

//-------------------------------------------------------Setup Initialization--------------------------------------------------------//
void setup() 
{
  // put your setup code here, to run once:
  // Serial.pins(0,1);  //TX RX
  // mySerial.begin(9600);    //INITIALIZE SOFTWARE SERIAL   //printer
  Serial.begin(115200);     //for serial print
  u8g2.begin();        //-----for oled 
  // Wire.begin();       //------for RTC
  // Time_update.start();   //start ticker
  // PrintData.start();     //start printdata ticker
  // setTime(00,38,16,3,21,12,22);   // FORMATE  SEC,MIN,HOUR,DAY(FRIDAY=5),DATE,MONTH,YEAR
  // Serial.swap(1);
  // Serial.begin(9600);
  Enco.SETUP();
  pinMode(enSW, INPUT_PULLUP); 
  pinMode(sensorPin, INPUT);  // Set the sensor pin as input
  // pinMode(enSW, INPUT); 
    
  attachInterrupt(digitalPinToInterrupt(sensorPin), sensorISR, CHANGE);
  attachInterrupt(pinA,PinA,RISING);
  attachInterrupt(pinB,PinB,RISING);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(enablem, OUTPUT);
  digitalWrite(dirPin,LOW);//-----  motor pins
  // digitalWrite(M0, LOW);
  // digitalWrite(M1, LOW);
  // digitalWrite(M2, LOW);
  motorspeed(4);
  digitalWrite(enablem, HIGH);
  button0.attachClick(click1);
  // button0.attachLongPressStart(longPress1);
  // button0.attachDoubleClick(double_click);
    button0.attachLongPressStart(handleLongPressStart);
  button0.attachLongPressStop(handleLongPressStop); 

 digitalWrite(buzzer,HIGH);
 u8g2.clearDisplay();
 u8g2.clearBuffer();
 u8g2.setFont(u8g2_font_helvB08_te);//u8g2_font_helvB08_te
 u8g2.drawStr(40,30,"BOROSIL");
 u8g2.drawStr(32,45,"SCIENTIFIC");//TECHNOLOGIES 20, 45 Scientific
 u8g2.sendBuffer();   
 delay(2000);
 u8g2.clearBuffer();
 u8g2.drawStr(40,35,"ADB V2.0");
 u8g2.sendBuffer(); 
 delay(1500);
 digitalWrite(buzzer,LOW);

//  EEPROM.get(101, User);
//  EEPROM.get(0, startAddress);
//  EEPROM.get(sizeof(int), endAddress);
 loadTitrations();
 menus = EEPROM.read(123);

 if(menus == 1)
 {
 EEPROM.get(8, totalDispense);
//  encoder0Pos = 1;
 }
  if (menus < 0 || menus > 3) 
    {
        menus = 2;  // Default to menu 2 if the saved value is invalid
    }

 display_screens();  
 encoder0Pos = 0;//doubt********after mains//
}

//-------------------------------------ISR-------------------------------------//
void sensorISR() {
  int state = digitalRead(sensorPin);
  objectDetected = (state == HIGH);
}
//----------------------------------Onebutton----------------------------------//

void handleLongPressStart() {
  pressStartTime = millis();  // Record the start time
}

void handleLongPressStop() {
  unsigned int pressDuration = millis() - pressStartTime;  // Calculate the press duration //long

  if (pressDuration >= 800 && pressDuration < 2000) {
    if(menu_flag == 1 && (menus ==1 || menus==2||menus==3))
    {
    longPress2();
    Seperate_menu();  // Call the function for 800ms press
    
    menus = 12;
    }
           else if (menus == 4 || menus == 5 || menus == 6 || menus == 7) {
            // Return to the previously stored main menu
            menus = previous_main_menu;
            menu_flag = 1;  // Return to main menu mode
        } 
    else {
    longPress2();

    if(menus != 1)
    {
    menus = 2;
    }
    else{
      menus = 1; //changed
            //    draw_flag = true;
            // menu_flag = 1;
            // singleClickCount = 0;

    }
    menu_flag = 1;
    }
    
  } 
  else if (pressDuration >= 2000) 
  {
    longPress1();  // Call the function for 2000ms press
  }
}
//------------------------------------------------------------Calibration formula function--------------------------------------------//
void updateStepcont() 
{
  stepc = 235.984 * voln * 1;
}

void updateStepcontd() {
  stepd = 235.984 * V_h * 1;
}

struct TitrationEntry 
{
    int userID;
    int titrationValue;  
};

TitrationEntry titrations[maxEntries];
const int eepromBaseAddress = 200; 

void saveTitration(int userID, int value) {
    // Save the titration in memory
    titrations[nextEntryIndex].userID = userID;
    titrations[nextEntryIndex].titrationValue = value;
    
    // Save the titration in EEPROM
    EEPROM.put(eepromBaseAddress + nextEntryIndex * sizeof(TitrationEntry), titrations[nextEntryIndex]);
    
    // Update total titrations and next entry index
    if (totalTitrations < maxEntries) {
        totalTitrations++;
    }
    
    nextEntryIndex = (nextEntryIndex + 1) % maxEntries;  // Move to the next index circularly

    // Save the updated totalTitrations count
    EEPROM.put(eepromBaseAddress - sizeof(totalTitrations), totalTitrations);
}

void loadTitrations() {
   EEPROM.get(eepromBaseAddress - sizeof(totalTitrations), totalTitrations);
    for (int i = 0; i < maxEntries; i++) {
        EEPROM.get(eepromBaseAddress + i * sizeof(TitrationEntry), titrations[i]);
    }

    if (totalTitrations > maxEntries) {
        totalTitrations = maxEntries;
    }
}


//-----------------------------------------------------------------ENCODER INTERRUPT--------------------------------------------------//
void PinA()
{
  cli(); //stop interrupts happening before we read pin values
//  reading = PIND & 0xC; 
  if(digitalRead(pinA) == HIGH && digitalRead(pinB) == HIGH && aFlag)
  { 
    encoderPos --; 
    bFlag = 0; 
    aFlag = 0; 
    checkflag=0;
  }
  else if (digitalRead(pinA) == HIGH) bFlag = 1; 
  sei(); //restart interrupts
  digitalWrite(buzzer, HIGH);
  buzz.interval(50);
  buzz.start();
  Enco.encode_increment();
}


void PinB()
{
  cli(); //stop interrupts happening before we read pin values
  if(digitalRead(pinA) == HIGH && digitalRead(pinB) == HIGH && bFlag) 
  { 
    encoderPos ++; 
    bFlag = 0; 
    aFlag = 0; 
    checkflag=1;
  }
  else if (digitalRead(pinB) == HIGH) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
//  cnt++;
//  if(cnt >= 4)
//  {
//    cnt = 0;
    Enco.encode_increment();
//  }
}

//--------------------------------------------------------SINGLE CLICK-------------------------------------------------------------//

void click1() {
  
    switch (menus) {
      
        case 0:
            menus = encoder0Pos + 1;
      
            encoder0Pos = 0;
            once = false;
            //---------------------------buzzer indication--------------//
            digitalWrite(buzzer, HIGH);
            buzz.interval(50);
            buzz.start();
            break;

        case 2:   //time dispense
            previous_main_menu = menus;
            // Serial.println(previous_main_menu);
          if(menu_flag)
          {
            if(sub_menus == 1)
            {
              parking_motor_flag = true;
            }
            if(sub_menus == 3)
            {
              parking_motor_flag = true;
            } 
            if(sub_menus == 2)
            {
              menu_flag = 0;
              // selecting_volume = true;
            }           
            
          } 
            if (!dispense_start) 
            {
                  if (singleClickCount == 0) {
                      singleClickCount++;  // First click
                      menu_flag = 0;
                      selecting_volumes = true;
                                             }
                  else if (singleClickCount==1) 
                  {
                    selecting_volumes = true;  // Enter volume selection on the second click
                           if (selecting_volumes) 
                           {
                             selecting_volumes = false;  // Switch to step selection
                             selecting_steps = true;
                           }
                           singleClickCount++;  // Move to the next click count
                  }
                 else if (singleClickCount >= 2) {
        // After the second click, start dispensing
        if (selecting_steps) 
        {
            dispense_start = true;  // Start dispensing
            selecting_steps = false;
            selecting_volumes = false;  // Reset for next round
            // Serial.println("enter 4");
            singleClickCount = 0;
            menu_flag = 1;
        }
          } 
                // else {
                //     dispense_start = true;  // Start dispensing
                //     dispensing = false;
                //     retracting = false;
                //     // revers = 2;
                // }
                //  singleClickCount++;  // Move to the next click count
            }
            break;

        case 1:  //----------when in dispense mode
        previous_main_menu = menus;
          if(menu_flag)
          {
            if(sub_menus == 1)
            {
              parking_motor_flag = true;

              // selecting_volume = false;
            }
            if(sub_menus == 3)
            {
              parking_motor_flag = true;
              menu_flag = 0;
              // selecting_volume = false;
            } 
            if(sub_menus == 2)
            {
              menu_flag = 0;
            }          
            
          }
            if (!dispense_start) {
    // First, ensure that selecting_volume is entered on the first click
    if (singleClickCount == 0) {
        singleClickCount++;  // First click
        menu_flag = 0;
        selecting_volume = true;
        // disable_sensor = false;
        // Serial.println("enter 1");
    } 
    else if (singleClickCount == 1 ) {
        selecting_volume = true;  // Enter volume selection on the second click
        // Serial.println("enter 2");

        if (selecting_volume) {
            selecting_volume = false;  // Switch to step selection
            selecting_steps = true;
            // Serial.println("enter 3");
        }

        singleClickCount++;  // Move to the next click count
    } 
    else if (singleClickCount >= 2) {
        // After the second click, start dispensing
        if (selecting_steps) {
            dispense_start = true;  // Start dispensing
            selecting_steps = false;
            selecting_volume = false;  // Reset for next round
            // Serial.println("enter 4");
            singleClickCount = 0;
            menu_flag = 1;
        }
    }
}
            break;

        case 6:  //----------when in user select mode //3-4
            // revers = 3;
            // if(motor.state() == STOPPED)
            // {
            //   change_flag = true;
            //   cal_flag++;
            // }
            if (User_set_flag == 1 && encoder0Pos >= 0 && encoder0Pos <= 9) {  // selecting the user name
                User = encoder0Pos;
                // uint8_t addr = (startAddress + scrollData) % maxValues;
                User_set_flag = 0;
                // Serial.print("user:");
                // Serial.println(User);
                EEPROM.put(170, User); 

                encoder0Pos = 0;
                menus = 1;
                // EEPROM.put(101, encoder0Pos); 
            }
            once_update = 1;
            break;

        case 7:  //-------when in priming mode //5-3
            priming++;
            break;

case 12:  //-------when in priming mode //5-3
    switch(encoder0Pos) {
        case 0:
            display_priming();
            once = false;
            menus = 7;
            break;

        case 1:
            menus = 6;
            break;

        case 2:
            menus = 5;
            break;

        case 3:
            menus = 4;
            break;
    }
    break;


        // case 4:  //--------------when in setting mode//4-5
        //     if (Print_flag == 1 && encoder0Pos == 1) {  //---------------printing data
        //         digitalWrite(buzzer, LOW);
        //         // PrintData.update();
        //     } else if (Print_flag == 1 && encoder0Pos == 0) {  //-------------saving data
        //         // Data_Save();
        //     }
        //     break;

          // case 1:  //titrate
          //     //  menu_flag = 1;
          //     previous_main_menu = menus;
          //   if(menu_flag)
          // {
          //   if(sub_menus == 1)
          //   {
          //     parking_motor_flag = true;
          //   }
          //   if(sub_menus == 3)
          //   {
          //     parking_motor_flag = true;
          //   } 
          //   if(sub_menus == 2)
          //   {
          //     menu_flag = 0;
          //   }           
            
          // }

          //     if (singleClickCount == 0 && sub_menus == 2) 
          //     {
          //         singleClickCount++;  // First click
          //         menu_flag = 0;
          //         encoder0Pos = 0;
          //         // EEPROM.get(8, totalDispense);
          //          draw_flag = false;
          //     }
          //     else{
          //       singleClickCount == 0;
                
          //     } 

        case 4:  // calibration
            //  revers = 3;
            if (motor.state() == STOPPED) {
                change_flag = true;
                cal_flag++;
            }
            break;

            
    }
    EEPROM.write(123, menus);
}
//-------------------------------------------------------------------------------------

// void double_click()
// {
//   menus = 12;
// }
//--------------------------------------------------------------------------------------
void Seperate_menu()
{
  u8g2.clearBuffer();          // clear the internal memory
          u8g2.setFont(u8g2_font_helvB08_te);
         u8g2.drawStr(38,18,"MENU"); 
        //  u8g2.setFont(u8g2_font_helvB08_te);
         u8g2.drawLine(130, 20, 0, 20);  //u8g2.drawLine(130 = length, 13 = start point (vertical position y) , 0 = horizontal start position (x), 13 = end point);
        //  u8g2.drawStr(25,25,"SELECT USER");
        //  u8g2.drawLine(130, 27, 0, 27); 
         u8g2.drawStr(0,30,">");
         u8g2.setCursor(10,31);
         u8g2.print(menu_option2[encoder0Pos]);
         u8g2.setCursor(10,43);
         u8g2.print(menu_option2[encoder0Pos+1]); 
         u8g2.setCursor(10,55);
         u8g2.print(menu_option2[encoder0Pos+2]); 
         u8g2.setCursor(10,67);
         u8g2.print(menu_option2[encoder0Pos+3]); 
         u8g2.sendBuffer();
}


void handleLongPressCommon(int finalMenu) {
    switch (menus) {
        // case 1: // In titration mode
        //     if (totalDispense != 0 && encoder0Pos == 0) {
        //         digitalWrite(buzzer, HIGH);
        //         buzz.interval(400);
        //         buzz.start();
        //     } 
        //     else if (totalDispense != 0 && encoder0Pos != 0) {
        //         saveTitration(User, totalDispense);
        //         digitalWrite(buzzer, HIGH);
        //         buzz.interval(400);
        //         buzz.start();
        //     } 
        //     else {
        //         menus = finalMenu;
        //     }
        //     totalDispense = 0;
        //     prev_stepperCount = 0;
        //     processRunning = false;
        //     initialPositioning = false;
        //     initialDetectionDone = true;
        //     tempVal = 0;
        //     draw_flag = true;
        //     menu_flag = 1;
        //     singleClickCount = 0;
        //     break;

        case 1: // In dispense mode
            if (dispense_val != 0 && menu_flag == 0) {
                if (!dispense_start) {
                    dispense_start = true;
                    repeats = 1;
                    singleClickCount = 0;
                    menu_flag = 1;
                    selecting_volume = false;
                } else {
                    exitDispenseMode = true;
                }
            } else {
                if (menu_flag) {
                    if (sub_menus == 1 || sub_menus == 3) {
                        sub_menus = 2;
                        dispense_start = false;
                        repeats = 0;
                    }
                } else {
                    menus = finalMenu;
                }
            }
            break;

        case 2: // Time dispensing
            if (!dispense_start) menus = 0;
            if (processStarted) {
                processStarted = false;
                exitDispenseMode = true;
                dispense_start = false;
                digitalWrite(enablem, HIGH);
            }
            break;

        default:
            menus = finalMenu;
            break;
    }

    stepperCount = 0;
    if (menus != 1) encoder0Pos = 0;

    once = false;
    entry_chk = true;
    previousPos = 0;
    cal_flag = 0;
    stopDispense();
    once_update = 1;
    menu_flag = 1;
    singleClickCount = 0;
    processRunning = false;
    initialPositioning = false;
    initialDetectionDone = true;
}

void longPress1() {
    handleLongPressCommon(0);  // Go back to main menu
}

void longPress2() {
    handleLongPressCommon(12);  // Go to special/exit menu
}



//------------------------------------------Main Loop------------------------------------------//
void loop()
{ 
// {  int sensorValue = analogRead(analogInPin);

  // Print the raw sensor value (0 to 1023) to the serial monitor
  // Serial.print("Analog Value: ");
  // Serial.println(sensorValue);
  button0.tick();    //---------------for button
  if(menus == 0)
  {
    digitalWrite(dirPin, HIGH);
    stopDispense();
    display_main_menu();
  }
  display_screens();
  if(menus == 1 || menus == 2 || menus == 3){
    dispense_menus();

    // if(menus == 1)
    // {
    //   // prev_stepperCount = stepperCount
    //   Serial.println("prev");
    //   Serial.println(prev_stepperCount);
    //   Serial.println(stepperCount);
    // }
  }


  // animate.update();
  motor.update();
  buzz.update();
  // Time_update.update();
  list_arrow();//----------to keep main menu visble 
  // if(menus==1){
  //   if(once==false){
  //     u8g2.clearBuffer();
  //      drawRectangleWithArrows(titrateMode);
  //      u8g2.sendBuffer();
  //      delay(50);
  //   }
  // }
}

//--------------------------------list of main menu--------------------------------------------------//
void list_arrow()
{
  if (encoder0Pos < scrollOffset) 
  {
    scrollOffset = encoder0Pos;
  } 
  else if (encoder0Pos >= scrollOffset + 4)                         //original 3
  {                     
    scrollOffset = encoder0Pos - 3;                                 //original-2
  }
}  

//------------------------------------------Menu Switch Loop---------------------------------------//
void display_screens()
{
  switch(menus)
  {
    default:
//          display_main_menu();
            break;

    case 1:
            uni_dispense();
            // display_calibrate();
            // select_mode();
            break;
    
    case 2: 
            uni_dispense();
            
            break;

    case 5: 
            setting_mode();
            
            break;
    case 4:

           Calibration();

           break;

    case 6:
            select_mode();
            // display_priming();
            break;

    case 7:
            display_priming();

            break;



    case 12:
          Seperate_menu();


  }
}

//-------------------------Dispense submenus UI screens-----------------------------------------//
void dispense_menus()
{
  switch(sub_menus)
  {
    case 1:
            display_parking();
             parking_display_flag = true;
              parking_once = true;
            // once = true;
            break;

    case 2:
            // display_dispense();//display_forced(); //priming
            if(menus == 1 || menus == 2)//dispense mode
            {
              
            uni_dispense();
            once = false;
            }
            // else if(menus == 1)//titration
            // {
            //   if(menu_flag == 1 )
            //   {  
            //     parking_display_flag = false;
            //     parking_once = false;
            //       // u8g2.clearBuffer(); 
            //       // u8g2.sendBuffer();
            //   //  parking_once = true;  // Use the dedicated flag
            //   //  if(sub_menus == 2)
            //   //  {
            //      once = false; 
            //    }

                
            //   // }
            //   display_titrate();

            //   // once = false;
            // }
            break;

    case 3:
            display_forced();
            parking_display_flag = true;
            parking_once = false;
            // once = true;
            // display_calibrate();
            // select_mode();
            break;


  }
}

//-------------------------------Parking function ------------------------------------------------//
void display_parking()
{   
      
  if(parking_display_flag == true && parking_once == true )
  {    
  u8g2.clearBuffer(); 
  u8g2.drawCircle(64, 40, 5, U8G2_DRAW_ALL); // Outer circle
  u8g2.drawStr(30, 24, "Parking");
  // Calculate the handle's new position based on the rotating angle
  float radAngle = (angle) * (0.017453);  // Removed the -90 offset
uint8_t handleXStart = 64 + 5 * sin(radAngle);  // Swapped cos/sin
uint8_t handleYStart = 40 + 5 * cos(radAngle);  // Swapped cos/sin
uint8_t handleXEnd = 64 + (15) * sin(radAngle);  // Swapped cos/sin
uint8_t handleYEnd = 40 + (15) * cos(radAngle);  // Swapped cos/sin
u8g2.drawLine(handleXStart, handleYStart, handleXEnd, handleYEnd);
u8g2.sendBuffer();
angle += 1;
if (angle >= 90) {
  angle = 0;
}
  parking_once = false;  // Use the dedicated flag
  parking_display_flag = false;
  }

  if(parking_motor_flag == true)
      { 
           u8g2.clearBuffer();
           u8g2.setFont(u8g2_font_helvB08_te);//u8g2_font_helvB08_te 
           u8g2.drawStr(45, 40, "Parked");
           drawRectangleWithArrows(3);
           u8g2.sendBuffer();

        digitalWrite(enablem, LOW);
        digitalWrite(dirPin, LOW);// digitalWrite(dirPin, HIGH);
        for(int i = 0; i<9020; i++)
        {
          motor_run();
        }
        delay(1000);
        digitalWrite(enablem, HIGH);
        // Serial.println("parking");
        parking_motor_flag = false;
        // while(1);
        parking_display_flag = false;
        sub_menus = 2;
        menu_flag = 1;
                selecting_volume = false;//changed
        singleClickCount = 0;
        dispense_start = false;
      }
}


void runMotor(int steps, bool direction) {
    digitalWrite(dirPin, direction ? LOW : HIGH);//HIGH:LOW
    for (int i = 0; i < steps; i++) {
        motor_run();
    }
}


//-------------------------------Forced Priming function ------------------------------------------------//
void display_forced() {
    if (parking_display_flag) {
        u8g2.clearBuffer(); 
  u8g2.drawCircle(64, 40, 5, U8G2_DRAW_ALL); // Outer circle
  u8g2.drawStr(30, 24, "Purge");
  // Calculate the handle's new position based on the rotating angle
  float radAngle = (angle-90) * (0.017453);  // Removed the -90 offset
uint8_t handleXStart = 64 + 5 * sin(radAngle);  // Swapped cos/sin
uint8_t handleYStart = 40 + 5 * cos(radAngle);  // Swapped cos/sin
uint8_t handleXEnd = 64 + (15) * sin(radAngle);  // Swapped cos/sin
uint8_t handleYEnd = 40 + (15) * cos(radAngle);  // Swapped cos/sin
u8g2.drawLine(handleXStart, handleYStart, handleXEnd, handleYEnd);
u8g2.sendBuffer();
angle += 1;
if (angle >= 90) {
  angle = 0;
}
        parking_display_flag = false;
    }

    if (parking_motor_flag) {
        // Initialize display
        u8g2.clearBuffer(); 
        drawRectangleWithArrows(3);
        u8g2.drawStr(45, 40, "Inprogress");
        u8g2.sendBuffer();

        // Control motor
        digitalWrite(enablem, LOW);

        // Run motor in one direction
        runMotor(8020, true);

        // Pause before reversing direction
        delay(100);

        // Run motor back and forth
        for (uint8_t j = 0; j < 3; j++) {
            runMotor(1000, false);  // Reverse direction
            delay(100);
            runMotor(1000, true);   // Forward direction
        }

        delay(100);
        runMotor(8020, false);
        delay(10);
        // Stop motor and reset flags
        digitalWrite(enablem, HIGH);
        parking_motor_flag = false;
        parking_display_flag = false;
        sub_menus = 2;
        menu_flag = 1;
        selecting_volume = false;//changed
        singleClickCount = 0;
    }
}

//----------------------------Main menu display-----------------------------//
void display_main_menu()
{  
  
   if(previousPos != encoder0Pos)           //encoder position (clockwise or anticlockwise)
      {
        previousPos = encoder0Pos;
        once = false;
      }
  if(once == false)
   {

     u8g2.clearBuffer();          // clear the internal memory   
     u8g2.setFont(u8g2_font_helvB08_te);     //to set font    
     u8g2.drawStr(30,13,"MAIN MENU");        //30- x ,10- y
     //  Disp_date_time();    
     u8g2.drawLine(130, 16, 0, 16);

    uint8_t visibleItems = 4; // Number of menu items to display at a time

  for (uint8_t i = 0; i < visibleItems; i++) {
    uint8_t menuItemIndex = i + scrollOffset;

    // Draw the ">" arrow next to the selected item
    if (menuItemIndex == encoder0Pos) {
      u8g2.drawStr(0, 31 + (i * 11), ">");
    }

    // Display the menu item text
    u8g2.drawStr(10, 31 + (i * 11), menu_option1[menuItemIndex]);
  }
     u8g2.sendBuffer();
    //once_update = 0;
    //  enco_mode = 0;
    //  Print_flag = 0;
   //  User_set_flag = 0;
    once = true;
  }
}

//--------------------------------Titrate Piston display----------------------------------//
// void Piston()
// {
//     u8g2.clearBuffer(); // Clear the display buffer
//     u8g2.setFont(u8g2_font_helvB08_te); // Set text size and font
//     u8g2.drawStr(5,18, "Wait Positioning");
//     u8g2.sendBuffer(); // Send the buffer to the display
// }

//----------------------------new titrate---------------------------------------------//



void drawRectangleWithArrows(int titrateModeSpeed) {

const uint8_t arrowSize = 10; // Size of the arrow
const uint8_t startY[3] = {25, 30, 40}; // Adjust positions as needed
const uint8_t x = 20; // x-coordinate of the arrowheads (centered horizontally in the rectangle)

  // Draw the rectangle outline
   u8g2.drawFrame(10, 22, 30, 35);

  // Calculate the number of arrows based on titrateModeSpeed (constrained to 1 to 3)
  uint8_t numArrows = constrain(titrateModeSpeed, 1, 3);

  // Calculate the spacing between arrows vertically
  uint8_t arrowSpacing = 35 / (numArrows + 1);

  // Draw the arrows inside the rectangle based on titrateModeSpeed
  for (uint8_t i = 0; i < numArrows; i++) {
    // Calculate y-position of the current arrow based on time (millis()) and arrow index
    // Apply a slight offset to millis() for the third arrow to adjust its timing
    uint8_t yPos;
    if (i == 2) {yPos = 20 + (((millis() + 200) / 10) % (35 - arrowSize));} // Apply offset for third arrow
    //  { // Check if it's the third arrow (index 2)
      // yPos = rectY + (((millis() + 200) / 10) % (rectHeight - arrowSize)); // Apply offset for third arrow
    // } 
    else {yPos = 20 + ((millis() / 10) % (35 - arrowSize));} // Regular animation for first two arrows
    //  {
      // yPos = rectY + ((millis() / 10) % (rectHeight - arrowSize)); // Regular animation for first two arrows
    // }

    // Offset yPos based on arrow index to stagger animation
    yPos += (i + 1) * arrowSpacing;

    // Constrain yPos to be within the rectangle
    yPos = constrain(yPos, 20, 55 - arrowSize);

    // Draw the arrowhead (V shape) at the calculated yPos
    uint8_t arrowX = 10 + (30 / 2); // Center the arrow horizontally
    u8g2.drawLine(arrowX, yPos, arrowX - 4, yPos - 4); // Draw arrowhead (left line)
    u8g2.drawLine(arrowX, yPos, arrowX + 4, yPos - 4); // Draw arrowhead (right line)
  }

  // Send the display buffer to the display
  // u8g2.sendBuffer();
}

//-------------------------------------------------------------DISPENSE MODE------------------------------------------------------//

void uni_dispense()
{
    // uint8_t sensorState = digitalRead(sensorPin);
    double volume_factor = dispense_val / 10.0;
    double E_CALB = (MC * (volume_factor - 0.5)) + E_LOW;
    // Serial.println(E_CALB);
    if(menus == 1)
    {
         double stepcont = 235.984 * volume_factor * 1;
         if ((previousPos != dispense_val && selecting_volume) || (previousPos != repeats && !selecting_volume)) 
         {
             pos1 = 0;
             previousPos = selecting_volume ? dispense_val : repeats;
             once = false;
         }
    }
    else if(menus == 2)
    {
          if ((previousPos != dispense_val && selecting_volumes) || (previousPos != dispense_time && !selecting_volumes)) 
          {
              pos1 = 0;
              previousPos = selecting_volumes ? dispense_val : dispense_time;
              once = false;
          }
    }
    if (once == false) 
    {
      
       u8g2.clearBuffer(); 
       u8g2.setFont(u8g2_font_helvB08_tr);
      //  u8g2.print(" ml");
//--------------------------Menu 4 (Time Dispense)----------------------------------//
       if(menus == 2)
      {
      //  u8g2.setFont(u8g2_font_helvB08_tr);
       u8g2.drawStr(20, 18, "Time Dispense");
       u8g2.drawLine(130, 21, 0, 21);
       u8g2.setCursor(20, 40);
       u8g2.print(volume_factor);

       u8g2.setCursor(20, 60);
       u8g2.print(dispense_time);
       u8g2.print(" min");
        if(selecting_volumes && singleClickCount == 1)
          {
            u8g2.drawStr(12,40, ">");
          }
        else if(selecting_steps)
          {
            u8g2.drawStr(12,60, ">");
          }
      }
//--------------------------Menu 3 (Normal Dispense)-----------------------------------//
      else if(menus = 1)
      {
      u8g2.setFont(u8g2_font_helvB12_tr);
      u8g2.setCursor(40, 30);
      u8g2.print(volume_factor);
      u8g2.setCursor(40, 50);
      u8g2.print(repeats);
        if(selecting_volume && singleClickCount == 1)
          {
            u8g2.drawStr(20,29, ">");
          }
        else if(selecting_steps)
          {
            u8g2.drawStr(20,50, ">");
          }
      }

        u8g2.sendBuffer();
        once = true;
    }
    if (objectDetected) 
    {
    if (objectDetected && dispense_start == true) 
     {
            digitalWrite(buzzer, HIGH);
            buzz.interval(500);
            buzz.start();
            titrateModeSpeed(4);
            float totalVolume = dispense_val / 10;

            if(menus == 1)
            {
              // yPos = 20;
              // encoder0Pos = 3;
              // uint8_t direction = 1;
              uint8_t total = repeats;
              dispense(totalVolume, 1,total);
            }
            else if(menus == 2)
            {
              float totalTimeInSeconds = dispense_time;
              exitDispenseMode = false;
              uint8_t initialMinutes = dispense_time;
              uint8_t initialSeconds = 0;
              updateDisplay(initialMinutes, initialSeconds);
              times(totalVolume, 1, totalTimeInSeconds);
              dispensing_complete = true; 
            }
            dispense_start = false;        
     }
    }
    else if(objectDetected == false && menu_flag == 0 && (menus == 1 || menus == 2))
    {
          digitalWrite(enablem, LOW);
          // digitalWrite(dirPin, LOW);
          digitalWrite(dirPin, HIGH);
          motorspeed(4);
        while (objectDetected == false) 
          {
            motor_run();
          }
          delay(500);
           for (int i = 0; i < 1500; i++) 
            {
        motor_run(); // Assuming motor_run() moves one step
            } 
          digitalWrite(enablem, HIGH);
        // sensorState = 0;
        encoder0Pos = 0;
    }
    // if (dispensing_complete) 
    // {
    //   //  u8g2.clearBuffer();
    //   //  u8g2.drawStr(10, 40, "Dispensing Complete");
    //   //  u8g2.sendBuffer();
    //   //  delay(3000); // Display the message for 3 seconds

    //    dispensing_complete = false; // Reset the flag
    //   //  selecting_volumes = true; // Reset to volume selection
    //    previousPos = 0; // Reset previous position to allow new settings
    //   //  menu_flag = 1;
    // }
}

//------------------------------------------------------------------------
void motor_run()
{
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(500); //220
  digitalWrite(stepPin, LOW);
  delayMicroseconds(500);
}

void motor_runs(bool timedMode = false) {
    unsigned int delayTime = timedMode ? 500 : 500; //200 : 300; // Use 500 µs in timed mode, otherwise 300 µs 230
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayTime);
}

void commonss()
{
    digitalWrite(enablem, LOW);
    digitalWrite(dirPin, HIGH);
}
//-----------------------------------------------------------------------time dispense-----------------------------------------------------------------//

void runCycle(float v, int m, int totalCycles, bool timedMode, float totalTime = 0) {
  uint8_t numCycles = ceil(v / 50); // Number of cycles required
  double E_CALB = (MC * (v - 0.5)) + E_LOW;

  if(E_CALB == 0)
  {
    E_CALB = 1;
  }
  // Serial.println(E_CALB);
  float remainingVolume = v;
  double stepcount; 
  unsigned long cycleDuration = 0, stepDelay = 0, timeRemaining = totalTime * 60;
  unsigned long previousMillis = millis();
  bool processStarted = false;

  // Enable motor driver
  digitalWrite(enablem, LOW);

  if (timedMode) {
    unsigned long totalDuration = totalTime * 60 * 1000;
    unsigned long suctionTimePerCycle = 50 * 235.984 * m / 1000;
    unsigned long adjustedTotalDuration = 1.5 * (totalDuration + suctionTimePerCycle * numCycles);
    cycleDuration = adjustedTotalDuration / numCycles;
  }

  // Repeat cycles only for non-timed mode
  int repeatCycles = timedMode ? 1 : totalCycles; // If timed mode, run only once

  for (uint8_t repeat = 0; repeat < repeatCycles; repeat++) {
    for (uint8_t cycle = 0; cycle < numCycles; cycle++) {
      if (exitDispenseMode) break;

      float cycleVolume = min(remainingVolume, 50);
      stepcount = (timedMode) ? (10 + (235.984 * cycleVolume * m)) : (235.984 * cycleVolume * 1 * E_CALB); //(195 + (235.984 * cycleVolume * 1 * E_CALB));//(235.984 * cycleVolume * 1 * 1); //(10 + (235.984 * cycleVolume * 1 * 1));
      
      // Serial.println(E_CALB);

      // Serial.println(stepcount);
      
      if (timedMode) {
        stepDelay = cycleDuration / (2 * stepcount);
        if (!processStarted) {
          processStarted = true;
          int minutes = timeRemaining / 60;
          int seconds = timeRemaining % 60;
          updateDisplay(minutes, seconds);
        }
      }

      digitalWrite(dirPin, LOW); // Set direction to dispense  //digitalWrite(dirPin, HIGH);
      for (long i = 0; i < stepcount; i++) {
        if (exitDispenseMode) break;
        motor_runs(timedMode);
        if (timedMode) delay(stepDelay);

        // unsigned long currentMillis = millis();
        if (timedMode && millis() - previousMillis >= 10000) { // Update every 10 seconds
          previousMillis = millis();
          timeRemaining = (timeRemaining > 10) ? timeRemaining - 10 : 0;
          updateDisplay(timeRemaining / 60, timeRemaining % 60);
        }

        button0.tick();
        if (exitDispenseMode) {
          processStarted = false;
          digitalWrite(enablem, HIGH); // Disable motor driver
          return;
        }
      }
      
      digitalWrite(enablem, HIGH); // Disable motor driver
      delay(3000);                 // Wait for 3 seconds
      digitalWrite(enablem, LOW);  // Re-enable motor driver

      // Reverse the direction to retract the piston
      digitalWrite(dirPin, HIGH); //digitalWrite(dirPin, LOW);
      for (long i = 0; i < stepcount; i++) {
        if (exitDispenseMode) break;
        motor_run();
        button0.tick();
        if (exitDispenseMode) {
          processStarted = false;
          digitalWrite(enablem, HIGH); // Disable motor driver
          return;
        }
      }

      delay(50); // Wait for suction

      remainingVolume -= cycleVolume;
      if (remainingVolume <= 0) break;
    }

    // Reset remaining volume for the next repeat cycle (only in non-timed mode)
    if (!timedMode) {
      remainingVolume = v;
    }
  }
   
  digitalWrite(enablem, HIGH); // Disable motor driver
  if (timedMode) {
    updateDisplay(0, 0); // Ensure display shows final time as 0
    processStarted = false;
    once = true;
    menu_flag = 1;
  }
}

void dispense(float v, int m, int total) {
  runCycle(v, m, total, false); // Non-timed mode
}

void times(float v, int m, float totalTime) {
  runCycle(v, m, 0, true, totalTime); // Timed mode
}

void updateDisplay(uint8_t minutes, uint8_t seconds) {
  u8g2.clearBuffer(); // Clear the display buffer

  // Display the remaining time in mm:ss format
  u8g2.setFont(u8g2_font_helvB14_tr);  // Set smaller font
  u8g2.setCursor(40, 40);  // Set cursor position
  
  if (minutes < 10) {  
    u8g2.print("0");
  }
  u8g2.print(minutes);  
  u8g2.print(":");  
  
  if (seconds < 10) { 
    u8g2.print("0");
  }
  u8g2.print(seconds); 

  u8g2.sendBuffer(); 
}

//------------------------------------------------------Calibration Mode---------------------------------------------------------//
void Calibration()
{ 
  // double voln = 
  // double stepcont = 235.984 *voln* 1;
  if(previousPos != cal_mode && cal_flag == 0)//------for cal mode change check
  {
    previousPos = cal_mode;
    entry_chk = true;
  }
  if(previousPos != CalVal && cal_flag >= 1)//--------for calibration Value enter check
  {
    previousPos = CalVal;
    change_flag = true;
  }
  if(entry_chk ==  true)
  {
  u8g2.clearBuffer(); // Clear the display buffer
  // Set font and text color
  // Display menu options based on cal_mode
  if (cal_mode) {
    u8g2.drawStr(0,30, "> Disp Calb");
    u8g2.drawStr(0,50, " Titr Calb");
  } else {
    u8g2.drawStr(0,30, "Disp Calb");
    u8g2.drawStr(0,50, "> Titr Calb");
  }
  u8g2.sendBuffer();
  entry_chk=false;
    // Serial.print("enterred calb");
  }
  /////////////////////////////////////////////
  switch(cal_flag)
  {
    // Serial.print("enterred calb");
    case 1:
      if(change_flag == true)
        {
        u8g2.clearBuffer(); // Clear the display buffer
        u8g2.drawStr(0, 16, "Enter Volume"); // Set cursor position for "Enter Vol1"
        u8g2.drawStr(0, 32, "P1 = 0.50");
      if (cal_mode) { // For titration calibration
        unsigned int disp_val = CalVal / 10; // Remove the unit digit of the number
        unsigned int disp_val_unit = CalVal % 10; // Take the unit digit of the number (after decimal num)
        // Serial.println("calmode1");
    // Display formatted calibration value
        u8g2.drawStr(0, 47, "P2="); // Set position for decimal point
        u8g2.setCursor(25, 47); // Set position for main part of the value
        u8g2.print(number3Digit(disp_val)); // Display main part of the value
        u8g2.drawStr(54,47, ".");
        u8g2.setCursor(64, 47); // Set position for unit digit
        u8g2.print(disp_val_unit); // Display unit digit
  } else { // For dispense calibration
        // Display raw calibration value
        // Serial.println("calmode2");
        u8g2.setCursor(0, 32); // Set position for the calibration value
        u8g2.print(CalVal); // Display the calibration value directly
      }
        // Serial.println(CalVal/100);
        u8g2.sendBuffer(); // Transfer internal memory to the display
        V_h = CalVal/10.0; //100
        // Serial.println(V_h);
        change_flag = false;
      }
    break;
       
case 2: // ----------------Common for cal_mode 0 and 1
    if (change_flag == true) {
        for (int i = 5; i > 0; i--) {
            u8g2.clearBuffer(); // Clear the display buffer
            // Display "Dispensing 0.5 ml"
            u8g2.drawStr(20, 30, "Dispensing 0.5 ml"); // Set cursor position for the first line of text
            // Display countdown
            u8g2.setCursor(55, 50); // Set cursor position for the second line of text
            u8g2.print(i); // Display the countdown number

            u8g2.sendBuffer(); // Send the buffer to the display
            delay(1000); // Wait for 1 second
            
        }
        // After the countdown, move to case 2
        change_flag = false;
        
        // cal_count = 1;
        // cal_flag = 2; // Assuming you have a variable to track the current case
    }
    cal_flag++;
    change_flag=true;
    break;

        case 3://-------------dispense rawlow volume i.e 1 ml
      if(change_flag == true)
      {
        voln = 0.5; // Update volume factor
        updateStepcont(); // Recalculate stepcont
        // dispense_calb = 1;
        if(cal_mode == 1)//------for titration calibreation
        { /*Serial.println("calmode3");*/
          titrateModeSpeed(4);//------High Speed
          // calbSteps = 800;
        }
        else
        {
          // Serial.println("calmode4");
          titrateModeSpeed(4);//------High Speed
          // calbSteps = 117.992;
          //  voln = 0.5;
//          titrateModeSpeed(0);//------Highest speed
//          calbSteps = 200;
        }
    //    cal_mode ? calbSteps = 800 : calbSteps = 200;
        calDispDisplay();
        change_flag = false;
      }
      // change_flag = true;
    break;

        case 4://--------------User Enter volume
      if(change_flag == true)
      {
        u8g2.clearBuffer(); // Clear the display buffer
        u8g2.drawStr(0,16, "Enter Vol1");

      if (cal_mode) { // For titration calibration
        unsigned int disp_val = CalVal / 10; // Remove the unit digit of the number
        unsigned int disp_val_unit = CalVal % 10; // Take the unit digit of the number (after decimal num)
        // Serial.println("calmode5");
    // Display formatted calibration value
        u8g2.setCursor(5, 32); // Set position for main part of the value
        u8g2.print(number3Digit(disp_val)); // Display main part of the value
        u8g2.drawStr(54,32, ".");
        u8g2.setCursor(64, 32); // Set position for unit digit
        u8g2.print(disp_val_unit); // Display unit digit
  } else { // For dispense calibration
        // Display raw calibration value
        // Serial.println("calmode6");
        u8g2.drawStr(5,32, CalVal);
      }

        u8g2.sendBuffer(); // Transfer internal memory to the display
         rawLow = CalVal/10.0; //10
        // Serial.println(rawLow);
        change_flag = false;
      }
    break;

        case 5://--------------Common for cal_mode 0 and 1 for Volume2 100 ml
       if (change_flag == true) {
        for (int i = 5; i > 0; i--) {
            u8g2.clearBuffer(); // Clear the display buffer
            // Display "Dispensing 0.5 ml"
            u8g2.drawStr(20,30, "Dispensing High ml");
            // Display countdown
            u8g2.setCursor(55, 50); // Set cursor position for the second line of text
            u8g2.print(i); // Display the countdown number
            u8g2.sendBuffer(); // Send the buffer to the display
            delay(1000); // Wait for 1 second
            
        }
        // After the countdown, move to case 2
        change_flag = false;
        
        // cal_count = 1;
        // cal_flag = 2; // Assuming you have a variable to track the current case
    }
    cal_flag++;
    change_flag=true;
    break;

    case 6://--------------dispense rawhigh volume 100 ml
      if(change_flag == true)
      { 
        // Serial.println(V_h);
         updateStepcontd();
        // dispense_calb = 100;
        if(cal_mode == 1)
        { /*Serial.println("calmode7");*/
          titrateModeSpeed(4);//------High Speed
          // calbSteps = 800;
        }
        else
        {
          // Serial.println("calmode8");
          titrateModeSpeed(4);//------High Speed
          // calbSteps = 800;
//          titrateModeSpeed(0);//------Highest speed
//          calbSteps = 200;
        }
    //    cal_mode ? calbSteps = 800 : calbSteps = 200;
        calDispDisplay();
        change_flag = false;
      }
    break;

    case 7://---------------User input volume 2 - 100 ml
      if(change_flag == true)
      {
        u8g2.clearBuffer(); // Clear the display buffer
        u8g2.drawStr(0,16, "Enter Vol 2");
      if (cal_mode) { // For titration calibration
        unsigned int disp_val = CalVal / 10; // Remove the unit digit of the number
        unsigned int disp_val_unit = CalVal % 10; // Take the unit digit of the number (after decimal num)
       // Display formatted calibration value
        u8g2.setCursor(5, 32); // Set position for main part of the value
        u8g2.print(number3Digit(disp_val)); // Display main part of the value
        u8g2.drawStr(54,32, ".");
        u8g2.setCursor(64, 32); // Set position for unit digit
        u8g2.print(disp_val_unit); // Display unit digit
  }   else { // For dispense calibration
       // Display raw calibration value
        u8g2.setCursor(0, 32); // Set position for the calibration value
        u8g2.print(CalVal); // Display the calibration value directly
      }

        u8g2.sendBuffer(); //------display all above set values
        // rawHigh = CalVal;//-------transfer value to rawHigh variable
        rawHigh = CalVal/10.0; //10
        // Serial.println(rawHigh,3);
        change_flag = false;
      }
    break;

    case 8://--------------------------Calibration calculation and Complete Message display
      if(cal_mode) 
      { 
        E_LOW = voln/rawLow;
        E_HIGH = V_h/rawHigh;
         EEPROM.put(104,E_LOW);
        MC = (E_HIGH-E_LOW)/(V_h-voln);
         EEPROM.put(4,MC);
                 //  EEPROM.put(4,dispenseSteps);  //-------save calibrated value in eeprom address 4
              //   Serial.print("ELOW:");
              //   Serial.println(E_LOW,3);
              //  Serial.print("EH:");
              //  Serial.println(E_HIGH,3);
              //  Serial.println(MC,3);
        // Serial.println(rawHigh);
        // Serial.println(rawLow); 
               // Serial.println(voln); 
               // Serial.println(V_h);
        // Serial.println("calmode11");
      }
      else
      { 
        titrationSteps = round(79200/((rawHigh-rawLow)/10)); //---we get rawhigh - 1000 and rawlow - 10 hence divide by 10 to get proper value.
        // EEPROM.put(1,titrationSteps); //-------save calibrated value in eeprom address 1

   
        // Serial.println("calmode12");
        // updatecalb();
        // dispenseSteps = /*round(19800/(rawHigh-rawLow));*/
       
      } 
//      cal_mode ? titrationSteps = round(79200/(rawHigh-rawLow)) : dispenseSteps = round(19800/(rawHigh-rawLow));
       u8g2.clearBuffer(); // Clear the display buffer
       u8g2.drawStr(0,16,"Calibration Done");
       u8g2.sendBuffer();
      //---------------------------buzzer indication--------------//
      digitalWrite(buzzer, HIGH);
      buzz.interval(700);
      buzz.start();
      //---------------------------------------------------------//
      delay(1000);
      cal_flag = 0;
      menus = 0;
      change_flag = true;
      entry_chk = true;
    break;
  }
  if (motor.state() == RUNNING) {
  if (cal_flag == 3 && stepperCount >= stepc) { // For Case 3 with stepc condition
  // Serial.println("enteredlow");
  // Serial.println(stepc);
    stopDispense(); // Stop the stepper motor
    stepperCount = 0; // Reset cycle count
    cal_flag++; // Increment flag
    CalVal = 0; // Reset the variable value
    change_flag = true;
    digitalWrite(buzzer, HIGH); // Buzzer indication
    buzz.interval(700);
    buzz.start();
  } else if (cal_flag == 6 && stepperCount >= stepd) { // For Case 6 with stepd condition
  // Serial.println("enteredHIGH");
    stopDispense(); // Stop the stepper motor
    stepperCount = 0; // Reset cycle count
    cal_flag++; // Increment flag
    CalVal = 0; // Reset the variable value
    change_flag = true;
    digitalWrite(buzzer, HIGH); // Buzzer indication
    buzz.interval(700);
    buzz.start();
  }
  }
}
//-------------------------------------PRIMING MODE------------------------------------//
void display_priming()
{ 
  //  int sensorState = digitalRead(sensorPin);
   pos1=0;
   previousPos = 0; 
  if(previousPos != encoder0Pos)//------------Check encoder value with previous 
  {
    previousPos = encoder0Pos;
    change_flag = true;
  }
  if(once == false)//------------------------Execute only once when "once = false"
  {
      encoder0Pos=0;
      // buzz.stop();
    u8g2.clearBuffer();          // clear the internal memory
      //  u8g2.setFont( u8g2_font_bitcasual_tu);   
      //  Disp_date_time();
    u8g2.setFont(u8g2_font_helvB08_te);
    u8g2.drawStr(20,19,"PRIMING MODE");
    u8g2.drawLine(130, 23, 0, 23);  

    u8g2.drawStr(13,37,"Press knob to start");
    u8g2.drawStr(42,51,"Priming");
    u8g2.sendBuffer();
    once = true;
  } 
  if(pos1==0)                       //(sensorState==HIGH)                              //(pos1==0)
  {
  if (priming == 1 && change_flag == true) // Start priming
{  
  //  startDispense();
    digitalWrite(enablem, LOW);
    digitalWrite(dirPin, HIGH);  // Motor direction anticlockwise.
    startDispense();
    titrateModeSpeed(4);
    u8g2.clearBuffer();  
    // Update the display before starting the motor loop
    u8g2.drawStr(20, 19, "PRIMING MODE");
    u8g2.drawLine(130, 23, 0, 23);
    u8g2.drawStr(13, 37, "Priming in Progress");
    u8g2.sendBuffer();
    change_flag = false; 
}

  }
  if(priming == 2)       //------------when press knob again,it will stop the priming.
  { 
    stopDispense();//----------stop motor
    if (objectDetected == false) {//pos1 == 1 && double_flag1 == 1
    // Serial.println("not optic");
    digitalWrite(enablem, LOW);
    digitalWrite(dirPin, LOW);
    motorspeed(4);
    while (objectDetected == false) {
    // for (int k = 0; k < 5000; k++) {
              motor_run();
    }
              delay(500);
           for (int i = 0; i < 800; i++) 
            {
        motor_run(); // Assuming motor_run() moves one step
            } 
    }
    // sensorState = 0;
    stopDispense();
    //---------------reset all the variables-----------//
    priming = 0;
    change_flag = true;
    once = false;
    previousPos = 0;
    encoder0Pos = 0;
    stepperCount=0;
    //---------------------------buzzer indication--------------//
    digitalWrite(buzzer, LOW);
    buzz.interval(700);
    buzz.start();
  }
}
//--------------------------Stepper motor Pulse Ticker Loop---------------------//
void stepperPulse()
{
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(stepPin, LOW);
  stepperCount++;
}

void motorspeed(uint8_t m)
{
   switch (m) {
        case 5: // High Speed (1/2 step)
            digitalWrite(M0, HIGH);
            digitalWrite(M1, LOW);
            digitalWrite(M2, LOW);
            break;

        case 4: // High Speed (1 step)
            digitalWrite(M0, LOW);
            digitalWrite(M1, LOW);
            digitalWrite(M2, LOW);
            break;

        case 3: // Mid Speed (1/4 step)
            digitalWrite(M0, LOW);
            digitalWrite(M1, HIGH);
            digitalWrite(M2, LOW);
            break;

        case 2: // low Moderate Speed (1/8 step)
            digitalWrite(M0, HIGH);
            digitalWrite(M1, HIGH);
            digitalWrite(M2, LOW);
            break;

        case 1: // Low Speed (1/32 step)
            digitalWrite(M0, LOW);
            digitalWrite(M1, HIGH);
            digitalWrite(M2, HIGH);
            break;
    }
}

//----------------------------titartion mode selection----------------------------------//
void titrateModeSpeed(int modeSpeed)
{
    // Setting motor pins based on the modeSpeed
    digitalWrite(dirPin, HIGH); // Set the direction pin
    switch (modeSpeed) {
        case 5: // High mid Speed (1/2 step)
            motorspeed(5);
            break;

        case 4: // High Speed (1/4 step)
            motorspeed(4);
            break;

        case 3: // Mid Speed (1/4 step)
            motorspeed(3);
            break;

        case 2: // Low Moderate Speed (1/8 step)
            motorspeed(2);
            break;

        case 1: // Low Speed (1/32 step)
            motorspeed(1);
            break;
    }
}
//-----------------------------Start Stepper Motor----------------------//
void startDispense()
{
  motor.start(); //timer for motor
  digitalWrite(enablem, LOW);    //active low
}
//----------------------------Stop Stepper Motor-----------------------//
void stopDispense()
{
  motor.stop(); //timer for motor
  digitalWrite(enablem, HIGH);  //active low
}
//--------------------------Stop Buzzer Ticker Loop----------------------//
void buzzer_stop()
{               
  digitalWrite(buzzer, LOW);
  buzz.stop();
}
//--------------------------------Convert Single digit nuber into double digit sting------------------------//
 String number2Digit(int number)
{
 char localString[] = "00";
 sprintf(localString,"%02d",number);
 return localString;
}
// --------------------------------Convert Single digit nuber into double digit sting------------------------//
 String number3Digit(int number)
{
 char localString[] = "000";
 sprintf(localString,"%03d",number);
 return localString;
}

// void setTime(byte SEC, byte MIN, byte HOUR, byte DAY, byte DATE, byte MONTH, byte YEAR) //FUNCTION FOR WRITE DATA TO RTC MODEULE
// {
//   Wire.beginTransmission(RTCAddress);    // START I2C COMMUNICATIONS WITH STARTING ADDRESS 0x68 
//   Wire.write(0);                         // WRITE THE FIRST ADDRESS i.e 0x00
//   // Wire.write(dec2bcd(SEC));           // WRITE SENCOND VALUE IN RTC MODULE
//   // Wire.write(dec2bcd(MIN));
//   // Wire.write(dec2bcd(HOUR));
//   // Wire.write(dec2bcd(DAY));
//   // Wire.write(dec2bcd(DATE));
//   // Wire.write(dec2bcd(MONTH));
//   // Wire.write(dec2bcd(YEAR));
//   Wire.endTransmission();                 // END THE I2C WIRE COMMUNICATION
// }

// void readTime(byte *second, byte *minute, byte *hour, byte *day, byte *date, byte *month, byte *year)   //FUNCTION FOR READ DATA FROM RTC MODEULE
// {
//   Wire.beginTransmission(RTCAddress);     // START I2C COMMUNICATIONS WITH STARTING ADDRESS 0x68 
//   Wire.write(0);                          // WRITE THE FIRST ADDRESS i.e 0x00
//   Wire.endTransmission();                 // END THE I2C WIRE COMMUNICATION                    
//   Wire.requestFrom(RTCAddress, 7);        // REQUEST THE DATA FROM I2C(RTC MODEULE DS-1307) AND HOW MANY BYTES YOU WANT TO READ (7)          
//   // *second = bcd2dec(Wire.read() & 0x7F);  // READ ADDRESS OF SECOND REGISTER IN RTC MODULE AND ASSIGN TO THE *second POINTER & WITH 0x7F(THIS IS THE SECOND ADDRESS IN RTC MODULE)
//   // *minute = bcd2dec(Wire.read());
//   // *hour = bcd2dec(Wire.read() & 0x3F);
//   // *day = bcd2dec(Wire.read());
//   // *date = bcd2dec(Wire.read());
//   // *month = bcd2dec(Wire.read());
//   // *year = bcd2dec(Wire.read());
// }

// void ReadTime()   //-------function for read time
// {

//   byte second, minute, hour, day, date, month, year;
//   readTime(&second, &minute, &hour, &day, &date, &month, &year);   //SECOND, MINUTE, DAT, MONT, YER;
// }

byte bcd2dec(byte val)                  // THIS FUNCTION USE TO CONVERT THE BCD INTO DEC DATA TO DISPLAY 
{
  return(( val / 16 * 10 ) + ( val % 16 ));
}

byte dec2bcd(byte val)                  // THIS FUNCTION USE TO CONVERT THE DEC INTO BCD DATA TO WRITE IN CHIP
{
  return(( val / 10 * 16 ) + ( val % 10 ));
}
//------------------------------user select mode---------------------------------//
void select_mode()
  { 
      if (once_update == 1)
       { 
         u8g2.clearBuffer();          // clear the internal memory
          u8g2.setFont(u8g2_font_helvB08_te);
         u8g2.drawStr(32,18,"SELECT USER"); 
        //  u8g2.setFont(u8g2_font_helvB08_te);
         u8g2.drawLine(130, 19, 0, 19);  //u8g2.drawLine(130 = length, 13 = start point (vertical position y) , 0 = horizontal start position (x), 13 = end point);
        //  u8g2.drawStr(25,25,"SELECT USER");
        //  u8g2.drawLine(130, 27, 0, 27); 
         u8g2.drawStr(0,29,">");
         u8g2.setCursor(10,30);
         u8g2.print(User_name[encoder0Pos]);
         u8g2.setCursor(10,41);
         u8g2.print(User_name[encoder0Pos+1]); 
         u8g2.setCursor(10,53);
         u8g2.print(User_name[encoder0Pos+2]); 
         u8g2.setCursor(10,65);
         u8g2.print(User_name[encoder0Pos+3]); 
         u8g2.sendBuffer();          // transfer internal memory to the display
         User_set_flag = 1;
       }
     } 
//-------------------------------------------------setting mode--------------------------------------------------//
void setting_mode() {
  u8g2.clearBuffer(); 
    // Get titration values for all users
  for (uint8_t i = 0; i < 4; i++) {
        uint8_t index = currentPageStart + i;
        //         Serial.print("currentPAGEs :");
        // Serial.println(currentPageStart);
        if (index >= totalTitrations) break;  // Stop if we've reached the end
        u8g2.setFont(u8g2_font_helvB08_te);
         u8g2.drawStr(40,18,"RECORD"); 
        //  u8g2.setFont(u8g2_font_helvB08_te);
         u8g2.drawLine(130, 19, 0, 19);  
        uint8_t yPosition = 32 + (i * 20);  // Adjust y position for each entry
                if (i == 0) {  // Assuming the first value in the visible list is the selected one
            u8g2.setCursor(0, yPosition);
            u8g2.print("=>");  // Draw arrow
        }
        u8g2.setCursor(15, yPosition);
        u8g2.print(titrations[index].titrationValue / 100);  // Integer part
        u8g2.drawStr(26, yPosition, ".");
        u8g2.setCursor(31, yPosition);
        u8g2.print(titrations[index].titrationValue % 100);  // Decimal part
        u8g2.setCursor(50, yPosition);
        u8g2.print(User_name[titrations[index].userID]);  // Display user name from the User_name array
    }
    u8g2.sendBuffer();  // Send buffer to the OLEDer();  // Send buffer to the OLED
}
//--------------------------CALIBRATION SUB FUNCTION-------------------------------//
void calDispDisplay()//------start motor and display message for vol1 and vol2 of calbaration
{
  startDispense();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB08_te); // Set text size and font
  u8g2.drawStr(20,30,"Dispensing Wait");
if (cal_flag == 3) 
{
    u8g2.drawStr(40,50,"0.5 ML");
} 
else if (cal_flag == 6) 
{
    u8g2.drawStr(40,50,"HIGH ML");
}
u8g2.sendBuffer();
}
