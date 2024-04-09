   /* Guidance system
1,2,3,4 - Align with with which motor
5 - turn leftf
6 - turn right
7 - Set alignment with chosen motor
8 - Set chosen direction

a. calculate the angle between chosen motor and chosen direction ABS
b. calculate all the other angles of all the other motors relative to the first motor
c. populate the propulsion algorithm motors with their angles
d. disable the startup posistion
e. propel


R - reset variables (including the above) as well as other measure devices - timer - laser trigger

C - Increase speed
D - Decrease speed

F - Forward propulsion
B - Backward propulsion

A - Turn left with 5 degrees when propelling
9 - Turn right with 5 degrees when propelling
 */

/*
 NOTES
 2021/11/21
1. Calibration can be implemented to give more stability to the BNO055
2. Rotation RPM can be governed to not go to high
3. delay statement of the BNO055 can be increated up and until a 100 mhz
2022/02/14
1.Heading will now be used, based on the magnotometer,plus/minus a number of degrees which will determine direction to move in (Heading)
2022/03/26
1.Rotation speed control
  Rotation will now continues and at a stable RPM. However, as seen in the past, the rotation accelerates too quickly for translation motors
  to work properly. If the 2 rotation motors power is reduced, they will not have enough impact to turn when competing with translational motors
  Therefore, 2 additional rotation motors will be switched, which provide thrust in the opposite direction as the other 2.
  They will run at a lower RPM then the original 2 motors. This will provide first and foremostly, rotation speed control and the ability to up
  scale the thrust - that is the theory. This will be tested.The difference between these will determin the RPM and both motors can run 80/90% strength
  or 20/30% strenght.
 */
 

#define DEBUG1
#define DEBUGG1
#define SD_ON //_OFF
#define SD_FILE_ONE_ON //_OFF
//************************************************
//PWM
//************************************************
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <avr/sleep.h>
//************************************************

//************************************************
//SERVO - STEERING
//************************************************
#include <Servo.h>

//************************************************
//RADIO
//************************************************
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//************************************************
//SD CARD - DATA LOGGER
//************************************************
//#include <SD.h>
#ifdef SD_ON
  #include "SdFat.h"
#endif

//************************************************
//RTC
//************************************************
#include <DS3231_Simple.h>

//************************************************
//INSTANCIATE
//************************************************
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#ifdef SD_ON
  SdFat SD;
#endif
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//************************************************
//CONSTANTS
//************************************************
#define BNO055_SAMPLERATE_DELAY_MS (40) //Magnometer can't go higher then 20hz...aka 50ms

//actualDirection can either be 0 or 1
static const int FORWARD = 0; 
static const int BACKWARD = 1;

#ifdef SD_ON
  #ifdef SD_FILE_ONE_ON
  char *FileN = "setup.txt";
  char *FileL = "loop.txt";
  char *FileK = "key.txt";
  char *FileR = "rf.txt";
  char *FileT = "time.txt";
  char *FileRF = "radiofound";
  #else
  char *FileN = "setup.txt";
  char *Pre = "data";
  char *Ext = ".txt";
  #endif 
#endif 

char *BNOF = "BNO055 fail";
char *BNOI = "BNO055 init";
char *co = ",";
bool blnTest=false;
bool blnCutOff=false;
//************************************************
//RADIO
//************************************************
 
//SPI
const int SD_Select = 4;
const int RF_CSN = 10;
const int RF_CE = 7; 

const uint64_t pipe = 0xE8E8F0F0E1LL;
RF24 radio(RF_CE,RF_CSN);


DS3231_Simple Clock;

char data[50] = "";
char key = 'N';

bool blnKey1 = false;
bool blnKey2 = false;
bool blnKey3 = false;
bool blnKey4 = false;
bool blnKey5 = false;
bool blnKey6 = false;
bool blnKey7 = false;
bool blnKey8 = false;
bool blnKey9 = false;
bool blnKeyA = false;
bool blnKeyB = false;
bool blnKeyC = false;
bool blnKeyD = false;
bool blnKeyE = false;
bool blnKeyF = false;
bool blnKeyR = false;

bool blnOnceOff=false;
bool blnSensOnce=false;
bool blnInitStart=false;
bool blnFirstRun=false;
int theRunSpeed; 
int safetyclock;//Restrict to 5 sec
int SetSec=3320; //Seconds running

//Max & Min Range for PWM
int HighVal;
int LowVal;
int xrot=0;
int xmag=0;
int xm; //Magnetometer
bool blnCalibrate=false;
int intSpdCnt=0;
int prev_xi=0;
bool blnSlower=false;
//int intPrevxi = 0;
//bool blnThreeSixty=false;

bool blStartRec=false;
//************************************************
String SetupdataString;
String LoopdataString="";
#ifdef SD_ON
  int FileCounter = 1; //Use to add uniqueness and sequence to files during resets
#endif
//************************************************
//Servo - steering
//************************************************
Servo Steering;
int pos = 90;
int posninety=0; //track 90 degree turns

//GUIDANCE
int MotorAlignedWith; //Once set, it will stay static, unless reset is hit. Can be 1,2,3 or 4
double MotorAlignedAngle; //The angle register with the chosen aligned motor
double ChosenPropulsionDirection; //The direction that the artifact will propel itself in.
bool IgnitionSet; //Once set, only a reset/switch off&on can make this value false again


//************************************************
//SETUP
//************************************************
void setup() {
  #ifdef DEBUGG
  Serial.begin(9600);
  #endif
  
  LoopdataString.reserve(30); //75
  SetupdataString.reserve(52);

  //LEDS
   pinMode(3, OUTPUT); //YELLOW
   pinMode(8, OUTPUT); //RED
   pinMode(9, OUTPUT); //GREEN

   

  pinMode(SD_Select,OUTPUT);
  digitalWrite(SD_Select, HIGH);

  //SERVO SETUP
  SetupdataString += "Servo init";
  Steering.attach(5);
  Steering.write(pos);
  //
  delay(1000);

  //pinMode (defaultSS,OUTPUT);
  //SPI.begin();
  
  #ifdef DEBUG
  Serial.begin(9600);
  Serial.println(F(""));
  #endif
  /* Use external crystal for better accuracy */
  /* Initialise the sensor */

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    #ifdef DEBUG
    Serial.print(F("no BNO055 detect"));
    #endif
    SetupdataString += BNOF;
    return;
  }
  else
  {
    SetupdataString += BNOI;
    bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  //sensor_t sensor;
  //bno.getSensor(&sensor);
  //delay(500);
  
  #ifdef SD_ON
  //Initialize SD Card
  digitalWrite(SD_Select, LOW);
  if (!SD.begin(SD_Select)) {
    #ifdef DEBUG
    Serial.println(F("SD fail"));
    #endif
    SetupdataString += "SD fail";
    return;
  }
  else
  {
    SetupdataString += "SD init";
  }
  #endif
  digitalWrite(SD_Select, HIGH);
  SPI.transfer(0);
  
  pwm.begin(); 
  pwm.setPWMFreq(400);
  SetupdataString += "PWM 16 init";

//Max & Min Range for PWM Freq 200
  HighVal = 3310;
  LowVal = 3040; //2770;
  
    delay(1000);
  pwm.setPWM(0,0,2700);
    delay(250);
  //pwm.setPWM(1,0,2700);
  //  delay(250);
  pwm.setPWM(2,0,2700);
    delay(250);
  pwm.setPWM(3,0,2700);
    delay(250);
  //pwm.setPWM(4,0,2700);
  //  delay(250);
  pwm.setPWM(5,0,2700);
    delay(250);
  //pwm.setPWM(6,0,2700);
  //  delay(250);
  //pwm.setPWM(7,0,2700);
  //  delay(250);
  pwm.setPWM(8,0,2700);
    delay(250);
  //pwm.setPWM(9,0,2700);
  //  delay(250);
  pwm.setPWM(10,0,2700);
    delay(250);
  pwm.setPWM(11,0,2700);
    delay(250);
  //pwm.setPWM(12,0,2700);
  //  delay(250);
  pwm.setPWM(13,0,2700);
    delay(250);
  //pwm.setPWM(14,0,2700);
  //  delay(250);
  //pwm.setPWM(15,0,2700);
  //  delay(250);
    
  delay(2000);

  theRunSpeed = 1450;

  //Receiver activated
  #ifdef DEBUG
  Serial.println(F("Receiver started"));
  #endif
  
  //Set Receiver as active on SPI
  digitalWrite(RF_CSN, LOW);
  delay(10);
  radio.begin();
  delay(10);
  radio.openReadingPipe(1,pipe);
  delay(10);
  radio.startListening(); 
  delay(10);
  digitalWrite(RF_CSN, HIGH);
  delay(10);
  //
  Clock.begin();
  delay(10);
  // Check to see if the file exists:
  digitalWrite(SD_Select, LOW);
  FileExists();
  delay(100);
  #ifdef SD_ON
  WriteSetupLog();
  digitalWrite(SD_Select, HIGH);
  delay(10);
  digitalWrite(RF_CSN, HIGH);
  delay(10);
  #endif
  //Indicate ready
  digitalWrite(9, HIGH);
  digitalWrite(8, HIGH);
  digitalWrite(3, HIGH);   
  delay(2000);                       
  digitalWrite(9, LOW);
  digitalWrite(8, LOW);
  digitalWrite(3, LOW); 
  delay(2000);
  
  blnCalibrate = false;
  blnFirstRun=false;

  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

//************************************************
//LOOP    
//************************************************
void loop() {
  digitalWrite(9, LOW);    
  DoCalibrate();

  if(blnOnceOff==false)
  {
    //readBNO(); //2022-01-31 switch off
  }
//Set Receiver as active on SPI
digitalWrite(RF_CSN, LOW);
bool tx_ds,tx_df,rx_dr;
radio.whatHappened(tx_ds,tx_df,rx_dr);    
         
      while(radio.available() ||(blnKey1 == true) || (blnKey2 == true) || (blnKey3 == true) || (blnKey4 == true) || (blnKey5 == true) || (blnKey6 == true) || (blnKey7 == true) || (blnKey8 == true)
                              || (blnKey9 == true) || (blnKeyA == true) || (blnKeyB == true) || (blnKeyC == true) || (blnKeyD == true) || (blnKeyE == true) || (blnKeyF == true))
      {                 
        readBNO(); //Main sensor read
        
        //Retrieve radio signal
        radio.read(data ,sizeof(data) );
        key = data[0];        
        digitalWrite(RF_CSN, HIGH);

        //Set SD Card as active on SPI
        digitalWrite(SD_Select, LOW);          
        #ifdef SD_ON
          WriteKeyPress();              
        #endif   
        digitalWrite(SD_Select, HIGH);          

        //receive transmission every second from timer - not implemented yet
        if(key == 'T')
        {
          //LoopdataString += String(data[12]) + ","; //Steering angle + ","
          //key = ""; //record time only once
        }
        
        //END TIME - check for Lazer broke time or remote control E button
        if(key == 'Z')
        {
          //LoopdataString += String(data[12]) + ",";
          ResetRoutine();
          readBNO();
          //key = ""; //record time only once
          exit(0); //End the loop
        }
        if ((key == '1') || (key == '2') || (key == '3') || 
            (key == '4') || (key == '5') || (key == '6') ||
            (key == '7') || (key == '8') || (key == '9') || 
            (key == 'A') || (key == 'B') || (key == 'C') ||  
            (key == 'D') || (key == 'E') || (key == 'F') ||
            (key == 'R') 
            || (blnKey1 == true) || (blnKey2 == true) || (blnKey3 == true) || (blnKey4 == true) || (blnKey5 == true) || (blnKey6 == true) || (blnKey7 == true) || (blnKey8 == true)
            || (blnKey9 == true) || (blnKeyA == true) || (blnKeyB == true) || (blnKeyC == true) || (blnKeyD == true) || (blnKeyE == true) || (blnKeyF == true))
            {  

              //If the key is R - Reset
              if (key == 'R')
              {
                ResetRoutine();
              }  
                    
                //Check any KeyVariable is active and that havent been cancelled
                  if ((blnKey1 == true) || (blnKey2 == true) || (blnKey3 == true) || (blnKey4 == true) || (blnKey5 == true) || (blnKey6 == true) || (blnKey7 == true) || (blnKey8 == true)
                      || (blnKey9 == true) || (blnKeyA == true) || (blnKeyB == true) || (blnKeyC == true) || (blnKeyD == true) || (blnKeyE == true) || (blnKeyF == true))
                  {
                    
                    //**********************************
                    //Various Guidance System Routines
                    //**********************************

                    //NOTES
                    //blnKey1 = false; 
                    //Maybe it is better to press ResetKey once the BasicTest run is to be stopped, 
                    //that means one program runs until cancelled.
                    //Also, don't use delay in routines like BasicTest if not needed.Keep it clean.
                    //Some keys can be used to increase global values that are related to a routine like BasicTest()

                                        
                    if (blnKey1 == true)
                    {
                      digitalWrite(3, HIGH);
                      blnKey1 = false;                                               
                    }

                    if (blnKey2 == true)
                    {
                      //digitalWrite(3, HIGH);
                      SetSec=2320;
                      blnKey2 = false;
                    }                    

                    if (blnKey3 == true)
                    {
                      
                      SetSec=3320;
                      delay(10);
                      //digitalWrite(3, HIGH);
                      blnKey3 = false;                    
                    }
                    
                    if (blnKey4 == true)
                    {
                      SetSec=4320;
                      delay(10);
                      //digitalWrite(4, HIGH);
                      blnKey4 = false;                    
                    }                    

                    if (blnKey5 == true)
                    {
                        SetSec=5320;
                        delay(10);
                        /*
                        if (pos <= 180 &&  pos >= 0) 
                        {
                          pos += 5;
                          if (pos>180)
                          pos=180;
                          Steering.write(pos);
                          //readBNO(); 
                          digitalWrite(9, HIGH);   
                          //delay(20);                       
                          //digitalWrite(9, LOW);                                      
                        }
                        else
                        {
                          pos = 180;
                        }
                        */
                      blnKey5 = false;                                       
                    }
                    
                    if (blnKey6 == true)
                    {
                        SetSec=6320;
                        delay(10);
                        /*
                        if (pos <= 180 &&  pos >= 0) 
                        {
                          pos += -5;
                          if (pos<0)
                            pos=0;
                          Steering.write(pos);
                          //readBNO(); 
                          digitalWrite(9, HIGH);   
                          //delay(20);                       
                          //digitalWrite(9, LOW);                                      
                        }
                        else
                        {
                          pos = 0;
                        }
                        */
                      blnKey6 = false;                                       
                    } 
                    
                    if (blnKey7 == true)
                    {
                      MotorAlignedAngle = pos; 
                      digitalWrite(8, HIGH);   
                      delay(20);                       
                      digitalWrite(8, LOW);                                      
                      blnKey7 = false;                                       
                    }                                        

                    if (blnKey8 == true)
                    {
                      ChosenPropulsionDirection = pos; 
                      //digitalWrite(9, HIGH);   
                      //delay(10);                       
                      //digitalWrite(9, LOW);                                      
                      blnKey8 = false;                              
                    }
                     
                    if (blnKey9 == true) //steer left
                    {
                      //BasicMotorSpinTest();
                      blnKey9 == false;
                    }
            
                    //**********************************
                    //Manipulation of Guidance System Routines
                    //**********************************
                    
                    //F-FORWARD
                    if (blnKeyF == true)
                    {
                       delay(1);   
                       blnKeyF = false;
                       //LoopdataString += (String)key + ",";                        
                    }  
                     
                    //B-BACKWARD
                    if (blnKeyB == true)
                    {   
                        blnCutOff=false; //Enable execution again 2022-01-11
                        blnInitStart=false;//Allow the initial 300ms to be executed again
                        delay(5);
                      blnKeyB = false;
                      //LoopdataString += (String)key + ",";                      
                    }
                    
                    //C-INCREASE 25 (Needs attention)
                    if (blnKeyC == true)
                    {
                      if (theRunSpeed < 1697) //450 is the max
                      {
                        theRunSpeed= theRunSpeed + 90;     //C-INCREASE 25 
                          if (theRunSpeed > 1697)
                             theRunSpeed = 1697;                   
                      }
                      else
                      {
                      theRunSpeed= 1697;
                      }
                      blnKeyC = false;
                      if (blnKeyA == true)  
                      GuidanceM2(xrot,theRunSpeed,xmag);                                      
                    }   
                    
                    //D-DECREASE 25 (Needs attention)
                    if (blnKeyD == true)
                    {
                      if (theRunSpeed >= 1337) //50 is min 1337
                      {
                      theRunSpeed= theRunSpeed - 90;     //D-DECREASE 25   
                        if (theRunSpeed < 1337)
                             theRunSpeed = 1337;                    
                      }
                      else
                      {
                        theRunSpeed = 1337;
                      }                    
                      blnKeyD = false;
                      if (blnKeyA == true)
                      GuidanceM2(xrot,theRunSpeed,xmag);
                    }

                    //A-Add 
                    if (blnKeyA == true)
                    {
                      if(blnCutOff==false) //only run code for 1 time, then reset mannually
                      {
                        blnCutOff=true;
                        safetyclock = 0;
                        delay(2);
                      }
                      //blnOnceOff=true; //disable to enable 4 second runs
                      blStartRec=true;
                      blnTest=false; //2022-01-30 Set to allow multiple recordings
                      //if(blnSensOnce==false)
                      //{
                      //  readBNO(); //Have to read sensor once before you start, otherwise their will be not initial orientation value
                        //delay(15); //2022-01-30 Added to see if initial run can be stabilised, since the reset routine have a readBNO in it and we don't want to run it manual before pressing A
                      //  blnSensOnce=true;
                      //}
                      GuidanceM2(xrot,theRunSpeed,xmag); 
                      delay(1); 
                      //blnKeyA = false;
                       //LoopdataString += (String)key + ",";                      
                    }

                    //E-Deduct 
                    if (blnKeyE == true)
                    {
                     
                      delay(1);   
                      blnKeyE = false;
                       //LoopdataString += (String)key + ",";                                             
                    }
                                                                                
                  }
                  else
                  {
                    //Check which key has been pressed
                   SwitchAKey (key);
                  }        
            }
              if(blnOnceOff==false)
              {              
                delay(BNO055_SAMPLERATE_DELAY_MS);
                safetyclock += BNO055_SAMPLERATE_DELAY_MS;
                if (safetyclock == SetSec) //This should ensure that the vehicle can only run 4 seconds (the 300ms is for the kick start in the beginning)
                {
                  blnTest=true;
                  ResetRoutine();
                  safetyclock = 0;
                  blnSensOnce=false; //2022-01-30 Added so that all restarts will start the same
                  blnKeyA = false;
                }     
              }       
        }
    //}
    /*
  if(blnOnceOff==false)
  {
    delay(BNO055_SAMPLERATE_DELAY_MS);
    safetyclock += BNO055_SAMPLERATE_DELAY_MS;
    if (safetyclock == SetSec) //This should ensure that the vehicle can only run 4 seconds (the 300ms is for the kick start in the beginning)
    {
      blnTest=true;
      ResetRoutine();
      safetyclock = 0;
      blnKeyA = false;
    }
  }
  */
  //delay(2);
}

void SwitchAKey (char keypress)
{
  switch (keypress)
  {
    case '1': //Key1 has been pressed
       blnKey1 = true;break;
    case '2': //Key2 has been pressed
       blnKey2 = true;break;
    case '3': //Key3 has been pressed
       blnKey3 = true;break;
    case '4': //Key4 has been pressed
       blnKey4 = true;break;
    case '5': //Key5 has been pressed
       blnKey5 = true;break;
    case '6': //Key6 has been pressed
       blnKey6 = true;break;
    case '7': //Key7 has been pressed
       blnKey7 = true;break;
    case '8': //Key8 has been pressed
       blnKey8 = true;break;
    case '9': //Key9 has been pressed
       blnKey9 = true;break;
    case 'R': //KeyR has been pressed
       blnKeyR = true;break;
    case 'A': //KeyA has been pressed
       blnKeyA = true;break;
    case 'B': //KeyB has been pressed
       blnKeyB = true;break;
    case 'C': //KeyC has been pressed
       blnKeyC = true;break;
    case 'D': //KeyD has been pressed
       blnKeyD = true;break;
    case 'E': //KeyE has been pressed
       blnKeyE = true;break;
    case 'F': //KeyF has been pressed
       blnKeyF = true;break;                                                                                                                                      
  }   
}

//**********************************  
//**********************************  
//ESSENTIAL ROUTINES
//**********************************  
//**********************************
#ifdef SD_ON
void FileExists()
{
    if (SD.exists(FileN)) {
      SD.remove(FileN);
  } 
}


void WriteSetupLog()
{
  File LogFile; 
  LogFile = SD.open(FileN, FILE_WRITE);

  // if the file is available, write to it:
  if (LogFile) {
    LogFile.println(SetupdataString);
    LogFile.close();
    #ifdef DEBUG
    Serial.println(SetupdataString);
    #endif
    SetupdataString = ""; //Clear used variable
  }
  // if the file isn't open, pop up an error:
  else {
    #ifdef DEBUG
    Serial.println(F("error opening setup.txt"));
    #endif
  }
}

void WriteLog(char *FN,double Val)
{
  File LogFile; 
    #ifdef SD_FILE_ONE_ON
      LogFile = SD.open(FN, FILE_WRITE);
    #endif
  if (LogFile) {
    LogFile.println(String(Val));
    LogFile.close();
  }  
}

void WriteLoopLog(int FileCounter)
{
    if (blnTest==false)
    {
      if(blStartRec==true)
      {
        //2021-09-27T23:06:21
      DateTime MyDateAndTime;
      MyDateAndTime = Clock.read();
      File LogFile; 
        #ifdef SD_FILE_ONE_ON
          LogFile = SD.open(FileL, FILE_WRITE);
        #else
          LogFile = SD.open(Pre + String(FileCounter) + Ext, FILE_WRITE);
        #endif
      if (LogFile) {
        Clock.printTo(LogFile); 
        LogFile.print(",");   
        //LogFile.print(Clock.read()); //String(MyDateAndTime.Year) + "-" + String(MyDateAndTime.Mo nth) + "-" + String(MyDateAndTime.Day) + " " + String(MyDateAndTime.Hour) + ":" + String(MyDateAndTime.Minute) + ":" + String(MyDateAndTime.Second)
        LogFile.println(LoopdataString);
        intSpdCnt += 1;
        //delay(2);
        LogFile.close();
        
       
      }
    }
  }  
}

void WriteKeyPress()
{
  if((key == '1') || (key =='2') || (key =='3') || (key =='4') || (key =='5') || (key =='6') || (key =='7') || (key =='8') || (key =='9') || (key =='A') || (key =='B') || (key =='C') || (key =='D') || (key =='E') || (key =='F'))
  {
    File LogFile; 
      #ifdef SD_FILE_ONE_ON
        LogFile = SD.open(FileK, FILE_WRITE);
      #endif
    if (LogFile) {
      LogFile.println(key);
      LogFile.close();
      #ifdef DEBUG
      Serial.println(key);
      #endif
    }  
  }
}

void WriteRadioFound()
{
  File LogFile; 
    #ifdef SD_FILE_ONE_ON
      LogFile = SD.open(FileR, FILE_WRITE);
    #endif
  if (LogFile) {
    LogFile.println(FileRF);
    LogFile.close();
    #ifdef DEBUG
    Serial.println(key);
    #endif
  }  
}
#endif

void DoCalibrate()
{
    unsigned long time;
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  if(blnCalibrate == false)
  {
    int calibration = gyro+accel+mag;
    while(calibration!=9)
    {
      time = millis();
      bno.getCalibration(&system, &gyro, &accel, &mag);
      delay(1);
      calibration = gyro+accel+mag;
      delay(1);
      //
      if (gyro==3){digitalWrite(3, HIGH);} //YELLOW
      if (accel==3){digitalWrite(8, HIGH);} //RED
      if (mag==3){digitalWrite(9, HIGH);} //GREEN
      while (millis() < time+1000);
    }
    blnCalibrate = true;
    digitalWrite(3, LOW);
    digitalWrite(8, LOW);
    delay(2000);                       
  }
  else
  {
    if (mag==3)
    {
      digitalWrite(9, HIGH); //GREEN
    } 
    else
    {
      digitalWrite(9, LOW); //GREEN
    }
    delay(1);
  }
}
void ResetRoutine()
{ 
  #ifdef DEBUG
  Serial.println(F("Key Reset pressed"));
  #endif

  //LEDS
  digitalWrite(3, LOW);
  digitalWrite(8, LOW);
  //digitalWrite(9, LOW); //Disabled so that it does not affect the magneto meter reading.
  
  //Reset all other keys bools
  blnKey1 = false;blnKey2 = false;blnKey3 = false;blnKey4 = false;
  blnKey5 = false;blnKey6 = false;blnKey7 = false;blnKey8 = false;
  blnKey9 = false;blnKeyA = false;blnKeyB = false;blnKeyC = false;
  blnKeyD = false;blnKeyE = false;blnKeyF = false;blnKeyR = false;
  
  //Stop any motors running
  pwm.setPWM(0,0,2700); //12
  pwm.setPWM(2,0,2700); //81 ROT N
  pwm.setPWM(3,0,2700); //51 ROT
  
  pwm.setPWM(5,0,2700); //21

  pwm.setPWM(8,0,2700);  //32
  pwm.setPWM(10,0,2700); //63 ROT N
  pwm.setPWM(11,0,2700); //73 ROT
  
  pwm.setPWM(13,0,2700); //41 

  //Ensure the next file created will be unique until bot drv switched off
  FileCounter = FileCounter+1;
  //sleep_mode();
  
}

//Guidance V7 Anti-ClockWise      HighVal = 1697;LowVal = 1337;
void GuidanceM2(double ixi,int TurnSpeed,double xmi)
{
  int xi;

  int MinDiff = 108; //135 75% 108 70%  216 40%  189 35%  162 30%
  int MaxDiff = 162; //162 70% 135 75%  162 70%
 
  
  //Motor Variable strength values
  int L0V; //Linear 0 Value
  int L8V; //Linear 8 Value
  int L5V; //Linear 5 Value
  int L13V; //Linear 5 Value
  int R3V; //Rotation High Value
  int R2V; //Rotation Low Value
  int LValConst = 10;  //Since highval motors start always, lowval doesnt trigger sometimes. So we increase it with constant

  bool blnswitcher=false;
  
  //float Const_350_360 = 1.028571429;  //360/350
  // float Const_350_360 = 1.058823529; //360/340
  //// float Const_350_360 = 1.125; //360/340
  //int Const_Val = 10;
  //int Const_Val = 20;
  int Const_Val = 40;
  
  //Heading threshold
  //int heading=122; //Heading is based on north; This value can later be input as parameter to the routine

  //Motor Rotation Range values
  
  //POSSIBLE NEW VALUES
  /*
  int RRFHV = 225; //Rotation Range From High Value Motor 3 & 11
  int RRTHV = 315; //Rotation Range To High Value Motor 3 & 11
  int RRFLV = 45;  //Rotation Range From Low Value Motor 3 & 11
  int RRTLV = 225; //Rotation Range To Low Value Motor 3 & 11
  int RRFMV = 315; //Rotation Range From Mid Value Motor 3 & 11
  int RRTMV = 45;  //Rotation Range To Mid Value Motor 3 & 11
  */
  

  
  float ROTSpeedOne=1.333; //68.5%  60   2022-02-14 adjusted and simplyfied
  float ROTSpeedTwo=1.43; //67%  60   2022-02-14 adjusted and simplyfied
  float ROTSpeedThree=1.38; //67%  60   2022-02-14 adjusted and simplyfied
  
  //float LINSpeedControl=1.43; //70%  80
  // 1.11 90%
  // 1.25 80%
  // 1.333 75%
  // 1.38 72.5%
  // 1.43 70%
  // 1.44927 69%
  // 1.45985 68.5%
  // 1.47058 68%
  // 1.49925 67%
  // 1.49 66.7%  
  // 1.50 66.6%
  // 1.52 66%
  // 1.54 65%
  // 1.61 62%
  // 1.67 60%
  // 2    50%
  // 2.5  40%
  // 3.33 30%
  // 4.00 25%
  // 5.00 20%
  // 6.67% 15%
  // 10   10%
  //2021-01-15 ROT 1.43 LIN 1.25 - Pull to the right
  //2021-01-16 ROT 2    LIN 1.25
  
  xi = (int)ixi;
  if (xm==0)
  xm = (int)xmi;
  else
  xm = (xm+(int)xmi)/2; //average over the last 2 values
  
//kick start
  if (blnInitStart==false) //Is to get going when starting
  {
    blnInitStart=true;
    pwm.setPWM(3,0,2700); 
    pwm.setPWM(11,0,2700); 
    pwm.setPWM(0,0,2700); //12
    pwm.setPWM(13,0,2700); //41   
    delay(5);
    pwm.setPWM(3,0,3100); 
    pwm.setPWM(11,0,2850); 
    pwm.setPWM(0,0,3100); //12
    pwm.setPWM(13,0,3100); //41     
    delay(300); 
    pwm.setPWM(3,0,2700); 
    pwm.setPWM(11,0,2700); 
    pwm.setPWM(0,0,2700); //12
    pwm.setPWM(13,0,2700); //41       
  }
//Linear 
//Notes NB - remember motor 0 & 13 (outside propulsion) have three if statements due to crossing 0 / 360 and the other 2 motors not.
//         - Rotation motor 3 next motor 0
//         - remember, there is a 45degree constant that the linear motors have to be adjusted with when setting these values 
//           due to the orientation of the bno055 sensor, that xi being 315 actually result in the linear motor at 270
//   
Serial.print("Degree ");   
Serial.print(xi);
Serial.print(",");
//
//MOTOR 0   12
  if (xi >=315 && xi < 360) //1st 90 degrees with 1st high to low motor speed
      {
        L0V = (int)round((LowVal+MinDiff)+((6-(0.022222222*MinDiff))*(xi-315)));
        //L0V = (LowVal+Const_Val)+((((xi-315)*4)/Const_350_360)/round(LINSpeedControl));
        pwm.setPWM(0,0,L0V);
        #ifdef DEBUGG
        Serial.print("M0 12 ");   
        Serial.print(L0V);
        Serial.print(",");
        #endif
      }   
  else if (xi >=0 && xi < 45)
      {
        L0V = (int)round((LowVal+MinDiff)+((6-(0.022222222*MinDiff))*(45+xi)));
        //L0V = (LowVal+Const_Val)+((((45+xi)*4)/Const_350_360)/round(LINSpeedControl)); //45 constant is to take into account the 45 degrees that has happend already, so that the speed of the motor is correctly adj.
        pwm.setPWM(0,0,L0V);
        #ifdef DEBUGG
        Serial.print("M0 12 ");   
        Serial.print(L0V);
        Serial.print(",");
        #endif
      }       
  else if (xi >=45 && xi < 135) //2nd 90 degrees with 2nd high to low motor speed
      {
        L0V=(int)round((HighVal-MaxDiff)+(((0.022222222*MaxDiff)-6)*(xi-45)));
        //L0V = HighVal-(((xi-45)*4)/round(LINSpeedControl));
        pwm.setPWM(0,0,L0V);
        #ifdef DEBUGG
        Serial.print("M0 12 ");   
        Serial.print(L0V);
        Serial.print(",");
        #endif
      }
  else
      {
        pwm.setPWM(0,0,2700);//Switch-off 
        #ifdef DEBUGG
        Serial.print("M0 12 ");   
        Serial.print(L0V);
        Serial.print(",");
        #endif
      }         

////MOTOR 13   41
  if (xi >=315 && xi < 360)  //1st 90 degrees with 1st high to low motor speed
      {
        L13V=(int)round((HighVal-MaxDiff)+(((0.022222222*MaxDiff)-6)*(xi-315)));
        //L13V = HighVal-(((xi-315)*4)/round(LINSpeedControl));
        pwm.setPWM(13,0,L13V);
        #ifdef DEBUGG
        Serial.print("M13 41 ");   
        Serial.print(L13V);
        Serial.print(","); 
        #endif       
      }   
  else if (xi >=0 && xi < 45)
      {
        L13V=(int)round((HighVal-MaxDiff)+(((0.022222222*MaxDiff)-6)*(45+xi)));
        //L13V = HighVal-(((45+xi)*4)/round(LINSpeedControl)); //45 constant is to take into account the 45 degrees that has happend already, so that the speed of the motor is correctly adj.
        pwm.setPWM(13,0,L13V);
        #ifdef DEBUGG
        Serial.print("M13 41 ");;   
        Serial.print(L13V);
        Serial.print(",");
        #endif           
      }     
  else if (xi >=225 && xi < 315) //2nd 90 degrees with 2nd high to low motor speed
      {
        L13V = (int)round((LowVal+MinDiff)+((6-(0.022222222*MinDiff))*(xi-225)));
        //L13V = (LowVal+Const_Val)+((((xi-225)*4)/Const_350_360)/round(LINSpeedControl));
        pwm.setPWM(13,0,L13V);
        #ifdef DEBUGG
        Serial.print("M13 41 ");   
        Serial.print(L13V);
        Serial.print(","); 
        #endif       
      }         
  else
      {
        pwm.setPWM(13,0,2700);//Switch-off 
        #ifdef DEBUGG
        Serial.print("M13 41 ");  
        Serial.print(L13V);
        Serial.print(",");
        #endif        
      }  



//MOTOR 8       32

  if (xi >=135 && xi < 225)
      {
        L8V = (int)round((LowVal+MinDiff)+((6-(0.022222222*MinDiff))*(xi-135)));
        //L8V = (LowVal+Const_Val)+((((xi-135)*4)/Const_350_360)/round(LINSpeedControl));
        pwm.setPWM(8,0,L8V);
        #ifdef DEBUGG
        Serial.print("M8 32 ");   
        Serial.print(L8V);
        Serial.print(","); 
        #endif       
      }         
  else if (xi >=225 && xi < 315)
      {
        L8V=(int)round((HighVal-MaxDiff)+(((0.022222222*MaxDiff)-6)*(xi-225)));
        //L8V = HighVal-(((xi-225)*4)/round(LINSpeedControl));
        pwm.setPWM(8,0,L8V);
        #ifdef DEBUGG
        Serial.print("M8 32 ");   
        Serial.print(L8V);
        Serial.print(",");
        #endif         
      }
  else
      {
        pwm.setPWM(8,0,2700);//Switch-off 
        #ifdef DEBUGG
        Serial.print("M8 32 "); 
        Serial.print(L8V);
        Serial.print(",");
        #endif         
      }

  
//MOTOR 5  21

  if (xi >=45 && xi < 135)
      {
        L5V = (int)round((LowVal+MinDiff)+((6-(0.022222222*MinDiff))*(xi-45)));
        //L5V = (LowVal+Const_Val)+((((xi-45)*4)/Const_350_360)/round(LINSpeedControl));
        pwm.setPWM(5,0,L5V);
        #ifdef DEBUGG
        Serial.print("M5 21 ");   
        Serial.print(L5V);
        Serial.print(",");
        #endif         
      }    
  else if (xi >=135 && xi < 225)
      {
        L5V=(int)round((HighVal-MaxDiff)+(((0.022222222*MaxDiff)-6)*(xi-135)));
        //L5V = HighVal-(((xi-135)*4)/round(LINSpeedControl));
        pwm.setPWM(5,0,L5V);
        #ifdef DEBUGG
        Serial.print("M5 21 ");   
        Serial.print(L5V);
        Serial.print(",");
        #endif         
      }    
  else
      {
        pwm.setPWM(5,0,2700);//Switch-off 
        #ifdef DEBUGG
        Serial.print("M5 21 ");   
        Serial.print(L5V);
        Serial.print(",");
        #endif         
      }

//ROTATE 
  //HighVal = 3310;
  //LowVal  = 2770;

//this happen every one rotation  
if (safetyclock >= 320) 
{
    if (blnFirstRun==false) //Startup Sequence
    {
    #ifdef DEBUGG
    Serial.print("blnFirstRunF");   
    #endif
    if ((xi>=110) && (xi<180))
      {
        intSpdCnt=0;
        blnSlower=true;
        #ifdef DEBUGG
          Serial.print("blnFirstRunT<20");   
        #endif        
      } 
    else if ((xi>=180) && (xi<240))
      {
        intSpdCnt=0;
        blnSlower=false;
        #ifdef DEBUGG
          Serial.print("blnFirstRunT<20");   
        #endif        
      } 
    else if ((xi>=240) && (xi<300))
      {
        intSpdCnt=0;
        blnSlower=true;
        #ifdef DEBUGG
          Serial.print("blnFirstRunT<20");   
        #endif        
      }   
    else if (xi>=300)
      {
        intSpdCnt=0;
        blnSlower=false;
        blnFirstRun=true;  
        #ifdef DEBUGG
          Serial.print("blnFirstRunT<20");   
        #endif        
      }         
    }
    else
    {
      //this happen every 120 degrees turn in rotations  
        #ifdef DEBUGG
        Serial.print("102 started");   
        #endif    
      if ((xi>=0) && (xi<120))
        {
          if(intSpdCnt>=9)
          {
            intSpdCnt=0;
            blnSlower=false;
          }
          else if (intSpdCnt<9)
          {
            intSpdCnt=0;
            blnSlower=true;
          }
        }
      else if ((xi>=120) && (xi<240))
        {
          if(intSpdCnt>=9)
          {
            intSpdCnt=0;
            blnSlower=false;
          }
          else if (intSpdCnt<9)
          {
            intSpdCnt=0;
            blnSlower=true;
          }
        }
      else if ((xi>=240) && (xi<359))
        {
          if(intSpdCnt>=9)
          {
            intSpdCnt=0;
            blnSlower=false;
          }
          else if (intSpdCnt<9)
          {
            intSpdCnt=0;
            blnSlower=true;
          }
        }
    }
 }


if (blnSlower==false)
{
    //If stable speed was needed consitantly 
            R3V = 2770+round(((3310-2770)/ROTSpeedOne));
    //MOTOR 3
            pwm.setPWM(3,0,R3V); 
    //MOTOR 11
            pwm.setPWM(11,0,R3V);       
            #ifdef DEBUGG
            Serial.print("blnSlowF");   
            #endif           
            R2V = 2770+round(((3310-2770)/ROTSpeedTwo));
    //newly added
    //MOTOR 2
            pwm.setPWM(2,0,R2V); 
    //MOTOR 10
            pwm.setPWM(10,0,R2V);           
}
else
{

    //If stable speed was needed consitantly 
            R3V = 2770+round(((3310-2770)/ROTSpeedTwo));
    //MOTOR 3
            pwm.setPWM(3,0,R3V); 
    //MOTOR 11
            pwm.setPWM(11,0,R3V);       
            Serial.print("blnSlowT");      
            R2V = 2770+round(((3310-2770)/ROTSpeedThree));
    //newly added
    //MOTOR 2
            pwm.setPWM(2,0,R2V); 
    //MOTOR 10
            pwm.setPWM(10,0,R2V); 
            #ifdef DEBUGG
            Serial.print("MR 2 10 ");   
            Serial.print(R2V);
            Serial.print(",");
            #endif              
}
#ifdef DEBUGG
 Serial.println("");
#endif             
}


//2022-01-29 Only retrief orientation data
void readBNO()
{
  sensors_event_t event; //declare event
  LoopdataString = ""; //Clear used variable
  
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER); //SENSOR_TYPE_ORIENTATION
  printEvent(&event);
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE); //SENSOR_TYPE_ROTATION_VECTOR
  printEvent(&event);
  //bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL); //SENSOR_TYPE_LINEAR_ACCELERATION
  //printEvent(&event);
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_MAGNETOMETER); //SENSOR_TYPE_MAGNETIC_FIELD
  printEvent(&event);
  //bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER); //SENSOR_TYPE_ACCELEROMETER
  // printEvent(&event);

  digitalWrite(SD_Select, LOW);
    //delay(2);          
  #ifdef SD_ON
    WriteLoopLog(FileCounter);            
  #endif   
  digitalWrite(SD_Select, HIGH); 
}

void printEvent(sensors_event_t* event) {
  double x = -111, y = -111 , z = -111; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    //Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    LoopdataString += String(x) + *co + String(y) + *co + String(z) + *co;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    //Serial.print("Orient:");  
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    LoopdataString += String(x) + *co + String(y) + *co + String(z) + *co; 
    xrot = x;   
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    //Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
    LoopdataString += String(x) + *co + String(y) + *co + String(z) + *co; 
    xmag = x;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    //Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    LoopdataString += String(x) + *co + String(y) + *co + String(z) + *co; 
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    //Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    LoopdataString += String(x) + *co + String(y) + *co + String(z) + *co;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    //Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    LoopdataString += String(x) + *co + String(y) + *co + String(z) + *co;
  }
}
/*
void BasicMotorSpinTest()
{
  //Serial.println("0");  
  delay(100);
  pwm.setPWM(0,0,2770);   //Unit 1 Outside - Forward 12
  delay(100);
  pwm.setPWM(0,0,2700);   //Unit 1 Outside - Forward 12
  delay(100);
  pwm.setPWM(0,0,2770);   //Unit 1 Outside - Forward 12
  delay(100);
  pwm.setPWM(0,0,2700);   //Unit 1 Outside - Forward 12
  delay(100);
  //Serial.println("2");
  pwm.setPWM(2,0,2770);   //Unit 8 next to Unit 1 - Rotating -New 81
  delay(100);
  pwm.setPWM(2,0,2700);   //Unit 8 next to Unit 1 - Rotating -New 81
  delay(100);
  pwm.setPWM(2,0,2770);   //Unit 8 next to Unit 1 - Rotating -New 81
  delay(100);
  pwm.setPWM(2,0,2700);   //Unit 8 next to Unit 1 - Rotating -New 81
  delay(100);
  //Serial.println("3");
  pwm.setPWM(3,0,2770);   //Unit 5 next to Unit 1 - Rotating 51
  delay(100);
  pwm.setPWM(3,0,2700);   //Unit 5 next to Unit 1 - Rotating 51
  delay(100);
  pwm.setPWM(3,0,2770);   //Unit 5 next to Unit 1 - Rotating 51
  delay(100);
  pwm.setPWM(3,0,2700);   //Unit 5 next to Unit 1 - Rotating 51
  delay(100);
  //Serial.println("5");
  pwm.setPWM(5,0,2770);   //Unit 2 Inside - Forward 21
  delay(100);
  pwm.setPWM(5,0,2700);   //Unit 2 Inside - Forward 21
  delay(100);
  pwm.setPWM(5,0,2770);   //Unit 2 Inside - Forward 21
  delay(100);
  pwm.setPWM(5,0,2700);   //Unit 2 Inside - Forward 21
  delay(100);
  pwm.setPWM(8,0,2770);   //Unit 3  Outside - Forward 32
  delay(100);
  pwm.setPWM(8,0,2700);   //Unit 3  Outside - Forward 32
  delay(100);
  pwm.setPWM(8,0,2770);   //Unit 3  Outside - Forward 32
  delay(100);
  pwm.setPWM(8,0,2700);   //Unit 3  Outside - Forward 32
  delay(100);
  //Serial.println("10");
  pwm.setPWM(10,0,2770);   //Unit 6 next to Unit 3 - Rotating 63
  delay(100);
  pwm.setPWM(10,0,2700);   //Unit 6 next to Unit 3 - Rotating 63
  delay(100);
  pwm.setPWM(10,0,2770);   //Unit 6 next to Unit 3 - Rotating 63
  delay(100);
  pwm.setPWM(10,0,2700);   //Unit 6 next to Unit 3 - Rotating 63
  delay(100);
  //Serial.println("11");
  pwm.setPWM(11,0,2770);   //Unit 7 next to Unit 3 - Rotation 73
  delay(100);
  pwm.setPWM(11,0,2700);   //Unit 7 next to Unit 3 - Rotation 73
  delay(100);
  pwm.setPWM(11,0,2770);   //Unit 7 next to Unit 3 - Rotation 73
  delay(100);
  pwm.setPWM(11,0,2700);   //Unit 7 next to Unit 3 - Rotation 73
  delay(100);
  //Serial.println("13");
  pwm.setPWM(13,0,2770);   //Unit 4 Inside - Forward 41
  delay(100);
  pwm.setPWM(13,0,2700);   //Unit 4 Inside - Forward 41
  delay(100);
  pwm.setPWM(13,0,2770);   //Unit 4 Inside - Forward 41
  delay(100);
  pwm.setPWM(13,0,2700);   //Unit 4 Inside - Forward 41
  delay(100);
}
*/






