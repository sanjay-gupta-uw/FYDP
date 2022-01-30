// Version 1.0.0 ALPHA - May 15 2020 - UniMatrix Technologies Coil Winder - rrichter@unimatrixtech.com. 
// Please keep in mind that I'm quite busy and may take time before I answer my emails.
// Multi Size Wire Coil Winder
// NOT TESTED ON ACTUAL MACHINE - JUST THE MOTORS HAVE BEEN TESTED ********************************************
// This work is licensed under a Creative Commons Attribution-ShareAlike 3.0 Unported License.

// This is the Arduino side of the Coil Winder Project written by Ray Richter of Unimatrix Technologies
// The Widows side was written also by Ray Richter in C#.
// This Arduino code is ment to be used with 2 TB6600 stepper controlers and 2 Nema 17 stepper motors.
// The winder motor is a 1:1 ratio but can have up to 6400 steps per revolution or 0.05625 degrees of rotation per pulse.
// The carriage motor drives a 10/32(TPI) threaded rod and at 6400 pulses per rev gives a resolution of 0.0000048828".
// at 200 pulses per rev (the lowest step count) it gives a resolution of 0.00015625".

//If using metric - example:
//Threaded rod's pitch = 2 mm. This means that one revolution will move the nut 2 mm.
//Default stepping = 400 step/revolution.
// 400 step = 1 revolution = 8 mm linear motion. (4 start 2 mm pitch screw)
// 1 cm = 10 mm =>> 10/8 * 400 = 4000/8 = 500 steps are needed to move the nut by 1 cm.

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <stdio.h> // for function sprintf
#include <EEPROM.h>

// Size of the incoming serial char array **** NOT OPTIMIZED ****
const byte    numChars = 100;

// temporary array for use by strtok() function
char tempChars[numChars];        

// variables to hold the parsed data
char receivedCommand[numChars] = {0};

// SET STEPPER TYPE and pins
AccelStepper winder(AccelStepper::DRIVER, 4, 5);;// pulses Digital D4 (CLK), direction Digital D5 (DIR)  1 = AccelStepper::DRIVER 2 pin setup for the TB6600 and similar
AccelStepper carriage(AccelStepper::DRIVER, 8, 9);; // pulses Digital D8 (CLK), direction Digital D9 (DIR) 1 = AccelStepper::DRIVER 2 pin setup for the TB6600 and similar

//User-defined values
// Variables used gobaly
bool          BOLcarLeft = false;         // Allow deny movig the carriage left
bool          BOLcarRight = false;        // Allow deny movig the carriage right
unsigned long carMax;                     // SETS Max carriage steps to the right
short         carMin = 0;                 // SETS Max carriage steps to the left
long          carTempPos = 1;             // Temp variable for the carriage position
bool          Coil_Build = false;         // Allow/deny coil building
unsigned long curPosition_C;              // The current carriage position
unsigned long curPosition_W = 0;          // The current winder position
unsigned long Home_Position = 0;          // The current carriage position BEFORE SETTING the HOME position
int           inTempPos = 0;              // Temp variable for the carriage position
unsigned int  LayerCount = 0;             // Count of the number of layers made
unsigned long limSwRight_Pos = 0;         // Limit Switch Right Steps Position
unsigned long limSwLeft_Pos = 0;          // Limit Switch Left Steps Position
int           limSwtLeft = 11;            // Pin number of the Left Limit Switch
int           limSwtRight = 12;           // Pin number of the Right Limit Switch
bool          newData = false;            // Whether or not there is new data from serial
long          PulsesPerLayer_Winder = 0;  // Pulses/Steps Per Layer Winder
char          receivedChars[numChars];    // The received Characters Array
long          receivedSteps = 0;          // Number of steps
long          receivedSpeed = 0;          // Speed or Steps/second
long          receivedAcceleration = 0;   // Accel Rate Steps/second^2
bool          runallowed = false;         // runallowed flag
long          StepCount = 0;              // Count Winder steps for the ratio
int           winder_direction = 1;       // (= 1: positive direction), (= -1: negative direction) used with radBtnDir
unsigned int  WindCount = 0;              // Used for the Windings per layer counter
long          windings_x = 0;             // The number of total winder steps for a given coil
long          WindCountTotal = 0;         // Keeps track of the total windings made


//SETTINGS - These are sent once to set the coil specs ------------------------------------------------------------------------------------------------
//String unicodeString = StartS + Sts + "," + strPPR_WindingM + "," + dblPPR_C + "," + PPWind_C + "," + WPLayer + "," + STEP_RATIO + "," + PPLayer_C + "," + Speed + "," + Accel + "," + radBtnDir + "," + PPI + "," + EndS;
// or                     <   S or C or t  ,      800            ,    800         ,      143       ,       218     ,       6          ,     31174       ,     650     ,     400     ,    CW or CCW    ,    6400   ,     >
// In order of transmission.
String  inType = "";       // Type of input
int     PPR_WindingM = 0;  // Winder Pulses per rev
int     PPR_C = 0;         // Carriage Pulses Per rev
float   PPWind_C = 0;      // Carriage Pulses Per Winding
float   WPLayer = 0;       // WINDINGS PER LAYER
float   STEP_RATIO = 0;    // Winder to Carriage stepper ratio
float   PPLayer_C = 0;     // Carriage steps per layer
int     Speed = 0;         // Speed Motors
int     Accel = 0;         // Acceleration
String  radBtnDir = "";    // Direction
float   PPI;               //PULSEs PER INCH
long    NWnum = 0;         //Number of windings
int     CalLayers = 0;     //Number of layers

//-------------------------------------------------------------------------
//COMMANDS - These can be sent anytime INDIVIDUALLY SENT!
//Winder Motor Enable, Carriage Motor  Enable, Left, Right, Pause, Continue, Wind_1_rev, Set_home, Start, Stop, Terminate/Clear, Status of positions
// Inorder of Received
int Lft;    // Jog/move carriage Left until pause, stop, limit switch, or set home
int Rit;    // Jog/move carriage Rightt until pause, stop, limit switch, or set home
int Pas;    // Pause
int Con;    // Continue
int W1r;    // Wind_1_revolution
int Sth;    // Set_home position, start of coil
int Str;    // Start the build
int Stp;    // Stop
int Clr;    // Terminate/Clear zero-out everything but carriage position
int Sts;    // Status of positions

//-------------------------------------------------------------------------------------
// STATUS / INDIACATORS - Sent when needed back to the computer(PC)
//indOUT, WME, CME, LED_left, LED_right, LED_paused, LED_H_set, W_PULSE, C_PULSE, WOWINDING, WOLAYER
//eg. I,100111,######,###  this what is sent to PC
// In order of transmission.
String indOUT = "I";      // Indiacator data sent to the PC
String STSout = "S";      // Status data sent to the PC 
String WME = "0";         // Winder Motor Enable
String CME = "0";         // Carriage Motor Enable
String LED_left = "0";    // The LED indiacators are either a "0" or a "1" and sent to the PC
String LED_right = "0";   //
String LED_paused = "0";  //
String LED_H_set = "0";   // Set home LED, sent to the PC
String W_PULSE = "0";     // Winder pulse count
String C_PULSE = "0";     // Carriage pulse count
char   WOWINDING[10];     // Character array for Workin On Winding count
char   WOLAYER[5];        // Character array for Workin On Layer count

//========================================================================================================================
void setup()
{
// EEPROM Locations and values:
// '0' position of Winder
// '4' position of Carriage
// '8' calculated value of carMax
// '12' Home_Position or start of the bobbin
// '16' limSwRight_Pos
// '20' limSwLeft_Pos
// '24'  NWnum - Number of total windings

// initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);             // Just for fun
  pinMode(limSwtLeft, INPUT_PULLUP);        // PIN D11 USING INTERNAL PULL-UPS
  pinMode(limSwtRight, INPUT_PULLUP);       // PIN D12 USING INTERNAL PULL-UPS
  
  winder.setEnablePin(6);                   // D6 ENABLE WINDER
  carriage.setEnablePin(10);                // D10 ENABLE CARRIAGE
  
  winder.setMaxSpeed(5000);                 //SPEED = Steps / second ***DEFAULT*** JUST ENCASE
  winder.setAcceleration(650);              //ACCELERATION = Steps /(second)^2 ***DEFAULT*** JUST ENCASE
  winder.disableOutputs();                  //disable outputs

  carriage.setMaxSpeed(5000);               //SPEED = Steps / second ***DEFAULT*** JUST ENCASE
  carriage.setAcceleration(650);            //ACCELERATION = Steps /(second)^2 ***DEFAULT*** JUST ENCASE
  carriage.disableOutputs();                //disable outputs
  unsigned int   minWidth = 20;
  winder.setMinPulseWidth(minWidth);        // self explanitory
  carriage.setMinPulseWidth(minWidth);      // self explanitory
  EEPROM.get(0, curPosition_W);             // Get stored winder Position
  winder.setCurrentPosition(curPosition_W); // Set winder Position
  delay(20);                                // Give EEPROM time to do it's job
  EEPROM.get(4, curPosition_C);               // Get stored carriage Position
  carriage.setCurrentPosition(curPosition_C); // Set carriage Position
  delay(20);
  EEPROM.get(8, carMax);                      // Get stored carMax Position
  delay(20);
  EEPROM.get(12, Home_Position);
  delay(20);
  EEPROM.get(16, limSwRight_Pos);
  delay(20);
  EEPROM.get(20, limSwLeft_Pos);
  delay(20);
limSwRight_Pos = 179200;  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  TO BE REMOVED ONCE THE WINDER STAND IS BUILT
limSwLeft_Pos = 0;  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  TO BE REMOVED ONCE THE WINDER STAND IS BUILT
  EEPROM.get(24, NWnum);
  delay(20);
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
   
  }
}

//  MAIN PROGRAM LOOP =======================================================================================================
void loop() 
{
// Check for new serial coms
    checkSerial();

// this temporary copy is necessary to protect the original data
// because strtok() replaces the commas with \0    
    if (newData == true)
    {
        strcpy(tempChars, receivedChars);
        parseData();
        newData = false;
    }
//function to handle the motor    
    RunTheMotor();
}

//CHECKS to see if anything is in the receive buffer  =======================================================================================================
void checkSerial() 
  {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

// While the serial port is available and no work is being done
    while (Serial.available() > 0 && newData == false) // 
    {
// Read the serial buffer
        rc = Serial.read();
        if (recvInProgress == true) 
        {
// Not at the end of the message
            if (rc != endMarker) 
            {
// Store characters in the array
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) 
                {
                    ndx = numChars - 1;
                }
            }
            else 
            {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) 
        {
            recvInProgress = true;
        }
    }
}

//PARSE RECEIVED DATA BY GROUP  ====================================================================================================
void parseData() 
{
// split the data into its parts
    
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    strcpy(receivedCommand, strtokIndx); // copy it to message
    inType = receivedCommand;         //char to string
       
    if(inType == "S")   // SETTINGS GROUP
    {
     Parse_Type_S();    // Go to the parse settings function
     newData = false;   // Work is done
     return;            // Go back to the calling function
    }

    if(inType == "C")   // COMMANDS GROUP
    {
      Parse_Type_C();   // Go to the parse commands function
      newData = false;  // Work is done
      return;           // Go back to the calling function
    }
    
   if(inType == "t")    // COM TEST reply
    {      
      Serial.println("CONNECTION IS GOOD");
      newData = false;
      return;
    }
}

// splits the SETTINGS into its parts ===============================================================================================================================
void Parse_Type_S()
{
// this is used by strtok() as an index  
    char * strtokIndx; 
      strtokIndx = strtok(NULL, ",");   // this continues where the previous call left off

// Winder Pulses per rev      
      PPR_WindingM = atoi(strtokIndx);  // atoi convert this part to an integer

// Carriage Pulses Per rev
      strtokIndx = strtok(NULL, ",");
      PPR_C = atoi(strtokIndx);         // atoi convert this part to an integer

// Carriage Pulses Per Winding
      strtokIndx = strtok(NULL, ",");
      PPWind_C = atof(strtokIndx);     // atof convert this part to a float

// WINDINGS PER LAYER
      strtokIndx = strtok(NULL, ",");
      WPLayer = atof(strtokIndx);      // atof convert this part to a float

// WINDER TO CARRIAGE RATIO
      strtokIndx = strtok(NULL, ",");
      STEP_RATIO = atof(strtokIndx);   // atof convert this part to a float

// Carriage steps(Pulses) per layer
      strtokIndx = strtok(NULL, ",");
      PPLayer_C = atof(strtokIndx);   // atof convert this part to a float
      
//Motor speed
      strtokIndx = strtok(NULL, ",");
      Speed = atoi(strtokIndx);         // atoi convert this part to an integer
      winder.setMaxSpeed(Speed);        //set speed
      carriage.setMaxSpeed(Speed);      //set speed
      
//Motor Acceleration Rate
      strtokIndx = strtok(NULL, ",");
      Accel = atoi(strtokIndx);         // atoi convert this part to an integer
      winder.setAcceleration(Accel);    //update the value of the variable
      carriage.setAcceleration(Accel);  //update the value of the variable
      
//Direction Control for the coil Winder
      strtokIndx = strtok(NULL, ",");
      radBtnDir = (strtokIndx);         // convert this part to a string
      if (radBtnDir == "0")
      {
        winder_direction = -1;          // Move winder CCW
      }
      if (radBtnDir == "1") 
      {
        winder_direction = 1;           // Move winder CW
      }

// Carriage Pulse Per Inch - USED to set the max width encase of limit switch failure >> Hopefully
      strtokIndx = strtok(NULL, ",");
      PPI = atof(strtokIndx);
      carMax = 6 * PPI;                 // Set carMax for a 7" inch long winder shaft or the useful work area, this allows for 1" space for the limit switches
      EEPROM.put(8, carMax);            // Put or store the carMax Position
      delay(20);    
      newData = false;

// Number of WINDINGS
      strtokIndx = strtok(NULL, ",");
      NWnum = atol(strtokIndx);         // atol convert this part to a long
      EEPROM.put(24, NWnum);            // Put or store the Total Number of Windings (NWnum)
      delay(20);
      newData = false;

// Number of LAYERS
      strtokIndx = strtok(NULL, ",");
      CalLayers = atoi(strtokIndx);     // atoi convert this part to an integer
      EEPROM.put(28, CalLayers);        // Put or store the Total number of Calculated Layers (CalLayers)
      delay(20);
      newData = false;
      return;
}
// PARSE THE RECEIVED COMMANDS  ===================================================================================================================================
void Parse_Type_C()
{
// split the COMMANDS into its parts
      char * strtokIndx;                              // this is used by strtok() as an index
      strtokIndx = strtok(NULL, ",");                 // this continues where the previous call left off
      strcpy(receivedCommand, strtokIndx);            // Split the bytes into an indexed array using the "," seperator

//LEFT JOG SETUP - move carriage Left until pause, stop, Limit Switches, or set home
      if(strcmp(receivedCommand, "Lft") == 0)
        {
          winder.disableOutputs();                    // disable winder
          carriage.enableOutputs();                   // disable carriage
          EEPROM.get(4, curPosition_C);               // Get stored carriage Position
          carriage.setCurrentPosition(curPosition_C); // Set the carriage Current Position to curPosition_C value
          carriage.moveTo(-carMin);                   // Move carriage to the negative/CCW rotation towards 0/carMin
          carriage.setMaxSpeed(Speed);
          carriage.setAcceleration(Accel);  //5000          
          
// Allow the carriage to move left      ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ ADD IN THE LIMIT SWITCH
          runallowed = true;
          BOLcarLeft = true;
          WME = "0";
          CME = "1";
          LED_left = "1";
          LED_right = "0";
          LED_paused = "0";
          LED_H_set = "0";          
          Send_Status();    // Send the Status values to the PC **** Send_Status() is at the bottom of this code ****
          Send_Progess();    // Send the Progess values to the PC **** Send_Progess() is at the bottom of this code ****
          newData = false;

// Carriage is all the way LEFT - STOP IT
          if (curPosition_C <= 0)
            {
              carriage.disableOutputs();
              Serial.print("<= 0 carMin - Carriage is all the way LEFT");
              Serial.println(carMax);
              delay(50);
              carriage.setCurrentPosition(0);
              curPosition_C = carriage.currentPosition();
              EEPROM.put(4, curPosition_C);
              delay(20);
              CME = "0";
              LED_left = "0";
              Send_Status();
              
      // THE USB PORT NEEDS A BIT OF TIME TO TX THE DATA
              delay(20);
              Send_Progess();
              delay(20);
              BOLcarLeft = false;
              newData = false;
              return; // Return to main loop
            }
          else
            {
              return; // Return to main loop and allow the Carriage to move LEFT
            }
          return; // Return to main loop if we are here by mistake
       }

//RIGHT JOG SETUP - move carriage Right until pause, stop, Limit Switches, or set home
      if(strcmp(receivedCommand, "Rit") == 0)
        {
          winder.disableOutputs();
          carriage.enableOutputs();
          EEPROM.get(4, curPosition_C);
          carriage.setCurrentPosition(curPosition_C);
          carriage.moveTo(carMax);
          carriage.setMaxSpeed(Speed);
          carriage.setAcceleration(5000);  //Accel
          
// Allow the carriage to move RIGHT      ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ ADD IN THE LIMIT SWITCH          
          runallowed = true;
          WME = "0";
          CME = "1";
          LED_left = "0";
          LED_right = "1";
          LED_paused = "0";
          LED_H_set = "0";          
          Send_Status();
          Send_Progess();
          BOLcarLeft = false;
          BOLcarRight = true;
          newData = false;
          runallowed == true;

// Carriage is all the way RIGHT - STOP IT          
          if (carriage.currentPosition() >= carMax)
          {
            carriage.disableOutputs();
            Serial.println("Carriage is all the way RIGHT");
            delay(20);
            curPosition_C = carriage.currentPosition();
            EEPROM.put(4, curPosition_C);
            delay(20);
            CME = "0";
            LED_right = "0";
            Send_Status();
            delay(20);
            Send_Progess();
            delay(20);
            newData = false;
            return;
          }
          else
            {
              return;
            }
          return;
        }

// Pause both motors
      if(strcmp(receivedCommand, "Pas") == 0)
        {
    // Disable the motors
    // Store there positions just encase we decide to cancel the build
          winder.disableOutputs(); //disable outputs
          carriage.disableOutputs(); //disable outputs
          EEPROM.put(0, curPosition_W);
          delay(20);
          curPosition_C = carriage.currentPosition();
          EEPROM.put(4, curPosition_C);
          delay(20);
          curPosition_W = winder.currentPosition();
          LED_paused = "1";
          Send_Status();
          delay(20);
          Send_Progess();
          delay(20);
          newData = false;
          return;
        }

// Continue - let the Motors conitue
      if(strcmp(receivedCommand, "Con") == 0)
        {
          winder.enableOutputs();
          carriage.enableOutputs();
          LED_paused = "0";
          Send_Status();
          delay(20);
          Send_Progess();
          delay(20);
          newData = false;
          return;
        }

// Wind 1 Revolution for 2 reasons, 1 last chance to align before buildig the coil, 2 tightens the wire on to the bobbin
      if(strcmp(receivedCommand, "W1r") == 0)
         {
            StepCount = 0;
            winder.enableOutputs();
            carriage.enableOutputs();
            
    // inTempPos = Direction multiplied by Pulses Per Revolution of Winder Motor which was calculated on the PC. Positive inTempPos (Position) is CW rotation        
            inTempPos = winder_direction * PPR_WindingM;  
    
    //Just encase the winder is not at 0        
            if (winder.currentPosition() > 0)
            {
              inTempPos = inTempPos + PPR_WindingM;
            }
            winder.moveTo(inTempPos); // Move 200, 800, or whatever steps per rev you setup in the PC
            
    // Slow winding        
            winder.setMaxSpeed(250);
            winder.setAcceleration(0);

    // Set the speed of the carriage by using the calculated STEP_RATIO from the PC
            inTempPos = Speed/STEP_RATIO;
            carriage.setSpeed(inTempPos);
            carriage.setAcceleration(Accel);  // setAcceleration(Accel) doesn't work with setSpeed but, this doesn't both it
            runallowed = true;
            WME = "1";
            CME = "1";
            Send_Status();
            delay(20);
            Send_Progess();
            delay(20);
            
            
// Call the Wind-1-rev function **** NOT USING THE RunTheMotor(); FUNCTION HERE ****           
            Wind1rev();

    // New winder and carriage position
            curPosition_W = winder.currentPosition();
            curPosition_C = carriage.currentPosition();            
            EEPROM.put(0, curPosition_W);                  // Store new winder position
            delay(20);
            
    // Reset speed & accel vales        
            winder.setMaxSpeed(Speed); //set speed
            carriage.setMaxSpeed(Speed); //set speed
            winder.setAcceleration(Accel); //update the value of the variable
            carriage.setAcceleration(Accel); //update the value of the variable
            runallowed = false;
            WME = "0";
            CME = "0";
            LED_left = "0";
            LED_right = "0";
            LED_paused = "0";
            Send_Status();
            Send_Progess();
            newData = false;
            return;                                                            
         }

//HOME POSITION -  Sets the start of coil position
      if(strcmp(receivedCommand, "Sth") == 0)
         { 
            winder.disableOutputs(); //disable outputs
            carriage.disableOutputs(); //disable outputs

    // Set the winder & carriage
            curPosition_W = winder.currentPosition();                        
            winder.setCurrentPosition(0);
            Home_Position = carriage.currentPosition();  // carriage home position in real carriage position (steps), need to restore this value after coil is built

            EEPROM.put(12, Home_Position);
            delay(20);
            
            carriage.setCurrentPosition(0); // Set current position to new "new home" or start of bobbin.
            curPosition_C = carriage.currentPosition();  // Set the variable curPosition_C to the current position "new home" or start of bobbin.

            EEPROM.put(0, curPosition_W);   // Store winder position
            delay(20);
            
            EEPROM.put(4, curPosition_C);   // Store carriage position
            delay(20);           
    // Get things ready to wind 1 rev
            WindCount = 0;
            LayerCount = 0;
            LED_H_set = 1;
            WME = "0";
            CME = "0";
            LED_left = "0";
            LED_right = "0";
            LED_paused = "0";         
            Send_Status();
            Send_Progess();
            newData = false;
            return;
         }

//Start building the coil using values sent from the PC
      if(strcmp(receivedCommand, "Str") == 0)
         {
            winder.enableOutputs();
            carriage.enableOutputs();            
            Coil_Build = true;
            runallowed = true;
            WindCount = 1;
            LayerCount = 1;

    // The total winding count
            WindCountTotal = 1;

    // windings_x is the total steps needed to build the coil, (WPLayer) Windings Per Layer, (CalLayers) Total Layers Required, (PPR_WindingM) Winder Pulses per rev
    // had to use the long() convert for this to work
            windings_x =  long(WPLayer) * winder_direction * long(CalLayers) * long(PPR_WindingM); //long(x)
            PulsesPerLayer_Winder = long(WPLayer) * long(PPR_WindingM);

    // I explained most of this earlier
            winder.moveTo(windings_x);
            winder.setMaxSpeed(Speed);
            winder.setAcceleration(Accel);  // 6000

    // IT'S IMPORTANT THAT THE SEQUENCE BELOW IS IN THE RIGHT ORDER TO WORK *********************************************
            carriage.moveTo(PPLayer_C); //PulsesPerLayer_Winder/STEP_RATIO
            inTempPos = Speed/STEP_RATIO;
            carriage.setSpeed(inTempPos);
            carriage.setAcceleration(Accel);  // 6000
            
            WME = "1";
            CME = "1";
            LED_left = "1";
            LED_right = "0";
            LED_paused = "0";
            Send_Status();
            Send_Progess();
            newData = false;
            return;
         }

// STOP - Disable the motors
      if(strcmp(receivedCommand, "Stp") == 0)
         {
            runallowed = false;
            winder.disableOutputs(); //disable outputs
            carriage.disableOutputs(); //disable outputs
            WME = "0";
            CME = "0";
            LED_left = "0";
            LED_right = "0";
            LED_paused = "0";
            LED_H_set = "0";
            Send_Progess();
            newData = false;
            return;
         }

// TERMINATE/CLEAR - Sent form the TERMINATE BUTTON on the PC - Stop everything now and Reset Everything including positions except max and min limits
      if(strcmp(receivedCommand, "Clr") == 0)
         {
      // Sets a new target position that causes the stepper to stop as quickly as possible, using the current speed and acceleration parameters.
      // NOT THE SAME AS disableOutputs()
            winder.stop(); //disable outputs
            carriage.stop(); //disable outputs            
            WME = "0";
            CME = "0";
            LED_left = "0";
            LED_right = "0";
            LED_paused = "0";
            LED_H_set = "0";
            WindCount = 0;
            LayerCount = 0;
            WindCountTotal = 0;
            winder.setCurrentPosition(0);
            
            EEPROM.put(0, 0); //winder zeroed out
            delay(20);
            
            curPosition_W = 0;//Clear the bobbin start position

    //restore the carriage.currentPosition to the saved True position **************************************
            EEPROM.get(12, Home_Position); 
            delay(20);
            
            curPosition_C = Home_Position;
            carriage.setCurrentPosition(curPosition_C);
            EEPROM.put(4, curPosition_C);
            delay(20);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            EEPROM.put(24, 0); //Clear the total number of windings required - NWnum
            delay(20);            
            
            Serial.print("Cleared ");
            Serial.println(Clr);
            delay(20);
            Send_Status();
            Send_Progess();
            newData = false;
            return;
         }

// Status Call for Winder and Carriage Positions
      if(strcmp(receivedCommand, "Sts") == 0)
         {
            Send_Status();
            return;
         }

// LIMIT SWITCHES - Call to Cycle Carriage motor RIGHT & LEFT to get the Limit Switches Positions. Called from PC.
      if(strcmp(receivedCommand, "Cyc") == 0)
         {
            Cycle_LimitS();
            return;
         }         
}

//========================================================================================================================================================================
//===============================================================================ROUTINES/FUNCTIONS=======================================================================

// BUILD the COIL from the calculated values from the PC
void Build_Coil()
  {
    winder.run();
    carriage.runSpeed();    // I can use runSpeed for the carriage because the drive rod is 32 threads per inch or about 153,600 steps for 6" of travel at 800 steps per rev.
    StepCount = StepCount + 1;

  // The Winding counter
    if (StepCount >= PPR_WindingM)
      {
        WindCount = WindCount + 1;
        WindCountTotal = WindCountTotal + 1;
        StepCount = 0;
      }

  // Have we made the number of windings for 1 LAYER, if yes reverse direction & and reset windings count
    if(WindCount >= WPLayer && inTempPos > 0)
      {
        winder.disableOutputs(); //disable outputs
        carriage.disableOutputs(); //disable outputs
        WindCount = 0;
        LayerCount = LayerCount + 1;
        curPosition_W = winder.currentPosition();
        curPosition_C = carriage.currentPosition();
        LED_left = "0";
        LED_right = "1";
        Send_Status();
        Send_Progess();
        delay(1000);   // a nice pause
        winder.enableOutputs();

    // IT'S IMPORTANT THAT THE SEQUENCE BELOW IS IN THE RIGHT ORDER TO WORK *********************************************
    // seems to need this reset
        carriage.moveTo(0);
        inTempPos = Speed/STEP_RATIO;
        carriage.setSpeed(-inTempPos);
        carriage.enableOutputs();
        return;
      }

  // Are we at the starting point of the coil, if yes reverse direction & and reset windings count
    if(carriage.currentPosition() <= 0)
      {
        winder.disableOutputs();
        carriage.disableOutputs();
        WindCount = 0;
        LayerCount = LayerCount + 1;
        curPosition_W = winder.currentPosition();
        curPosition_C = carriage.currentPosition();
        LED_left = "1";
        LED_right = "0";
        Send_Status();
        Send_Progess();
        delay(1000);    
        winder.enableOutputs();            
        carriage.moveTo(PPLayer_C);
        inTempPos = Speed/STEP_RATIO;
        carriage.setSpeed(inTempPos);
        carriage.enableOutputs();
        return;
      }

  // Have we made the number of windings needed, if YES stop building and send carriage back to the start position if needed "GoHome() function"
    if(WindCountTotal >= NWnum)
      {       
        GoHome();
        Serial.println("FINISHED BUILDING COIL ");        
        winder.disableOutputs(); //disable outputs
        carriage.disableOutputs(); //disable outputs        
        runallowed = false;
        BOLcarLeft == false;
        BOLcarRight = false;
        Coil_Build = false;
        WME = "0";
        CME = "0";
        LED_left = "0";
        LED_right = "0";
        LED_paused = "0";
        LED_H_set = "0";
        curPosition_W = winder.currentPosition();
        
   //restore the carriage.currentPosition to the saved True position **************************************
        EEPROM.get(12, Home_Position); 
        delay(20);
            
        curPosition_C = Home_Position;
        carriage.setCurrentPosition(curPosition_C);
        EEPROM.put(4, curPosition_C);
        delay(20);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
        Send_Status();
        Send_Progess();
        delay(1000);
        return;
      }     
    return;
}

// Move carriage Left until pause, stop, Limit Switch, or set home
void carLeft()
  {
    if (runallowed == true &&  (carriage.currentPosition() >= carMin)) //allow running - this allows entering the carriage.run() function.
      {
        checkSerial();
        carriage.run();

    // Check to see if the limit switch is closed, grounding the pin 11, once per step - STOP and store postion if low
        if (digitalRead(limSwtLeft) == LOW)
          { 
    // limit switch is closed
            carriage.disableOutputs();

    // Store Left Limit Position
            limSwLeft_Pos = carriage.currentPosition();
            EEPROM.get(20, limSwLeft_Pos);
            delay(20);
            
    // Store where we are        
            carriage.setCurrentPosition(0);
            curPosition_C = carriage.currentPosition();
            EEPROM.put(4, curPosition_C);

    // Stop running
            runallowed = false;
            BOLcarLeft = false;            
            carMin = 0;
            CME = "0";
            LED_left = "0";   
            Send_Status();
            Send_Progess();
            Serial.println("carriage is at LEFT ZERO " + carriage.currentPosition());
            delay(20);
          }
      }
    else
      {

    // Are we at the new 0 location, if so stop and set position
        if(carriage.currentPosition() <= carMin)
          {
            carriage.disableOutputs();
            carMin = 0;
            CME = "0";
            LED_left = "0";
            carriage.setCurrentPosition(0);
            curPosition_C = 0;
            EEPROM.put(4, curPosition_C);
            delay(20);
            Send_Status();
            Send_Progess();
            Serial.println("carriage is at LEFT ZERO " + carriage.currentPosition());
            delay(20);
          }
        else
          {
    // Just encase we got here by mistake
            runallowed = false;
            BOLcarLeft = false;
            //carriage.stop();
            carriage.disableOutputs();
            CME = "0";
            LED_left = "0";
            Serial.println("Outputs Disabled RUN NOT ALLOWED");
            delay(20);
            Send_Status();
            delay(20);
            Send_Progess();
            delay(20);
          }        
      }
  }

// move carriage Right until pause, stop, limit Switch, or set home =============================================================================
// SAME AS THE carLEFT FUNCTION - EXCEPT OTHER DIRECTION
void carRight()
  {
    if (runallowed == true &&  (carriage.currentPosition() <= carMax)) //allow running - this allows entering the carriage.run() function.
      {
        checkSerial();
        carriage.run();      
        if (digitalRead(limSwtRight) == LOW)
          { 
// limit switch is closed - STOP and store postion if pin 12 is low
            carriage.disableOutputs();
            runallowed = false;
            BOLcarRight = false;
            limSwRight_Pos = carriage.currentPosition();
            EEPROM.put(16, limSwRight_Pos);
            delay(20);
            runallowed = false;
            CME = "0";
            LED_right = "0";
            curPosition_C = limSwRight_Pos; //carriage.currentPosition();
            EEPROM.put(4, curPosition_C);
            delay(20);
            Send_Status();
            Send_Progess();
          }
      }
    else
      {
        if(carriage.currentPosition() >= carMax)
          {
            runallowed = false;
            BOLcarRight = false;
            carriage.disableOutputs();
            Serial.println("carriage is at carMax");
            delay(20);
            CME = "0";
            LED_right = "0";
            curPosition_C = carMax;
            EEPROM.put(4, curPosition_C);
            delay(20);
            Send_Status();
            Send_Progess();
          }
        else
          {
            runallowed = false;
            BOLcarRight = false;
            //carriage.stop();
            carriage.disableOutputs();
            CME = "0";
            LED_right = "0";
            curPosition_C = carriage.currentPosition();
            EEPROM.put(4, curPosition_C);
            delay(20);
            Serial.println("Outputs Disabled RUN NOT ALLOWED");
            delay(20);
            Send_Status();
            Send_Progess();
          }
      }
  }

//function for RUNNING the motors - CALLED FROM MAIN LOOP ==========================================================================================
void RunTheMotor() 
{
  if (runallowed == true)
    {
      checkSerial();
    
      if (BOLcarLeft == true && carriage.currentPosition() > limSwLeft_Pos)
        {
  //step the motor LEFT (this will step the motor by 1 step at each loop)        
          carLeft();        
          return;
        }
   
      if (BOLcarRight == true && carriage.currentPosition() < limSwRight_Pos)
        {
  //step the motor RIGHT (this will step the motor by 1 step at each loop)        
          carRight();
          return;
        }

      if(Coil_Build = true)
        {
          Build_Coil();
        }
    
      if (BOLcarLeft == false && BOLcarRight == false)
        {
          return;
        }
    }
  
  //program enters this part if the runallowed is FALSE, we do not do anything
  else 
    {
      runallowed = false;
      BOLcarLeft == false;
      BOLcarRight = false;
      winder.disableOutputs(); //disable outputs
      carriage.disableOutputs(); //disable outputs
      return;
    }
}

// Sends the Carriage to the Home Position when coil is done ================================================================================================
void GoHome()
{ 
  for(int i = 0; i <= 5; i++) // Just for fun
    {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level) ************ FOR FEEDBACK TO PC USE ***************
      delay(500);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(500);                       // wait for a second ************ FOR FEEDBACK TO PC USE ***************
    }
  
  if (carriage.currentPosition() == 0)
    {
      Serial.println("We are at the home position.");
      winder.disableOutputs(); //disable power
      carriage.disableOutputs(); //disable power
      WME = "0";
      CME = "0";
      runallowed = false;
      Send_Status();
      Send_Progess();
    }
  else
    {
      winder.disableOutputs(); //disable power
      carriage.setMaxSpeed(400); //set speed manually to 400. In this project 400 is 400 step/sec = 1 rev/sec.
      carriage.enableOutputs(); //enable pins
      WME = "0";
      CME = "1";    
      carriage.moveTo(0); //set abolute distance to move
      CME = "0";
      Send_Status();
      Send_Progess();
    }
}

// Winds 1 winding on the bobbin ============================================================================================
void Wind1rev()
{
  if (runallowed == false)
    {
      winder.disableOutputs();
      carriage.disableOutputs();
      Serial.println("Outputs Disabled RUN NOT ALLOWED");
      WME = "0";
      CME = "0";
      WindCount = 0;
      LayerCount = 0;
      Send_Status();
      Send_Progess();
      return;
    }

  if (runallowed == true)
    {
      Send_Progess();
      do
        {
          winder.run();      
          StepCount = StepCount + 1;
          if (StepCount >= STEP_RATIO)
            {
              carriage.move(carTempPos);
              carriage.runSpeed();
              carTempPos = carTempPos + 1;
              StepCount = 0;
            }
        
          if(winder.currentPosition() == inTempPos)
            {
              runallowed == false;
              winder.disableOutputs();
              carriage.disableOutputs();
              carTempPos = 0;
              WME = "0";
              CME = "0";
              curPosition_W = winder.currentPosition();
              EEPROM.put(0, curPosition_W);
              delay(20);
              curPosition_C = carriage.currentPosition();
              EEPROM.put(4, curPosition_C);
              delay(20);
              WindCount = WindCount + 1;
              WindCountTotal = WindCountTotal + 1;
              LayerCount = 1;
              Send_Status();
              Send_Progess();
              delay(500);
              break;
            }
        } while(winder.currentPosition() != inTempPos);
      return;
    }
}

//Cycles the Carriage to find the Limit Switches and store their position values ==========================================================================
void Cycle_LimitS()
{
  {
    winder.disableOutputs();
    carriage.enableOutputs();

 // Get the carriage position   
    EEPROM.get(4, curPosition_C);
    carriage.setCurrentPosition(curPosition_C);

 // Are we at less than Max right, if so move to Max right
    if (curPosition_C <= carMax)
      {
        carriage.moveTo(carMax);
        carriage.setMaxSpeed(Speed);
        carriage.setAcceleration(Accel);
        WME = "0";
        CME = "1";
        LED_left = "0";
        LED_right = "1";
        LED_paused = "0";
        LED_H_set = "0";
        Send_Status();
        Send_Progess();

    // LOOP - Run the carriage to the right until carMax or the limit switch closes
        while(curPosition_C <= carMax)
          {
            carriage.run();
            if (digitalRead(limSwtRight) == LOW)
              { // limit switch is closed
                carriage.disableOutputs();
                limSwRight_Pos = 1;
                carMax = carriage.currentPosition();
                EEPROM.put(8, carMax);                              
                delay(20);
                break;
              }          
          }

    // Store MAX RIGHT position
        limSwRight_Pos = carMax;
        EEPROM.put(16, limSwRight_Pos);
        delay(20);
        curPosition_C = carriage.currentPosition();
        EEPROM.put(4, curPosition_C);
        delay(20);
        CME = "0";
        LED_right = "0";
        Send_Status();
        Send_Progess();
        Serial.println("Carriage is at RIGHT LIMIT");
        delay(1000);
      }
      else

    // We are at MAX RIGHT already
        {            
          Serial.println("Carriage is all the way RIGHT");
          delay(20);
          curPosition_C = carriage.currentPosition();
          delay(20);
          carriage.disableOutputs();
          EEPROM.put(4, curPosition_C);
          delay(20);
          CME = "0";
          LED_right = "0";
          Send_Status();
          Send_Progess();
        }
    delay(500);    // A little pause before changing directions

    // move left to carriage limit left limit or to carMin
    if (digitalRead(limSwtRight) == LOW) 
      {
        EEPROM.get(4, curPosition_C);
        delay(20);
          if (curPosition_C > 0)
            {
              carriage.enableOutputs();
              carriage.setCurrentPosition(curPosition_C);

    // The negative "-" changes MOTOR direction
              carriage.moveTo(-carMin);
              carriage.setMaxSpeed(Speed);
              carriage.setAcceleration(Accel);
              WME = "0";
              CME = "1";
              LED_left = "1";
              LED_right = "0";
              LED_paused = "0";
              LED_H_set = "0";
              Send_Status();
              Send_Progess();

    // LOOP - Run the carriage to the left until carMax or the limit switch closes
              while(carriage.currentPosition() > 0)
                {
                  carriage.run();
                  if (digitalRead(limSwtLeft) == LOW)
                    { // limit switch is closed
                      limSwLeft_Pos = 1;
                      break;
                    }
                }

    // Store MAX LEFT position                
              carMin = carriage.currentPosition();            
              EEPROM.put(20, carMin);
              delay(20);
              carriage.setCurrentPosition(0);
              curPosition_C = carriage.currentPosition();
              EEPROM.put(4, curPosition_C);
              delay(20);            
              CME = "0";
              LED_left = "0";
              Send_Status();
              Send_Progess();
              Serial.println("Carriage is at LEFT LIMIT");
              delay(20);
          }
        else
          {
    // We are at MAX left already
            carriage.setCurrentPosition(0);
            curPosition_C = carriage.currentPosition();
            EEPROM.put(4, curPosition_C);
            delay(20);
            CME = "0";
            LED_left = "0";
            Send_Status();
            Send_Progess();
            Serial.println("Carriage is all the way LEFT");
            delay(20);
          }        
      }
      return;
  }
}

//INDIACATORS ===========================================================================================================================
void Send_Progess()
{
//This is sent back to the PC
//WME, CME, LED_left, LED_right, LED_paused, LED_H_set, WOWINDING, WOLAYER
//eg. I,11100111,######,###  this what is sent to PC
// + ',' + WME + ',' + CME + ',' + LED_left + ',' + LED_right + ',' + LED_paused + ',' + LED_H_set + ',' + WOWINDING + ',' + WOLAYER
//eg.I      1           0             1                 0                 0                 0                000001            001   //18 characters

  sprintf(WOWINDING, "%09d", WindCountTotal);
  sprintf(WOLAYER, "%04d", LayerCount);
          
  String dataout = (indOUT + WME + CME + LED_left + LED_right + LED_paused + LED_H_set + WOWINDING + WOLAYER); // + "\n"
  Serial.println(dataout);
  delay(20);
  return;
}

// =====================================================================================================================================
// Send the positions of the winder and carriage to the PC in # of steps !!!CAN NOT BE CALLED TOO OFTEN - WILL SLOW THE STEPPERS A LOT
void Send_Status()
{
  char W[10];
  char C[10];
  
  sprintf(W, "%09ld", curPosition_W);
  sprintf(C, "%09ld", curPosition_C);
  String strSts = (STSout + W + C);
  Serial.println(strSts);
  delay(20);
}
// End of PROGRAM---------------------------------------------------------------------------------------------------------------------------------
