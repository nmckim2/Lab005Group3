#include <MSE2202_Lib.h>

/*

 MSE 2202 MSEBot code for Final Project
 Language: Arduino
 Authors: Nicklaus Mckim and Ryan McCuaig
 
 Rev 1 - Initial version 2016
 Rev 2 - Update for MSEduino V0.2
 ...
 Rev 4 - revisit for MSEDuino V4.2 2023

 */

//  To program and use ESP32-S3
//   
//  File->Preferences:
//  Additional Boards Manager URLs: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
//
//
//  Tools->:
//  Board: "Adafruit Feather ESP32-S3 No PSRAM"
//  Upload Speed: "921600"
//  USB CDC On Boot: "Enabled"
//  USB Firmware MSC on Boot: "Disabled"
//  USB DFU On Bot: "Disabled"
//  Upload Mode:"UART0/Hardware CDC"
//  SPU Frequency: "240MHz (WiFi)"
//  Flash Mode: "QIO 80MHz"
//  Flash SIze: "4MB (32Mb)"
//  Partition Scheme: "Default 4MB with spiffs (1.2MB app/1.5MB SPIFFS)"
//  Core Debug Level: "Verbose"
//  PSRAM: 'Disabled"
//  Arduino Runs On: "Core 1"
//  Events Run On: "Core 1"
//
//  To program, press and hold the reset button then press and hold program button, release the reset button then 
//  release the program button 
//

#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>

// Uncomment keywords to enable debugging output
#define DEBUG_DRIVE_SPEED 1
#define DEBUG_ENCODER_COUNT 1

// Port pin constants

#define MODE_BUTTON         0   // GPIO0  pin 27 for Push Button 1
                                   
#define LEFT_MOTOR_A        35  // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36  // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37  // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38  // GPIO38 pin 31 (J38) Motor 2 B
                                   
#define SHOULDER_SERVO      41  // GPIO41 pin 34 (J41) Servo 1
#define CLAW_SERVO          42  // GPIO42 pin 35 (J42) Servo 2
                                   
#define ENCODER_LEFT_A      15  // When DIP Switch S1-1 is on, Left encoder A signal is connected to pin 8 GPIO15 (J15)
                                // When DIP Switch S1-1 is off, J15 can be used as analog AD2-4
#define ENCODER_LEFT_B      16  // When DIP Switch S1-2 is on, Left encoder B signal is connected to pin 9 GPIO16 (J16)
                                // When DIP Switch S1-2 is off, J16 can be used as analog AD2-5
#define ENCODER_LEFT_DIR    17  // When DIP Switch S1-3 is on, Left encoder Direction signal is connected to pin 10 GPIO17 (J17)
                                // When DIP Switch S1-3 is off, J17 can be used as analog AD2-6
#define ENCODER_LEFT_SPD    18  // When DIP Switch S1-4 is on, Left encoder Speed signal is connected to pin 11 GPIO18 (J18)
                                // When DIP Switch S1-4 is off, J18 can be used as analog AD2-7
                                   
#define MOTOR_ENABLE_SWITCH 3   // DIP Switch S1-5 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
                                // When DIP Switch S1-5 is off, J3 can be used as analog AD1-2
                                   
#define ENCODER_RIGHT_A     11  // When DIP Switch S1-7 is on, Right encoder A signal is connected to pin 19 GPIO11 (J11)
                                // When DIP Switch S1-7 is off, J11 can be used as analog AD2-0
#define ENCODER_RIGHT_B     12  // When DIP Switch S1-8 is on, Right encoder B signal is connected to pin 20 GPIO12 (J12)
                                // When DIP Switch S1-8 is off, J12 can be used as analog AD2-1
#define ENCODER_RIGHT_DIR   13  // When DIP Switch S1-9 is on, Right encoder Direction signal is connected to pin 21 GPIO13 (J13)
                                // When DIP Switch S1-9 is off, J13 can be used as analog AD2-2
#define ENCODER_RIGHT_SPD   14  // When DIP Switch S1-10 is on, Right encoder Speed signal is connected to pin 22 GPIO14 (J14)
                                // When DIP Switch S1-10 is off, J14 can be used as analog AD2-3
                                   
#define BRDTST_POT_R1       1   // When DIP Switch S1-12 is on, Analog AD1-0 (pin 39) GPIO1 is connected to Poteniometer R1

#define STEPPER_DIR         39  // GPIO39 pin 32 (J39) STEPPER Motor direction pin
#define STEPPER_CLK         40  // GPIO40 pin 33 (J40) stepper motor clock pin
                                   
#define SMART_LED           21  // When DIP Switch S1-11 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1   // Number of SMART LEDs in use

#define IR_DETECTOR         9   // GPIO9 pin 17 (J9) IR detector input

#define LEFT                1   // Indicates left direction for (stepper) motor
#define RIGHT               0   // Indicates right direction for (stepper) motor



// Constants

const int ci_Display_Update = 100;                                            // Update interval for Smart LED in milliseconds

//=====================================================================================================================
//
// IMPORTANT: The constants in this section need to be set to appropriate values for your robot. 
// You will have to experiment to determine appropriate values.

const int ci_Claw_Servo_Open = 1650;                                          // Value for open position of claw
const int ci_Claw_Servo_Closed = 1880;                                        // Value for closed position of claw
const int ci_Shoulder_Servo_Retracted = 690;                                  // Value for shoulder of arm fully retracted
const int ci_Shoulder_Servo_Extended = 1300;                                  // Value for shoulder of arm fully extended

//
//=====================================================================================================================

// Variables

boolean bt_Motors_Enabled = true;                                             // Motors enabled flag
boolean bt_3_S_Time_Up = false;                                               // 3 second timer elapsed flag
boolean bt_2_S_Time_Up = false;                                               // 2 second timer elapsed flag
boolean bt_200_mS_Time_Up = false;                                            // 200 millisecond timer elapsed flag
boolean bt_Direction;                                                         // Stepper motor direction
boolean bt_Stepper_Step;                                                      // Stepper motor step flag

unsigned char uc_Drive_Speed;                                                 // Motor drive speed (0-255)
unsigned char uc_Drive_Index;                                                 // State index for run mode (1)
unsigned char uc_Stepper_Index;                                               // State index for stepper test mode (2)

int i_MaxStepsFromCentre = 900;                                               // Allowable number of steps from centre point
int i_StepperCentre = i_MaxStepsFromCentre;                                   // Centre point of stepper range, in steps
int i_StepCounter = 0;                                                        // Number of steps to take to reach setpoint
int i_StepperPosition;                                                        // Current position of stepper, in steps

unsigned int ui_CurrentPotValue;                                              // Current value of pot as raw ADC value
unsigned int ui_PreviousPotValue = 0;                                         // Previous value of pot as raw ADC value
unsigned int ui_DeadZone = 50;                                                // Allowable difference between current stepper set point and
                                                                              // value from pot. Accounts for noise in ADC readings.
unsigned int ui_PotArmSetpoint;                                               // Desired position of arm stepper motor read from pot
unsigned int ui_PotClawSetpoint;
unsigned int ui_PotClawSetpoint1;  
unsigned int ui_PotClawSetpoint2;// Desired position of claw servo read from pot
unsigned int ui_PotShoulderSetpoint;                                          // Desired position of shoulder servo read from pot
unsigned int ui_Mode_PB_Debounce;                                             // Pushbutton debounce timer count

unsigned long ul_3_Second_timer = 0;                                          // 3 second timer count in milliseconds
unsigned long ul_2_Second_timer = 0;                                          // 2 second timer count in milliseconds
unsigned long ul_200_mS_timer = 0;                                            // 200 millisecond timer count in milliseconds
unsigned long ul_Display_Time;                                                // Heartbeat LED update timer
unsigned long ul_Previous_Micros;                                             // Last microsecond count
unsigned long ul_Current_Micros;                                              // Current microsecond count

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {5,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,
                                       240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15};

unsigned int  ui_Robot_Mode_Index = 0;                                        // Robot operational state                              
unsigned int  ui_Mode_Indicator[7] = {                                        // Colours for different modes
  SmartLEDs.Color(255,0,0),                                                   //   Red - Stop
  SmartLEDs.Color(0,255,0),                                                   //   Green - Run
  SmartLEDs.Color(0,0,255),                                                   //   Blue - Test stepper
  SmartLEDs.Color(255,255,0),                                                 //   Yellow - Test claw servo
  SmartLEDs.Color(255,0,255),                                                 //   Magenta - Test shoulder servo
  SmartLEDs.Color(0,255,255),                                                 //   Cyan - Test IR receiver
  SmartLEDs.Color(255,165,0)                                                  //   Orange - empty case
};                                                                            

// Motor and encoder objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();                                                        // Instance of Motion for motor control
Encoders driveEncoders = Encoders();                                          // Instance of Encoders for encoder data
IR Scan = IR();                                                               // Instance of IR for beacon detection

// Interrupt Service Routines
void IRAM_ATTR LeftSpd_EncoderISR()                                           // ISR to update left encoder count
{
   driveEncoders.LeftSpd_Encoder_ISR();
}

void IRAM_ATTR RightSpd_EncoderISR()                                          // ISR to update right encoder count
{
	driveEncoders.RightSpd_Encoder_ISR();
}

// Function Declarations
void Indicator();                                                             // For mode/heartbeat on Smart LED


unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;
int buttonState;
int ledState = HIGH;
int lastButtonState = HIGH;


long prevTime;

const int trigPin = 5;
const int echoPin = 6;
long duration;
int distance;

void setup()
{
   Serial.begin(9600);

   Scan.Begin(IR_DETECTOR);                                                    //set up IR Detection
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

   // Set up motors and encoders
   Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B); // Set up motors as Drive 1
   Bot.servoBegin("S1",CLAW_SERVO);   
   Bot.servoBegin("S2",SHOULDER_SERVO);

   driveEncoders.Begin(0, LeftSpd_EncoderISR, RightSpd_EncoderISR);           // Set up encoders

   // Set up SmartLED
   SmartLEDs.begin();                                                         // Initialize smart LEDs object (REQUIRED)
   SmartLEDs.clear();                                                         // Clear pixel
   SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                         // Set pixel colors to 'off'
   SmartLEDs.show();                                                          // Send the updated pixel colors to the hardware

   // Set up mode pushbutton
   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);                                // Set up motor enable switch with internal pullup
   ui_Mode_PB_Debounce = 0;                                                   // Reset debounce timer count
   
   //Stepper motor 
   pinMode(STEPPER_CLK, OUTPUT);                                              // Set up stepper step/clock pin as output
   pinMode(STEPPER_DIR, OUTPUT);                                              // Set up stepper direction pin as output

   pinMode(MODE_BUTTON, INPUT_PULLUP);                                        //set up mode button with internal pullup

   
}

void loop()
{
  int reading = digitalRead(10);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
  // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
  // whatever the reading is at, it's been there for longer than the debounce
  // delay, so take it as the actual current state:

  // if the button state has changed:
    if (reading != buttonState) {
    buttonState = reading;

  // only toggle the LED if the new button state is HIGH
     if (buttonState == LOW) {
        ledState = !ledState;
      }
    }
    
    }




  if (ledState == HIGH){
    ledState ==LOW;
  }
  lastButtonState = reading;


  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  //Serial.print("Distance: ");
  //Serial.println(distance);

  
    
   ul_Current_Micros = micros();                                              // Get current time in microseconds
   if ((ul_Current_Micros - ul_Previous_Micros) >= 1000)                      // Enter when 1 ms has elapsed
   {
      ul_Previous_Micros = ul_Current_Micros;                                 // Record current time in microseconds

      // 3 second timer, counts 3000 milliseconds
      ul_3_Second_timer = ul_3_Second_timer + 1;                              // Increment 3 second timer count
      if(ul_3_Second_timer > 3000)                                            // If 3 seconds have elapsed
      {
         ul_3_Second_timer = 0;                                               // Reset 3 second timer count
         bt_3_S_Time_Up = true;                                               // Indicate that 3 seconds have elapsed
      }
   
      // 2 second timer, counts 2000 milliseconds
      ul_2_Second_timer = ul_2_Second_timer + 1;                              // Increment 2 second timer count
      if(ul_2_Second_timer > 2000)                                            // If 2 seconds have elapsed
      {
         ul_2_Second_timer = 0;                                               // Reset 2 second timer count
         bt_2_S_Time_Up = true;                                               // Indicate that 2 seconds have elapsed
      }
   
      // 200 millisecond timer, counts 200 milliseconds
      ul_200_mS_timer = ul_200_mS_timer + 1;                                  // Increment 200 millisecond timer count
      if(ul_200_mS_timer > 200)                                               // If 200 milliseconds have elapsed
      {
         ul_200_mS_timer = 0;                                                 // Reset 200 millisecond timer count
         bt_200_mS_Time_Up = true;                                            // Indicate that 200 milliseconds have elapsed
      }

      // Mode pushbutton debounce and toggle
      if(!digitalRead(MODE_BUTTON))                                           // If pushbutton GPIO goes LOW (nominal push)
      {
         // Start debounce
         if(ui_Mode_PB_Debounce <= 25)                                        // 25 millisecond debounce time
         {
            ui_Mode_PB_Debounce = ui_Mode_PB_Debounce + 1;                    // Increment debounce timer count
            if(ui_Mode_PB_Debounce > 25)                                      // If held for at least 25 mS
            {
               ui_Mode_PB_Debounce = 1000;                                    // Change debounce timer count to 1 second
            }
         }
         if(ui_Mode_PB_Debounce >= 1000)                                      // Maintain 1 second timer count until release
         {
            ui_Mode_PB_Debounce = 1000;
         }
      }
      else                                                                    // Pushbutton GPIO goes HIGH (nominal release)
      {
         if(ui_Mode_PB_Debounce <= 26)                                        // If release occurs within debounce interval
         {
            ui_Mode_PB_Debounce = 0;                                          // Reset debounce timer count
         }
         else
         {
            ui_Mode_PB_Debounce = ui_Mode_PB_Debounce + 1;                    // Increment debounce timer count
            if(ui_Mode_PB_Debounce >= 1025)                                   // If pushbutton was released for 25 mS
            {
               ui_Mode_PB_Debounce = 0;                                       // Reset debounce timer count
               ui_Robot_Mode_Index++;                                         // Switch to next mode
               ui_Robot_Mode_Index = ui_Robot_Mode_Index & 7;                 // Keep mode index between 0 and 7
               ul_3_Second_timer = 0;                                         // Reset 3 second timer count
               bt_3_S_Time_Up = false;                                        // Reset 3 second timer         
            }
         }
      }
  
      // check if drive motors should be powered
      bt_Motors_Enabled = !digitalRead(MOTOR_ENABLE_SWITCH);                  // If SW1-5 is on (low signal), then motors are enabled

      // modes 
      // 0 = Default after power up/reset. Robot is stopped.
      // 1 = Press mode button once to enter. Run robot.
      // 2 = Press mode button twice to enter. Test stepper motor. 
      // 3 = Press mode button three times to enter. Test claw servo. 
      // 4 = Press mode button four times to enter. Test arm shoulder servo.
      // 5 = Press mode button five times to enter. Test IR receiver. 
      // 6 = Press mode button six times to enter.  //add your code to do something 
      switch(ui_Robot_Mode_Index)
      {
         case 0: // Robot stopped
         {
            Bot.Stop("D1");                                                   // Stop Drive 1
            uc_Drive_Index = 0;                                               // Reset drive index
            uc_Stepper_Index = 0;                                             // Reset stepper index
            driveEncoders.clearEncoder();                                     // Clear encoder counts
            bt_2_S_Time_Up = false;                                           // Reset 2 second timer flag
            break;
         }  
      
         case 1: // Run robot
         {
            if(bt_3_S_Time_Up)                                                // Pause for 3 sec before running Case 1 code
            {
               // Read pot to update drive motor speed
               uc_Drive_Speed = map(analogRead(BRDTST_POT_R1), 0, 4096, 150, 255);
#ifdef DEBUG_DRIVE_SPEED 
               Serial.print(F("Drive Speed: Pot R1 = "));
               Serial.print(analogRead(BRDTST_POT_R1));
               Serial.print(F(", mapped = "));
               Serial.println(uc_Drive_Speed);
#endif
#ifdef DEBUG_ENCODER_COUNT
               driveEncoders.getEncoderRawCount();
               driveEncoders.getEncoderRawSpeed();
               Serial.print(F("Left Encoder count = "));
               Serial.print(driveEncoders.lRawEncoderLeftCount);
               Serial.print(F("   Right Encoder count = "));
               Serial.print(driveEncoders.lRawEncoderRightCount);
               Serial.print(F("   Left Encoder speed = "));
               Serial.print(driveEncoders.lRawEncoderLeftSpeed);
               Serial.print(F("   Right Encoder speed = "));
               Serial.println(driveEncoders.lRawEncoderRightSpeed);
#endif
               if(bt_Motors_Enabled)                                          // Run motors only if enabled
               {
                  if(bt_200_mS_Time_Up)                                          // Update drive state after 2 seconds
                  {
                     bt_200_mS_Time_Up = false;                                  // Reset 2 second timer
                     
                     switch(uc_Drive_Index)                                   // Cycle through drive states
                     {
                        case 0: // Set servo to beginning position
                        {
                           Bot.ToPosition("S1", 600);
                           uc_Drive_Index = 1;
                           break;
                        }
                        case 1: // Set servo to flat position in increments (case 1-9)
                        {
                          
                           Bot.ToPosition("S1", 650);
                           uc_Drive_Index = 2;
                           break;
                        }
                        case 2: // Set servo to flat position in increments (case 1-9)
                        {
                           Bot.ToPosition("S1", 700);
                           uc_Drive_Index = 3;                               
                           break;
                        }
                        case 3: // Set servo to flat position in increments (case 1-9)
                        {
                          Bot.ToPosition("S1", 750);
                           uc_Drive_Index = 4;                                
                           break;
                        }
                        case 4: // Set servo to flat position in increments (case 1-9)
                        {
                           Bot.ToPosition("S1", 800);
                           uc_Drive_Index = 5;                                
                           break;
                        }
                        case 5:// Set servo to flat position in increments (case 1-9)
                        {
                         Bot.ToPosition("S1", 850);
                          uc_Drive_Index = 6;                                
                          break;
                        }
                        case 6:// Set servo to flat position in increments (case 1-9)
                        {
                          Bot.ToPosition("S1", 900);
                          uc_Drive_Index = 7;                                
                          break;
                        }
                        case 7:// Set servo to flat position in increments (case 1-9)
                        {
                          Bot.ToPosition("S1", 950);
                          uc_Drive_Index = 8;
                          break;
                        }
                        case 8:// Set servo to flat position in increments (case 1-9)
                        {
                          Bot.ToPosition("S1", 1000);
                          uc_Drive_Index = 9;
                          break;
                        }
                        case 9:// Set servo to flat position in increments (case 1-9)
                        {
                          Bot.ToPosition("S1", 1050);
                          uc_Drive_Index = 10;
                          break;
                        }
                        case 10: // Make robot drive forward until edge is detected
                        {
                          Bot.Forward("D1", uc_Drive_Speed, uc_Drive_Speed);
                          if (distance > 25){ // Once ultrasonic seonsor receives feedback of large distance robot will stop
                            Bot.Stop("D1");

                            uc_Drive_Index = 11;
                          }
                          break;
                        }

                        case 11: // Set servo to inclined position to reach opposing edge in increments (case 11-15)
                        {
                          Bot.ToPosition("S1", 1100);
                          uc_Drive_Index = 12;
                          break;
                        }
                        case 12: // Set servo to inclined position to reach opposing edge in increments (case 11-15)
                        {
                          Bot.ToPosition("S1", 1150);
                          uc_Drive_Index = 13;
                          break;
                        }
                        case 13: // Set servo to inclined position to reach opposing edge in increments (case 11-15)
                        {
                          Bot.ToPosition("S1", 1200);
                          uc_Drive_Index = 14;
                          break;
                        }
                        case 14: // Set servo to inclined position to reach opposing edge in increments (case 11-15)
                        {
                          Bot.ToPosition("S1", 1250);
                          uc_Drive_Index = 15;
                          break;
                        }
                        case 15: // Set servo to inclined position to reach opposing edge in increments (case 11-15)
                        {
                          Bot.ToPosition("S1", 1300);
                          uc_Drive_Index = 16;
                          prevTime = millis(); // Set prevTime to millis() so robot knows how long to drive for
                          break;
                        }
                        case 16: // Drive forward for 5 seconds
                        {
                          Bot.Forward("D1", uc_Drive_Speed, uc_Drive_Speed);
                          if((millis() - prevTime >= 5000)){ // Stop after 5 seconds
                            Bot.Stop("D1");
                            uc_Drive_Index = 17;
                          }  
                          break;
                        }
                        case 17: // Set servo to make extending section flat in increments (case 17-24)
                        {
                          Bot.ToPosition("S1", 1250);
                          uc_Drive_Index = 18;
                          break;
                        }
                        case 18: // Set servo to make extending section flat in increments (case 17-24)
                        {
                          Bot.ToPosition("S1", 1200);
                          uc_Drive_Index = 19;
                          break;
                        }
                        case 19: // Set servo to make extending section flat in increments (case 17-24)
                        {
                          Bot.ToPosition("S1", 1150);
                          uc_Drive_Index = 20;
                          break;
                        }
                        case 20: // Set servo to make extending section flat in increments (case 17-24)
                        {
                          Bot.ToPosition("S1", 1100);
                          uc_Drive_Index = 21;
                          break;
                        }
                        case 21: // Set servo to make extending section flat in increments (case 17-24)
                        {
                          Bot.ToPosition("S1", 1050);
                          uc_Drive_Index = 22;
                          break;
                        }
                        case 22: // Set servo to make extending section flat in increments (case 17-24)
                        {
                          Bot.ToPosition("S1", 1000);
                          uc_Drive_Index = 23;
                          break;
                        }
                        case 23: // Set servo to make extending section flat in increments (case 17-24)
                        {
                          Bot.ToPosition("S1", 950);
                          uc_Drive_Index = 24;
                          break;
                        }
                        case 24: // Set servo to make extending section flat in increments (case 17-24)
                        {
                          Bot.ToPosition("S1", 900);
                          uc_Drive_Index = 25;
                          prevTime = millis(); // Set prevTime to millis() so robot knows how long to drive for
                          break;
                        }

                        case 25: // Drive forward for 4 seconds
                        {
                          Bot.Forward("D1", uc_Drive_Speed, uc_Drive_Speed);
                          if ((millis() - prevTime) >= 4000){ // Stop after 4 seconds
                            Bot.Stop("D1");
                            uc_Drive_Index = 26;
                          }  
                          break;
                        }
                        case 26: // Set servo to flat position in increments (case 26-30)
                        {
                          Bot.ToPosition("S1", 850);
                          uc_Drive_Index = 27;
                          break;
                        }
                        case 27: // Set servo to flat position in increments (case 26-30)
                        {
                          Bot.ToPosition("S1", 900);
                          uc_Drive_Index = 28;
                          break;
                        }
                        case 28: // Set servo to flat position in increments (case 26-30)
                        {
                          Bot.ToPosition("S1", 950);
                          uc_Drive_Index = 29;
                          break;
                        }
                        case 29: // Set servo to flat position in increments (case 26-30)
                        {
                          Bot.ToPosition("S1", 1000);
                          uc_Drive_Index = 30;
                          break;
                        }
                        case 30: // Set servo to flat position in increments (case 26-30)
                        {
                          Bot.ToPosition("S1", 1050);
                          uc_Drive_Index = 31;
                          break;
                        }
                        case 31: // Make robot drive forward until edge is detected
                        {
                          Bot.Forward("D1", uc_Drive_Speed, uc_Drive_Speed);
                          if (distance > 25){// Once ultrasonic seonsor receives feedback of large distance robot will stop
                            Bot.Stop("D1");
                            uc_Drive_Index = 32;
                            prevTime = millis(); // Set prevTime to millis() so robot knows how long to drive for 
                          }
                          break;
                        }
                        case 32: // Set servo to starting position in increments (case 32-40)
                        {
                           Bot.ToPosition("S1", 1000);
                           uc_Drive_Index = 33;
                           break;
                        }
                        case 33: // Set servo to starting position in increments (case 32-40)
                        {
                          
                           Bot.ToPosition("S1", 950);
                           uc_Drive_Index = 34;                               
                           break;
                        }
                        case 34: // Set servo to starting position in increments (case 32-40)
                        {
                           Bot.ToPosition("S1", 900);
                           uc_Drive_Index = 35;                               
                           break;
                        }
                        case 35: // Set servo to starting position in increments (case 32-40)
                        {
                          Bot.ToPosition("S1", 850);
                           uc_Drive_Index = 36;                               
                           break;
                        }
                        case 36: // Set servo to starting position in increments (case 32-40)
                        {
                           Bot.ToPosition("S1", 800);
                           uc_Drive_Index = 37;                             
                           break;
                        }
                        case 37: // Set servo to starting position in increments (case 32-40)
                        {
                         Bot.ToPosition("S1", 750);
                          uc_Drive_Index = 38;                               
                          break;
                        }
                        case 38: // Set servo to starting position in increments (case 32-40)
                        {
                          Bot.ToPosition("S1", 700);
                          uc_Drive_Index = 39;                                
                          break;
                        }
                        case 39: // Set servo to starting position in increments (case 32-40)
                        {
                          Bot.ToPosition("S1", 650);
                          uc_Drive_Index = 40;
                          break;
                        }
                        case 40: // Set servo to starting position in increments (case 32-40)
                        {
                          Bot.ToPosition("S1", 600);
                          uc_Drive_Index = 41;
                          break;
                        }
                        case 41: // Drive backwards for 10 seconds
                        {
                          Bot.Reverse("D1", uc_Drive_Speed, uc_Drive_Speed);
                          if((millis()-prevTime) >= 10000){ // Stop after 10 seconds
                            Bot.Stop("D1");
                            uc_Drive_Index = 42;
                          }
                          break;
                        }
                        case 42: // Signify that task is complete
                        {
                          ui_Robot_Mode_Index = 2;
                          break;
                        }
                        
                     }
                  }
               }
               else                                                           // Stop when motors are disabled
               {  
                  Bot.Stop("D1");
               }
            }
            break;
         } 
                   
         case 2: //Change colour of LED to signify completed task
         {
              SmartLEDs.setPixelColor(0, ui_Mode_Indicator[4]);         // Set pixel colors to = mode 
              SmartLEDs.show();
            
            break;
         } 
      }

      // Update brightness of heartbeat display on SmartLED
      ul_Display_Time++;                                                      // Count milliseconds
      if(ul_Display_Time > ci_Display_Update)                                 // When display update period has passed
      {
         ul_Display_Time = 0;                                                 // Reset display counter
         LEDBrightnessIndex++;                                                // Shift to next brightness level
         if(LEDBrightnessIndex > sizeof(LEDBrightnessLevels))                 // If all defined levels have been used
         {
            LEDBrightnessIndex = 0;                                           // Reset to starting brightness
         }
         SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);    // Set brightness of heartbeat LED
         Indicator();                                                         // Update LED
      }
   }
}   

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator()
{
  SmartLEDs.setPixelColor(0, ui_Mode_Indicator[ui_Robot_Mode_Index]);         // Set pixel colors to = mode 
  SmartLEDs.show();                                                           // Send the updated pixel colors to the hardware
}
