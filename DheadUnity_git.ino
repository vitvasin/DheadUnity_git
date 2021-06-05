//Library declare
#include <Dynamixel2Arduino.h>
#include <Tic.h>
#include "pitches.h"
//add to git

//Controller Declare
#if defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
// For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
// Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
#define DXL_SERIAL   Serial3
//  #define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#endif

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 1.0;
//initialize dynamixel motor
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//initialize Tic
TicI2C tic;


using namespace ControlTableItem; //This namespace is required to use Control table item names (Dynamixel)

//String process variable
String incoming = "";
String inputString = "";
bool stringComplete = false;
String OutPut[8] = {"", "", "", "", "", "", "", ""};

//LED variable
int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };
bool LEDFlag[4] = {false, false, false, false};

//Timer variable
unsigned long period = 500; //time interval
unsigned long lasttime = 0; //

void setup() {

  // Set up I2C. >> for Tic
  Wire.begin();
  Wire.setClock(400000);
  delay(500);
  // Set up Serial Port >> for Connect with Unity
  Serial.begin(57600);
   //while(!Serial);
   delay(500);
Serial.println("Serial Port Started at 115200");


  // Set Port baudrate to 1M. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);

  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
 delay(20);
  //Define pin mode
  pinMode(led_pin_user[0], OUTPUT);
  pinMode(led_pin_user[1], OUTPUT);
  pinMode(led_pin_user[2], OUTPUT);
  pinMode(led_pin_user[3], OUTPUT);


  // initialize Servo
  initializeServo();

  LEDRun();

  // Give the Tic some time to start up.
  delay(20);

  // Set the Tic's current position to 0, so that when we command
  // it to move later, it will move a predictable amount.
  tic.haltAndSetPosition(0);

  // Tells the Tic that it is OK to start driving the motor.  The
  // Tic's safe-start feature helps avoid unexpected, accidental
  // movement of the motor: if an error happens, the Tic will not
  // drive the motor again until it receives the Exit Safe Start
  // command.  The safe-start feature can be disbled in the Tic
  // Control Center.
  tic.exitSafeStart();

   tic.setTargetPosition(-10000);
   waitForPosition(-10000);


  tic.haltAndSetPosition(0);


  sound1();
  Serial.println("initialized");
}
int z = 0;

void loop()
{

  if (stringComplete)
  {
    inputString.trim();
    LEDController(); // For debug send "on1","on2","on3"
    //S,512,512,512,512,512,512,F >>>> default command;
    if (inputString[0] == 'S' && inputString[inputString.length() - 1 ] == 'F') //Check if Headder and Tail are correct then read and process the entire protocal
    {
      Spliter();
      //control Motor (Servo)
      dxl.setGoalPosition(1, OutPut[1].toInt());
      dxl.setGoalPosition(2, OutPut[2].toInt());
      dxl.setGoalPosition(3, OutPut[3].toInt());
      dxl.setGoalPosition(4, OutPut[4].toInt());
      dxl.setGoalPosition(5, OutPut[5].toInt());
      //control Motor (Linear Actuator)
      //Serial.println(OutPut[6]);
      z = map(OutPut[6].toInt(), 0, 1023, 0, 3000);
      tic.setTargetPosition(z);

    }
    if (inputString[0] == 'T') //Check if Headder and Tail are correct then read and process the entire protocal
    {
      Serial.println(tic.getErrorsOccurred());
        tic.clearDriverError();

    }


    // clear the string:
    delay(1);

    inputString = "";
    stringComplete = false;
  }

  /*
    if (millis() - lasttime > 10)
    {
      lasttime = millis();
      // send this every Period (500 ms).
      resetCommandTimeout();
    }
  */
  resetCommandTimeout();
 // delayWhileResettingCommandTimeout(1);
  //Serial.println(tic.getErrorsOccurred());
}

void initializeServo()
{
  // initialize
  for (int i = 1; i <= 5 ; i++)
  {

    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(i);
    dxl.setOperatingMode(i, OP_POSITION);
    dxl.torqueOn(i);
    // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
    dxl.writeControlTableItem(PROFILE_VELOCITY, i, 0);
    dxl.setGoalPosition(i, 512); // set zero position of motor Center position(HOME)
    delay(200);
  }

}







void LEDController()
{
  if (inputString == "on1")
  {
    LEDOn(0);
    //    Serial.println("On LED 1");
  }
  if (inputString == "on2")
  {
    LEDOn(1);
  }
  if (inputString == "on3")
  {
    LEDOn(2);
  }
  if (inputString == "on4")
  {
    LEDOn(3);
  }
  if (inputString == "Home")
  {
    Home();
  }

}

void LEDOn(int i)
{
  digitalWrite(led_pin_user[i], !digitalRead(led_pin_user[i]));
}

void writeString(String stringData) { // Used to serially push out a String with Serial.write()

  for (int i = 0; i < stringData.length(); i++)
  {
    Serial.write(stringData[i]);   // Push each char 1 by 1 on each loop pass
  }
  // Serial.flush();
  // delay(20);
}// end writeString

int orientation = 0;

void Home()
{
  getOrientation();
  if (orientation == 0)
  {

    for (int i = 1; i <= 5 ; i++)
    {
      dxl.setGoalPosition(i, 512); // set zero position of motor Center position(HOME)
      delay(200);
    }


  }

  tic.haltAndSetPosition(0);
  tic.exitSafeStart();
  tic.setTargetPosition(-10000);
  waitForPosition(-10000);


  tic.haltAndSetPosition(0);

}

void getOrientation()
{

}
void LEDRun()
{
  for ( int i = 0; i < 4; i++ )
  {
    digitalWrite(led_pin_user[i], LOW);
    delay(100);
  }
  for (int i = 0; i < 4; i++ )
  {
    digitalWrite(led_pin_user[i], HIGH);
    delay(100);
  }
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      // Serial.flush();
    }
  }
}



void Spliter()
{
  //String input = "123,456";
  int r = 0, t = 0;

  for (int i = 0; i < inputString.length(); i++)
  {
    if (inputString.charAt(i) == ',')
    {
      OutPut[t] = inputString.substring(r, i);
      r = (i + 1);
      t++;
    }
  }
}

void sound1()
{
   for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BDPIN_BUZZER, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(BDPIN_BUZZER);
  }
}


void resetCommandTimeout()
{
  tic.resetCommandTimeout();
}


void delayWhileResettingCommandTimeout(uint32_t ms)
{
  uint32_t start = millis();
  do
  {
    resetCommandTimeout();
  } while ((uint32_t)(millis() - start) <= ms);
}

void waitForPosition(int32_t targetPosition)
{
  do
  {
    resetCommandTimeout();
  } while (tic.getCurrentPosition() != targetPosition);
}
