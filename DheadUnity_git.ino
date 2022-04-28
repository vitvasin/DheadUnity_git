
/*
//25-4-2022 Update Simplify version ->> remove unnecessary version
  // Not test on HW yet!
  
-remove scservo
-remove publisher feedback and  feedback2

*/
//rev3- This version adjust parameter with JOI unity code
//30-7-2021 -rev4- fix reverse pitch angle line 55 >>> swap 0 and 1023
//18-8-2021 -rev5- add manual calibration function >> home_adj();
// Need to use sync write >>tested
//Library declare
#include <Dynamixel2Arduino.h>
#include <DynamixelWorkbench.h>
#include <Tic.h>
#include "pitches.h"
//#include <SCServo.h>

DynamixelWorkbench dxl_wb;
int32_t goal_position[2] = {0, 1023};
const uint8_t handler_index = 0;

// ros library
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
ros::NodeHandle nh;
std_msgs::String str_msg;
std_msgs::Float32MultiArray vec6_msg;
std_msgs::Int8MultiArray setup_msg;

String rev_msg = "";
bool ROSFlag = false;
String inputString = "";
float inputVector[6];
float inputSetup[10];
bool stringComplete = false;
float a[6] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
int oroll, opitch, oyaw, fe, lb;
float initial_z = 0.0, prev_z = 0.0 , relative_z = 0.0;
int z = 0, Zgain = 10;
float return_fe = 0.0, return_lb = 0.0;
//SMS_STS sms_sts;
//const int handID = 1;

//Controller Declare
#if defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
// For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
// Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
#define DXL_SERIAL   Serial3
//  #define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#endif
//ros::Publisher feedback("Head_Feedback", &vec6_msg);
//ros::Publisher feedback2("Hand_Feedback", &str_msg);

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 1.0;
//initialize dynamixel motor
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
ParamForSyncWriteInst_t sync_write_param;

//initialize Tic
TicI2C tic;


using namespace ControlTableItem; //This namespace is required to use Control table item names (Dynamixel)

//String process variable
String incoming = "";
//String inputString = "";

String OutPut[8] = {"", "", "", "", "", "", "", ""};

//LED variable
int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };
bool LEDFlag[4] = {false, false, false, false};
bool inv_roll = false, inv_pitch = false, inv_yaw = false;
// Input Variable
int limitTop = 2, limitBot = 3;

//Timer variable
unsigned long period = 500; //time interval
unsigned long lasttime = 0; //

int minValue = 0, maxValue = 1023;
//bool handState = true;
void commandfromHMD( const std_msgs::Float32MultiArray& msg)
{

  ROSFlag = true;

  //  if (vec6_msg.data[20] != msg.data[20])
  //  {
  //
  //  }
  vec6_msg = msg;
  vec6_msg.data = msg.data;
  inputVector[0] = msg.data[0]; // flex
  inputVector[1] = msg.data[1]; //Camera.transform.position.y;
  inputVector[2] = msg.data[2]; //Camera.transform.position.z;
  inputVector[3] = msg.data[3]; // pitch Camera.transform.eulerAngles.x
  inputVector[4] = msg.data[4]; //yaw  Camera.transform.eulerAngles.y
  inputVector[5] = msg.data[5]; // roll Camera.transform.eulerAngles.z



  //Assign initial z
  //initial_z = abs(inputVector[2]) * Zgain;
  //relative_z = initial_z ;//- prev_z;
  // z = map(abs(inputVector[1]), 0, 100 , 0, 3000); // test with S20-15-30-B need to adjust when change
  z = abs(inputVector[1]);
  //
  fe = inputVector[0];
  lb = inputVector[2];



  // map angle from 0-360 to 0 to 180, 0 to -180
  if (inputVector[3] > 180) inputVector[3] -= 360.00;
  if (inputVector[4] > 180) inputVector[4] -= 360.00;
  if (inputVector[5] > 180) inputVector[5] -= 360.00;

  // map angle to motor (0 position start from 150)
  inputVector[3] += 150;
  inputVector[4] += 150;
  inputVector[5] += 150;

  // Convert angle to 0-1023
  if (inv_roll == false) oroll = map(inputVector[5], 0, 300, maxValue, 0); else oroll = map(inputVector[5], 0, 300, 0, maxValue);
  if (inv_pitch == false)opitch = map(inputVector[3], 0, 300, maxValue, 0); else opitch = map(inputVector[3], 0, 300, 0, maxValue);
  if (inv_yaw == false)oyaw = map(inputVector[4], 0, 300, maxValue, 0); else oyaw = map(inputVector[4], 0, 300, 0, maxValue);

/* calculation and return value
  float m1_r = (dxl.getPresentPosition(1) - 512) * 0.29;
  float m2_r = (dxl.getPresentPosition(2) - 512) * 0.29;
  //vec6_msg.data[0] = return_fe;//
  vec6_msg.data[0] = (m1_r - m2_r) / 4;
  vec6_msg.data[1] =  (m1_r + m2_r) / 2 * -1;

  //vec6_msg.data[1] = return_lb;//(dxl.getPresentPosition(2)-512) *-0.29;
  //vec6_msg.data[1] = ((dxl.getPresentPosition(2)-512) *0.29)-(2*fe);
  vec6_msg.data[2] = (dxl.getPresentPosition(3) - 512) * -0.29;
  vec6_msg.data[3] = (dxl.getPresentPosition(4) - 512) * 0.29;
  vec6_msg.data[4] = (dxl.getPresentPosition(5) - 512) * -0.29;
  */
/*
  str_msg.data = "callback";
  if (msg.data[20] == 1.0)
  {
    handState = true;
   // handControl(true);
    str_msg.data = "close";
  } else if (msg.data[20] == 0)
  {
    handState = false;
   // handControl(false);
    str_msg.data = "open";
  } else
  {
    str_msg.data = "error";
    // error
  }
*/

  stringComplete = true;

}


void setupfromHMD( const std_msgs::Int8MultiArray& st_msg)
{
  setup_msg = st_msg;
  setup_msg.data = st_msg.data;
  inputSetup[0] = st_msg.data[0]; // roll_inverse
  inputSetup[1] = st_msg.data[1]; // pitch_inverse
  inputSetup[2] = st_msg.data[2]; // yaw_inverse

  if (inputSetup[0] >= 1) inv_roll = true;
  else inv_roll = false;
  if (inputSetup[1] >= 1) inv_pitch = true;
  else inv_pitch = false;
  if (inputSetup[2] >= 1) inv_yaw = true;
  else inv_yaw = false;


}

ros::Subscriber<std_msgs::Float32MultiArray> sub("head_command", &commandfromHMD );

ros::Subscriber<std_msgs::Int8MultiArray> sub2("head_setup", &setupfromHMD );


// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};


void setup() {

  nh.initNode();
  //nh.advertise(feedback);
  //nh.advertise(feedback2);
  nh.subscribe(sub);
  nh.subscribe(sub2);
  // Set up I2C. >> for Tic
  Wire.begin();
  Wire.setClock(400000);
  delay(500);
  // Set up Serial Port >> for Connect with Unity
  Serial.begin(57600);
  Serial1.begin(1000000);
  //sms_sts.pSerial = &Serial1;
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
  pinMode(limitTop, INPUT_PULLUP);
  pinMode(limitBot, INPUT_PULLUP);
  pinMode(BDPIN_GPIO_18, INPUT_PULLUP);
  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);
  pinMode(BDPIN_DIP_SW_1, INPUT);
  pinMode(BDPIN_DIP_SW_2, INPUT);
  ////////////////////////////////////////////////////

  sync_write_param.addr = 30; //Goal position of DYNAMIXEL-AX series
  sync_write_param.length = 2;
  sync_write_param.xel[0].id = 1;
  sync_write_param.xel[1].id = 2;
  sync_write_param.id_count = 2;
  dxl.writeControlTableItem(RETURN_DELAY_TIME, 254, 0);






  // initialize Servo
  initializeServo();

  LEDRun();

  // Give the Tic some time to start up.
  delay(20);
/*
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

  tic.setTargetPosition(-22000);
  waitForPosition(-22000);


  tic.haltAndSetPosition(0);



  //hand Init
  handInit();
  */
  //initialized
  sound1();
  Serial.println("initialized");
}


void loop()
{
  //limitCheck(); // check limit switches >> turn motor torque off until sw1 is press and limit is not pressed. // not tested yet
  //home_adj(); // manual calibration from on-board Switch
  //KILLSW();

  if (ROSFlag)
  {
    //resetCommandTimeout();
    //waisttoMotor(fe, lb);
    dxl.setGoalPosition(3, oyaw);
    dxl.setGoalPosition(4, opitch);
    dxl.setGoalPosition(5, oroll );
    //handControl(handState);
    //control Motor (Linear Actuator)
   // tic.setTargetPosition(z);
    // prev_z = initial_z ;
    // readEncoder();


  }
 // feedback.publish( &vec6_msg );
 // feedback2.publish( &str_msg );
  //delay(1);
  ROSFlag = false;
  
  //////////////////////////////// test ros node
  nh.spinOnce();
  delay(1);
}


void readEncoder()
{
  //float[] mtr_angle[6];
  vec6_msg.data[0] = return_fe;//dxl.getPresentPosition(1);
  vec6_msg.data[1] = return_lb;//dxl.getPresentPosition(2);
  vec6_msg.data[2] = dxl.getPresentPosition(3);
  vec6_msg.data[3] = dxl.getPresentPosition(4);
  vec6_msg.data[4] = dxl.getPresentPosition(5);
  // mtr_angle[5] = dxl.getPresentPosition(1);
//  feedback.publish( &vec6_msg );
}




void initializeServo()
{
  dxl.torqueOff(254);
  dxl.setOperatingMode(254, OP_POSITION);
  delay(200);
  dxl.torqueOn(254);
  dxl.writeControlTableItem(MOVING_SPEED, 254, 50);
  dxl.setGoalPosition(254, 512); // set zero position of motor Center position(HOME)
  delay(5000);
  dxl.writeControlTableItem(MOVING_SPEED, 3, 800);
  dxl.writeControlTableItem(MOVING_SPEED, 4, 800);
  dxl.writeControlTableItem(MOVING_SPEED, 5, 800);
  waisttoMotor(0, 0);
}

void KILLSW()
{
  if (digitalRead(BDPIN_GPIO_18) == HIGH)
  {

    dxl.torqueOff(254);
    dxl.ledOn(254);
    while (1)
    {
      Serial.println("EMERTRIGGER please restart");
    }
  }
}


void home_adj()
{
  static int32_t pos1;
  static int32_t pos2;
  int c1 = dxl.getPresentPosition(1);
  int c2 = dxl.getPresentPosition(2);
  int adjust_offset = 10;
  if (digitalRead(BDPIN_DIP_SW_1) == LOW && digitalRead(BDPIN_DIP_SW_2) == LOW) // Lateral setup
  {
    if (digitalRead(BDPIN_PUSH_SW_1) == HIGH)
    {
      dxl.setGoalPosition(1, c1 + adjust_offset); // set zero position of motor Center position(HOME)
      dxl.setGoalPosition(2, c2 + adjust_offset); // set zero position of motor Center position(HOME)
      delay(10);
    }
    if (digitalRead(BDPIN_PUSH_SW_2) == HIGH)
    {
      dxl.setGoalPosition(1, c1 - adjust_offset); // set zero position of motor Center position(HOME)
      dxl.setGoalPosition(2, c2 - adjust_offset); // set zero position of motor Center position(HOME)
      delay(10);
    }
  } else if (digitalRead(BDPIN_DIP_SW_1) == LOW && digitalRead(BDPIN_DIP_SW_2) == HIGH)  // FE setup
  {
    if (digitalRead(BDPIN_PUSH_SW_1) == HIGH)
    {
      dxl.setGoalPosition(1, c1 + adjust_offset); // set zero position of motor Center position(HOME)
      dxl.setGoalPosition(2, c2 - adjust_offset); // set zero position of motor Center position(HOME)

      delay(10);
    }
    if (digitalRead(BDPIN_PUSH_SW_2) == HIGH)
    {
      dxl.setGoalPosition(1, c1 - adjust_offset); // set zero position of motor Center position(HOME)
      dxl.setGoalPosition(2, c2 + adjust_offset); // set zero position of motor Center position(HOME)
      delay(10);
    }
  } else if (digitalRead(BDPIN_DIP_SW_1) == HIGH && digitalRead(BDPIN_DIP_SW_2) == LOW)  // FE setup 45 deg
  {
    adjust_offset = map(8, 0, 300, 0, 1023);
    int c11, c21;


    if (digitalRead(BDPIN_PUSH_SW_1) == HIGH)
    {
      if (c1 + adjust_offset > 1023)
      {
        c11 = 1023;
      } else c11 = c1 + adjust_offset;
      if (c2 - adjust_offset < 0)
      {
        c21 = 0;
      } else c21 = c2 - adjust_offset;
      //pos1 = c11;
      //pos2 = c21;
      //pos1 = 1023;
      //pos2 = 0;
      //memcpy(sync_write_param.xel[0].data, &pos1, sizeof(pos1));
      //memcpy(sync_write_param.xel[1].data, &pos2, sizeof(pos2));

      //dxl.syncWrite(sync_write_param);
      //delay(500);
      dxl.writeControlTableItem(MOVING_SPEED, 1, 50);
      dxl.writeControlTableItem(MOVING_SPEED, 2, 50);
      /*

        dxl.setGoalPosition(1, 1023); // set zero position of motor Center position(HOME)
        //delay(1);
        dxl.setGoalPosition(2, 0); // set zero position of motor Center position(HOME)
        delay(5000);
        dxl.setGoalPosition(1, 0); // set zero position of motor Center position(HOME)
        //delay(1);
        dxl.setGoalPosition(2, 1023); // set zero position of motor Center position(HOME)
        delay(5000);
      */

      waisttoMotor(-90, 0);
      delay(7000);
      waisttoMotor(90, 0);
      delay(13000);
      waisttoMotor(0, 0);

      /*
        dxl.setGoalPosition(1, 1023);
        dxl.setGoalPosition(2, 0);
        delay(7000);
        dxl.setGoalPosition(1, 0);
        dxl.setGoalPosition(2, 1023);
        delay(13000);
        dxl.setGoalPosition(1, 512);
        dxl.setGoalPosition(2, 512);
        dxl.writeControlTableItem(MOVING_SPEED, 1, 128);
        dxl.writeControlTableItem(MOVING_SPEED, 2, 128);
      */
    }
    if (digitalRead(BDPIN_PUSH_SW_2) == HIGH)
    {
      if (c1 - adjust_offset < 0)
      {
        c11 = 0;
      } else c11 = c1 - adjust_offset;
      if (c2 + adjust_offset > 1023)
      {
        c21 = 1023;
      } else c21 = c2 + adjust_offset;
      /*
        pos1 = c11;
        pos2 = c21;
      */
      /*
            pos1 = 0;
            pos2 = 1023;
            memcpy(sync_write_param.xel[0].data, &pos1, sizeof(pos1));
            memcpy(sync_write_param.xel[1].data, &pos2, sizeof(pos2));
            // delay(3000);
            dxl.syncWrite(sync_write_param);
            delay(500);*/
      dxl.writeControlTableItem(MOVING_SPEED, 1, 50);
      dxl.writeControlTableItem(MOVING_SPEED, 2, 50);

      waisttoMotor(0, -90);
      delay(10000);
      waisttoMotor(0, 90);
      delay(10000);
      waisttoMotor(0, 0);


      dxl.writeControlTableItem(MOVING_SPEED, 1, 128);
      dxl.writeControlTableItem(MOVING_SPEED, 2, 128);
    }


  } else if (digitalRead(BDPIN_DIP_SW_1) == HIGH && digitalRead(BDPIN_DIP_SW_2) == HIGH)
  {
    adjust_offset = map(30, 0, 300, 0, 1023);
    int c11, c21;
    if (digitalRead(BDPIN_PUSH_SW_1) == HIGH)
    {
      if (c1 + adjust_offset > 1023)
      {
        c11 = 1023;
      } else c11 = c1 + adjust_offset;
      if (c2 + adjust_offset > 1023)
      {
        c21 = 1023;
      } else c21 = c2 + adjust_offset;
      pos1 = c11;
      pos2 = c21;
      memcpy(sync_write_param.xel[0].data, &pos1, sizeof(pos1));
      memcpy(sync_write_param.xel[1].data, &pos2, sizeof(pos2));
      // delay(3000);
      dxl.syncWrite(sync_write_param);
      delay(500);
    }
    if (digitalRead(BDPIN_PUSH_SW_2) == HIGH)
    {
      if (c1 - adjust_offset < 0)
      {
        c11 = 0;
      } else c11 = c1 - adjust_offset;
      if (c2 - adjust_offset < 0)
      {
        c21 = 0;
      } else c21 = c2 - adjust_offset;
      pos1 = c11;
      pos2 = c21;
      memcpy(sync_write_param.xel[0].data, &pos1, sizeof(pos1));
      memcpy(sync_write_param.xel[1].data, &pos2, sizeof(pos2));
      // delay(3000);
      dxl.syncWrite(sync_write_param);
      delay(500);
    }

  }
}


void limitCheck()
{
  if (limitTop == LOW)
  {
    LEDOn(2);
    while (true)
    {
      // clear input if limit is detected
      inputString = "";
      stringComplete = false;
      //
      // off motor torque
      dxl.torqueOff(1);
      dxl.torqueOff(2);
      //

      // check button 1 (onboard "Push sw1") and limit switch
      if ( (digitalRead(BDPIN_PUSH_SW_1) == HIGH) && (limitTop == HIGH) )
      {
        LEDOn(2);
        dxl.torqueOn(1);
        dxl.torqueOn(2);

        break;
      }

    }


  } else if (limitBot == LOW)
  {
    LEDOn(3);
    while (true)
    {
      // clear input if limit is detected
      inputString = "";
      stringComplete = false;
      //
      // off motor torque
      dxl.torqueOff(1);
      dxl.torqueOff(2);
      //

      // check button 1 (onboard "Push sw1") and limit switch
      if ( (digitalRead(BDPIN_PUSH_SW_1) == HIGH) && (limitBot == HIGH) )
      {
        LEDOn(3);
        dxl.torqueOn(1);
        dxl.torqueOn(2);

        break;
      }

    }

  }


}

void waisttoMotor(int fe, int lb) //input in angle
{
  //fe,lb is the real angle
  //calculate m1 and m2 angle
  int m1_angle = (2 * fe - lb) + 150; // fe=-90 lb =0 >>m1 =-180
  int m2_angle = (-1 * (2 * fe + lb)) + 150; //
  return_fe = (m1_angle - 150 + lb) / 2;
  return_lb = ((m2_angle - 150) * (-1) - lb) / 2;
  //  Serial.println("M1 =" + String(m1_angle)+ ", M2 = " + String(m2_angle));
  // map value to drive motor

  if (m1_angle >= 300.0) m1_angle = 300.0;
  if (m2_angle >= 300.0) m2_angle = 300.0;
  if (m2_angle <= 0 ) m2_angle = 0.0;
  if (m1_angle <= 0 ) m1_angle = 0.0;
  int m1_drive = map(m1_angle, 0, 300, 0, 1023);
  int m2_drive = map(m2_angle, 0, 300, 0, 1023);
  //Serial.println("D1 =" + String(m1_drive) + ", M2 = " + String(m2_drive));



  dxl.setGoalPosition(1, m1_drive);
  dxl.setGoalPosition(2, m2_drive);

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

/*
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
*/


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
//  unsigned long previousMillis = 0;
//  unsigned long currentMillis = millis();
//  const long interval = 5000;
  do
  {
    resetCommandTimeout();
//    if (currentMillis - previousMillis >= interval)
//    {
//      LEDOn(3);
//      break;
//    }
  } while (tic.getCurrentPosition() != targetPosition);

}



//hand
void handControl(bool state)
{
//  if (state == false) sms_sts.WritePosEx(handID, 1000, 0, 0);
//  else sms_sts.WritePosEx(handID, 4000, 0, 0);

}
void handInit()
{
  handControl(true);
  delay(1500);
  handControl(false);

}
