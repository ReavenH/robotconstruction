// WIFI settings in app_httpd.cpp

// OLED screen display.
// === PAGE 1 ===
// Row1: [WIFI_MODE]:IP ADDRESS
//       [WIFI_MODE]: 1 as [AP] mode, it will not connect other wifi.
//                    2 as [STA] mode, it will connect to known wifi.
// Row2: [RSSI]
// Row3: [STATUS] [A] [B] [C] [D]
//       [A]: 1 as forward. -1 as backward.
//       [B]: 1 as turn right. -1 as turn left.
//       [C]: 0 as not in the debugMode. Robot can be controled.
//            1 as in the debugMode. You can debug the servos.
//       [D]: 0 as not in any function mode. Robot can be controled to move around.
//            1 as in steady mode. keep balancing.
//            2 as stayLow action.
//            3 as handshake action.
//            4 as jump action.
//            5, 6 and 7 as ActionA, ActionB and ActionC.
//            8 as servos moving to initPos, initPos is the middle angle for servos.
//            9 as servos moving to middlePos, middlePos is the middle angle for program.
// Row4: [BATTERY]
// === PAGE 2 ===
// [SHOW] DebugMode via wire config.
// [ . . . o o ]  LED G21 G15 G12 3V3
// [ . . . . . ]  TX  RX  GND  5V  5V
//    <SWITCH>
extern IPAddress IP_ADDRESS = (0, 0, 0, 0);
extern byte WIFI_MODE = 0; // select WIFI_MODE in app_httpd.cpp
extern void getWifiStatus();
extern int WIFI_RSSI = 0;

// gait type ctrl
// 0: simpleGait(DiagonalGait).
// 1: triangularGait.
extern int GAIT_TYPE = 0;

int CODE_DEBUG = 0;


// ctrl interface.
// refer to OLED screen display for more detail information.
extern int moveFB = 0;
extern int moveLR = 0;
extern int debugMode = 0;
extern int funcMode  = 0;
float gestureUD = 0;
float gestureLR = 0;
float gestureOffSetMax = 15;
float gestureSpeed = 2;
int STAND_STILL = 0;

const char* UPPER_IP = "";
// const char* UPPER_RASP4B_IP = "";
String UPPER_RASP4B_IP = "0.0.0.0";
String UPPER_RASP4B_SSID = "no wifi";
int UPPER_TYPE = 0;
unsigned long LAST_JSON_SEND;
int JSON_SEND_INTERVAL;


// import libraries.
#include "InitConfig.h"
#include "ServoCtrl.h"
#include "PreferencesConfig.h"
#include <ArduinoJson.h>
// NEW by Z. Huang: include math library
#include <math.h>

StaticJsonDocument<200> docReceive;
StaticJsonDocument<100> docSend;
TaskHandle_t threadings;

// NEW by Z. Huang: define the thread handler for sending IMU data
TaskHandle_t handle_send_IMU_data;
// NEW by Z. Huang: define the thread handler for calculating the pose
TaskHandle_t handle_calculate_pose;

// NEW by Z. Huang: define the global flag for deciding whether to start / stop sending IMU data
// stop_sending = true -> does not send IMU data to RPi;
// default: true.
bool stop_sending = false;

// NEW by Z. Huang: define the global flag for deciding whether to start calculating the IMU data
// stop_calculating_imu = true -> does not update or calculate the pose
bool stop_calculating_imu = true;
// whether to use built-in pitch and roll algorithm 
bool use_default_pitch_roll_algo = false;

// NEW by Z. Huang: define the global flag of whether to print the readings through Serial
// will not print results if connected to Rpi, default: false.
bool print_debug = true;

// placeHolders.
void webServerInit();

// NEW by Z. Huang: global flag for whether the sendIMU2Rpi is initiallized for the first time.
bool have_never_sent = true;
// NEW by Z. Huang: initialize the global IMU data to send.
unsigned int current_stamp = 0, last_stamp = 0, last_stamp_sent = 0;
float pitch_a = 0.0, pitch_g = 0.0, roll_a = 0.0, roll_g = 0.0, mag_x = 0.0, mag_y = 0.0;
float dt_send, yaw_send, pitch_send, roll_send, x_send, y_send, z_send;

// NEW by Z. Huang: define a function sendIMU2Rpi() to be an individial thread (task)
// detect whether to send: 
// initialized at setup(), never terminates
// if send -> if first time: initialize the calculation thread;
// if stop -> only terminate the sending module.
// TODO: fix the problem that calculating is around 5 times faster than Serial.print().
void sendIMU2Rpi(void *pvParameter){
  if(print_debug){
    // TODO: check if Serial.available()
    Serial.println("Running sendIMU2Rpi...");
  }
  // looping to detect whether to send
  while(true){
    if(stop_sending == false){
      if(have_never_sent == true){
        // TODO: initialize the calculation thread.
        stop_calculating_imu = false;
        xTaskCreate(&calculatePose, "calculatePose", 4000, NULL, 5, &handle_calculate_pose);
        have_never_sent = false;
      }
      // write the new data into serial, wait until channel available
      if(current_stamp != last_stamp_sent){  // TODO: consider if two stamps is NOT continuous
        docSend["stamp"] = current_stamp;
        docSend["dt"] = dt_send;
        docSend["yaw"] = yaw_send;
        docSend["pitch"] = pitch_send;
        docSend["roll"] = roll_send;
        docSend["x"] = x_send;
        docSend["y"] = y_send;
        docSend["z"] = z_send;
        /*
        while(Serial.available() != true){
          ;  // blank, wait until the channel is available
        }
        serializeJson(docSend, Serial);
        */
        last_stamp_sent = current_stamp;
        if(print_debug){
          /*
          while(Serial.available() != true){
            ;  // blank, wait until the channel is available
          }*/
          Serial.print("Calculated Pose: stamp[");
          Serial.print(last_stamp_sent);
          Serial.print("], dt[");
          Serial.print(delta_time*1000);
          Serial.print("]ms, Yaw[");
          Serial.print(yaw_send);
          Serial.print("]Deg, Pitch[");
          Serial.print(pitch_send);
          Serial.print("]Deg, Roll[");
          Serial.print(roll_send);
          Serial.print("]Deg, X[");
          Serial.print(x_send);
          Serial.print("]m, Y[");
          Serial.print(y_send);
          Serial.print("]m, Z[");
          Serial.print(z_send);
          Serial.print("]m, ");
          Serial.print("mag_x[");
          Serial.print(mag_x);
          Serial.print("], mag_y[");
          Serial.print(mag_y);
          Serial.print("], MAG_X[");
          Serial.print(MAG_X);
          Serial.print("], MAG_Y[");
          Serial.print(MAG_Y);
          Serial.print("], MAG_Z[");
          Serial.print(MAG_Z);
          Serial.println("].");
        }
      }
    }
  }
}

void calculatePose(void *pvParameter){
  if(print_debug){
    // TODO: check if Serial.available()
    Serial.println("Running calculatePose...");
  }
  // calculate the new pose using freshed IMU data
  while(true){
    if(use_default_pitch_roll_algo != true){
      // get the corrected accelerations
      accXYZUpdate();  // changes globals: corrected acc and gyro readings along the x, y, z axis, the raw magnetometer values, the delta_time and two time stamps.
      pitch_a = atan2(ACC_Y, ACC_Z)*180/M_PI;
      pitch_g += GYR_X*delta_time;  // variables updated from InitConfig.h
      pitch_send = (pitch_send + GYR_X*delta_time)*0.9 + pitch_a*0.1;  // complementary filter
      roll_a = atan2(ACC_X, ACC_Z)*180/M_PI;
      roll_g += GYR_Y*delta_time;
      roll_send = (roll_send + GYR_Y*delta_time)*0.9 + roll_a*0.1; // complementary filter
      mag_x = MAG_Y*cos(roll_send*M_PI/180) - MAG_Z*sin(roll_send*M_PI/180);
      mag_y = MAG_X*cos(pitch_send*M_PI/180) + MAG_Y*sin(roll_send*M_PI/180)*sin(pitch_send*M_PI/180);
      yaw_send = atan2(mag_y,mag_x)*180/M_PI;
      last_stamp = current_stamp;
      current_stamp += 1;
    }
    else if(use_default_pitch_roll_algo == true){
      accXYZUpdate(true);
      pitch_send = pitch_default;
      roll_send = roll_default;
      mag_x = MAG_Y*cos(roll_send*M_PI/180) - MAG_Z*sin(roll_send*M_PI/180);
      mag_y = MAG_X*cos(pitch_send*M_PI/180) + MAG_Y*sin(roll_send*M_PI/180)*sin(pitch_send*M_PI/180) + MAG_Z*cos(roll_send*M_PI/180)*sin(pitch_send*M_PI/180);
      yaw_send = atan2(mag_y,mag_x)*180/M_PI;
      last_stamp = current_stamp;
      current_stamp += 1;
    }
  }
}

// var(variable), val(value).                  
void serialCtrl(){
  if (Serial.available()){
    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(docReceive, Serial);

    if (err == DeserializationError::Ok){
      UPPER_TYPE = 1;
      docReceive["val"].as<int>();

      int val = docReceive["val"];

      if(docReceive["var"] == "funcMode"){
        debugMode = 0;
        gestureUD = 0;
        gestureLR = 0;
        if(val == 1){
          if(funcMode == 1){funcMode = 0;Serial.println("Steady OFF");}
          else if(funcMode == 0){funcMode = 1;Serial.println("Steady ON");}
        }
        else{
          funcMode = val;
          Serial.println(val);
        }
      }

      else if(docReceive["var"] == "move"){
        debugMode = 0;
        funcMode  = 0;
        digitalWrite(BUZZER, HIGH);
        switch(val){
          case 1: moveFB = 1; Serial.println("Forward");break;
          case 2: moveLR =-1; Serial.println("TurnLeft");break;
          case 3: moveFB = 0; Serial.println("FBStop");break;
          case 4: moveLR = 1; Serial.println("TurnRight");break;
          case 5: moveFB =-1; Serial.println("Backward");break;
          case 6: moveLR = 0; Serial.println("LRStop");break;
        }
      }

      else if(docReceive["var"] == "ges"){
        debugMode = 0;
        funcMode  = 0;
        switch(val){
          case 1: gestureUD += gestureSpeed;if(gestureUD > gestureOffSetMax){gestureUD = gestureOffSetMax;}break;
          case 2: gestureUD -= gestureSpeed;if(gestureUD <-gestureOffSetMax){gestureUD =-gestureOffSetMax;}break;
          case 3: break;
          case 4: gestureLR -= gestureSpeed;if(gestureLR <-gestureOffSetMax){gestureLR =-gestureOffSetMax;}break;
          case 5: gestureLR += gestureSpeed;if(gestureLR > gestureOffSetMax){gestureLR = gestureOffSetMax;}break;
          case 6: break;
        }
        pitchYawRollHeightCtrl(gestureUD, gestureLR, 0, 0);
      }

      else if(docReceive["var"] == "light"){
        switch(val){
          case 0: setSingleLED(0,matrix.Color(0, 0, 0));setSingleLED(1,matrix.Color(0, 0, 0));break;
          case 1: setSingleLED(0,matrix.Color(0, 32, 255));setSingleLED(1,matrix.Color(0, 32, 255));break;
          case 2: setSingleLED(0,matrix.Color(255, 32, 0));setSingleLED(1,matrix.Color(255, 32, 0));break;
          case 3: setSingleLED(0,matrix.Color(32, 255, 0));setSingleLED(1,matrix.Color(32, 255, 0));break;
          case 4: setSingleLED(0,matrix.Color(255, 255, 0));setSingleLED(1,matrix.Color(255, 255, 0));break;
          case 5: setSingleLED(0,matrix.Color(0, 255, 255));setSingleLED(1,matrix.Color(0, 255, 255));break;
          case 6: setSingleLED(0,matrix.Color(255, 0, 255));setSingleLED(1,matrix.Color(255, 0, 255));break;
          case 7: setSingleLED(0,matrix.Color(255, 64, 32));setSingleLED(1,matrix.Color(32, 64, 255));break;
        }
      }

      else if(docReceive["var"] == "buzzer"){
        switch(val){
          case 0: digitalWrite(BUZZER, HIGH);break;
          case 1: digitalWrite(BUZZER, LOW);break;
        }
      }

      else if(docReceive["var"] == "r4bIp"){
        UPPER_RASP4B_IP = docReceive["ip"].as<String>();
      }

      else if(docReceive["var"] == "r4bSsid"){
        UPPER_RASP4B_SSID = docReceive["ip"].as<String>();
      }

      else if(docReceive["var"] == "IMU_send"){
        // 1. send the calculated yaw, pitch, roll, x, y, z during the past period;
        // 2. start calculating the IMU data, set an origin value
        // data frame: {"stamp": unsigned int, "dt": float, "yaw": float, "pitch": float, "roll", float, "x": float, "y": float, "z": float}
        // sending the IMU data in a constant time rate.
        stop_sending = false;
      }

      else if(docReceive["var"] == "IMU_stop"){
        // stop sending, but the IMU calculation goes on.
        stop_sending = true;
      }

      else if(docReceive["var"] == "IMU_stop_cal"){
        // stop calculating IMU data.
        stop_calculating_imu = true;
      }
    }


      // else if(docReceive['var'] == "ip"){
      //     UPPER_IP = docReceive['ip'];
      // }
     

    else {
      while (Serial.available() > 0)
        Serial.read();
    }
  }
}


void jsonSend(){
  if(millis() - LAST_JSON_SEND > JSON_SEND_INTERVAL || millis() < LAST_JSON_SEND){
    docSend["vol"] = loadVoltage_V;
    serializeJson(docSend, Serial);
    LAST_JSON_SEND = millis();
  }
}


void robotThreadings(void *pvParameter){
  delay(3000);
  while(1){
    serialCtrl();
    delay(25);
  }
}


void threadingsInit(){
  xTaskCreate(&robotThreadings, "RobotThreadings", 4000, NULL, 5, &threadings);
}


void setup() {
  Wire.begin(S_SDA, S_SCL);
  Serial.begin(115200);

  // WIRE DEBUG INIT.
  wireDebugInit();
  
  // INA219 INIT.
  InitINA219();

  // BUZZER INIT.
  InitBuzzer();

  // RGB INIT
  InitRGB();

  // PCA9685 INIT.
  ServoSetup();

  // SSD1306 INIT.
  InitScreen();

  // EEPROM INIT.
  preferencesSetup();

  // Standup for ICM20948 calibrating.
  delay(100);
  setSingleLED(0,matrix.Color(0, 128, 255));
  setSingleLED(1,matrix.Color(0, 128, 255));
  standMassCenter(0, 0);GoalPosAll();delay(1000);
  setSingleLED(0,matrix.Color(255, 128, 0));
  setSingleLED(1,matrix.Color(255, 128, 0));
  delay(500);

  // ICM20948 INIT.
  InitICM20948();

  // WEBCTRL INIT. WIFI settings included.
  webServerInit();

  // RGB LEDs on.
  delay(500);
  setSingleLED(0,matrix.Color(0, 32, 255));
  setSingleLED(1,matrix.Color(255, 32, 0));

  // update data on screen.
  allDataUpdate();

  // threadings start.
  threadingsInit();

  // NEW: by Z. Huang, initialize the magnetometer
  myIMU.setMagOpMode(AK09916_CONT_MODE_20HZ);

  // NEW: by Z. Huang, activate new thread for detecting whether to send IMU data
  xTaskCreate(&sendIMU2Rpi, "send_IMU_data", 4000, NULL, 4, &handle_send_IMU_data);

}


// main loop.
void loop() {
  robotCtrl();
  allDataUpdate();  // fresh the screen
  wireDebugDetect();
}


// <<<<<<<<<<=== Devices on Board ===>>>>>>>>>>>>

// --- --- ---   --- --- ---   --- --- ---
// ICM20948 init. --- 9-axis sensor for motion tracking.
// InitICM20948();

// read and update the pitch, raw and roll data from ICM20948.
// accXYZUpdate();


// --- --- ---   --- --- ---   --- --- ---
// INA219 init. --- DC current/voltage sensor.
// InitINA219();

// read and update the voltage and current data from INA219.
// InaDataUpdate();


// --- --- ---   --- --- ---   --- --- ---
// RGB INIT. --- the 2 RGB LEDs in front of the robot. LED_NUM = 0, 1.
// InitRGB();

// control RGB LED. 0 <= R, G, B <= 255.
// setSingleLED(LED_NUM, matrix.Color(R, G, B));


// --- --- ---   --- --- ---   --- --- ---
// SSD1306 INIT. --- OLED Screen
// InitScreen();

// show the newest data on the OLED screen.
// for more information you can refer to <OLED screen display>.
// screenDataUpdate();


// --- --- ---   --- --- ---   --- --- ---
// BUZZER INIT. --- the device that make a sound.
// InitBuzzer();

// BUZZER on.
// digitalWrite(BUZZER, HIGH);

// BUZZER off.
// digitalWrite(BUZZER, LOW);


// --- --- ---   --- --- ---   --- --- ---
// PCA9685 INIT.
// ServoSetup();

// all servos move to the middle position of the servos.
// initPosAll();


// <<<<<<<<<<<<=== Servos/Legs/Motion Ctrl ===>>>>>>>>>>>>>>>

// --- --- ---   --- --- ---   --- --- ---
// all servos move to the middle position of the program. 
// the position that you have to debug it to make it moves to.
// middlePosAll();

// control a single servo by updating the angle data in GoalPWM[].
// once it is called, call GoalPosALL() to move all of the servos.
// goalPWMSet(servoNum, angleInput);

// all servos move to goal position(GoalPWM[]).
// GoalPosAll();

// Ctrl a single leg of WAVEGO, once it is called, call GoalPosALL() to move all of the servos.
// input (x,y) position and return angle alpha and angle beta.
//     O  X  O                 O ------ [BODY]      I(1)  ^  III(3)
//    /         .              |          |               |
//   /    |        O           |          |               |
//  O     y     .              |          |         II(2) ^  IV(4)
//   \.   |  .                 |          |
//    \.  .                    |          |
//     O  |                    |          |
//  .                          |          |
//   \.   |                    |          |
//    \-x-X                    X----z-----O
// ---------------------------------------------------------------
// x, y, z > 0
// singleLegCtrl(LEG_NUM, X, Y, Z);


// --- --- ---   --- --- ---   --- --- ---
// a simple gait to ctrl the robot.
// GlobalInput changes between 0-1.
// use directionAngle to ctrl the direction.
// once it is called, call GoalPosALL() to move all of the servos.
// simpleGait(GlobalInput, directionAngle);

// a triangular gait to ctrl the robot.
// GlobalInput changes between 0-1.
// use directionAngle to ctrl the direction.
// once it is called, call GoalPosALL() to move all of the servos.
// triangularGait(GlobalInput, directionAngle);


// --- --- ---   --- --- ---   --- --- ---
// Stand and adjust mass center.
//     ^
//     a
//     |
// <-b-M
// a,b > 0
// standMassCenter(aInput, bInput);


// --- --- ---   --- --- ---   --- --- ---
// ctrl pitch yaw and roll.
// pitchInput (-, +), if > 0, look up.
// yawInput   (-, +), if > 0, look right.
// rollInput  (-, +), if > 0, lean right.
// 75 < input < 115
// pitchYawRoll(pitchInput, yawInput, rollInput);


// --- --- ---   --- --- ---   --- --- ---
// balancing function.
// once it is called, call GoalPosALL() to move all of the servos.
// balancing();


// --- --- ---   --- --- ---   --- --- ---
// the default function to control robot.
// robotCtrl();


// <<<<<<<<<<<<<<<=== Save Data Permanently ===>>>>>>>>>>>>>>>>>>

// EEPROM INIT.
// preferencesSetup();

// save the current position of the servoNum in EEPROM.
// servoConfigSave(servoNum);

// read the saved middle position data of the servos from EEPROM.
// middleUpdate();