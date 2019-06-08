//------------------------------------------------------------------//
//Supported MCU:   ESP32 (M5Stack)
//File Contents:   HoverSat Satellite1
//Version number:  Ver.1.0
//Date:            2019.06.08
//------------------------------------------------------------------//
 
//This program supports the following boards:
//* M5Stack(Gray version)
 
//Include
//------------------------------------------------------------------//
#include <M5Stack.h>
#include <Servo.h>
#include <Wire.h>
#include <WiFi.h>
#include <time.h>
#include "utility/MPU9250.h"
#include "BluetoothSerial.h"


//Define
//------------------------------------------------------------------//
#define   TIMER_INTERRUPT     10      // ms
#define   LCD
#define   STEPMOTOR_I2C_ADDR  0x70
#define   STEP_PER_LENGTH     0.092   // 230 / 400 / 6.25
#define   STEP_PER_INPUT      0.001
#define   ONE_ROTATION_LENGTH 230
// #define  STEPMOTOR_I2C_ADDR 0x71

#define BufferRecords 128
#define STEPPER_BUFFER  80



//Global
//------------------------------------------------------------------//
int     pattern = 0;
bool    hover_flag = false;
int     cnt10 = 0;

byte    counter;
char charBuf[100];
long  abslength = 0;
boolean inc_flag = false;
long steps;
float speed;
String  stepper_status;
char status_buffer[STEPPER_BUFFER];
boolean hasData = false;
String label = "Tick";
static const int Limit1Pin = 17;
static const int Limit2Pin = 34;
int  Limit1State = 1;
int  Limit2State = 1;


BluetoothSerial bts;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Buffalo-G-0CBA";
char pass[] = "hh4aexcxesasx";

// Time
char ntpServer[] = "ntp.jst.mfeed.ad.jp";
const long gmtOffset_sec = 9 * 3600;
const int  daylightOffset_sec = 0;
struct tm timeinfo;
String dateStr;
String timeStr;

File file;
const char* fname = "/log/btnevent_log.tsv";

typedef struct {
    String  log_time;
    int     log_pattern;
    String  log_status;
} RecordType;

static RecordType buffer[2][BufferRecords];
static volatile int writeBank = 0;
static volatile int bufferIndex[2] = {0, 0};


// MPU9250
MPU9250 IMU; 

// DuctedFan
static const int DuctedFanPin = 15;
unsigned char hover_val = 0;
Servo DuctedFan;

// Timer Interrupt
volatile int interruptCounter;
volatile int interruptCounterS;
int totalInterruptCounter;
int iTimer10;


hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


//Global
//------------------------------------------------------------------//
void IRAM_ATTR onTimer(void);
void SendByte(byte addr, byte b);
void SendCommand(byte addr, char *c);
void Timer_Interrupt( void );
void stepper(long ex_length, float ex_speed);
void getTimeFromNTP(void);
void getTime(void);
void writeData(void);
void writeDataInitial(void);
void ReceiveStepperData( void );


//Setup
//------------------------------------------------------------------//
void setup() {

  M5.begin();
  Wire.begin();
  M5.Lcd.clear();
  M5.Lcd.drawJpgFile(SD, "/Image/Picture.jpg");
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(88, 160);
  M5.Lcd.println("HoverSat");
  M5.Lcd.setCursor(82, 200);
  M5.Lcd.println("Sattelite");
  
  delay(1000);

  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(GREEN ,BLACK);
  M5.Lcd.fillScreen(BLACK);

  bts.begin("M5Stack Mother");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
  }

  // timeSet
  getTimeFromNTP();

  pinMode(Limit1Pin, INPUT);
  pinMode(Limit2Pin, INPUT);
    

  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer);

  SendCommand(STEPMOTOR_I2C_ADDR, "$0=1000"); // mm per step
  SendCommand(STEPMOTOR_I2C_ADDR, "$1=1000"); // mm per step
  SendCommand(STEPMOTOR_I2C_ADDR, "$2=1000"); // mm per step
  SendCommand(STEPMOTOR_I2C_ADDR, "$8=0.5"); // Accel
  SendCommand(STEPMOTOR_I2C_ADDR, "$16=0"); // Hard Limit

//SendCommand(STEPMOTOR_I2C_ADDR, "G1 X99Y99Z99 F100"); // Accel


  
}




//Main
//------------------------------------------------------------------//
void loop() {

  Timer_Interrupt();
  getTime();
  ReceiveStepperData();

  if( (timeinfo.tm_hour == 18) &&  (timeinfo.tm_min == 3) && (timeinfo.tm_sec == 0) ) {
    pattern = 31;
  }

  int readBank = !writeBank;

  if (bufferIndex[readBank] >= BufferRecords) {
    static RecordType temp[BufferRecords];

    memcpy(temp, buffer[readBank], sizeof(temp));
    bufferIndex[readBank] = 0;
    file = SD.open(fname, FILE_APPEND);
    for (int i = 0; i < BufferRecords; i++) {
        file.print(temp[i].log_time);
        file.print(" ");
        file.print(temp[i].log_pattern);
        file.print(" ");
        //file.print(temp[i].log_status);
        file.println(" ");
    }
    file.close();
  }

  switch (pattern) {
    case 0:
      break;

    case 11:
    
      stepper( 5300, 170 );
      pattern = 12;
      cnt10 = 0;
      break;

    case 12:
      if( cnt10 >= 200 )
      pattern = 0;
      break;

    case 21:
      //( length, speed, accel )
      stepper( -5800, 170 );
      pattern = 22;
      cnt10 = 0;
      break;

    case 22:
      if( cnt10 >= 200 )
      pattern = 0;
      break;

    case 31:
      DuctedFan.attach(DuctedFanPin);
      DuctedFan.write(0);
      cnt10 = 0;
      pattern = 32;
      break;

    case 32:
      if( cnt10 >= 300 )
        pattern = 33;
      break;

    case 33:
      DuctedFan.write(50);
      pattern = 0;
      break;
    
  }


  bts.print(pattern);
  bts.print(",  ");
  bts.print(Limit1State);
  bts.println(",  ");
  //bts.print(stepper_status);
  //bts.println(", ");
  /*
  bts.println(hx711_data);
  */

  // Send Data to Module.
  while (Serial.available() > 0) {
    int inByte = Serial.read();
    SendByte(STEPMOTOR_I2C_ADDR, inByte);
  }

      
  // Button Control
  M5.update();
  if (M5.BtnA.pressedFor(700)) {
  hover_flag = !hover_flag;
  // Hover Control
    if(hover_flag) {
      M5.Lcd.clear();
      DuctedFan.attach(DuctedFanPin);
      DuctedFan.write(0);
    } else {
      M5.Lcd.clear();
      DuctedFan.detach();
      hover_val = 0;
    }
  } else if (M5.BtnA.wasPressed()) {
    if(hover_val<100) hover_val+=5;
    DuctedFan.write(hover_val);    
  } else if (M5.BtnB.wasPressed() && pattern == 0) {      
    inc_flag = true;
    pattern = 11;
  } else if (M5.BtnC.wasPressed() && pattern == 0) {    
    inc_flag = true;
    pattern = 21;
  }
  
  if(Limit1State==0 && Limit2State==0) {
    //SendByte(STEPMOTOR_I2C_ADDR, '!');
    SendByte(STEPMOTOR_I2C_ADDR, 0x18);
    delay(1000);
    SendCommand(STEPMOTOR_I2C_ADDR, "$X");
    pattern = 0;
  }

}


// Timer Interrupt
//------------------------------------------------------------------//
void Timer_Interrupt( void ){
  if (interruptCounter > 0) {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);

    cnt10++;

  if( pattern >= 11 ) {
    if (bufferIndex[writeBank] < BufferRecords) {
      RecordType* rp = &buffer[writeBank][bufferIndex[writeBank]];
      rp->log_time = timeStr;
      rp->log_pattern = pattern;
      //rp->log_status = status_buffer;
      if (++bufferIndex[writeBank] >= BufferRecords) {
          writeBank = !writeBank;
      }      
    }
  }

    Limit1State = digitalRead(Limit1Pin);
    Limit2State = digitalRead(Limit2Pin);




//    totalInterruptCounter++;

    iTimer10++;
    switch( iTimer10 ) {
    case 1:
      if(hover_flag) {
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(140, 105);
        M5.Lcd.println(hover_val);
      } else {
        M5.Lcd.setCursor(100, 105);
        M5.Lcd.println("Disable");
      }
    break;

    case 2:
      break;
    
    case 10:
      iTimer10 = 0;
      break;

    }

  }
}


// IRAM
//------------------------------------------------------------------//
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  portEXIT_CRITICAL_ISR(&timerMux);
 }


// Stepper (mm, mm/s)
//------------------------------------------------------------------//
void stepper( long ex_length, float ex_speed ) {
  String command;

  memset( charBuf , '\0' , 100 );

  if( inc_flag ) {
    abslength = abslength + ex_length;
    inc_flag = false;
  }
  steps =  abslength / STEP_PER_LENGTH * STEP_PER_INPUT;
  speed = ex_speed / 0.88412;
  
  command.concat("G1 ");
  command.concat("X");
  command.concat(String(steps));
  command.concat("Y");
  command.concat(String(steps));
  command.concat("Z");
  command.concat(String(steps));
  command.concat(" ");
  command.concat("F");
  command.concat(String(speed));
  command.toCharArray(charBuf, 100);
  SendCommand(STEPMOTOR_I2C_ADDR, charBuf);
  Serial.println(charBuf);

 }


//SendByte
//------------------------------------------------------------------//
void SendByte(byte addr, byte b) {
  Wire.beginTransmission(addr);
  Wire.write(b);
  Wire.endTransmission();
}

//SendCommand
//------------------------------------------------------------------//
void SendCommand(byte addr, char *c) {
  Wire.beginTransmission(addr);
  while ((*c) != 0) {
    Wire.write(*c);
    c++;
  }
  Wire.write(0x0d);
  Wire.write(0x0a);
  Wire.endTransmission();
}

//Receive Data From Module
//------------------------------------------------------------------//
void ReceiveStepperData( void ) {

  int index_stepper = 0;
  Wire.requestFrom(STEPMOTOR_I2C_ADDR, 1);
  while (Wire.available() > 0) {
    hasData = true;
    status_buffer[index_stepper] = Wire.read();
    index_stepper++;
    if (index_stepper >= STEPPER_BUFFER-1) {
      break;
    }
    delayMicroseconds(10); 
  }

  if (hasData == true) {
    //stepper_status = status_buffer;
    //label = status_buffer;
    //label.trim();
  } 
  //bts.print(label);

}


//Get Time From NTP
//------------------------------------------------------------------//
void getTimeFromNTP(void){
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  while (!getLocalTime(&timeinfo)) {
    delay(1000);
  }
}

//Get Convert Time
//------------------------------------------------------------------//
void getTime(void){
  getLocalTime(&timeinfo);
  dateStr = (String)(timeinfo.tm_year + 1900)
          + "/" + (String)(timeinfo.tm_mon + 1)
          + "/" + (String)timeinfo.tm_mday;
  timeStr = (String)timeinfo.tm_hour
          + ":" + (String)timeinfo.tm_min
          + ":" + (String)timeinfo.tm_sec;
}


//Write SD Initial Data
//------------------------------------------------------------------//
void writeDataInitial(void) {
  file = SD.open(fname, FILE_APPEND);
  file.println("Tether extension experiment");
  file.println("Parameters");
  file.println("Time, Pattern, Pattern");
  file.close();
}


//Write SD 
//------------------------------------------------------------------//
void writeData(void) {
  file = SD.open(fname, FILE_APPEND);
  file.println(timeStr + "," + pattern + "," + pattern);
  file.close();
}

