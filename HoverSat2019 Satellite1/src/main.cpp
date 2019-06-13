//------------------------------------------------------------------//
//Supported MCU:   ESP32 (M5Stack)
//File Contents:   HoverSat Satellite1
//Version number:  Ver.1.1
//Date:            2019.06.14
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
#include <EEPROM.h>
#include "utility/MPU9250.h"
#include "BluetoothSerial.h"


//Define
//------------------------------------------------------------------//
#define   TIMER_INTERRUPT     10      // ms
#define   LCD
#define   STEPMOTOR_I2C_ADDR  0x70
#define   STEP_PER_LENGTH     0.575  // 230 / 400 
#define   ONE_ROTATION_LENGTH 230
// #define  STEPMOTOR_I2C_ADDR 0x71

#define BufferRecords 16
#define STEPPER_BUFFER  80



//Global
//------------------------------------------------------------------//
int     pattern = 0;
int     tx_pattern = 0;
int     rx_pattern = 0;
int     rx_val = 0;
bool    hover_flag = false;
bool    log_flag = false;
int     cnt10 = 0;

unsigned long time_ms;
unsigned long time_stepper = 0;
unsigned long time_buff = 0;
unsigned char current_time = 0; 
unsigned char old_time = 0;  

byte    counter;
char charBuf[100];
char charBuf2[100];
long  abslength = 0;
boolean inc_flag = false;
long steps;
float speed;
boolean hasData = false;
String label = "Tick";
static const int Limit1Pin = 17;
static const int Limit2Pin = 34;
int  Limit1State = 1;
int  Limit2State = 1;
const int stepper_enable = 0;
char  stepper_enable_status = 1;

// progress
float  current_length;
float  current_speed;
float  current_accel;
float  old_length=0;
char stepper_pattern=10;

BluetoothSerial bts;
String  bts_rx;
char bts_rx_buffer[16];
int bts_index = 0;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Buffalo-G-0CBA";
char pass[] = "hh4aexcxesasx";

//char ssid[] = "Macaw";
//char pass[] = "1234567890";

// Time
char ntpServer[] = "ntp.jst.mfeed.ad.jp";
const long gmtOffset_sec = 9 * 3600;
const int  daylightOffset_sec = 0;
struct tm timeinfo;
String dateStr;
String timeStr;

File file;
String fname_buff;
const char* fname;

String accel_buff;
const char* accel_out;

typedef struct {
    String  log_time;
    int     log_pattern;
    String  log_time_ms;
    float   log_length;
    float   log_speed;
    float   log_accel;
} RecordType;

static RecordType buffer[2][BufferRecords];
static volatile int writeBank = 0;
static volatile int bufferIndex[2] = {0, 0};


// MPU9250
MPU9250 IMU; 

// DuctedFan
static const int DuctedFanPin = 15;
Servo DuctedFan;

// Timer Interrupt
volatile int interruptCounter;
volatile int interruptCounterS;
int totalInterruptCounter;
int iTimer10;


hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Parameters
unsigned char hover_val = 0;
unsigned int ex_length = 2000;
unsigned int ex_speed = 200;
unsigned int ex_accel = 5;
unsigned char wait = 5;
unsigned char limit_flag = 1;


//Global
//------------------------------------------------------------------//
void IRAM_ATTR onTimer(void);
void SendByte(byte addr, byte b);
void SendCommand(byte addr, char *c);
void Timer_Interrupt( void );
void stepper(long ex_length, int ex_speed, int ex_accel);
void getTimeFromNTP(void);
void getTime(void);
void writeData(void);
void writeDataInitial(void);
void ReceiveStepperData( void );
void bluetooth_rx(void);
void bluetooth_tx(void);
void eeprom_write(void);
void eeprom_read(void);


//Setup
//------------------------------------------------------------------//
void setup() {

  M5.begin();
  Wire.begin();
  EEPROM.begin(128);
  SD.begin(4, SPI, 24000000, "/sd");
  M5.Lcd.clear();
  M5.Lcd.drawJpgFile(SD, "/Image/Picture.jpg");
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(88, 160);
  M5.Lcd.println("HoverSat");
  M5.Lcd.setCursor(82, 200);
  M5.Lcd.println("Satellite1");
  
  delay(1000);

  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(GREEN ,BLACK);
  M5.Lcd.fillScreen(BLACK);

  bts.begin("M5Stack Satellite1");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
  }

  eeprom_read();

  // timeSet
  getTimeFromNTP();
  getTime();
  fname_buff  = "/log/Satellite1_log_"
              +(String)(timeinfo.tm_year + 1900)
              +"_"+(String)(timeinfo.tm_mon + 1)
              +"_"+(String)timeinfo.tm_mday
              +"_"+(String)timeinfo.tm_hour
              +"_"+(String)timeinfo.tm_min
              +".csv";
  fname = fname_buff.c_str();

  pinMode(Limit1Pin, INPUT);
  pinMode(Limit2Pin, INPUT);
  pinMode(stepper_enable, INPUT);

  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer);

  SendCommand(STEPMOTOR_I2C_ADDR, "$0=10.8696"); // step/mm
  SendCommand(STEPMOTOR_I2C_ADDR, "$1=10.8696"); // step/mm
  SendCommand(STEPMOTOR_I2C_ADDR, "$2=10.8696"); // step/mm
  //SendCommand(STEPMOTOR_I2C_ADDR, "$8=200"); // Accel

  file = SD.open(fname, FILE_APPEND);
  if( !file ) {
    M5.Lcd.setCursor(0, 160);
    M5.Lcd.println("Failed to open sd");
  } else {
    file.print("NTP");
    file.print(",");
    file.print("Pattern");
    file.print(",");
    file.print("Time");
    file.print(",");
    file.print("Length");
    file.print(",");
    file.print("Speed");
    file.print(",");
    file.print("Accel");
    file.println(",");
    file.close();
  }


}




//Main
//------------------------------------------------------------------//
void loop() {

  Timer_Interrupt();
  //ReceiveStepperData();
  bluetooth_rx();
  bluetooth_tx();

  int readBank = !writeBank;

  if (bufferIndex[readBank] >= BufferRecords) {
    static RecordType temp[BufferRecords];

    memcpy(temp, buffer[readBank], sizeof(temp));
    bufferIndex[readBank] = 0;
    file = SD.open(fname, FILE_APPEND);
    for (int i = 0; i < BufferRecords; i++) {
        file.print(temp[i].log_time);
        file.print(",");
        file.print(temp[i].log_pattern);
        file.print(",");
        file.print(temp[i].log_time_ms);
        file.print(",");
        file.print(temp[i].log_length);
        file.print(",");
        file.print(temp[i].log_speed);
        file.print(",");
        file.print(temp[i].log_accel);
        file.println(",");
    }
    file.close();
  }

  switch (pattern) {
    case 0:
      break;

    case 11:    
      stepper( ex_length, ex_speed, ex_accel );
      pattern = 12;
      cnt10 = 0;
      break;

    case 12:
      if( cnt10 >= 100 ) {
        pattern = 13;
      }
      break;

    case 13:
      if( stepper_enable_status==1 ) {
        pattern = 0;
      }
      break;
    
    

    case 21:
      //( length, speed, accel )
      stepper( ex_length*-1-1000, ex_speed, ex_accel );
      pattern = 22;
      cnt10 = 0;
      break;

    case 22:
      if( cnt10 >= 100 ) {
        pattern = 13;
      }
      break;

    case 23:
      if( stepper_enable_status==1 ) {
        pattern = 0;
      }
      break;
    


    // CountDown
    case 111:    
      if( current_time >= 52  ) {
        cnt10 = 0;
        pattern = 113;      
        hover_flag = true;
        M5.Lcd.clear();
        DuctedFan.attach(DuctedFanPin);
        DuctedFan.write(0);
        break;
      }
      bts.println( 60 - current_time );
      break;

    case 112:    
      if( current_time < 1 ) {
        pattern = 111;
        break;
      }
      bts.println( 60 - current_time + 60 );
      break;

    case 113:    
      if( cnt10 >= 300 ) {
        DuctedFan.write(hover_val);
        bts.println(" - Start within 5 seconds -");
        pattern = 114;
        cnt10 = 0;
        break;
      }    
      bts.println( 60 - current_time );
      break;

    case 114:    
      if( cnt10 >= 500 ) {
        time_buff = millis();
        pattern = 115;
        cnt10 = 0;
        bts.println( "\n - Sequence start -" );
        break;
      }        
      break;

    case 115:   
      time_stepper = time_ms;
      stepper_pattern = 0;
      inc_flag = true; 
      stepper( ex_length, ex_speed, ex_accel );
      pattern = 116;
      tx_pattern = 11;
      log_flag = true;
      cnt10 = 0;
      break;

    case 116:   
      if( cnt10 >= 100 ) {
        pattern = 117;
        break;
      }
      break;

    case 117:   
      if( stepper_enable_status==1 ) {
        pattern = 118;
        cnt10 = 0;
        break;
      }
      break;

    case 118:   
      if( cnt10 >= wait*100 ) {
        pattern = 119;
        time_stepper = time_ms;
        stepper_pattern = 4;
        break;
      }
      break;

    case 119:
      //( length, speed, accel )
      inc_flag = true;
      stepper( ex_length*-1-1000, ex_speed, ex_accel  );
      pattern = 120;
      cnt10 = 0;
      break;

    case 120:   
      if( cnt10 >= 100 ) {
        pattern = 121;
        break;
      }
      break;

    case 121:
      if( stepper_enable_status==1 ) {
        pattern = 0;
        break;
      }
      break;


    
  }


  // Send Data to Module.
  while (Serial.available() > 0) {
    int inByte = Serial.read();
    SendByte(STEPMOTOR_I2C_ADDR, inByte);
  }

      
  // Button Control
  M5.update();
  if (M5.BtnA.wasPressed()) {
    hover_flag = !hover_flag;
    // Hover Control
    if(hover_flag) {
      M5.Lcd.clear();
      DuctedFan.attach(DuctedFanPin);
      DuctedFan.write(0);
      delay(3000);
      DuctedFan.write(hover_val);
    } else {
      M5.Lcd.clear();
      DuctedFan.detach();
    } 
  } else if (M5.BtnB.wasPressed() && pattern == 0) {      
    inc_flag = true;
    pattern = 11;
  } else if (M5.BtnC.wasPressed() && pattern == 0) {    
    inc_flag = true;
    pattern = 21;
  }

  if( limit_flag == 1 ) {
    if(Limit1State==0 && Limit2State==0 ) {
      if(pattern == 13 || pattern == 23 || pattern == 120) {
        //SendByte(STEPMOTOR_I2C_ADDR, '!');
        SendByte(STEPMOTOR_I2C_ADDR, 0x18);
        delay(1000);
        SendCommand(STEPMOTOR_I2C_ADDR, "$X");
        pattern = 0;
        stepper_pattern = 10;
        current_accel = 0;
        current_speed = 0;
        current_length = 0;
      }
    }
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
    time_ms = millis()-time_buff;
    
    getTime();

    if (bufferIndex[writeBank] < BufferRecords && log_flag) {
      RecordType* rp = &buffer[writeBank][bufferIndex[writeBank]];
      rp->log_time = timeStr;
      rp->log_pattern = pattern;
      rp->log_time_ms = time_ms;
      rp->log_length = current_length;
      rp->log_speed = current_speed;
      rp->log_accel = current_accel;
      if (++bufferIndex[writeBank] >= BufferRecords) {
          writeBank = !writeBank;
      }      
    }

    Limit1State = digitalRead(Limit1Pin);
    Limit2State = digitalRead(Limit2Pin);
    stepper_enable_status = digitalRead(stepper_enable);

    if( pattern > 115 && pattern <= 121 ) {
      switch( stepper_pattern ) {
      case 0:
        current_accel = ex_accel;
        current_speed = ex_accel * (float(time_ms)/1000);
        current_length = 0.5 * ex_accel * (float(time_ms)/1000) * (float(time_ms)/1000);
        if( time_ms >= float(ex_speed/ex_accel)*1000 ) {
          stepper_pattern = 1;
          time_stepper = time_ms;
          old_length = current_length;
          break;
        }
        break;

      case 1:
        current_accel = 0;
        current_speed = ex_speed;
        current_length = old_length + ex_speed * ((float(time_ms)-float(time_stepper))/1000);
        if( ex_length - current_length <= old_length ) {
          time_stepper = time_ms;
          old_length = current_length;
          stepper_pattern = 2;
          break;
        }
        break;

      case 2:
        current_accel = ex_accel;
        current_speed = ex_speed - ex_accel * ((float(time_ms)-float(time_stepper))/1000);
        current_length = old_length + ex_speed * (float(time_ms)-float(time_stepper))/1000 - 0.5 * ex_accel * ((float(time_ms)-float(time_stepper))/1000) * ((float(time_ms)-float(time_stepper))/1000);
        if( time_ms - time_stepper >= float(ex_speed/ex_accel)*1000 ) {
          current_length = ex_length;
          current_speed = 0;
          current_accel = 0;
          stepper_pattern = 3;
          break;
        }
        break;

      case 3:
        break;

      case 4:
        current_accel = ex_accel;
        current_speed = ex_accel * (float(time_ms)-float(time_stepper))/1000;
        current_length = ex_length - 0.5 * ex_accel * ((float(time_ms)-float(time_stepper))/1000) * ((float(time_ms)-float(time_stepper))/1000);
        if( float(time_ms)-float(time_stepper) >= float(ex_speed/ex_accel)*1000 ) {
          stepper_pattern = 5;
          time_stepper = time_ms;
          old_length = current_length;
          break;
        }
        break;

      case 5:
        current_accel = 0;
        current_speed = ex_speed;
        current_length = old_length - ex_speed * ((float(time_ms)-float(time_stepper))/1000);
        /*if( current_length <= ex_length - old_length ) {
          time_stepper = time_ms;
          old_length = current_length;
          stepper_pattern = 6;
          break;
        }*/
        break;

      case 6:
        current_accel = ex_accel;
        current_speed = ex_speed - ex_accel * ((float(time_ms)-float(time_stepper))/1000);
        current_length = old_length - ex_speed * (float(time_ms)-float(time_stepper))/1000 - 0.5 * ex_accel * ((float(time_ms)-float(time_stepper))/1000) * ((float(time_ms)-float(time_stepper))/1000);
        if( time_ms - time_stepper >= float(ex_speed/ex_accel)*1000 ) {
          current_length = 0;
          current_speed = 0;
          current_accel = 0;
          stepper_pattern = 10;
          break;
        }
        break;

      case 10: 
        break;

      }
    }



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


// EEPROM Write
//------------------------------------------------------------------// 
void eeprom_write(void) {
  EEPROM.write(0, hover_val);
  EEPROM.write(1, (ex_length & 0xFF));
  EEPROM.write(2, (ex_length>>8 & 0xFF));
  EEPROM.write(3, (ex_length>>16 & 0xFF));
  EEPROM.write(4, (ex_length>>24 & 0xFF));
  EEPROM.write(5, (ex_speed & 0xFF));
  EEPROM.write(6, (ex_speed>>8 & 0xFF));
  EEPROM.write(7, (ex_speed>>16 & 0xFF));
  EEPROM.write(8, (ex_speed>>24 & 0xFF));
  EEPROM.write(9, (ex_accel & 0xFF));
  EEPROM.write(10, (ex_accel>>8 & 0xFF));
  EEPROM.write(11, (ex_accel>>16 & 0xFF));
  EEPROM.write(12, (ex_accel>>24 & 0xFF));
  EEPROM.write(13, wait);
  EEPROM.write(14, limit_flag);
  EEPROM.commit();
}

// EEPROM Read
//------------------------------------------------------------------// 
void eeprom_read(void) {
    hover_val = EEPROM.read(0);
    ex_length = EEPROM.read(1) + (EEPROM.read(2)<<8) + (EEPROM.read(3)<<16) + (EEPROM.read(4)<<24);
    ex_speed = EEPROM.read(5) + (EEPROM.read(6)<<8) + (EEPROM.read(7)<<16) + (EEPROM.read(8)<<24);
    ex_accel = EEPROM.read(9) + (EEPROM.read(10)<<8) + (EEPROM.read(11)<<16) + (EEPROM.read(12)<<24);
    wait = EEPROM.read(13);
    limit_flag = EEPROM.read(14);
}


// Bluetooth RX
//------------------------------------------------------------------//
void bluetooth_rx(void) {

  while (bts.available() > 0) {
    bts_rx_buffer[bts_index] = bts.read();
    bts.write(bts_rx_buffer[bts_index]);
    
    if( bts_rx_buffer[bts_index] == '/' ) {
      bts.print("\n\n"); 
      if( tx_pattern == 1 ) {
        rx_pattern = atoi(bts_rx_buffer);
      } else {
        rx_val = atof(bts_rx_buffer);
      }
      bts_index = 0;
      
      switch ( rx_pattern ) {
          
      case 0:
        tx_pattern = 0;
        break;
        
      case 11:
        rx_pattern = 0;
        tx_pattern = 11;
        break;

      case 20:
        rx_pattern = 0;
        tx_pattern = 20;
        if( current_time >= 52 ) {   
          pattern = 112;
          break;
        } else {
          pattern = 111;
          break;
        }
        break;
        
      case 21:
        rx_pattern = 0;
        tx_pattern = 21;
        hover_flag = !hover_flag;
        if(hover_flag) {
          M5.Lcd.clear();
          DuctedFan.attach(DuctedFanPin);
          DuctedFan.write(0);
          delay(3000);
          DuctedFan.write(hover_val);
        } else {
          M5.Lcd.clear();
          DuctedFan.detach();
        }
        break;

      case 22:
        rx_pattern = 0;
        tx_pattern = 22;    
        inc_flag = true;
        pattern = 11;
        break;

      case 23:
        rx_pattern = 0;
        tx_pattern = 23;    
        inc_flag = true;
        pattern = 21;
        break;

      case 24:
        rx_pattern = 0;
        tx_pattern = 24;    
        //SendByte(STEPMOTOR_I2C_ADDR, '!');
        SendByte(STEPMOTOR_I2C_ADDR, 0x18);
        delay(1000);
        SendCommand(STEPMOTOR_I2C_ADDR, "$X");
        pattern = 0;
        break;

      case 31:
        tx_pattern = 31;
        rx_pattern = 41;
        break;

      case 41:
        hover_val = rx_val;
        eeprom_write();
        tx_pattern = 0;
        rx_pattern = 0;
        break;

      case 32:
        tx_pattern = 32;
        rx_pattern = 42;
        break;

      case 42:
        ex_length = rx_val;
        eeprom_write();
        tx_pattern = 0;
        rx_pattern = 0;
        break;

      case 33:
        tx_pattern = 33;
        rx_pattern = 43;
        break;

      case 43:
        ex_speed = rx_val;
        eeprom_write();
        tx_pattern = 0;
        rx_pattern = 0;
        break;

      case 34:
        tx_pattern = 34;
        rx_pattern = 44;
        break;

      case 44:
        ex_accel = rx_val;
        //accel_buff = "$8=" + (String)ex_accel;
        //accel_out = accel_buff.c_str();
        SendCommand(STEPMOTOR_I2C_ADDR, "$8="); // Accel
        SendByte(STEPMOTOR_I2C_ADDR, ex_accel/10);
        eeprom_write();
        tx_pattern = 0;
        rx_pattern = 0;
        break;

      case 35:
        tx_pattern = 35;
        rx_pattern = 45;
        break;

      case 45:
        wait = rx_val;
        eeprom_write();
        tx_pattern = 0;
        rx_pattern = 0;
        break;

      case 36:
        tx_pattern = 36;
        rx_pattern = 46;
        break;

      case 46:
        limit_flag = rx_val;
        eeprom_write();
        tx_pattern = 0;
        rx_pattern = 0;
        break;
          

      }
      
    } else {
        bts_index++;
    }
  }


}


// Bluetooth TX
//------------------------------------------------------------------//
void bluetooth_tx(void) {

    switch ( tx_pattern ) {
            
    case 0:
      delay(30);
      bts.print("\n\n\n\n\n\n");
      bts.print(" HoverSat Satellite1 (M5Stack version) "
                         "Test Program Ver1.00\n");
      bts.print("\n");
      bts.print(" Satellite control\n");
      bts.print(" 11 : Telemetry\n");
      bts.print(" 12 : Read log\n");
      bts.print("\n");
      bts.print(" 20 : Sequence Control\n");
      bts.print(" 21 : Start/Stop Hovering\n");
      bts.print(" 22 : Start Extruding\n");
      bts.print(" 23 : Start Winding\n");
      bts.print(" 24 : Pause\n");
      bts.print("\n");
      bts.print(" Set parameters  [Current val]\n");
      bts.print(" 31 : DuctedFan Output [");
      bts.print(hover_val);
      bts.print("%]\n");
      bts.print(" 32 : Extension length [");
      bts.print(ex_length);
      bts.print("mm]\n");
      bts.print(" 33 : Extension Speed [");
      bts.print(ex_speed);
      bts.print("mm/s]\n");
      bts.print(" 34 : Extension Accel [");
      bts.print(ex_accel);
      bts.print("mm/s^2]\n");
      bts.print(" 35 : Sequence Wait [");
      bts.print(wait);
      bts.print("s]\n");
      bts.print(" 36 : LimitSwitch Enable [");
      bts.print(limit_flag);
      bts.print("]\n");
      
      bts.print("\n");
      bts.print(" Please enter 11 to 35  ");
      
      tx_pattern = 1;
      break;
        
    case 1: 
      break;
        
    case 2:
      break;
        
    case 11:
      bts.print(time_ms);
      bts.print(", ");
      bts.print(stepper_pattern);
      bts.print(", ");
      bts.print(current_length);
      bts.print(", ");
      bts.print(current_speed);
      bts.print(", ");
      bts.println(current_accel);
      break;

    case 20:
      bts.print(" Starting Sequence...\n");
      tx_pattern = 1;
      break;

    case 21:
      if(hover_flag) {
        bts.print(" Start Hovering...\n");
      } else {
        bts.print(" Stop Hovering...\n");
      }
      delay(1000);
      tx_pattern = 0;
      break;

    case 22:
      bts.print(" Start Extruding...\n");
      tx_pattern = 11;
      break;

    case 23:
      bts.print(" Start Winding...\n");
      tx_pattern = 11;
      break;

    case 24:
      bts.print(" Pause...\n");
      tx_pattern = 11;
      break;

              
    case 31:
      bts.print(" DuctedFan Output [%] -");
      bts.print(" Please enter 0 to 100 ");
      tx_pattern = 2;
      break;

    case 32:
      bts.print(" Extension Length [mm] -");
      bts.print(" Please enter 0 to 10,000 ");
      tx_pattern = 2;
      break;

    case 33:
      bts.print(" Extension Speed [mm/s] -");
      bts.print(" Please enter 0 to 500 ");
      tx_pattern = 2;
      break;

    case 34:
      bts.print(" Extension Accel [mm/s^2] -");
      bts.print(" Please enter 1 to 50 ");
      tx_pattern = 2;
      break;

    case 35:
      bts.print(" Sequence Wait [s] -");
      bts.print(" Please enter 0 to 255 ");
      tx_pattern = 2;
      break;

    case 36:
      bts.print(" LimitSwitch Enable -");
      bts.print(" Please enter 0 or 1 ");
      tx_pattern = 2;
      break;
                 
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
void stepper( long ex_length, int ex_speed, int ex_accel ) {
  String command;
  String command2;

  memset( charBuf , '\0' , 100 );
  memset( charBuf2 , '\0' , 100 );

  if( inc_flag ) {
    abslength = abslength + ex_length;
    inc_flag = false;
  }
  steps =  abslength;
  speed = ex_speed * 100;

  command2.concat("$8=");
  command2.concat(String(ex_accel*2));
  command2.toCharArray(charBuf2, 100);
  SendCommand(STEPMOTOR_I2C_ADDR, charBuf2);
  
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
  //Serial.println(charBuf);

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
/*void ReceiveStepperData( void ) {

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

}*/


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
  current_time = timeinfo.tm_sec;
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

