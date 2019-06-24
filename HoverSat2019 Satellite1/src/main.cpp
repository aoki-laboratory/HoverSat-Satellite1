//------------------------------------------------------------------//
//Supported MCU:   ESP32 (M5Stack)
//File Contents:   HoverSat Satellite1
//Version number:  Ver.1.2
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

#define BufferRecords 32
#define STEPPER_BUFFER  80



//Global
//------------------------------------------------------------------//
int     pattern = 0;
int     tx_pattern = 0;
int     rx_pattern = 0;
int     rx_val = 0;
bool    hover_flag = false;
bool    log_flag = false;
bool    telemetry_flag = false;
int     cnt10 = 0;

unsigned long time_ms;
unsigned long time_stepper = 0;
unsigned long time_buff = 0;
unsigned long time_buff2 = 0;
unsigned char current_time = 0; 
unsigned char old_time = 0;  

byte    counter;
char charBuf[100];
char charBuf2[100];
long  abslength = 0;
boolean inc_flag = false;
long steps;
float velocity;
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
float  current_velocity;
float  current_accel;
float  old_length=0;
char stepper_pattern=10;

BluetoothSerial bts;
String  bts_rx;
char bts_rx_buffer[16];
int bts_index = 0;

//String ssid_buff;
//String pass_buff;
//const char* ssid;
//const char* pass;

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
    float   log_velocity;
    float   log_accel;
    float   log_ax;
    float   log_ay;
    float   log_az;
    float   log_gz;
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
unsigned int ex_velocity = 200;
unsigned int ex_accel = 5;
unsigned char wait = 5;
unsigned char limit_flag = 1;
unsigned char ssid_pattern = 0;



//Global
//------------------------------------------------------------------//
void IRAM_ATTR onTimer(void);
void SendByte(byte addr, byte b);
void SendCommand(byte addr, char *ci);
void Timer_Interrupt( void );
void stepper(long ex_length, int ex_velocity, int ex_accel);
void getTimeFromNTP(void);
void getTime(void);
void writeData(void);
void writeDataInitial(void);
void ReceiveStepperData( void );
void bluetooth_rx(void);
void bluetooth_tx(void);
void eeprom_write(void);
void eeprom_read(void);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t subAddress);


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

  eeprom_read();
  
  delay(1000);

  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(GREEN ,BLACK);
  M5.Lcd.fillScreen(BLACK);

  /*
  switch( ssid_pattern ) {
      case 0:
        ssid_buff = "Buffalo-G-0CBA";
        pass_buff = "hh4aexcxesasx";
        break;

      case 1:
        ssid_buff = "Macaw";
        pass_buff = "1234567890";
        break;
  }
  ssid = ssid_buff.c_str();
  pass = pass_buff.c_str();
  */

  Serial.begin(115200);
  bts.begin("M5Stack Satellite1");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
  }

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
    M5.Lcd.setCursor(5, 160);
    M5.Lcd.println("Failed to open sd");
  }

  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  delay(100);
  //IMU.initMPU9250();

  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | 0 << 3; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | 0 << 3; // Set full scale range for the accelerometer
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting


  
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
        file.print(temp[i].log_velocity);
        file.print(",");
        file.print(temp[i].log_accel);
        file.print(",");
        file.print(temp[i].log_ax);
        file.print(",");
        file.print(temp[i].log_ay);
        file.print(",");
        file.print(temp[i].log_az);
        file.print(",");
        file.print(temp[i].log_gz);
        file.println(",");
    }
    file.close();
  }

  if( telemetry_flag ) {
    bts.print(time_ms);
    bts.print(", ");
    bts.print(pattern);
    bts.print(", ");
    bts.print(current_length);
    bts.print(", ");
    bts.print(current_velocity);
    bts.print(", ");
    bts.print(current_accel);
    bts.print(", ");
    bts.print(IMU.ax);
    bts.print(", ");
    bts.print(IMU.ay);
    bts.print(", ");
    bts.println(IMU.gz);
    telemetry_flag = false;
  }

  switch (pattern) {
    case 0:
      break;

    case 11:    
      stepper( ex_length, ex_velocity, ex_accel );
      time_buff2 = millis();



      pattern = 12;
      break;

    case 12:
      if( millis() - time_buff2 >= 1000 ) {
        pattern = 13;
      }
      break;

    case 13:
      if( stepper_enable_status==1 ) {
        pattern = 0;
      }
      break;
    
    

    case 21:
      //( length, velocity, accel )
      stepper( ex_length*-1-1000, ex_velocity, ex_accel );
      pattern = 22;
      time_buff2 = millis();
      break;

    case 22:
      if( millis() - time_buff2 >= 1000 ) {
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
        time_buff2 = millis();
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
      if( millis() - time_buff2 >= 3000 ) {
        DuctedFan.write(hover_val);
        bts.println(" - Start within 5 seconds -");
        time_buff2 = millis();
        pattern = 114;
        break;
      }    
      bts.println( 60 - current_time );
      break;

    case 114:   
      if( millis() - time_buff2 >= 5000 ) {
        time_buff = millis();
        pattern = 115;
        bts.println( "\n - Sequence start -" );
        break;
      }        
      break;

    case 115:   
      time_stepper = time_ms;
      stepper_pattern = 0;
      inc_flag = true; 
      stepper( ex_length, ex_velocity, ex_accel );
      pattern = 116;
      //tx_pattern = 11;
      log_flag = true;
      time_buff2 = millis();
      break;

    case 116:   
      if( millis() - time_buff2 >= 1000 ) {
        pattern = 117;
        break;
      }
      break;

    case 117:   
      if( stepper_enable_status==1 ) {
        pattern = 118;
        time_buff2 = millis();
        break;
      }
      break;

    case 118:   
      if( millis() - time_buff2 >= wait*1000 ) {
        pattern = 119;
        time_stepper = time_ms;
        stepper_pattern = 4;
        break;
      }
      break;

    case 119:
      //( length, velocity, accel )
      inc_flag = true;
      stepper( ex_length*-1-1000, ex_velocity, ex_accel  );
      pattern = 120;
      time_buff2 = millis();
      break;

    case 120:   
      if( millis() - time_buff2 >= 1000 ) {
        pattern = 121;
        break;
      }
      break;

    case 121:
      if( stepper_enable_status==1 ) {
        time_buff2 = millis();
        pattern = 131;
        break;
      }
      break;

    case 131:
      if( millis() - time_buff2 >= 5000 ) {
        log_flag = false;
        pattern = 0;
        tx_pattern = 0;
        hover_flag = false;
        M5.Lcd.clear();
        DuctedFan.detach();
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
    Wire.begin();
    inc_flag = true;
    pattern = 11;
  } else if (M5.BtnC.wasPressed() && pattern == 0) {   
    Wire.begin(); 
    inc_flag = true;
    pattern = 21;
  }

  if( limit_flag == 1 ) {
    if(Limit1State==0 && Limit2State==0 ) {
      if(pattern == 13 || pattern == 23) {
        //SendByte(STEPMOTOR_I2C_ADDR, '!');
        SendByte(STEPMOTOR_I2C_ADDR, 0x18);
        delay(1000);
        SendCommand(STEPMOTOR_I2C_ADDR, "$X");
        pattern = 0;
      }
      if(pattern == 121) {
        //SendByte(STEPMOTOR_I2C_ADDR, '!');
        SendByte(STEPMOTOR_I2C_ADDR, 0x18);
        delay(1000);
        SendCommand(STEPMOTOR_I2C_ADDR, "$X");
        time_buff2 = millis();
        pattern = 131;
        stepper_pattern = 10;
        current_accel = 0;
        current_velocity = 0;
        current_length = 0;
      }
    }
  }

  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    IMU.readAccelData(IMU.accelCount);
    IMU.getAres();

    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes;
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes;
    IMU.az = (float)IMU.accelCount[2] * IMU.aRes;

    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;
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
      rp->log_velocity = current_velocity;
      rp->log_accel = current_accel;
      rp->log_ax = IMU.ax;
      rp->log_ay = IMU.ay;
      rp->log_az = IMU.az;
      rp->log_gz = IMU.gz;
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
        current_velocity = ex_accel * (float(time_ms)/1000);
        current_length = 0.5 * ex_accel * (float(time_ms)/1000) * (float(time_ms)/1000);
        if( time_ms >= float(ex_velocity/ex_accel)*1000 ) {
          stepper_pattern = 1;
          time_stepper = time_ms;
          old_length = current_length;
          break;
        }
        break;

      case 1:
        current_accel = 0;
        current_velocity = ex_velocity;
        current_length = old_length + ex_velocity * ((float(time_ms)-float(time_stepper))/1000);
        if( ex_length - current_length <= old_length ) {
          time_stepper = time_ms;
          old_length = current_length;
          stepper_pattern = 2;
          break;
        }
        break;

      case 2:
        current_accel = ex_accel;
        current_velocity = ex_velocity - ex_accel * ((float(time_ms)-float(time_stepper))/1000);
        current_length = old_length + ex_velocity * (float(time_ms)-float(time_stepper))/1000 - 0.5 * ex_accel * ((float(time_ms)-float(time_stepper))/1000) * ((float(time_ms)-float(time_stepper))/1000);
        if( time_ms - time_stepper >= float(ex_velocity/ex_accel)*1000 ) {
          current_length = ex_length;
          current_velocity = 0;
          current_accel = 0;
          stepper_pattern = 3;
          break;
        }
        break;

      case 3:
        break;

      case 4:
        current_accel = ex_accel;
        current_velocity = ex_accel * (float(time_ms)-float(time_stepper))/1000;
        current_length = ex_length - 0.5 * ex_accel * ((float(time_ms)-float(time_stepper))/1000) * ((float(time_ms)-float(time_stepper))/1000);
        if( float(time_ms)-float(time_stepper) >= float(ex_velocity/ex_accel)*1000 ) {
          stepper_pattern = 5;
          time_stepper = time_ms;
          old_length = current_length;
          break;
        }
        break;

      case 5:
        current_accel = 0;
        current_velocity = ex_velocity;
        current_length = old_length - ex_velocity * ((float(time_ms)-float(time_stepper))/1000);
        /*if( current_length <= ex_length - old_length ) {
          time_stepper = time_ms;
          old_length = current_length;
          stepper_pattern = 6;
          break;
        }*/
        break;

      case 6:
        current_accel = ex_accel;
        current_velocity = ex_velocity - ex_accel * ((float(time_ms)-float(time_stepper))/1000);
        current_length = old_length - ex_velocity * (float(time_ms)-float(time_stepper))/1000 - 0.5 * ex_accel * ((float(time_ms)-float(time_stepper))/1000) * ((float(time_ms)-float(time_stepper))/1000);
        if( time_ms - time_stepper >= float(ex_velocity/ex_accel)*1000 ) {
          current_length = 0;
          current_velocity = 0;
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
      if( tx_pattern == 11 ) {
        telemetry_flag = true;
      }
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
  EEPROM.write(5, (ex_velocity & 0xFF));
  EEPROM.write(6, (ex_velocity>>8 & 0xFF));
  EEPROM.write(7, (ex_velocity>>16 & 0xFF));
  EEPROM.write(8, (ex_velocity>>24 & 0xFF));
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
    ex_velocity = EEPROM.read(5) + (EEPROM.read(6)<<8) + (EEPROM.read(7)<<16) + (EEPROM.read(8)<<24);
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
        file = SD.open(fname, FILE_APPEND);
        file.print("NTP");
        file.print(",");
        file.print("Pattern");
        file.print(",");
        file.print("Time [ms]");
        file.print(",");
        file.print("Length [mm]");
        file.print(",");
        file.print("Velocity [mm/s]");
        file.print(",");
        file.print("Accel [mm/s^2]");
        file.print(",");
        file.print("IMUaX [G]");
        file.print(",");
        file.print("IMUaY [G]");
        file.print(",");
        file.print("IMUgZ [deg/s]");
        file.println(",");
        file.close();

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
        ex_velocity = rx_val;
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
                         "Test Program Ver1.20\n");
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
      bts.print(" 33 : Extension velocity [");
      bts.print(ex_velocity);
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
      //Telemetry @ Interrupt
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
      tx_pattern = 0;
      break;

    case 23:
      bts.print(" Start Winding...\n");
      tx_pattern = 0;
      break;

    case 24:
      bts.print(" Pause...\n");
      tx_pattern = 0;
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
      bts.print(" Extension velocity [mm/s] -");
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
void stepper( long ex_length, int ex_velocity, int ex_accel ) {
  String command;
  String command2;

  memset( charBuf , '\0' , 100 );
  memset( charBuf2 , '\0' , 100 );

  if( inc_flag ) {
    abslength = abslength + ex_length;
    inc_flag = false;
  }
  steps =  abslength;
  velocity = ex_velocity * 100;

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
  command.concat(String(velocity));
  command.toCharArray(charBuf, 100);
  SendCommand(STEPMOTOR_I2C_ADDR, charBuf);
  //Serial.println(charBuf);

}


//writeByte
//------------------------------------------------------------------//
 void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  M5.I2C.writeByte(address, subAddress, data);
}

//readByte
//------------------------------------------------------------------//
uint8_t readByte(uint8_t address, uint8_t subAddress) {
  uint8_t result;
  M5.I2C.readByte(address, subAddress,&result);
  return (result);
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
void SendCommand(byte addr, char *ci) {
  Wire.beginTransmission(addr);
  while ((*ci) != 0) {
    Wire.write(*ci);
    ci++;
  }
  Wire.write(0x0d);
  Wire.write(0x0a);
  Wire.endTransmission();
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

