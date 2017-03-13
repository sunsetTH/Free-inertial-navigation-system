
//Перечень адресов
#define MPU                         0x68
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define HMC5883L_DEFAULT_ADDRESS    0x1E
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAY_H         0x07

#include <TinyGPS.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//Результирующие переменные гироскопов, акселерометров и магнитометров
float GyX = 0;
float GyY = 0;
float GyZ = 0;
float AcX = 0;
float AcY = 0;
float AcZ = 0; 
float MgX = 0;
float MgY = 0;
float MgZ = 0;

//Широта и долгота в формате char для максимально возможной дочности
float Lat;
float Lon;

int pass = 0;       //Количество настраиваний датчиков

MPU6050        mpu (0x68);       //Обьект класса MPU6050
TinyGPS        gps;              //Обьект класса TinyGPSPlus
SoftwareSerial SerialGPS (6, 5); //Обьект класса SoftwareSerial

void setup () {
  //Настройка пинов для мультиплексора
  pinMode (9, OUTPUT);
  pinMode (10, OUTPUT);
  
  //Инициализация шины
  Wire.begin ();   
  
  //Инициализайия порта
  Serial.begin (9600);
  
  //Инициализация GPS
  SerialGPS.begin (9600);
}

void loop () {

  /*0 LOW LOW*/
  /*1 HIGH HIGH ДЛИННЫЙ ПРОВОД*/
  /*2 HIGH LOW ОРТОГОНАЛЬНЫЙ*/
  /*3 LOW HIGH*/

  char com;
  byte pin9;
  byte pin10;
  while (Serial.available () > 0) {
    com = Serial.read ();
    switch (com) {
      case '0': {
        pin9 = LOW;
        pin10 = LOW;
        digitalWrite (9, pin9);
        digitalWrite (10, pin10);
        calib (0);
      }
      break;
      case '1': {
        pin9 = HIGH;
        pin10 = HIGH;
        digitalWrite (9, pin9);
        digitalWrite (10, pin10);
        calib (1);
      }
      break;
      case '2': {
        pin9 = HIGH;
        pin10 = LOW;
        digitalWrite (9, pin9);
        digitalWrite (10, pin10);
        calib (2);
      }
      break;
      case '3': {
        pin9 = LOW;
        pin10 = HIGH;
        digitalWrite (9, pin9);
        digitalWrite (10, pin10);
        calib (3);
      }
      break;
      case '4':
      while (1) start ();
      break;
    }  
  }
}

//Запуск датчиков
void start () {
 unsigned long t_now = millis();
 while (SerialGPS.available () > 0) {
  if (gps.encode (SerialGPS.read ())) {
    gps.f_get_position (&Lat, &Lon);
  }
}

byte pin9  = LOW;
byte pin10 = LOW;

int num  = 0;    
for (int i = 0; i < 2; i++) {
  for (int j = 0; j < 2; j++) {
    if (i == 0) pin9 = LOW;
    else pin9 = HIGH;
    if (j == 0) pin10 = LOW;
    else pin10 = HIGH;
    digitalWrite (9, pin9);
    digitalWrite (10, pin10);
    getMotion (&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
    Print (num);
    num++;
  }
}
  //PrintGPS ();
delay (200);
}

//Калибровка 
void calib (int num) {
  mpu.initialize();
  switch(num) {
    case 1:
    mpu.setXGyroOffset(93);
    mpu.setYGyroOffset(-15);
    mpu.setZGyroOffset(30);
    mpu.setXAccelOffset(-2500);
    mpu.setYAccelOffset(1783);
    mpu.setZAccelOffset(877);
    break;
    case 2:
    mpu.setXGyroOffset(93);
    mpu.setYGyroOffset(-15);
    mpu.setZGyroOffset(30);
    mpu.setXAccelOffset(-2500);
    mpu.setYAccelOffset(1783);
    mpu.setZAccelOffset(877);
    break;
    case 3:
    mpu.setXGyroOffset(93);
    mpu.setYGyroOffset(-15);
    mpu.setZGyroOffset(30);
    mpu.setXAccelOffset(-2500);
    mpu.setYAccelOffset(1783);
    mpu.setZAccelOffset(877);
    break;
  }
}

//Вспомогательная функция -> configMPU()
void set_mpu_axis (int mode, byte address) {
 mpu.setSlaveAddress         (mode, HMC5883L_DEFAULT_ADDRESS | 0x80); 
 mpu.setSlaveRegister        (mode, address);
 mpu.setSlaveEnabled         (mode, true);
 mpu.setSlaveWordByteSwap    (mode, false);
 mpu.setSlaveWriteMode       (mode, false);
 mpu.setSlaveWordGroupOffset (mode, false);
 mpu.setSlaveDataLength      (mode, 2);
}

//Настройка датчиков для работы магнитометра
void configMPU () {
  mpu.setI2CMasterModeEnabled (0);
  mpu.setI2CBypassEnabled     (1);
  Wire.beginTransmission      (HMC5883L_DEFAULT_ADDRESS);
  Wire.write (0x02); 
  Wire.write (0x00);  
  Wire.endTransmission ();
  delay (5);

  Wire.beginTransmission (HMC5883L_DEFAULT_ADDRESS);
  Wire.write (0x00);
  Wire.write (B00011000); 
  Wire.endTransmission ();
  delay (5);

  mpu.setI2CBypassEnabled (0); 

  set_mpu_axis (0, HMC5883L_RA_DATAX_H);
  set_mpu_axis (1, HMC5883L_RA_DATAY_H);
  set_mpu_axis (2, HMC5883L_RA_DATAZ_H);

  mpu.setI2CMasterModeEnabled (1);
}

//Масштабирование акселерометров
float scaleAccel (int16_t accel_) { 
  return (float) (accel_/16384);
}

//Масштабирование гироскопов
float scaleGyro (int16_t gyro_) {
  return (float) (gyro_/131);
}

void getMotion (float* Ax_, float* Ay_, float* Az_, float* Gx_, float* Gy_, float* Gz_, float* Mx_, float* My_, float* Mz_) {
  //Настройка должна проводится только раз для каждого датчика
  if (pass < 4) {
    configMPU ();
    pass++;
  }
  uint8_t buffer[14];
  Wire.beginTransmission (MPU);
  Wire.write (0x6B);  
  Wire.write (0);
  Wire.endTransmission (true);
  I2Cdev::readBytes (MPU, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
  int16_t ax = (((int16_t)buffer[0]) << 8)  | buffer[1];
  int16_t ay = (((int16_t)buffer[2]) << 8)  | buffer[3];
  int16_t az = (((int16_t)buffer[4]) << 8)  | buffer[5];
  int16_t gx = (((int16_t)buffer[8]) << 8)  | buffer[9];
  int16_t gy = (((int16_t)buffer[10]) << 8) | buffer[11];
  int16_t gz = (((int16_t)buffer[12]) << 8) | buffer[13];
  int16_t mx = mpu.getExternalSensorWord (0);
  int16_t my = mpu.getExternalSensorWord (2);
  int16_t mz = mpu.getExternalSensorWord (4);
  *Ax_ = scaleAccel (ax);
  *Ay_ = scaleAccel (ay);
  *Az_ = scaleAccel (az);
  *Gx_ = scaleGyro  (gx);
  *Gy_ = scaleGyro  (gy);
  *Gz_ = scaleGyro  (gz);
  *Mx_ = mx;
  *My_ = my;
  *Mz_ = mz;
}

void Print (int num_) {
  Serial.print (num_); Serial.print ("$"); 
  Serial.print (GyX);  Serial.print ("$"); Serial.print (GyY); Serial.print ("$"); Serial.print (GyZ); Serial.print ("$"); 
  Serial.print (AcX);  Serial.print ("$"); Serial.print (AcY); Serial.print ("$"); Serial.print (AcZ); Serial.print ("$"); 
  Serial.print (MgX);  Serial.print ("$"); Serial.print (MgY); Serial.print ("$"); Serial.print (MgZ); Serial.print ("$");
  Serial.print (Lat);  Serial.print ("$"); Serial.print (Lon); Serial.println ("$");
}

void PrintGPS () {
 Serial.print   (Lat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : Lat, 6); 
 Serial.print   ("$"); 
 Serial.println (Lon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : Lon, 6);
}


