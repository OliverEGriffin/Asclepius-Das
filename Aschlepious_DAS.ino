#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <EnvironmentCalculations.h>
#include <SPI.h>
#include <SPIMemory.h>

Adafruit_MPU6050 mpu;
SPIFlash flash(7);  // Use your CS pin (usually 10 on Uno)
unsigned long address = 0x00ff;
int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0  
    ? (int)&__heap_start : (int) __brkval);  
}
BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_16,
   BME280::SpiEnable_False,
   BME280I2C::I2CAddr_0x76
);

BME280I2C bme(settings);
BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_Pa);
EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;

const int telemitry_setup_packet_number = 32;
int mode = 0x00;
const float takeoff_g = 2;
float g = 9.81;
float est_launch_pressure = 100000;
float est_launch_temp = 20;
const float calibration_altitude_error = 0.5;
float launch_angle_c = 0;
float launch_angle_s = 0;
unsigned long packet_number=0;
unsigned long launch_time = 0;

const uint8_t breakwire_pin_foward = 1;
const uint8_t breakwire_pin_rear = 2;
const uint8_t breakwire_pin_stage = 3;
const uint8_t breakwire_pin_takeoff = 4;
uint8_t slow_telemitry_log_counter = 0x00;
void pc(){
  Serial.print(F(","));
}
class Telemitry{
  public:

    // General (20 Bytes)
    unsigned long up_time_us;
    unsigned long est_liftoff_time_us;
    unsigned long est_time_plus_us;
    float bat_voltage;
    unsigned long packet_number_ulong;
    

    // General (3 Bytes)
    byte connections;
    bool serial_connection;
    byte mode; 
    
    // IMU (28Bytes)
    sensors_vec_t imu_gyro_rads;
    sensors_vec_t imu_accel_m_s;;
    float imu_temp;
    
    // BME (16Bytes)
    float bme_pressure;
    float bme_altitude;
    float bme_temp;
    float bme_hum;

    
    //67 Bytes in a packet
    
    Telemitry(byte Mode = 0x00, unsigned long pn=0x00000000){
      up_time_us = micros();
      est_liftoff_time_us = launch_time;
      est_time_plus_us = up_time_us-est_liftoff_time_us;
      bat_voltage = 7.6;
      packet_number_ulong = pn;
      connections = digitalRead(breakwire_pin_foward)<<4 + digitalRead(breakwire_pin_rear)<<3 + digitalRead(breakwire_pin_stage)<<2 + digitalRead(breakwire_pin_takeoff);
      connections += connections<<2;
      
      serial_connection = Serial;
      mode = Mode;
      
      // ---- IMU DATA ----
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      
      imu_gyro_rads = g.gyro;
      imu_accel_m_s = a.acceleration;
      imu_temp = temp.temperature;

      // ---- BME 280 ----
      
      
      bme.read(bme_pressure, bme_temp, bme_hum, tempUnit, presUnit);

      
      //bme_altitude = 8;
      
      //bme_pressure = 1;
      //bme_temp = 1;
      //bme_hum = 1;
          
      bme_altitude = EnvironmentCalculations::Altitude(bme_pressure, envAltUnit, est_launch_pressure, est_launch_temp, envTempUnit);;
    };
    
    void Store(unsigned int Address){
      flash.writeAnything(Address, *this);
    }
    
    Telemitry Read(unsigned int Address){
      Telemitry T;
      flash.readAnything(Address, T);
      return T;
    }
    void printPacketHeaders(){
      Serial.println(F("Time,Packet Number,Estiamated Liftoff Time,Estimated T+,Battery Voltage,serial connection,Connection 1,Connection 2,Connection 3,Connection 4,Connection 5,Connection 6,Connection 7,Connection 8,Mode,Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z, IMU Temp, BME Pressure,BME Altitude,BME Temperature,BME humidity"));
    }
    
    void printPacket() {
      Serial.print(up_time_us);
      pc();
      Serial.print(packet_number_ulong);
      pc();
      Serial.print(est_liftoff_time_us);
      pc();
      Serial.print(est_time_plus_us);
      pc();
      Serial.print(bat_voltage);
      pc();
      Serial.print(serial_connection);
      pc();
      for (int i=0;i<8;i++)
      {
       Serial.print((connections >> i) & 1 == 1 ? "1" : "0"); // will reverse bit order!
        pc();
      }
      Serial.print(mode);
      pc();
      Serial.print(imu_accel_m_s.x);
      pc();
      Serial.print(imu_accel_m_s.y);
      pc();
      Serial.print(imu_accel_m_s.z);
      pc();
      Serial.print(imu_gyro_rads.x);
      pc();
      Serial.print(imu_gyro_rads.y);
      pc();
      Serial.print(imu_gyro_rads.z);
      pc();
      Serial.print(imu_temp);
      pc();
      Serial.print(bme_pressure);
      pc();
      Serial.print(bme_altitude);
      pc();
      Serial.print(bme_temp);
      pc();
      Serial.println(bme_hum);
    }
};    



int MPU6050Setup() {
  Serial.println(F("Setup MPU6050 chip"));
    while (!mpu.begin()) {
        Serial.println(F("Failed to find MPU6050 chip"));
      delay(100);
    }
    //Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

    return 0;
}


int BME280Setup(){
  Serial.println(F("BME280 setup!"));
  while(!bme.begin())
  {
    Serial.println(F("Could not find BME280 sensor!"));
    delay(1000);
  }

  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println(F("Found BME280 sensor! Success."));
       break;
     case BME280::ChipModel_BMP280:
       Serial.println(F("Found BMP280 sensor! No Humidity available."));
       break;
     default:
       Serial.println(F("Found UNKNOWN sensor! Error!"));
  }
  
  return 0;
}
void setup(){
  Serial.begin(115200);
  Serial.println(F("Program Start"));

  while (!flash.begin()) {  
  }
  //flash.eraseSector(0);
  
  if (!Serial){
    Serial.println(F("r/E"));
    while (!Serial.available()) {
    }
      
    String k = Serial.readString();
    if(k=="E"){
      Serial.println(F("y/n"));
      while (!Serial.available()) {
      }
      if(Serial.readString()=="y"){
        Serial.println(F("Beginning Erase"));
        flash.eraseChip();
        Serial.println(F("Erase Completer"));
          
      }
    }
    if(k=="r"){
    Telemitry T;
    T.printPacketHeaders();
    for (address=address;address <= 0xffffff-sizeof(Telemitry); address += sizeof(Telemitry)){
      
      T = T.Read(address);
      T.printPacket();
    }}
    //mode==0x80;
  }

  
  BME280Setup();
  MPU6050Setup();

  pinMode(breakwire_pin_foward, INPUT);
  pinMode(breakwire_pin_rear, INPUT);
  pinMode(breakwire_pin_stage, INPUT);
  pinMode(breakwire_pin_takeoff, INPUT);
  
 
  
  Serial.println(F("Settup Start"));
  // Callibrate launch
  Telemitry telemitry_packet = Telemitry(mode, packet_number);
  est_launch_pressure = telemitry_packet.bme_pressure;
  float av=0;
  for (int i =0; i<10; i++){
    Telemitry telemitry_packet = Telemitry(mode, packet_number);
    av += telemitry_packet.bme_pressure;
    est_launch_pressure = av/(i+1);
  }
    
  
  Telemitry telemitry_setup_packet = Telemitry(mode, packet_number);
  
  telemitry_setup_packet.imu_accel_m_s.x = 0;
  telemitry_setup_packet.imu_accel_m_s.y = 0;
  telemitry_setup_packet.imu_accel_m_s.z = 0;
  
  for (int i=0; i<telemitry_setup_packet_number;i++){
    Telemitry telemitry_next_setup_packet = Telemitry(mode, packet_number);
    telemitry_setup_packet.imu_accel_m_s.x += telemitry_next_setup_packet.imu_accel_m_s.x/telemitry_setup_packet_number;
    telemitry_setup_packet.imu_accel_m_s.y += telemitry_next_setup_packet.imu_accel_m_s.y/telemitry_setup_packet_number;
    telemitry_setup_packet.imu_accel_m_s.z += telemitry_next_setup_packet.imu_accel_m_s.z/telemitry_setup_packet_number;
  }
  
  float g = sqrt(sq(telemitry_setup_packet.imu_accel_m_s.x)+sq(telemitry_setup_packet.imu_accel_m_s.y)+sq(telemitry_setup_packet.imu_accel_m_s.z));
  float takeoff_accel = takeoff_g*g;
  launch_angle_c = -max(max(telemitry_setup_packet.imu_accel_m_s.x,telemitry_setup_packet.imu_accel_m_s.y),telemitry_setup_packet.imu_accel_m_s.z)/g;
  launch_angle_s = sin(acos(launch_angle_c));
  Serial.print("g=");Serial.println(g);
  Serial.print(F("launch_angle_c="));Serial.println(launch_angle_c);
  Serial.print(F("launch_angle_s="));Serial.println(launch_angle_s);
  Serial.print(F("takeoff_accel="));Serial.println(takeoff_accel);
  
  flash.writeAnything(0x0000F0,0x10);
  mode = 0x01;
}

void loop() {
  Telemitry telemitry_packet = Telemitry(mode, packet_number);
  //  ---- Print Data Logic ----
  if(mode==0x80){
    telemitry_packet.printPacket();
  }
  
  // ---- Store Data Logic ----
  if(mode == 0x01 || mode == 0x0F){
    if(slow_telemitry_log_counter == 199){
      telemitry_packet.Store(address);
      slow_telemitry_log_counter = 0x00;
      address += sizeof(Telemitry);
    }
    else{
      slow_telemitry_log_counter+=1;
    }
  }
  else{
    telemitry_packet.Store(address);
    
    address += sizeof(Telemitry);
  }
  packet_number+=1;


  // ---- Exit Data Logic ----
  // Quits if we run out of memory
  if (address > (256*256*254)/sizeof(Telemitry)){
    flash.writeAnything(0x0000F0,0x30);
  }

  // Quits when 28 Seconds remain
  if(telemitry_packet.up_time_us>=0xff000000){
  
    flash.writeAnything(0x0000F0,0x70);
  }
  
}



/*
 * Mode Description
 * 
 * 0x00 - SETUP
 * 0x01 - PAD IDLE
 * 0x02 - LAUNCH DETECT
 * 0x03 - BOOST
 * 0x03 - COAST
 * 0x04 - APOGEE
 * 0x05 - DESENT
 * 0x0F - LANDED
 * 0x80 - GROUND LOOP
 */
