#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>

/* ---- FLIGHT PERAMETERS ----*/
const bool debug = false;
const uint8_t pad_idle_time = 25000; //ms
const float boost_time = 1.25; // Seconds
const float apogee_lockout_time = 3.0; // Seconds 
const float apogee_lockout_altitude = 200.0; // m
const float main_deploy_altitude = NULL; // m | NULL is single deploy  

/* ----- SENSORS AND INPUTS-----  */
Adafruit_LSM303_Accel_Unified LSM303_Accelerometer = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified LSM303_Magnotometer = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_L3GD20_Unified L3GD20_Gyroscope = Adafruit_L3GD20_Unified(20);
Adafruit_BMP085_Unified BMP180_Barometer = Adafruit_BMP085_Unified(10085);
// DPS310
// GPS


#define switch_analogue A2    // Analogue Switch to acticate program
const int switch_analogue_threshold_on = 500;
const int switch_analogue_threshold_off = 480;

#define battery_voltage A0    // Measures half the battery coltage
// ---- Errors and Modes ----
uint8_t error_code = 0x00;


/* 
  Mode and Error Codes
X---  ----    0 Setup | 1 Loop
-X--  ----    0 Pad | 1 Flight
--X-  ----    0 Pre-Apogee | 1 Post-Apogee
---X  ----    0 Pre-Landing | 1 Post-Landing

    MODE CODE
----  XXXX    CODE
0000  0000    0x00    Setup
0000  0100    0x04    Sensor Turn On
0000  1000    0x08    Loop Detect
0000  1100    0x0C    Sensor Calibrate
0000  1111    0x0F    Unexpected Turnoff During Idle 

1000  0000    0x80    Pad Idle
1000  0100    0x84    Launch Detect

1100  0000    0xC0    Boost
1100  0100    0xC4    Coast
1100  1000    0xC8    Apogee Detect

1110  0000    0xE0    Desent Under Drogue
1110  0100    0xE4    Desent Under Main
1110  1000    0xE8    Landing Detect

1111  0000    0xF0    Touchdown
1111  0100    0xF4    Touchdown Cooloff
1111  1000    0xF8    Touchdown Idle
1111  1100    0xFC    Turnoff 
1111  1111    0xFF    Unexpected Turnoff During Loop 
    ERROR CODE
*/
const uint8_t mode_setup = 0x00;
const uint8_t mode_sensor_turn_on = 0x04;
const uint8_t mode_loop_detect = 0x08;
const uint8_t mode_sensor_calibrate = 0x0C;
const uint8_t mode_setup_complete = 0x0F;

const uint8_t mode_pad_idle = 0x80;
const uint8_t mode_launch_detect = 0x84;

const uint8_t mode_boost = 0xC0;
const uint8_t mode_coast = 0xC4;
const uint8_t mode_apogee_detect = 0xC8;

const uint8_t mode_desent_under_drogue = 0xE0;
const uint8_t mode_desent_under_main = 0xE4;
const uint8_t mode_landing_detect = 0xE8;

const uint8_t mode_touchdown = 0xF0;
const uint8_t mode_touchdown_cooloff = 0xF4;
const uint8_t mode_touchdown_idle = 0xF8;
const uint8_t mode_turnoff = 0xFC;

const uint8_t mode_unexpected_turnoff_during_loop  = 0xFF;


uint8_t mode = mode_setup; 






/* ----- OUTPUTS ----- */
class Buzzer{
  private:
    const uint8_t duration_short = 50;
    const uint8_t duration_long = 200;
    uint8_t apogee_last_digit;
  public:
    const uint8_t pin = 5;
    bool state = false;

  Buzzer(){};
  void state_setup(){
    pinMode(pin, OUTPUT);
    digitalWrite(pin, state);
  }
  void state_toggle(){
    state = !state;
    digitalWrite(pin, state);
  }
  void state_short(){
    state_toggle();
    delay(duration_short);
    state_toggle();
  }
  void state_long(){
    state_toggle();
    delay(duration_long);
    state_toggle();
  }
  void state_startup(){
    for(uint8_t i=0;i<1;i++){state_short();delay(duration_long);}
    delay(duration_long);
    for(uint8_t i=0;i<3;i++){state_short();delay(duration_short);}
  }
  void state_error(){
    if(error_code != 0 ){
      for(uint8_t i=0;i<3;i++){state_long();delay(duration_long);}
      for(uint8_t i=0;i<error_code;i++){state_long();delay(duration_long);}
    }
  }
  void state_pad_wait(){
    for(uint8_t i=0;i<2;i++){state_long();delay(duration_long);}
  }
  void state_pad_idle_start(){
    for(uint8_t i=0;i<5;i++){state_short();delay(duration_short);}
  }
  void state_pad_idle(){
    for(uint8_t i=0;i<30;i++){state_short();delay(1000);}
  }
  void state_shutdown_nominal(){
    for(uint8_t i=0;i<5;i++){state_long();delay(duration_long);}
  }
  void state_shutdown_error(){
    for(uint8_t i=0;i<5;i++){state_long();delay(duration_short);}
    delay(1000);
    for(uint8_t i=0;i<8;i++){(error_code<<i)%2?state_long() : state_short();delay(duration_long);}
  }
  void state_altitude(float apogee_altitude){
    for(int8_t j=3;j>0;j--){
      apogee_last_digit = (((int32_t)round(apogee_altitude))/(round(pow(10,j))))%10;
      if(!bool(apogee_last_digit)){delay(2000);}else{for(uint8_t i=0;i<apogee_last_digit;i++){state_short();delay(duration_long);}}
      for(uint8_t i=0;i<3;i++){delay(duration_long);}
    }
  }
};
Buzzer buzzer = Buzzer();
//SD Card

/* ----- TELEMITRY ----- */
sensors_event_t sensors_event;  // Used to log the current data from every Adafruite I2C sensor

class Telemitry{
  private:
    const float BMP_error = 0.17/3;
    const float DPS310_error = 0.02/3;
  public:
  float up_time_micros = micros();
  float Apogee_Estimation = 153.6;
  float Altitude_Estimation = 0;
  /* ----- SENSORS AND INPUTS-----  */
  int analogue_switch_value;
  sensors_event_t LSM303_Accelerometer_Data;
  sensors_event_t LSM303_Magnotometer_Data;
  sensors_event_t L3GD20_Gyroscope_Data;
  sensors_event_t BMP180_Barometer_Data;
  // Baro DPS310
  // GPS
  /* ----- OUTPUTS ----- */



  Telemitry(){
    analogue_switch_value = analogRead(switch_analogue);
    up_time_micros = micros();
    //LSM303_Accelerometer_Data.getEvent(&sensors_event);
    //serial.println(type(sensors_event.acceleration));
    /*
    LSM303_Magnotometer_Data = LSM303_Magnotometer_Data.getEvent(&sensors_event);
    L3GD20_Gyroscope_Data = L3GD20_Gyroscope_Data.getEvent(&sensors_event);
    BMP180_Barometer_Data = BMP180_Barometer_Data.getEvent(&sensors_event);
    


    Altitude_Estimation = 
    Apogee_Estimation = max(Apogee_Estimation,Altitude_Estimation);
    */
  };
  
  void log_telemitry(){
    int x = 1;
  }
};



Telemitry current_telemitry;

void setup() {
  Serial.begin(115200);
  delay(1000);
  mode = mode_sensor_turn_on;
  
  /* ----- OUTPUTS ----- */
  buzzer.state_setup();
  
  /* ----- SENSORS AND INPUTS-----  */
  pinMode(switch_analogue, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  

  L3GD20_Gyroscope.enableAutoRange(true);
  LSM303_Magnotometer.enableAutoRange(true);
  buzzer.state_startup();
  /* Initialise the sensor */
  while(true){
  if(!LSM303_Accelerometer.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    error_code = 0x01;
  }
  /* Initialise the sensor */
  else if(!LSM303_Magnotometer.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    error_code = 0x02;
  }
  /* Initialise the sensor */
  else if(!L3GD20_Gyroscope.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    error_code = 0x03;
  }
  else if(!BMP180_Barometer.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    error_code = 0x04;
  }
  else{
    break;
  }
  buzzer.state_error();
  delay(5000);
  }
  
  mode = mode_loop_detect;  // Loop detect
  while(analogRead(switch_analogue)<switch_analogue_threshold_on){
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("IDLE    "); Serial.print(analogRead(switch_analogue)); Serial.print("  ");Serial.print(mode,HEX);Serial.print("  ");Serial.println(error_code,HEX);
    if(millis()%30000<100){
      buzzer.state_pad_wait();
      delay(100);
    }
    if(analogRead(switch_analogue)<switch_analogue_threshold_off){
      error_code = 0x0A;
      buzzer.state_shutdown_error();
      exit(0);
    }
  }
  
  mode = mode_sensor_calibrate;  // Sensor Callibrate
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  buzzer.state_pad_idle_start();

  mode = mode_setup_complete;
}

unsigned long buzzer_idle_time_start;
unsigned long mode_switch_time;

void loop() {
  if (debug==true){Serial.print("LOOP    ");  Serial.print("  "); Serial.print(analogRead(switch_analogue)); Serial.print("  ");Serial.print(mode,HEX);Serial.print("  ");Serial.println(error_code,HEX);}
  current_telemitry = Telemitry();

  switch(mode) {
    //   ----- FROM SETUP -----
    case mode_setup_complete:
      mode=mode_pad_idle;
      buzzer_idle_time_start = micros();
      mode_switch_time = micros();
      delay(2);
      break;

    //   ----- ON PAD -----
    case mode_pad_idle: 
      if(micros()% (unsigned long)1e6 < 1e5){buzzer.state_short();current_telemitry.log_telemitry();delay(10);}
      if(current_telemitry.up_time_micros-buzzer_idle_time_start > pad_idle_time){mode = mode_launch_detect;mode_switch_time = current_telemitry.up_time_micros;}
      break;
    case mode_launch_detect:
      mode = mode_touchdown;
      break;
    //   ----- ASSENT -----
    //   ----- DESENT -----
    //   ----- LANDING -----
    case mode_touchdown:
      mode = mode_touchdown_cooloff;
      break;

    case mode_touchdown_cooloff:
      mode = mode_touchdown_idle;
      break;
    
    case mode_touchdown_idle :
      if(millis()%20000 < 100){buzzer.state_altitude(1234);}
      break;
  }

  if(current_telemitry.analogue_switch_value < switch_analogue_threshold_off && (mode==mode_touchdown || mode==mode_touchdown_idle || mode==mode_touchdown_cooloff||mode==mode_pad_idle ||mode==mode_launch_detect )){
    if(mode != 0xF8){
      error_code = 0xFA;
      buzzer.state_shutdown_error();
      exit(0);
    }
    mode = mode_turnoff; // Turnoff
    
    delay(1000);
    buzzer.state_shutdown_nominal();
    exit(0);
  }
}

  /* Get a new sensor event 
  sensors_event_t event; 
  gyro.getEvent(&event);
 
  Serial.print("X rads: "); Serial.print(event.gyro.x); Serial.print("  ");
  Serial.print("Y rads: "); Serial.print(event.gyro.y); Serial.print("  ");
  Serial.print("Z rads: "); Serial.print(event.gyro.z); Serial.print("  ");

  accel.getEvent(&event);
  Serial.print("X m/s^2: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y m/s^2: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z m/s^2: "); Serial.print(event.acceleration.z); Serial.print("  ");
  
  mag.getEvent(&event);
  Serial.print("X uT: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y uT: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z uT: "); Serial.print(event.magnetic.z); Serial.print("  ");


  bmp.getEvent(&event);
  Serial.print("BMP 180 Pressure bar: ");Serial.print(event.pressure); Serial.print("  ");
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("BMP 180 Temperature: ");Serial.print(temperature); Serial.print("  ");

    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print("BMP 180 Altitude:    "); Serial.print(bmp.pressureToAltitude(seaLevelPressure, event.pressure)); Serial.println(" ");
  
  }*/ 
