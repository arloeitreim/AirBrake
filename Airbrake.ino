#include "SparkFun_BMP581_Arduino_Library.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include "FS.h"
#include "SD.h"
#include "ESP32Servo.h"

BMP581 bmp; 
Adafruit_MPU6050 mpu;

float averagex = 0;
float averagey = 0;
float averagez = 0;

float accelerationx;
float accelerationy;
float accelerationz;

float groundLevel; // meters
float temperature = 15; // celcius 
float pressure; // pascals

float velocity = 0;
float previous_velocity = 0;
float previous_velocity_2 = 0;

float previous_altitude;

float seconds;
float previous_seconds;

bool stage_1 = true;
bool stage_2 = false;
bool stage_3 = false;
bool stage_4 = false;

int angle = 21;

Servo servo;

int start_delay = 30000;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  bmp.beginI2C();
  SD.begin();
  mpu.begin();
  bmp.setMode(BMP5_POWERMODE_CONTINOUS);
  
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
  servo.attach(13);

  bmp5_osr_odr_press_config osrMultipliers =
  {
      .osr_t = BMP5_OVERSAMPLING_1X,
      .osr_p = BMP5_OVERSAMPLING_1X,
      0,0 // Unused values, included to avoid compiler warnings-as-error
    };
    bmp.setOSRMultipliers(&osrMultipliers);
  
    int8_t err = BMP5_OK;
    bmp5_iir_config config =
     {
        .set_iir_t = BMP5_IIR_FILTER_COEFF_1, // Set filter coefficient
        .set_iir_p = BMP5_IIR_FILTER_COEFF_127, // Set filter coefficient
        .shdw_set_iir_t = BMP5_ENABLE, // Store filtered data in data registers
        .shdw_set_iir_p = BMP5_ENABLE, // Store filtered data in data registers
        .iir_flush_forced_en = BMP5_DISABLE // Flush filter in forced mode
    };
    err = bmp.setFilterConfig(&config);
  //delay(120000);
  delay(start_delay);
  servo.write(137);
  delay(210);
  servo.write(21);

  bmp5_sensor_data data = {0,0};
  err = bmp.getSensorData(&data);
  pressure = data.pressure;

  int count = 0;
  
  while ((millis() <= start_delay + 4000)) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    averagex += a.acceleration.x;
    averagey += a.acceleration.y;
    averagez += a.acceleration.z;

    count++;

  }

  averagex /= count;
  averagey /= count;
  averagez /= count;

  groundLevel = getAltitude();
  writeFile(SD, "/Data.txt", String("Ground Level: ").c_str());
  appendFile(SD, "/Data.txt", String(groundLevel).c_str());
  appendFile(SD, "/Data.txt", String("Pressure / Altitude / Projected Altitude / Velocity / Averaged Velocity / Acceleration / Angle / Time ").c_str());
}

void loop() {
  int8_t err;
  String timestamped_data;
  float current_altitude;
  float projected_altitude;
  float velocity_averaged;

  bmp5_sensor_data data = {0,0};
  err = bmp.getSensorData(&data);
  pressure = data.pressure;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accelerationx = a.acceleration.x;
  accelerationy = a.acceleration.y;
  accelerationz = a.acceleration.z;

  String acceleration = String(accelerationx) + " , " + String(accelerationy) + " , " + String(accelerationz);

  current_altitude = getAltitude() - groundLevel;

  seconds = millis() / 1000.0;

  previous_velocity_2 = previous_velocity;
  previous_velocity = velocity;
  velocity = (current_altitude - previous_altitude) / (seconds - previous_seconds);
  velocity_averaged = (previous_velocity_2 + previous_velocity + velocity) / 3;

  previous_altitude = current_altitude;
  previous_seconds = seconds;

  projected_altitude = find_projected_altitude(current_altitude, velocity_averaged);

  timestamped_data = String(data.pressure) + " / " + String(current_altitude) + " / " + String(projected_altitude) + " / " + String(velocity) + " / " + String(velocity_averaged) + " / " + String(acceleration) + " / " + String(stages(current_altitude, projected_altitude)) + " / " + String(seconds);
  Serial.println("Pressure / Altitude / Projected Altitude / Velocity / Averaged Velocity / Acceleration / Angle / Time  -->  " + timestamped_data);
  appendFile(SD, "/Data.txt", timestamped_data.c_str());

  
}

int stages(float current_altitude, float projected_altitude) {
  float target_altitude = 259.08;

  if (stage_1) {
    if (accelerationz < 10.0) {
      stage_1 = false;
      stage_2 = true;
      stage_3 = true;
      stage_4 = true;
      servo.write(137);
      angle = 137;
    }
  }
  else if ((projected_altitude <= target_altitude + 20  || projected_altitude >= target_altitude - 20 ) && stage_2) {
    stage_2 = false;
    servo.write(78.5);
    angle = 78.5;
  }
  else if ((projected_altitude <= target_altitude + 10 || projected_altitude >= target_altitude - 10) && stage_3) {
    stage_3 = false;
    servo.write(55);
    angle = 55;
  }
  else if ((projected_altitude <= target_altitude + 5 || projected_altitude >= target_altitude - 5) && stage_4) {
    stage_4 = false;
    servo.write(40);
    angle = 40;
  }
  else if (projected_altitude <= target_altitude + 2 || projected_altitude >= target_altitude - 2) {
    servo.write(21);
    angle = 21;
  }
  return angle;
}

float getAltitude() {
  float altitude;
  float pressure_hPa;

  pressure_hPa = pressure / 100;

  altitude = 44330.0 * (1.0 - pow(pressure_hPa / 1013.25, 0.1903));

  return altitude;
}

float sealevelForAltitude(float altitude, float pressure) {
  altitude = altitude * 0.0065;
  return pressure * pow(1 - (altitude / ( temperature + altitude + 273.15)), -5.257);
}

float find_projected_altitude(float current_altitude, float velocity) {
  float projected_altitude; // projected altitude in meters
  float p = 1.225; // density of air in kilograms per meter cubed 
  float v_2 = pow(velocity, 2); // velocity squared in meters per second 
  float A_v = 0.0040218; // area in meters per second squared
  float g = 9.81; // gravity in meters per second squared
  float C_v= pow(22.3458, (velocity + 0.312704)) + 57.0175;  // coefficient of drag
  float m = 0.662; // mass in kilograms 
  float kv;

  kv = 0.5 * p * A_v * C_v;

  projected_altitude = m / (2.0 * kv) * log((kv * v_2) / m * g + 1.0) + current_altitude;

  return projected_altitude;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.println(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}


