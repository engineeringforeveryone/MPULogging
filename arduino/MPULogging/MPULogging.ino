// MPULogging.ino
// This sketch is based on TechMonkeyBusiness's GY87 reader test.
// 
// // ORIGINAL COMMENTS
// It gets all sensors on a GY87 IMU board reporting (something) to the
// Serial.  The challenge is the Digital Compass which is nromally blocked by
// the Accelerometer.  This is remedied by using an I2C bypass command.  Thanks to
// pistolero992000 for this solution.
// Sensors on board the GY87 are:
// MPU6050 Accelerometer.  Address is 0x68
// HMC5883L Digital Compass.  Address is 0x1E
// BMP180 Barometer and Temperature Sensor.  Address is 0x77

// I2C Connections as per device default
// SCL and SDA
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// 250 Hz sampling rate
#define SAMPLING_RATE 250

//MPU6050 Accelerometer 
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

//HMC5883L Digital Compass
const int hmc5883Address = 0x1E; //0011110b, I2C 7bit address for compass
const byte hmc5883ModeRegister = 0x02;
const byte hmcContinuousMode = 0x00;
const byte hmcDataOutputXMSBAddress = 0x03;

int x,y,z; //triple axis data from HMC5883L.


float seaLevelPressure = 101325;

int LEDPin = 13;
bool blinkState = false;

void setup()
{
    Wire.begin();
    Serial.begin(9600);

    // initialise I2C Devices
    Serial.println("Initialising I2C devices...");
    accelgyro.initialize();

    // set accelerometer to 8g
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    // set gyro range to 1000 degrees per second
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    accelgyro.setI2CBypassEnabled(true);  //This sets the bypass so the HMC5883L gets a look in.

    
          
    //Initialise the Digital Compass
    Wire.beginTransmission(hmc5883Address);  //Begin communication with compass
    Wire.write(hmc5883ModeRegister);  //select the mode register
    Wire.write(hmcContinuousMode); //continuous measurement mode
    Wire.endTransmission();
    
        
    // configure Arduino LED for
    pinMode(LEDPin, OUTPUT);
    delay(1000);
}

// sampling rate 500 hz
// mainly inspired from TechMonkeyBusiness's main loop, stripped down to get the data that we want.
unsigned long int current_micros = 0;
unsigned long int prev_sample_micros= 0;
// get sampling rate in microseconds 
const long int SAMPLING_PERIOD = 1000/SAMPLING_RATE * 1000;


// timing test
// microseconds has a resolution of 4 microseconds
unsigned long int start_time = 0;
unsigned long int end_time = 0;

long long last_serial_ms = 0;

void loop()
{
    // sample every SAMPLING_PERIOD microseconds
    current_micros = micros();
    if (current_micros - prev_sample_micros > SAMPLING_PERIOD)
    {
        start_time = micros();
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        
        Wire.beginTransmission(hmc5883Address);
        Wire.write(hmcDataOutputXMSBAddress);  //Select register 3, X MSB register
        Wire.endTransmission();
    
        //Read data from each axis of the Digital Compass
        Wire.requestFrom(hmc5883Address,6);
        if(6<=Wire.available())
        {
            x = Wire.read()<<8; //X msb
            x |= Wire.read();   //X lsb
            z = Wire.read()<<8; //Z msb
            z |= Wire.read();   //Z lsb
            y = Wire.read()<<8; //Y msb
            y |= Wire.read();   //Y lsb    
        }
        end_time = micros();
        prev_sample_micros = current_micros;

        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); 
        Serial.print(",");
        Serial.print(gx); Serial.print(",");
        Serial.print(gy); Serial.print(",");
        Serial.print(gz); Serial.print(",");
        Serial.print(x); Serial.print(",");
        Serial.print(y); Serial.print(",");
        Serial.print(z); 
        Serial.print('\n');
    }


}


void print_debug()
{
  if (millis() - last_serial_ms > 500)
  {
    last_serial_ms = millis();
    Serial.print(start_time); Serial.print(","); Serial.print(end_time); 
    Serial.print(",");  
    int res = int(end_time - start_time);
    Serial.print(res);
    
  //        Serial.print(end_time - start_time); 
    Serial.print(" us.");
    Serial.println();
  }
}

