#include<Wire.h>
// ESP32-Accelerometer connection: GND on GND, 9 on SCL, 8 on SDA, 3v3 on VCC
const int MPU_addr=0x68; // Accelerometer's I2C address

void compressionCount(float acc);
bool periodStartCheck(float oldValue, float newValue, float detectionTreshold);
int measurementCounter=0;
int compCounter=0;
const int compLimit = 10;
unsigned long compStartTime = 0, compEndTime = 0;
double compFreq = 0;
bool currentPeriod=false;
int riseCounter = 0;
float gAcc[2]={}; // Stores the acceleration in m/s²

void setup(){
  Wire.begin(); // Starts I2C communication
  Wire.beginTransmission(MPU_addr); 
  Wire.write(0x6B); 
  Wire.write(0); // Wakes up the MPU 6050 
  Wire.endTransmission(true);

  // Sensor sensitivity config
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);                  
  Wire.write(0x10);                  // Defines the measurement scale as +/- 8g
  Wire.endTransmission(true);

  Serial.begin(9600);
}

void loop(){  
  int16_t rawAcc; // Stores acceleration from the sensor

  
  Wire.beginTransmission(MPU_addr); // Transmission start
  Wire.write(0x3F); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,2,true); // The only value being collected is the Z acceleration, so only 2 registers are read
  rawAcc=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  gAcc[measurementCounter]=(rawAcc/4096.0)*9.80665;

  compressionCount(gAcc);

  measurementCounter = (measurementCounter>0) ? 0:1;

  Serial.println(gAcc[measurementCounter]);
  // Serial.print(" ; ");
  Serial.println(millis());
  // Serial.print(" ; ");
  Serial.println(compFreq);
  delay(7);
}

void compressionCount(float *acc) {
  float oldAcc = (measurementCounter>0) ? acc[0] : acc[1];
  float newAcc=acc[measurementCounter];

  if(periodStartCheck(oldAcc,newAcc,11)) { // periodStartCheck monitors when a period starts, which means it'll activate every time a compression movement is fully completed. the detection treshold has been tested, and upon empirical analysis, 11 m/s² was defined as the best number to guarantee noise doesn't cause errors
    compCounter++;
    if(compCounter==1){ // aumentar as condições de compcounter para evitar que no começo do programa ele pegue movimentação e interprete como compressão
      compStartTime=millis();
    }
    if(compCounter==compLimit){
      compEndTime=millis();
      compCounter=0;
      double compPeriod = (double)(compEndTime - compStartTime);
      compFreq = (double)(600000/compPeriod);
    }
  }
}

bool periodStartCheck(float oldValue, float newValue, float detectionTreshold) {
  bool periodStart = false;

  if(newValue<oldValue && abs(newValue)>detectionTreshold && ((oldValue-newValue<3))) { // Tracks movement as long as the collected value is higher than the defined treshold
    riseCounter++;
  } else if (newValue>oldValue && abs(newValue)<8 ) {
    currentPeriod=false;
    riseCounter=0;
  }

  if(newValue>oldValue && riseCounter>0 && !currentPeriod) { // Tracks the lowest point of a "senoid" by looking for the moment it stops the upwards trend and starts a downwards one.
    periodStart = true;
    currentPeriod=true;
  }

  return periodStart;
}
