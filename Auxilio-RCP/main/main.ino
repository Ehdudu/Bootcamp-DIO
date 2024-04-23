#include<Wire.h>
const int MPU_addr=0x68; // Accelerometer's I2C address
const int accCollectionLimit=1000; // Defines a hard cap used for resetting the acceleration array once it reaches its limit. 1000 points = +-25 compressions
const int compLimit=25; // defines the amount of compressions necessary for the compression frequency to be calculated
float acceleration[accCollectionLimit]={}; // acceleration values collected over time.
float accTrend=0, velTrend=0; // trend is used for detrending
float compressionDepth=0; // stores depth of compression 
int measurementCounter=0; // used for passing values to the acceleration array
int compCounter=0; // compCounter tracks amount of compressions made
int riseCounter=0; // riseCounter is used for tracking the movement of a graph similar to a senoid, being used to identify the moment it stops rising and starts a downwards trend.
int startingPoint=0; // This will be used to keep track of when the first compression started, so a trend can be calculated later
unsigned long compStartTime=0, compEndTime=0; // compStartTime tracks the instant of time of the first compression, compEndTime tracks the instant of time after a specific number of compressions defined on compressionCount()
unsigned long collectAccTime[accCollectionLimit]={}; // time of collected acceleration value
double compFreq=0; // compFreq tracks the average amount of compressions per minute 

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

  Serial.begin(115200); // Starts serial communication
  Serial.println("Programa iniciado, 5 segundos até cálculo de trend");
  delay(5000);

  // Measuring the average accelerometer readings during rest.
  int i=0;
  do {
    acceleration[i] = collectAcc();
    i++;
  }while(i<=500); // stops after measuring 500 values]
  accTrend = trendCalc(500,acceleration);
  //memset(acceleration,0,sizeof(acceleration)); // resetta todos os valores do array. Nao e necessario, mas...

  Serial.print("Trend calculada:"); Serial.println(accTrend); // For initial test purposes only
}

void loop(){  
  acceleration[measurementCounter]= collectAcc();
  collectAccTime[measurementCounter]=millis();
  compressionCount(); // Tracks compressions based on collected and detrended acceleration values
  measurementCounter++;
  if(measurementCounter>=(accCollectionLimit-1)) {
    measurementCounter=0;
    compCounter=0;
    riseCounter=0;
    startingPoint=0;
    //compressionDepth = depthMeasure();
    //memset(acceleration,0,sizeof(acceleration)); // não é necessário, mas caso seja...
  } 
  
  
  //Serial.print(millis()); Serial.print(" ; "); Serial.print(acc); Serial.print(" ; "); Serial.print(compCounter); Serial.print(" ; "); Serial.print(compPeriod); Serial.print(" ; "); Serial.println(compFreq);
  
  delay(7);
}

float collectAcc() {
  int16_t rawAcc; // Stores acceleration from the sensor
  float gAcc; // Stores the acceleration in m/s²
  
  Wire.beginTransmission(MPU_addr); // Transmission start
  Wire.write(0x3F); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,2,true); // The only value being collected is the Z acceleration, so only 2 registers are read
  rawAcc=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  gAcc=(rawAcc/4096.0)*9.80665; 
  
  return gAcc;
}

void compressionCount() {
  float oldAcc = (measurementCounter>0) ? acceleration[measurementCounter-1] : acceleration[accCollectionLimit-1]; 
  float newAcc=acceleration[measurementCounter];

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

float depthMeasure() {
  int timeDelta[accCollectionLimit]={};
  float acc5PointMovingAvg[accCollectionLimit]={}, accOffset[accCollectionLimit]={};
  float accTrend=0;

  movingAverage(acc5PointMovingAvg,acceleration,5,accCollectionLimit);
  accTrend = trendCalc(startingPoint-100,acc5PointMovingAvg); // 100 is an arbitrary value. It is used so that the inicial decline of the slope isn't taken in account during trend calculation.
  
  for(int i=1;i<accCollectionLimit;i++) {
    timeDelta[i]=collectAccTime[i]-collectAccTime[i-1]; // Calculates time between measurements
    accOffset[i]=accOffset[i]-accTrend;
  }
}

bool periodStartCheck(float oldValue, float newValue, float detectionTreshold) {
  bool periodStart = false;

  if(newValue<oldValue && abs(newValue)>detectionTreshold) { // Tracks movement as long as the collected value is higher than the defined treshold
    riseCounter++;
  }

  if(newValue>oldValue && riseCounter>3) { // Tracks the lowest point of a "senoid" by looking for the moment it stops the upwards trend and starts a downwards one.
    periodStart = true;
    if(startingPoint==0) { // This will be used to keep track of when the first compression started, so a trend can be calculated later
      startingPoint=measurementCounter;
    }
    riseCounter=0;
  }

  return periodStart;
}

float trendCalc(int stoppingMeasurement, float measurement[]) {
  double sum=0;
  float trend=0;
  for(int i = 0;i<stoppingMeasurement;i++) {
    sum += measurement[i];
  }

  trend = sum/stoppingMeasurement;
  return trend;
}

void integralCalculator(float *integral, float *data, float *period,int stoppingMeasurement) {
  integral[stoppingMeasurement]={};
  for(int i=1;i<stoppingMeasurement;i++) {
    integral[i]=(((data[i-1]+data[i])*(period[i]/1000))/2)+integral[i-1]; // calculates que area between two points (an integral). THe period is divided by 1000 to convert from ms to s
  }
}

void movingAverage(float *avg,float *data,int numOfPoints,int stoppingMeasurement) {
  for(int i=0;i<stoppingMeasurement;i++) {
    if(i>numOfPoints) {
      avg[i]=(data[i-2]+data[i-1]+data[i]+data[i+1]+data[i+2])/5;
    } else {
      avg[i]=data[i];
    }
  }
}