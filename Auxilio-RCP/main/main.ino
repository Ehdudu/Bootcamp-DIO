#include<Wire.h>
const int MPU_addr=0x68; // Accelerometer's I2C address
const int accCollectionLimit=1000; // Defines a hard cap used for resetting the acceleration array once it reaches its limit. 1000 points = +-25 compressions
const int compLimit=25; // defines the amount of compressions necessary for the compression frequency to be calculated
float acceleration[accCollectionLimit]={}; // acceleration values collected over time.
float compressionDepth=0; // stores depth of compression
float accTrend=0, velTrend=0;
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
  Serial.println("Por favor, coloque o acelerometro virado para baixo. O calculo  de trend se inicia em 5 segundos");
  delay(5000);

  unsigned long timeDelta[500]={};
  float acc5PointMovingAvg[500]={}, accOffset[500]={}, velocity[500]={};

  for(int i=0;i<500;i++) {
    acceleration[i]= collectAcc();
    collectAccTime[i]=millis();
  }

  movingAverage(acc5PointMovingAvg,acceleration,5); // Calculated a moving average of the acceleration in order to reduce noise
  accTrend = trendCalc(500,acc5PointMovingAvg); // Calculates the noise trend before compressions start. 100 is an arbitrary value, used so to make sure that the first slope decline isn't included in the trend calculation.
  for(int i=1;i<500;i++) {
    timeDelta[i]=collectAccTime[i]-collectAccTime[i-1]; // Calculates time between measurements
    accOffset[i]=acc5PointMovingAvg[i]-accTrend; // detrends acceleration
  }
  integralCalculator(velocity,accOffset,timeDelta); // Calculates the velocity, which is the integral of the acceleration
  velTrend=trendCalc(500,velocity); // Calculates the noise trend in the velocity.

  Serial.println("Trend calculada."); Serial.println("Aceleracao: ."); Serial.println(accTrend); Serial.println("\nVelocidade: ."); Serial.println(velTrend);
  Serial.println("\n\nInicie as compressoes");
  delay(5000);
}

void loop(){  
  acceleration[measurementCounter]= collectAcc();
  collectAccTime[measurementCounter]=millis();
  compressionCount(); // Tracks compressions based on collected and detrended acceleration values

  if(measurementCounter>=accCollectionLimit) {
    compressionDepth = depthMeasure();
    //memset(acceleration,0,sizeof(acceleration)); // não é necessário, mas caso seja...
    measurementCounter=0;
    compCounter=0;
    riseCounter=0;
  }
  Serial.print(millis()); Serial.print(" ; ");
  Serial.print(acceleration[measurementCounter]); Serial.print(" ; ");
  Serial.print(compFreq); Serial.print(" ; ");
  Serial.print(compressionDepth); Serial.print(" ; ");

  measurementCounter++;
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
  unsigned long timeDelta[accCollectionLimit]={};
  float acc5PointMovingAvg[accCollectionLimit]={}, accOffset[accCollectionLimit]={};
  float velocity[accCollectionLimit]={},velOffset[accCollectionLimit]={};
  float displacement[accCollectionLimit]={}, disp121PointMovingAvg[accCollectionLimit]={},dispInCentimeters[accCollectionLimit]={};
  float maxValue[accCollectionLimit]={}, minValue[accCollectionLimit]={};
  float amplitude[accCollectionLimit]={}, avgAmplitude[accCollectionLimit]={};


  movingAverage(acc5PointMovingAvg,acceleration,5); // Calculated a moving average of the acceleration in order to reduce noise
  
  for(int i=1;i<accCollectionLimit;i++) {
    timeDelta[i]=collectAccTime[i]-collectAccTime[i-1]; // Calculates time between measurements
    accOffset[i]=acc5PointMovingAvg[i]-accTrend; // detrends acceleration
  }
  integralCalculator(velocity,accOffset,timeDelta); // Calculates the velocity, which is the integral of the acceleration

  for(int i=1;i<accCollectionLimit;i++) {
    velOffset[i]=velocity[i]-velTrend; // detrends velocity
  }

  integralCalculator(displacement,velOffset,timeDelta); // Calculates displacement, which is the integral of the velocity
  movingAverage(disp121PointMovingAvg,displacement,121);
  for(int i=0;i<accCollectionLimit;i++) {
    dispInCentimeters[i]=(displacement[i]-disp121PointMovingAvg[i])*100;
  }
  getMax(maxValue,dispInCentimeters,61);
  getMin(minValue,dispInCentimeters,61);
  for(int i=0;i<accCollectionLimit;i++) {
    amplitude[i]=maxValue[i]-minValue[i];
  }

  movingAverage(avgAmplitude,amplitude,250);
  double sum=0;
  for(int i=150;i<=(accCollectionLimit-250);i++) { // this is calculating the average amplitude, but only of the points affected by the moving average. Technically, I could've used 125 instead of 150, but i decided to shoot higher for a safe margin
    sum += avgAmplitude[i];
  }

  return (sum/(accCollectionLimit-250-150)); // it's the upper condition (accCollectionLimit-250) minus the bottom condition (150). This returns the average amplitude.
}

bool periodStartCheck(float oldValue, float newValue, float detectionTreshold) {
  bool periodStart = false;

  if(newValue<oldValue && abs(newValue)>detectionTreshold) { // Tracks movement as long as the collected value is higher than the defined treshold
    riseCounter++;
  }

  if(newValue>oldValue && riseCounter>3) { // Tracks the lowest point of a "senoid" by looking for the moment it stops the upwards trend and starts a downwards one.
    periodStart = true;
    riseCounter=0;
  }

  return periodStart;
}

float trendCalc(int stoppingMeasurement, float *measurement) {
  double sum=0;
  float trend=0;
  for(int i = 0;i<stoppingMeasurement;i++) {
    sum += measurement[i];
  }

  trend = sum/stoppingMeasurement;
  return trend;
}

void integralCalculator(float *integral, float *data, unsigned long *period) {
  int stoppingMeasurement = accCollectionLimit;

  integral[stoppingMeasurement]={};
  for(int i=1;i<stoppingMeasurement;i++) {
    integral[i]=((data[i-1]+data[i])*((float)period[i]/1000)/2)+integral[i-1];
  }
}

void movingAverage(float *avg,float *data,int period) {
  int stoppingMeasurement = accCollectionLimit;
  int halfPeriod = (period-1)/2;

  for(int i=0;i<stoppingMeasurement;i++) {
    float sum=0.0;
    for(int j=i-halfPeriod;j<=i+halfPeriod && j>=0 && i+halfPeriod<stoppingMeasurement;j++) { // If j ends up being less than 0, the moving average must not be calculated at all.
      sum += data[j];
    }
    avg[i]= (i+halfPeriod>stoppingMeasurement || i-halfPeriod<0) ? data[i] : sum/period;
  }
}

void getMax(float *maximum, float *data,int period) {
  int stoppingMeasurement = accCollectionLimit;
  int halfPeriod = (period-1)/2;

  for(int i=0;i<stoppingMeasurement;i++) {
    float max=0; // the reason max is created every iteration is that so the maxium of one iteration does not carry over to the next one, to avoid that the maximum value detected isn't actually one from a previous iteration that isn't covered by the current one
    for(int j=i-halfPeriod;j<=i+halfPeriod && j>=0 && i+halfPeriod<stoppingMeasurement;j++) {
      if(data[j]>max) {
        max = data[j];
      }
    }
    maximum[i]=max;
  }
}

void getMin(float *minimum, float *data,int period) {
  int stoppingMeasurement = accCollectionLimit;
  int halfPeriod = (period-1)/2;

  for(int i=0;i<stoppingMeasurement;i++) {
    float min=9999;
    for(int j=i-halfPeriod;j<=i+halfPeriod && j>=0 && i+halfPeriod<stoppingMeasurement;j++) {
      if(data[j]<min) {
        min = data[j];
      }
    }
    minimum[i]=min;
  }
}
