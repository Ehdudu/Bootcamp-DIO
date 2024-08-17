#include <iostream>
#include <fstream>
#include <unistd.h> // biblioteca pra função sleep


const int accCollectionLimit = 1500; // Defines a hard cap used for resetting the acceleration array once it reaches its limit. 1000 points = +-25 compressions
float acceleration[accCollectionLimit] = {}; // acceleration values collected over time.
float compressionDepth = 0; // stores depth of compression
float accTrend = 0, velTrend = 0;
int measurementCounter = 0; // used for passing values to the acceleration compLimit
unsigned long collectAccTime[accCollectionLimit] = {}; // time of collected acceleration value
float compFreq = 0; // compFreq tracks the average amount of compressions per minute

float depthMeasure();
float trendCalc(int stoppingMeasurement, float *measurement);
void integralCalculator(float *integral, float *data, unsigned long *period, int lastNumber);
void movingAverage(float *avg, float *data, int period, int lastNumber);
void getMax(float *maximum, float *data, int period);
void getMin(float *minimum, float *data, int period);
void printFreq(float freq);
void printDepth(float depth);

int main() {
    std::cout << "Por favor, coloque o acelerometro virado para baixo. O calculo de trend ira iniciar brevemente..." << std::endl;
    std::ifstream serial("/dev/ttyACM0");
    do {
        unsigned long timeDelta[1500] = {};
        float acc5PointMovingAvg[1500] = {}, accOffset[1500] = {}, velocity[1500] = {};
        bool skipFirstCollection=true;
        for (measurementCounter = 0; measurementCounter < accCollectionLimit; measurementCounter++) {
            float acc;
            unsigned long time;
            serial >> acc;
            serial >> time;
            serial >> compFreq;
            if (serial.fail()) {
                std::cerr << "Error: Failed to read from /dev/ttyACM0 during trend calc" << std::endl;
            }
            acceleration[measurementCounter] = acc;
            collectAccTime[measurementCounter] = time;
            if(skipFirstCollection&&measurementCounter>50) { // this is done because the first few values read from the sensor are mixed with gibberish values
              measurementCounter=0;
              acceleration[measurementCounter] = acc;
              skipFirstCollection=false;
            }
        }
        movingAverage(acc5PointMovingAvg, acceleration, 5, 1500); // Calculated a moving average of the acceleration in order to reduce noise
        accTrend = trendCalc(1500, acc5PointMovingAvg); // Calculates the noise trend before compressions start.

        for (int i = 1; i < 1500; i++) {
            timeDelta[i] = collectAccTime[i] - collectAccTime[i - 1]; // Calculates time between measurements
            accOffset[i] = acc5PointMovingAvg[i] - accTrend; // detrends acceleration
        }

        integralCalculator(velocity, accOffset, timeDelta, 1500); // Calculates the velocity, which is the integral of the acceleration
        velTrend = trendCalc(1500, velocity); // Calculates the noise trend in the velocity.

        std::cout << "accTrend:" << accTrend << " velTrend: " << velTrend << std::endl;
    }while(false);

    if (!serial.is_open()) {
        std::cerr << "Error: Failed to open /dev/ttyACM0 midway" << std::endl;
        return 1;
    }

    std::cout << "Iniciando loop principal" << std::endl;

    // Main data collection loop
    while (true) {
        for (measurementCounter = 0; measurementCounter < accCollectionLimit; measurementCounter++) {
            float acc;
            unsigned long time;
            serial >> acc;
            serial >> time;
            serial >> compFreq;
            if (serial.fail()) {
                std::cerr << "Error: Failed to read from /dev/ttyACM0" << std::endl;
            }
            acceleration[measurementCounter] = acc;
            collectAccTime[measurementCounter] = time;
            std::cout << compFreq << " " ;
            printFreq(compFreq);
            std::cout << "" << std::endl;
            std::cout << compressionDepth <<" " ;
            printDepth(compressionDepth);
            std::cout << "" << std::endl;
        }
        std::cout << "1500 pontos coletados" << std::endl;
        compressionDepth = depthMeasure();
    }

    serial.close();

    return 0;
}

float depthMeasure() {
  unsigned long timeDelta[accCollectionLimit]={};
  float acc5PointMovingAvg[accCollectionLimit]={}, accOffset[accCollectionLimit]={};
  float velocity[accCollectionLimit]={},velOffset[accCollectionLimit]={};
  float displacement[accCollectionLimit]={}, disp121PointMovingAvg[accCollectionLimit]={},dispInCentimeters[accCollectionLimit]={};
  float maxValue[accCollectionLimit]={}, minValue[accCollectionLimit]={};
  float amplitude[accCollectionLimit]={}, avgAmplitude[accCollectionLimit]={};


  movingAverage(acc5PointMovingAvg,acceleration,5,accCollectionLimit); // Calculated a moving average of the acceleration in order to reduce noise


  for(int i=1;i<accCollectionLimit;i++) {
    timeDelta[i]=collectAccTime[i]-collectAccTime[i-1]; // Calculates time between measurements
    accOffset[i]=acc5PointMovingAvg[i]-accTrend; // detrends acceleration
  }
  integralCalculator(velocity,accOffset,timeDelta,accCollectionLimit); // Calculates the velocity, which is the integral of the acceleration

  for(int i=1;i<accCollectionLimit;i++) {
    velOffset[i]=velocity[i]-velTrend; // detrends velocity
  }

  integralCalculator(displacement,velOffset,timeDelta,accCollectionLimit); // Calculates displacement, which is the integral of the velocity
  movingAverage(disp121PointMovingAvg,displacement,121,accCollectionLimit);
  for(int i=1;i<accCollectionLimit;i++) {
    dispInCentimeters[i]=(displacement[i]-disp121PointMovingAvg[i])*100;
  }
  getMax(maxValue,dispInCentimeters,61);
  getMin(minValue,dispInCentimeters,61);
  for(int i=0;i<accCollectionLimit;i++) {
    amplitude[i]=maxValue[i]-minValue[i];
  }

  movingAverage(avgAmplitude,amplitude,250,accCollectionLimit);
  double sum=0;
  for(int i=250;i<=(accCollectionLimit-250);i++) { // this is calculating the average amplitude, but only of the points affected by the moving average. Technically, I could've used 125 instead of 150, but i decided to shoot higher for a safe margin
    sum += avgAmplitude[i];
  }

  return (sum/(accCollectionLimit-250-250)); // it's the upper condition (accCollectionLimit-250) minus the bottom condition (250). This returns the average amplitude.
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

void integralCalculator(float *integral, float *data, unsigned long *period, int lastNumber) {
  int stoppingMeasurement = lastNumber;

  for(int i=1;i<stoppingMeasurement;i++) {
    integral[i]=((data[i-1]+data[i])*((float)period[i]/1000)/2)+integral[i-1];
  }
}

void movingAverage(float *avg,float *data,int period, int lastNumber) {
  int stoppingMeasurement = lastNumber;
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

void printFreq(float freq) {
  if(freq>140) {
    std::cout << "Diminua a frequencia"<<std::endl;
  }else if(freq>120) {
    std::cout << "Diminua um pouco a frequencia"<<std::endl;
  }else if(freq<80) {
    std::cout << "Acelere a frequencia"<<std::endl;
  }else if(freq<100) {
    std::cout << "Acelere um pouco a frequencia"<<std::endl;
  }else {
    std::cout << "Mantenha a frequencia"<<std::endl;
  }
}

void printDepth(float depth) { // the ideal depth is between 5 and 6cm. Since I'm using the amplitude, I've decided on a margin of 10-12cm
  if(depth>15) {
    std::cout << "Comprima com menos força"<<std::endl;
  }else if(depth>12) {
    std::cout << "Comprima com um pouco menos força"<<std::endl;
  }else if(depth<7) {
    std::cout << "Comprima com mais força"<<std::endl;
  }else if(depth<10) {
    std::cout << "Comprima com um pouco mais força"<<std::endl;
  }else {
    std::cout << "Mantenha a força"<<std::endl;
  }
}
