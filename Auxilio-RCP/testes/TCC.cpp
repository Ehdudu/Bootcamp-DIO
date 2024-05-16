#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

const int accCollectionLimit=1805; // Defines a hard cap used for resetting the acceleration array once it reaches its limit. 1000 points = +-25 compressions
float compressionDepth=0; // stores depth of compression
int measurementCounter=0; // used for passing values to the acceleration array
int startingPoint=200; // This will be used to keep track of when the first compression started, so a trend can be calculated later
float acceleration[accCollectionLimit]={}; // acceleration values collected over time.
unsigned long collectAccTime[accCollectionLimit]={}; // time of collected acceleration value

float trendCalc(int stoppingMeasurement, float measurement[]);
void integralCalculator(float *integral, float *data, unsigned long *period);
void movingAverage(float *avg,float *data,int period);
void getMax(float *maximum, float *data,int period);
void getMin(float *minimum, float *data,int period);
float depthMeasure();

int main(){
    // Open the CSV file
    std::ifstream file("teste.csv");
    if (!file.is_open()) {
        std::cerr << "Error opening file.\n";
        return 1;
    }

    std::string line;
    int index = 0;
    while (std::getline(file, line) && index < accCollectionLimit) {
        std::stringstream ss(line);
        std::string token;

        // Read double value
        if (std::getline(ss, token, ',')) {
            acceleration[index] = std::stod(token);
        } else {
            std::cerr << "Error reading double value.\n";
            return 1;
        }

        // Read unsigned long value
        if (std::getline(ss, token, ';')) {
            collectAccTime[index] = std::stoul(token);
        } else {
            std::cerr << "Error reading unsigned long value.\n";
            return 1;
        }

        ++index;
    }

    // Close the file
    file.close();


  compressionDepth = depthMeasure();
  std::cout<<compressionDepth;
  return 0;
}

float depthMeasure() {
  unsigned long timeDelta[accCollectionLimit]={};
  float acc5PointMovingAvg[accCollectionLimit]={}, accOffset[accCollectionLimit]={};
  float velocity[accCollectionLimit]={},velOffset[accCollectionLimit]={};
  float displacement[accCollectionLimit]={}, disp121PointMovingAvg[accCollectionLimit]={}, dispInCentimeters[accCollectionLimit]={};
  float maxValue[accCollectionLimit]={}, minValue[accCollectionLimit]={};
  float amplitude[accCollectionLimit]={}, avgAmplitude[accCollectionLimit]={};
  float accTrend=0, velTrend=0;

  movingAverage(acc5PointMovingAvg,acceleration,5); // validado

  accTrend = trendCalc(startingPoint,acc5PointMovingAvg);// validado



  for(int i=1;i<accCollectionLimit;i++) { // ambos os valores foram validados
    timeDelta[i]=collectAccTime[i]-collectAccTime[i-1];
    accOffset[i]=acc5PointMovingAvg[i]-accTrend;
  }


  integralCalculator(velocity,accOffset,timeDelta); // validado
  velTrend=trendCalc(startingPoint,velocity); // validado






  for(int i=1;i<accCollectionLimit;i++) { // validado
    velOffset[i]=velocity[i]-velTrend; //
  }


  integralCalculator(displacement,velOffset,timeDelta); // validado
  movingAverage(disp121PointMovingAvg,displacement,121);// validado




  for(int i=0;i<accCollectionLimit;i++) { // validado
    dispInCentimeters[i]=(displacement[i]-disp121PointMovingAvg[i])*100;
  }


  getMax(maxValue,dispInCentimeters,61); // validado
  getMin(minValue,dispInCentimeters,61); // validado




  for(int i=0;i<accCollectionLimit;i++) { // validado
    amplitude[i]=maxValue[i]-minValue[i];
  }


  movingAverage(avgAmplitude,amplitude,250);

/*  std::cout<<velTrend;
  for(int i=0;i<accCollectionLimit;i++) {
    std::cout<<i<<" "<<avgAmplitude[i]<<"\n";
  }*/


  double sum=0;
  for(int i=560;i<=(accCollectionLimit-250);i++) { // this is calculating the average amplitude, but only of the points affected by the moving average. Technically, I could've used 125 instead of 150, but i decided to shoot higher for a safe margin
    sum += avgAmplitude[i];
  }

  return (sum/(accCollectionLimit-250-560)); // the 300 comes from the fact it's the upper condition (accCollectionLimit-15) minus the bottom condition (150). This returns the average amplitude.
}

float trendCalc(int stoppingMeasurement, float *measurement) { // checado e ok
  double sum=0;
  float trend=0;
  for(int i = 0;i<stoppingMeasurement;i++) {
    sum += measurement[i];
  }

  trend = sum/stoppingMeasurement;
  return trend;
}

void integralCalculator(float *integral, float *data, unsigned long *period) { // checado e ok
  integral[accCollectionLimit]={};
  for(int i=1;i<accCollectionLimit;i++) {
    integral[i]=((data[i-1]+data[i])*((float)period[i]/1000)/2)+integral[i-1];
  }
}

void movingAverage(float *avg,float *data,int period)   { // checado e ok
  int halfPeriod = (period-1)/2;
  for(int i=0;i<accCollectionLimit;i++) {
    float sum=0.0;
    for(int j=i-halfPeriod;j<=i+halfPeriod && j>=0 && i+halfPeriod<accCollectionLimit;j++) { // If j ends up being less than 0, the moving average must not be calculated at all.
      sum += data[j];
    }
    avg[i]= (i+halfPeriod>accCollectionLimit || i-halfPeriod<0) ? data[i] : sum/period;
  }
}

void getMax(float *maximum, float *data,int period) { // checado e ok
  int halfPeriod = (period-1)/2;

  for(int i=0;i<accCollectionLimit;i++) {
    float max=0; // the reason max is created every iteration is that so the maxium of one iteration does not carry over to the next one, to avoid that the maximum value detected isn't actually one from a previous iteration that isn't covered by the current one
    for(int j=i-halfPeriod;j<=i+halfPeriod && j>=0 && i+halfPeriod<accCollectionLimit;j++) {
      if(data[j]>max) {
        max = data[j];
      }
    }
    maximum[i]=max;
  }
}

void getMin(float *minimum, float *data,int period) { // checado e ok
  int halfPeriod = (period-1)/2;

  for(int i=0;i<accCollectionLimit;i++) {
    float min=9999;
    for(int j=i-halfPeriod;j<=i+halfPeriod && j>=0 && i+halfPeriod<accCollectionLimit;j++) {
      if(data[j]<min) {
        min = data[j];
      }
    }
    minimum[i]=min;
  }
}
