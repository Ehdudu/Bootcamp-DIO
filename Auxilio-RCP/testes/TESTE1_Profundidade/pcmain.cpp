#include <iostream>
#include <fstream>
#include <chrono> // biblioteca pra gerenciar runtime
#include <unistd.h> // biblioteca pra função sleep


const int accCollectionLimit = 1500; // Defines a hard cap used for resetting the acceleration array once it reaches its limit. 1000 points = +-25 compressions
const int compLimit = 25; // defines the amount of compressions necessary for the compression frequency to be calculated
float acceleration[accCollectionLimit] = {}; // acceleration values collected over time.
float compressionDepth = 0; // stores depth of compression
float accTrend = 0, velTrend = 0;
int measurementCounter = 0; // used for passing values to the acceleration array
int compCounter = 0; // compCounter tracks amount of compressions made
int riseCounter = 0; // riseCounter is used for tracking the movement of a graph similar to a senoid, being used to identify the moment it stops rising and starts a downwards trend.
int startingPoint = 0; // This will be used to keep track of when the first compression started, so a trend can be calculated later
unsigned long compStartTime = 0, compEndTime = 0; // compStartTime tracks the instant of time of the first compression, compEndTime tracks the instant of time after a specific number of compressions defined on compressionCount()
unsigned long collectAccTime[accCollectionLimit] = {}; // time of collected acceleration value
double compFreq = 0; // compFreq tracks the average amount of compressions per minute 

unsigned long millis();
void compressionCount();
float depthMeasure();
bool periodStartCheck(float oldValue, float newValue, float detectionTreshold);
float trendCalc(int stoppingMeasurement, float *measurement);
void integralCalculator(float *integral, float *data, unsigned long *period, int lastNumber);
void movingAverage(float *avg, float *data, int period, int lastNumber);
void getMax(float *maximum, float *data, int period);
void getMin(float *minimum, float *data, int period);

int main() {
    std::cout << "Por favor, coloque o acelerometro virado para baixo. O calculo de trend se inicia em 5 segundos" << std::endl;
    sleep(5);
    std::ifstream serial("/dev/ttyACM0");
    // Cálculo de trend do sensor. Ele está com esse do while para que as variáveis criadas dentro dele fiquem dentro dele e sejam liberadas após início
    do {
        unsigned long timeDelta[1500] = {};
        float acc5PointMovingAvg[1500] = {}, accOffset[1500] = {}, velocity[1500] = {};

        // Coleta de dados para o cálculo de trend
        for (int i = 0; i < 1500; i++) {

            float value;
            serial  >>  value;
            if (serial.fail()) {
                std::cerr << "Error: Failed to read from /dev/ttyACM0" << std::endl;
                return 1;
            }
            acceleration[i] = value;
            collectAccTime[i] = millis();
            std::cout <<  acceleration[i] << ";" << collectAccTime[i] <<  std::endl;
            usleep(7000);
        }
        serial.close(); // fecha a porta serial para que nao acumulem dados antigos nela que interfiram nas proximas coletas.

        //Inicio dos calculos
        movingAverage(acc5PointMovingAvg, acceleration, 5, 1500); // Calculated a moving average of the acceleration in order to reduce noise
        accTrend = trendCalc(1500, acc5PointMovingAvg); // Calculates the noise trend before compressions start.

        for (int i = 1; i < 1500; i++) {
            timeDelta[i] = collectAccTime[i] - collectAccTime[i - 1]; // Calculates time between measurements
            accOffset[i] = acc5PointMovingAvg[i] - accTrend; // detrends acceleration
        }

        integralCalculator(velocity, accOffset, timeDelta, 1500); // Calculates the velocity, which is the integral of the acceleration
        velTrend = trendCalc(1500, velocity); // Calculates the noise trend in the velocity.

        sleep(2);
        std::cout << "accTrend:" << accTrend << " velTrend: " << velTrend << std::endl;
    } while (false);


    std::cout << "Calculo de trend finalizado, programa iniciando" << std::endl;
    sleep(5);
    std::cout << "Programa iniciado" << std::endl;

    std::ofstream outFile("compFreq.csv");
    bool condTemp = true;
    while (condTemp) {
        serial.open("/dev/ttyACM0");
        for (measurementCounter = 0; measurementCounter < accCollectionLimit; measurementCounter++) {
            float value;
            serial >> value;
            if (serial.fail()) {
                std::cerr << "Error: Failed to read from /dev/ttyACM0" << std::endl;
                return 1;
            }
            acceleration[measurementCounter] = value;
            collectAccTime[measurementCounter]=millis();
            std::cout <<  acceleration[measurementCounter] << ";" << collectAccTime[measurementCounter] << std::endl;
            compressionCount();
            outFile << compFreq<< std::endl;
            usleep(7000);
        }
        serial.close(); // fecha a porta sempre que as coletas terminaram para nao acumular dados antigos
        compressionDepth = depthMeasure();
        compCounter=0;
        riseCounter=0;

        condTemp = false;
    }
    outFile.close();
    std::cout << "Compression Depth: " << compressionDepth << std::endl;
    return 0;
}

unsigned long millis() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
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

bool periodStartCheck(float oldValue, float newValue, float detectionTreshold) {
  bool periodStart = false;

  if(newValue<oldValue && abs(newValue)>detectionTreshold) { // Tracks movement as long as the collected value is higher than the defined treshold
    riseCounter++;
  }

  if(newValue>oldValue && riseCounter>1) { // Tracks the lowest point of a "senoid" by looking for the moment it stops the upwards trend and starts a downwards one.
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
