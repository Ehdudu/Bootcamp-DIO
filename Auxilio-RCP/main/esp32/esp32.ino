#include<Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

const int MPU_addr=0x68; // Accelerometer's I2C address
const int accCollectionLimit=1000; // Defines a hard cap used for resetting the acceleration array once it reaches its limit. 1000 points = +-25 compressions
float acceleration[accCollectionLimit]={}; // acceleration values collected over time.
unsigned long collectAccTime[accCollectionLimit]={}; // time of collected acceleration value
int measurementCounter=0; // used for passing values to the acceleration array

const char* ssid = "TCC";
const char* password = "12345678";

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

  Serial.begin(115200);

  // Starts WiFi connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(""); Serial.println("Connected to the WiFi network.");
  Serial.print("IP Address:"); Serial.println(WiFi.localIP());
  




  //Serial.println("Por favor, coloque o acelerometro virado para baixo. O calculo de trend se inicia em 5 segundos");
  delay(5000); // realizar o calculo de trend no PC.
  
  for(int i=0;i<500;i++) {
    acceleration[i]=collectAcc();
    collectAccTime[i]=millis();
  }
}

void loop(){  
  acceleration[measurementCounter]=collectAcc();
  collectAccTime[measurementCounter]=millis();

  if(measurementCounter>=accCollectionLimit) {
    measurementCounter=0;
  }

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
