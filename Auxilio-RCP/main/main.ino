#include<Wire.h>
const int MPU_addr=0x68; //Endereço do sensor
float newAcc=0; // usado em compressionCount()
int riseCounter=0,compCounter=0; //usados em compressionCount()
unsigned long compStartTime=0, compEndTime=0; //usados em compressionCount()
double compFreq=0,compPeriod=0; //usado em compressionCount()

void setup(){
  Wire.begin(); //Inicia a comunicação I2C
  Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
  Wire.write(0x6B); // Aponta para a "localização" do acelerômetro
  Wire.write(0); // Manda 0 e "acorda" o MPU 6050
  Wire.endTransmission(true);

  // Configuração de sensibilidade de aceleração
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);                  // Aponta para as configurações de aceleração
  Wire.write(0x10);                  // Envia 00010000, que define a escala da aceleração como +/- 8g
  Wire.endTransmission(true);

  // Configuração do filtro de banda
  /*Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);
  Wire.write(0x04);
  Wire.endTransmission(true);*/

  Serial.begin(115200); //Inicia a comunicaçao serial (para exibir os valores lidos)
  delay(5000); //delay de 5 segundos antes do programa iniciar para que o teste possa ser arrumado antes do início do programa
}

void loop(){
  float acc;
  acc = collectAcc();
  compressionCount(acc);



  
  Serial.print(compCounter); Serial.print(";"); Serial.print(compPeriod); Serial.print(";"); Serial.println(compFreq);
  //Serial.print(millis()); Serial.print(";"); Serial.print(acc); Serial.print(";"); Serial.println(compFreq);
  
  delay(7);
}

float collectAcc() {
  int16_t rawAcc; //Variaveis para pegar os valores medidos
  float gAcc; //Variável para armazenar o valor real de aceleração
  
  Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
  Wire.write(0x3F); // registrador dos dados medidos (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,2,true); // faz um "pedido" para ler 2   registradores, que serão os registrados com os dados medidos
  rawAcc=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  gAcc=(rawAcc/4096.0)*9.80665;
  
  return gAcc;
}

void compressionCount(float acc) {    
  float oldAcc = newAcc;
  newAcc = acc;

  if(newAcc>oldAcc && riseCounter>3) {
    riseCounter=0;
    compCounter++;
    if(compCounter==1){
      compStartTime=millis();
    }
    if(compCounter==10){
      compEndTime=millis();
      compCounter=0;
      compPeriod = (double)(compEndTime - compStartTime);
      compFreq = (double)(600000/compPeriod);
    }
  } 

  if(newAcc<oldAcc && abs(newAcc)>11) {
    riseCounter++;
  }
}
