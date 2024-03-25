const int MPU_addr=0x68; //Endereço do sensor
float newAcc=0, compFreq;
int riseCounter=0;
unsigned long = newTime=0;

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
  compFreq = testForFreq(acc);
  
  Serial.print(micros()); Serial.print(";"); Serial.print(AcZreal); Serial.print(";"); Serial.print(compFreq);  Serial.println("");
  
  delay(7);
}

int collectAcc() {
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

int testForFreq(float acc) {
  bool risingCheck=false;
  unsigned long compressionTime;
  int compressionFreq;
  
  if(abs(acc)>11) {
    float oldAcc = newAcc;
    newAcc = acc;
    
    if(newAcc>oldAcc) {
      riseCounter++;
      risingCheck = (riseCounter>3) ? true : false;
      if(risingCheck) {
        unsigned long oldTime = newTime;
        newTime = millis();
        
        compressionTime = newTime-oldTime;
        compressionFreq = compressionTime*60000; // extrapolando o tempo de compressão de millisegundos para minutos 
      }
    } else {
      riseCounter=0;
    }  
  }
  
  return compressionFreq;
}