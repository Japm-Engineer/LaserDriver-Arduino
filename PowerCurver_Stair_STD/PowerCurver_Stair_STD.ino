#include <Wire.h> //I2C library
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "Adafruit_SHT31.h"
#define PotentiometerAdress 0x2F //i2c adress of digital potenciometer 

Adafruit_ADS1115 ads;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

const int Steps = 128; // number of datas
float dataADC[Steps];
float dataTemp[Steps];

int DeltaTime = 25;
int flag = 0;
int j;
// int command = 0x01;
int data = 1023; //
int data_to_send;// = (0x01 << 10) | (data) ;

void PotentiometerValue(int IN) {
  Wire.beginTransmission(PotentiometerAdress);
  Wire.write(0x1C); // Disable write protect on RDAC
  Wire.write(0x03); // Disable write protect on RDAC
  data_to_send = (0x01 << 10) | (IN) ;
  Wire.write(data_to_send >> 8);
  Wire.write(data_to_send & 0xFF);
  Wire.endTransmission();
}

void reading (){ //// command by serial
  String input = Serial.readStringUntil('\n');
  String cmd = input.substring(0, input.indexOf(' '));
  cmd.trim();
  float value = input.substring(input.indexOf(' ') + 1).toFloat();
  // self.dispatch(cmd, value)
  if (cmd == "start"){flag = 1;
  } else{ Serial.print(input);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(PotentiometerAdress);
  if (!Wire.endTransmission() == 0) {
    Serial.println("Error de Conexion Potenciometro");while (1) delay(1); }
  if (!sht31.begin(0x44)) {Serial.println("Couldn't find SHT31"); while (1) delay(1); }
  if (!ads.begin()) {Serial.println("Failed to initialize ADS."); while (1) delay(1); }
  delay(500);
  data = 1023;
  PotentiometerValue(data);
}

void loop() {
  if (Serial.available()){
    reading();
    delay(100);
  }
  if (flag == 1){
    Serial.print("Potentiometer,adc,adc_std,temp,temp_std\n");
     for (data = 500; data <= 1023 ; data += 10){
      PotentiometerValue(1023 - data);
      delay(2000);
      for (j = 0; j < Steps; j++){
        dataADC[j] = ads.readADC_SingleEnded(0);
        dataTemp[j] = sht31.readTemperature();
        //Serial.println(sht31.readTemperature(),6);
        delay(20);
      }
      float meanADC = 0; float meanTemp = 0;
      float stdADC = 0; float stdTemp = 0;
      for (int m = 0; m < Steps; m++){
        meanADC = meanADC + dataADC[m];
        meanTemp = meanTemp + dataTemp[m];   
      }
      meanTemp = meanTemp / Steps;
      meanADC = meanADC / Steps;
      for (int m = 0; m < Steps; m++){
        stdADC = stdADC + pow(dataADC[m] - meanADC,2); 
        stdTemp = stdTemp + pow(dataTemp[m] - meanTemp,2);   
      }
      stdADC = sqrt(stdADC/(Steps - 1));
      stdTemp = sqrt(stdTemp/(Steps - 1));
      Serial.print(data);Serial.print(",");
      Serial.print(meanADC,0);Serial.print(",");
      Serial.print(stdADC,2);Serial.print(",");
      Serial.print(meanTemp,2);Serial.print(",");
      Serial.print(stdTemp,4);Serial.print("\n");
   }
   flag = 0;
   PotentiometerValue(1023);
  }
  delay(500);
}
