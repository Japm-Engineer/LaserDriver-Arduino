#include <Wire.h>  //I2C library
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_SHT31.h>
#include <ESP32Time.h>
#define PotentiometerAdress 0x2F  //i2c adress of digital potenciometer
#define vect1 20
#define vect2 20

Adafruit_ADS1115 ads;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
ESP32Time rtc;
// software Parameters.
int data_to_send;  // = (0x01 << 10) | (data) ;
int data;
int flag = 0;  // star routine
char buffer[40];
char buffer_2[40];
float t0;          // time zero.
float t = 0;       // current time
float tf = 0;      // past time
int STEP = 33000;  //sampling step
float systemtime = 0.031;
float tempvec[vect1];
float templine[vect2];
int cutoff = 0;
int j;
int m = -1;
int past_step = 1;
int filter_step = 5;
float support = 1;
// Power calibration.
// Power Calib Model::: Power = Ap*Vadc + Bp*Temp(ºC) + Cp ::: "02 - 04 - 2024"
const float Ap = 363.51E-6;
const float Bp = 15.14E-4;
const float Cp = -0.1039;

// Current to Count Calib
// Current Model::: R(count) = Ai * I + Bi :  I in mA
const float Ai = 51.030;
const float Bi = 19.64;

//Threshold current model
// I_{th} = Ot + At*EXP(Bt * Temperature)
float Ot = 7.023;
float At = 73.51e-5;
float Bt = 0.030;

// variable declaration
int vadc;
float temp;
float temp_past;
float power;
// PID parametters
float kp = 1;
float ti = 1;
float powerref = 3.8;
float current;

// PID parameter programming
float e;
float ei = 0;
float uact;  //float eD0;

//PI support.
float I_support = 0;

// potentiometer routine.
void PotentiometerValue(int IN) {
  Wire.beginTransmission(PotentiometerAdress);
  Wire.write(0x1C);  // Disable write protect on RDAC
  Wire.write(0x03);  // Disable write protect on RDAC
  data_to_send = (0x01 << 10) | (IN);
  Wire.write(data_to_send >> 8);
  Wire.write(data_to_send & 0xFF);
  Wire.endTransmission();
}

void reading() {  //// command by serial
  String input = Serial.readStringUntil('\n');
  String cmd = input.substring(0, input.indexOf(' '));
  cmd.trim();
  float value = input.substring(input.indexOf(' ') + 1).toFloat();
  // self.dispatch(cmd, value)
  if (cmd == "start") {
    flag = 1;
    if (cutoff == 0) {
      cutoff = 1;
      ei = 0;
    }
  } else if (cmd == "stop") {
    flag = 0;
    m = -1;
    cutoff = 0;
  } else if (cmd == "kp") {
    kp = value;
  } else if (cmd == "ti") {
    ti = value;
  } else if (cmd == "ref") {
    powerref = value;
  } else if (cmd == "support") {
    support = value;
  } else if (cmd == "past") {
    past_step = int(value);
  } else if (cmd == "filter") {
    filter_step = int(value);
  } else {
    Serial.print(input);
  }
}

void setup() {
  Serial.begin(115200);
  rtc.setTime(0, 0, 0, 1, 1, 2021);
  Wire.begin();
  Wire.beginTransmission(PotentiometerAdress);
  if (!Wire.endTransmission() == 0) { Serial.println("Error de Conexion Potenciometro"); }
  if (!sht31.begin(0x44)) {
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }  // Set to 0x45 for alternate i2c addr
  delay(500);
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }
  data = 1023;
  PotentiometerValue(data);  //SET control signal to 0
  // put your setup code here, to run once:
}


void loop() {
  if (Serial.available()) {
    reading();
    delay(100);
    if (flag == 1) {
      sprintf(buffer_2, "supported=%0.0f,filtersteps=%d,paststeps=%d,pref=%0.1f,Kp=%0.2f,ti=%0.2f\n", support, filter_step, past_step, powerref, kp, ti);
      Serial.print(buffer_2);
      Serial.print("t,powerref,power,current,Isupport,temp\n");
      for (j = (vect1-filter_step); j < vect1; j++){tempvec[j] = sht31.readTemperature();}
      temp = 0;
      for (j = (vect1-filter_step); j < vect1; j++){temp = temp + tempvec[j];}
      temp = temp/filter_step;
      for (j = (vect2-filter_step); j < vect2; j++){templine[j] = temp;}
      temp_past = sht31.readTemperature();
      t0 = float(3600 * rtc.getHour(true) + 60 * rtc.getMinute() + rtc.getSecond()) + float(rtc.getMillis()) / 1000;
      m = 0;
      I_support = 0;
    }
  }


  temp = sht31.readTemperature();
  tf = t;

  // read parameters
  for (j = (vect1-filter_step); j < vect1-1; j++){tempvec[j] = tempvec[j+1];}
  tempvec[vect1-1]= sht31.readTemperature();
  temp = 0;
  for (j = (vect1-filter_step); j < vect1; j++){temp = temp + tempvec[j];}
  temp = temp/filter_step;
  for (j = (vect2-filter_step); j < vect2-1; j++){templine[j] = templine[j+1];}
  templine[vect2-1]= round(100*temp)/100;
  

  //act support;
  I_support =support*(Ot + At * exp(Bt * (temp + 273.15)));
  temp_past = temp;
  current = cutoff * (kp * e + (kp / ti) * ei + I_support);

  vadc = ads.readADC_SingleEnded(0);
  // PID

  power = Ap * vadc + Bp * temp + Cp;
  e = powerref - power;
  ei = ei + e * systemtime;  // riemann rule
  current = cutoff * (kp * e + (kp / ti) * ei + I_support);
  uact = Ai * current + Bi;
  if (uact < 0) {
    uact = 0;
  } else if (uact > 1023) {
    uact = 1023;
  }
  data = 1023 - int(uact);
  PotentiometerValue(data);
  if (m >= 0 && m <= STEP) {
    t = float(3600 * rtc.getHour(true) + 60 * rtc.getMinute() + rtc.getSecond()) + float(rtc.getMillis()) / 1000;
    sprintf(buffer, "%0.3f,%0.2f,%0.2f,%0.3f,%0.3f,%0.3f\n", t - t0, powerref, power, (int(uact) - Bi) / Ai, I_support, temp);
    Serial.print(buffer);
    m++;
  }
}