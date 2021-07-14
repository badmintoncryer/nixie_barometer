/*
BME280 I2C Test.ino

This code shows how to record data from the BME280 environmental sensor
using I2C interface. This file is an example file, part of the Arduino
BME280 library.

GNU General Public License

Written: Dec 30 2015.
Last Updated: Oct 07 2017.

Connecting the BME280 Sensor:
Sensor              ->  Board
-----------------------------
Vin (Voltage In)    ->  3.3V
Gnd (Ground)        ->  Gnd
SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro

 */

#include <BME280I2C.h>
#include <Wire.h>
#include <Ticker.h>

/* Definition of constant value */
#define SERIAL_BAUD 115200
#define TUBE1 0
#define TUBE2 1
#define TUBE3 2
#define TUBE4 3
#define TUBE5 4
/* Dot enable flag.
   It's used at showing temperature & humidity */
#define DOT_DISABLE 0
#define DOT_ENABLE 1
/* Index flag */
#define PRESSURE 0
#define TEMPERATURE 1
#define HUMIDITY 2
#define OUT_OF_RANGE 3
/* Definition of numbers corresponding to the symbols want to display on IN-19A. */
/* Original nixie-board version (ニキシー管を半田している基板)
 *    1 : k
 *    2 : n
 *    3 : micro
 *    4 : None
 *    5 : C
 *    6 : %
 *    7 : None
 *    8 : M
 *    9 : P */ 
#define P_EXPRESS_NUM 9
#define DEGREE_EXPRESS_NUM 5
#define PERCENT_EXPRESS_NUM 6
/* New nixie-board version */
//#define P_EXPRESS_NUM 1
//#define DEGREE_EXPRESS_NUM 6
//#define PERCENT_EXPRESS_NUM 8
/* True/False */
#define TRUE 1
#define FALSE 0
#define TEST_PIN 13
/* ESP32 pin definition. */
#define K155ID1_A 26
#define K155ID1_B 33
#define K155ID1_C 32
#define K155ID1_D 25
#define TUBE1_ENABLE_PIN 17
#define TUBE2_ENABLE_PIN 16
#define TUBE3_ENABLE_PIN 0
#define TUBE4_ENABLE_PIN 2
#define TUBE5_ENABLE_PIN 15
#define DOT_ENABLE_PIN 4
/* Definition of lighting time */
#define LIGHT_DELAY_MSEC 2
/* Definition of temperature calibration straight line associated with case mounting. */
#define TEMPERATURE_SLOPE 0.8852
#define TEMPERATURE_INTERCEPT -3.7191
/* Definition of delay time for ghost countermeasures. */
#define DELAY_MICROSECOND 400

/* Definition of global variable */
volatile uint8_t sign_flag = 0;
volatile uint8_t get_data_flag = 0;
volatile uint8_t dot_flag = 0;
uint8_t num_array[5] = {0};
hw_timer_t *timer = NULL;
portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED;

/* Definition of the structure of measure data */
typedef struct measure_data {
  float pressure;
  float temperature;
  float humidity;
} measure_data_t;
measure_data_t measure_data;

/* Interrupt Service Routin */
void IRAM_ATTR on_timer(){
  portENTER_CRITICAL_ISR(&timer_mux);
  get_data_flag = TRUE;
  portEXIT_CRITICAL_ISR(&timer_mux);
}

BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

uint8_t get_bme280_data(Stream* client, uint8_t* num_array);
uint8_t create_tube_num(uint8_t* num_array, measure_data_t* measure_data);
uint8_t light_tube(uint8_t light_num, uint8_t tube_num, uint8_t dot_flag);

//////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(SERIAL_BAUD);
  while(!Serial) {
  } // Wait
  Wire.begin();

  while(!bme.begin()) {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch(bme.chipModel()){
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }

  get_bme280_data(&Serial, &measure_data);
  create_tube_num(num_array, &measure_data);

  /* 必要なpinMode()を一通りセットする */
  pinMode(TEST_PIN, OUTPUT);
  pinMode(TUBE1_ENABLE_PIN, OUTPUT);
  pinMode(TUBE2_ENABLE_PIN, OUTPUT);
  pinMode(TUBE3_ENABLE_PIN, OUTPUT);
  pinMode(TUBE4_ENABLE_PIN, OUTPUT);
  pinMode(TUBE5_ENABLE_PIN, OUTPUT);
  pinMode(K155ID1_A, OUTPUT);
  pinMode(K155ID1_B, OUTPUT);
  pinMode(K155ID1_C, OUTPUT);
  pinMode(K155ID1_D, OUTPUT);
  pinMode(DOT_ENABLE_PIN, OUTPUT);
  digitalWrite(TEST_PIN, HIGH);
  digitalWrite(TUBE1_ENABLE_PIN, LOW);
  digitalWrite(TUBE2_ENABLE_PIN, LOW);
  digitalWrite(TUBE3_ENABLE_PIN, LOW);
  digitalWrite(TUBE4_ENABLE_PIN, LOW);
  digitalWrite(TUBE5_ENABLE_PIN, LOW);
  digitalWrite(K155ID1_A, LOW);
  digitalWrite(K155ID1_B, LOW);
  digitalWrite(K155ID1_C, LOW);
  digitalWrite(K155ID1_D, LOW);
  digitalWrite(DOT_ENABLE_PIN, LOW);

  /* タイマーの設定 */
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &on_timer, true);
  /* 3sに1回表示を切り替える */
  timerAlarmWrite(timer, 3000000, true);
  timerAlarmEnable(timer);
}

//////////////////////////////////////////////////////////////////
void loop()
{
   /* 1管ずつ切り替えながら点灯を行う(ダイナミック点灯) */
   light_tube(num_array[0], TUBE1, DOT_DISABLE);
   light_tube(num_array[1], TUBE2, dot_flag);
   light_tube(num_array[2], TUBE3, DOT_DISABLE);
   light_tube(num_array[3], TUBE4, DOT_DISABLE);
   light_tube(num_array[4], TUBE5, DOT_DISABLE);

   /* タイマー割込みが入る(get_data_flagがFALSE->TRUEになる)たびに、指標の取得＆表示の切り替えを行う */
   if (get_data_flag == TRUE) {
      get_bme280_data(&Serial, &measure_data);
      correct_temp_and_hum(&measure_data);
      create_tube_num(num_array, &measure_data);
      get_data_flag = FALSE;
   }
}

//////////////////////////////////////////////////////////////////
void printBME280Data(Stream* client)
{
  if(client == NULL) {
    /* error */
  }
   
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   client->print("Temp: ");
   client->print(temp);
   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
   client->print("\t\tHumidity: ");
   client->print(hum);
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(pres);
   client->println(" Pa");

   delay(1);
}

uint8_t get_bme280_data(Stream* client, measure_data_t* measure_data)
{
  if (client == NULL || measure_data == NULL) {
    /* error */
  }
   
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   measure_data->pressure = pres;
   measure_data->temperature = temp;
   measure_data->humidity = hum;

   return 0;
}

/* ケース装着に伴う温度と湿度の校正を行う
　 校正式はケース有無での温度の散布図を基に回帰直線から値を決定した */
uint8_t correct_temp_and_hum(measure_data_t* measure_data)
{
  if (measure_data == NULL) {
    /* error */
  }
  
  float temp, cor_temp, cor_hum, water_vapor_amount, sat_water_vapor_amount, cor_sat_water_vapor_amount;

  temp = measure_data->temperature;
  cor_temp = temp * TEMPERATURE_SLOPE + TEMPERATURE_INTERCEPT;

  /* Tetens(1930)の式を用いて湿度を補正 */
  sat_water_vapor_amount = 217 * (6.1078 * pow(10, ((7.5 * temp) / (temp + 237.3)))) / (temp + 273.15);
  water_vapor_amount = sat_water_vapor_amount * measure_data->humidity / 100;
  cor_sat_water_vapor_amount = 217 * (6.1078 * pow(10, ((7.5 * cor_temp) / (cor_temp + 237.3)))) / (cor_temp + 273.15);
  cor_hum = water_vapor_amount / cor_sat_water_vapor_amount * 100;

  measure_data->temperature = cor_temp;
  measure_data->humidity = cor_hum;

  return 0;
}

/* 各管で表示させる数字を決定する関数 */
uint8_t create_tube_num(uint8_t* num_array, measure_data_t* measure_data)
{
  if (num_array == NULL || measure_data == NULL) {
    /* error */
  }

  uint16_t pres_val, temp_val, hum_val;
  uint8_t  i;

  sign_flag++;
  if (sign_flag == OUT_OF_RANGE) {
    sign_flag = PRESSURE;
  }

  switch (sign_flag) {
    /* pressure */
    case PRESSURE:
      pres_val = (uint16_t)(measure_data->pressure / 100);
      num_array[0] = pres_val / 1000;
      num_array[1] = (pres_val - num_array[0] * 1000) / 100;
      num_array[2] = (pres_val - num_array[0] * 1000 - num_array[1] * 100) / 10;
      num_array[3] = (pres_val - num_array[0] * 1000 - num_array[1] * 100 - num_array[2] * 10);
      num_array[4] = P_EXPRESS_NUM;
      dot_flag = DOT_DISABLE;
      break;

    /* temperature */
    case TEMPERATURE:
      if (measure_data->temperature < 0) {
        /* 温度がマイナスの場合、0℃表示にする
        (マイナス環境で使わないでしょう...) */
        for (i=0; i<4; i++){
          num_array[i] = 0;
        }
      } else {
        /* 温度がプラスの場合 */
        temp_val = (uint16_t)(measure_data->temperature * 100);
        num_array[0] = temp_val / 1000;
        num_array[1] = (temp_val - num_array[0] * 1000) / 100;
        num_array[2] = (temp_val - num_array[0] * 1000 - num_array[1] * 100) / 10;
        num_array[3] = (temp_val - num_array[0] * 1000 - num_array[1] * 100 - num_array[2] * 10);
      }
      num_array[4] = DEGREE_EXPRESS_NUM;
      dot_flag = DOT_ENABLE;
      break;

    /* humidity */
    case HUMIDITY:
      hum_val = (uint16_t)(measure_data->humidity * 100);
      num_array[0] = hum_val / 1000;
      num_array[1] = (hum_val - num_array[0] * 1000) / 100;
      num_array[2] = (hum_val - num_array[0] * 1000 - num_array[1] * 100) / 10;
      num_array[3] = (hum_val - num_array[0] * 1000 - num_array[1] * 100 - num_array[2] * 10);
      num_array[4] = PERCENT_EXPRESS_NUM;
      dot_flag = DOT_ENABLE;
      break;

    default:
      return 1;
  }

  return 0;
}

uint8_t light_tube(uint8_t light_num, uint8_t tube_num, uint8_t dot_flag)
{
  if (light_num > 9 || tube_num > TUBE5 || dot_flag > DOT_ENABLE) {
    /* error */
  }

  switch (light_num) {
    case 0:
      digitalWrite(K155ID1_A, LOW);
      digitalWrite(K155ID1_B, LOW);
      digitalWrite(K155ID1_C, LOW);
      digitalWrite(K155ID1_D, LOW);
      break;
    case 1:
      digitalWrite(K155ID1_A, HIGH);
      digitalWrite(K155ID1_B, LOW);
      digitalWrite(K155ID1_C, LOW);
      digitalWrite(K155ID1_D, LOW);
      break;
    case 2:
      digitalWrite(K155ID1_A, LOW);
      digitalWrite(K155ID1_B, HIGH);
      digitalWrite(K155ID1_C, LOW);
      digitalWrite(K155ID1_D, LOW);
      break;
    case 3:
      digitalWrite(K155ID1_A, HIGH);
      digitalWrite(K155ID1_B, HIGH);
      digitalWrite(K155ID1_C, LOW);
      digitalWrite(K155ID1_D, LOW);
      break;
    case 4:
      digitalWrite(K155ID1_A, LOW);
      digitalWrite(K155ID1_B, LOW);
      digitalWrite(K155ID1_C, HIGH);
      digitalWrite(K155ID1_D, LOW);
      break;
    case 5:
      digitalWrite(K155ID1_A, HIGH);
      digitalWrite(K155ID1_B, LOW);
      digitalWrite(K155ID1_C, HIGH);
      digitalWrite(K155ID1_D, LOW);
      break;
    case 6:
      digitalWrite(K155ID1_A, LOW);
      digitalWrite(K155ID1_B, HIGH);
      digitalWrite(K155ID1_C, HIGH);
      digitalWrite(K155ID1_D, LOW);
      break;
    case 7:
      digitalWrite(K155ID1_A, HIGH);
      digitalWrite(K155ID1_B, HIGH);
      digitalWrite(K155ID1_C, HIGH);
      digitalWrite(K155ID1_D, LOW);
      break;
    case 8:
      digitalWrite(K155ID1_A, LOW);
      digitalWrite(K155ID1_B, LOW);
      digitalWrite(K155ID1_C, LOW);
      digitalWrite(K155ID1_D, HIGH);
      break;
    case 9:
      digitalWrite(K155ID1_A, HIGH);
      digitalWrite(K155ID1_B, LOW);
      digitalWrite(K155ID1_C, LOW);
      digitalWrite(K155ID1_D, HIGH);
      break;
/*    default:
       error */
  }

  if (dot_flag == DOT_ENABLE) {
    digitalWrite(DOT_ENABLE_PIN, HIGH);
  }

  switch (tube_num){
    case TUBE1:
      digitalWrite(TUBE1_ENABLE_PIN, HIGH);
      delay(LIGHT_DELAY_MSEC);
      digitalWrite(TUBE1_ENABLE_PIN, LOW);
      break;
    case TUBE2:
      digitalWrite(TUBE2_ENABLE_PIN, HIGH);
      delay(LIGHT_DELAY_MSEC);
      digitalWrite(TUBE2_ENABLE_PIN, LOW);
      break;
    case TUBE3:
      digitalWrite(TUBE3_ENABLE_PIN, HIGH);
      delay(LIGHT_DELAY_MSEC);
      digitalWrite(TUBE3_ENABLE_PIN, LOW);
      break;
    case TUBE4:
      digitalWrite(TUBE4_ENABLE_PIN, HIGH);
      delay(LIGHT_DELAY_MSEC);
      digitalWrite(TUBE4_ENABLE_PIN, LOW);
      break;
    case TUBE5:
      digitalWrite(TUBE5_ENABLE_PIN, HIGH);
      delay(LIGHT_DELAY_MSEC);
      digitalWrite(TUBE5_ENABLE_PIN, LOW);
      break;
  }
  /* ゴースト対策
     必要な待機時間はトライ&エラーで決定 */
  delayMicroseconds(DELAY_MICROSECOND);

  if (dot_flag == DOT_ENABLE) {
    digitalWrite(DOT_ENABLE_PIN , LOW);
  }

  return 0;
}
