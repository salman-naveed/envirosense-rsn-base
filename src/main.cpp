#include <GP2YDustSensor.h>
#include <SD.h>
#include <SPI.h>
#include "RTClib.h"
#include "MQ135.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"
#include <String.h>
#include <ArduinoJson.h>


#define runAvgSamples 1000 //number of samples taken for the running average
#define dust_Pin A7     // Mega Pin A7 (connect to Dust Sensor Pin 5)
#define SenseLED_pin 12 //Mega Pin D12 (connect to Dust Sensor Pin 3)
#define DHTPIN 30 //Connect Mega Pin 30 to Data pin of DHT22
#define DHTTYPE DHT22//22  //DHT version

const uint8_t SHARP_LED_PIN = SenseLED_pin;       // Dust sensor LED Pin (Pin 3 on Dust sensor)
const uint8_t SHARP_VO_PIN = dust_Pin;          // Dust sensor analog output pin (Pin 5 on Dust Sensor)
const uint16_t RUN_AVG_SAMPLES = runAvgSamples; // number of samples taken for the running average
const int MQ135_VO_PIN=A0;   //MQ-135(NOx Sensor) analog output pin
const int MQ7_VO_PIN = A1;//MQ-7(CO Sensor) analog output pin
const int MQ7_feedback_pin = A3;
const int MQ7_PWM_PIN = 9;//MQ-7(CO Sensor) PWM pin
const int FAN =   32; // Dust sensor fan pin
const int chipSelect = 53; // SPI CS pin for SD CARD

unsigned long time_now = 0;


const int PWR_CTRL = 5;
uint32_t pos = 0;
float reference_resistor_kOhm = 4.7; //fill correct resisor value if you are using not 10k reference
float sensor_reading_clean_air = 470; //fill raw sensor value at the end of measurement phase (before heating starts) in clean air here! That is critical for proper calculation
float sensor_reading_100_ppm_CO = -1; //if you can measure it 
//using some CO meter or precisely calculated CO sample, then fill it here
//otherwise leave -1, default values will be used in this case
float sensor_100ppm_CO_resistance_kOhm; //calculated from sensor_reading_100_ppm_CO variable
float sensor_base_resistance_kOhm; //calculated from sensor_reading_clean_air variable
float sens_val = 0; //current smoothed sensor value
float last_CO_ppm_measurement = 0; //CO concentration at the end of previous measurement cycle

float Pressure = 0;

float Co_level = 0;

int temperature[2] = {};

float Nox = 0;

unsigned long epoch_sec = 0;

bool stat = 0;
char character ;
int line_count =0 ;
float pm = 0;
float temp= 0;
float humidity = 0;


GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1010AU0F, SHARP_LED_PIN, SHARP_VO_PIN, RUN_AVG_SAMPLES);
RTC_DS3231 rtc;
File dataFile;
Adafruit_BMP280 bmp;
DHT dht(DHTPIN, DHTTYPE);
MQ135 gasSensor = MQ135(MQ135_VO_PIN);
String json = "{\"ts\":0,\"values\":{\"humidity\":0, \"coCon\":0,\"no2Con\":0,\"temperature\":0,\"pressure\":0,\"pm25Con\":0,\"latitude\":0,\"longitude\":0}}";
DynamicJsonDocument doc(2048);
DeserializationError err =  deserializeJson(doc, json);
JsonObject obj = doc.as<JsonObject>();

float raw_value_to_CO_ppm(float value);
void setTimer2PWM(byte chA, byte chB);
void startMeasurementPhase();
void startHeatingPhase();
float GetCOLevel();
float GetPressureLevel();
void GetTempAndHumidLevel(int *temp, int siz);
float GetNOxLevel();
float GetPM25Level();
void Delay();
bool sendCardData();
void getStatus();

////////////////////////////////////////////////////////////////////////////////////






////////////////////FUNCTIONS RELATED TO CO///////////////////////////////////////////////////////////////////////
void setTimer2PWM(byte chA, byte chB) //pins D11 and D3
{
  TCCR2A = 0b10100011; //OCA,OCB, fast pwm
  TCCR2B = 0b001; //no prescaler
  OCR2A = chA; //0..255
  OCR2B = chB;
}




float raw_value_to_CO_ppm(float value)
{
  if(value < 1) return -1; //wrong input value
  sensor_base_resistance_kOhm = reference_resistor_kOhm * 1023 / sensor_reading_clean_air - reference_resistor_kOhm;
  if(sensor_reading_100_ppm_CO > 0)
  {
    sensor_100ppm_CO_resistance_kOhm = reference_resistor_kOhm * 1023 / sensor_reading_100_ppm_CO - reference_resistor_kOhm;
  }
  else
  {
    sensor_100ppm_CO_resistance_kOhm = sensor_base_resistance_kOhm * 0.25;
  }
  float sensor_R_kOhm = reference_resistor_kOhm * 1023 / value - reference_resistor_kOhm;
  float R_relation = sensor_100ppm_CO_resistance_kOhm / sensor_R_kOhm;
  float CO_ppm = 134 * R_relation - 35;
  if(CO_ppm < 0) CO_ppm = 0;
  return CO_ppm;
}

void startMeasurementPhase()
{
  setTimer2PWM(0, 4);
}

void startHeatingPhase()
{
  setTimer2PWM(0, 255);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  pinMode(FAN,OUTPUT);//Dust sensor FAN 
  pinMode(PWR_CTRL,OUTPUT); //Power control
  pinMode(MQ7_PWM_PIN, OUTPUT); //MQ-7 pwm
  pinMode(MQ7_VO_PIN, INPUT); // MQ-7 output
  pinMode(MQ135_VO_PIN, INPUT); // MQ-135 output
  pinMode(MQ7_feedback_pin,INPUT); // MQ-7 feedback
  analogReference(DEFAULT);
  Serial.begin(9600);
  Serial2.begin(9600);
  delay(100);
  digitalWrite(FAN,LOW);
  digitalWrite(PWR_CTRL,HIGH);
  //delay(100);
  dht.begin();
  Serial.println("DHT begin successful");
  //delay(1000);
  dustSensor.setBaseline(0.68);            // Sets the voltage at no dust according to your own experiments
  
  dustSensor.begin();                     //Initialize sensor. Sets SenseLED_pin (D12) as output
  Serial.println("Sharp begin successful");
  rtc.begin();
  delay(1000);
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
  }
   if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
 // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
               //   Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
               //   Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
               //   Adafruit_BMP280::FILTER_X16,      /* Filtering. */
               //   Adafruit_BMP280::STANDBY_MS_1000);
  SD.begin(chipSelect);
  Serial.println("SD card begin successful");
  bmp.begin(0x76,0x58);
  Serial.println("BMP begin successful");
  
digitalWrite(PWR_CTRL,LOW);

}

void loop()   
{

  ///////////////////////////////TURN ON MAIN POWER//////////////////////////////////////
     
    digitalWrite(PWR_CTRL,HIGH);
    digitalWrite(FAN,HIGH);
    Serial.println("FAN ON"); 
    delay(10000);       
    

///////////////////////////////GET SENSORS DATA//////////////////////////////////////
    delay(10000);
   
    Co_level = GetCOLevel();
    Serial.println("GetCOLevel() Complete");
       
    Pressure = round(GetPressureLevel());
    Serial.println("GetPressureLevel() Complete");
    
    Nox = GetNOxLevel();
    Serial.println("GetNOxLevel() Complete");
    
    pm = GetPM25Level();
    Serial.println("GetPM25Level() Complete");
    
    digitalWrite(FAN,LOW);
    Serial.println("FAN OFF");
    
    GetTempAndHumidLevel(temperature,2);    
    temp = round(temperature[1]);
    humidity = round(temperature[0]);
    Serial.println("GetTempAndHumidLevel() Complete");
/////////////////////////////////////////////////////////////////////////////////////


//////////////////GET RTC TIME AND CONVERT INTO STRING OF MILLISECOND////////////////
DateTime Now = rtc.now();
epoch_sec = Now.unixtime()-18000L; 
/*if(epoch_sec>1623150000)
{
  epoch_sec = 1622650989;
}*/
String kVal = String(epoch_sec)+"000";
 
////////////////////////////////////////////////////////////////////////////////////


////////////////STORING DATA IN RESPECTIVE JSON TOPICS////////////////////////////
obj[String("ts")] = kVal;
obj[String("values")][String("coCon")] = Co_level;
obj[String("values")][String("no2Con")] = Nox;
obj[String("values")][String("pm25Con")] = pm;
obj[String("values")][String("pressure")] = Pressure;
obj[String("values")][String("humidity")] = humidity;
obj[String("values")][String("temperature")] = temp;
obj[String("values")][String("longitude")] = 67.056237;
obj[String("values")][String("latitude")] = 24.960722;
String output;
serializeJson(doc, output);
//////////////////////////////////////////////////////////////////////////////////


///////////////////////////GET ESP CONNECTIVITY STATUS///////////////////////
getStatus();
////////////////////////////////////////////////////////////////////////////////


stat = 0;

while(!SD.begin(chipSelect)){
    SD.begin(chipSelect);
    Serial.println("Setting up SD CARD");
    
  }
if(character == 'E'){  
//////////////////////// Sending data to ESP-01///////////////////////////////
Serial.println("Sending Data to ESP-01");

//////////////CHECK WHETHER SD CARD CONTAINS DATALOG/////////////
bool file_exist =  SD.exists("DATALOG.TXT");

if (file_exist){
Serial.println("Sending SD Card Data to ESP-01");
Serial2.print('C');
stat = sendCardData();
Serial.print("Data Send:");
Serial.print(stat);
}
///////////////////////////////////////////////////////////////
file_exist = SD.exists("DATALOG.TXT");
if((file_exist==0)){ 
/////////////////SENDING REAL TIME DATA TO ESP///////////////
if(stat){
  Serial.print("Send SD-CARD to cloud status: ");
  Serial.println(stat);
  delay(100);
  Serial2.print(output);
  delay(100);
  Serial2.print('>');
  Serial.println(output);
  }
  else{
Serial2.print('R');
Serial2.flush();
Serial2.print(output);
Serial.println(output);
Serial.flush();
delay(100);
Serial2.print('>');
}
}
////////////////////////////////////////////////////////////
}
////////////////////////////////////////////////////////////////////////////


else if (stat == 0 || character == 'F')
  {
///////////////////// SENDING DATA TO SD CARD //////////////////////////////
    while(!SD.begin(chipSelect)){
    SD.begin(chipSelect);
    Serial.println("Setting up SD CARD");
    
  }
if(SD.begin(chipSelect)){
  Serial.println("Sending Data to SD CARD");
 dataFile = SD.open("DATALOG.TXT",FILE_WRITE);
  if (dataFile) {
    dataFile.println(output);
    dataFile.close();
    // print to the serial port too:
    Serial.println(output);
    SD.end();
 
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  
}


}
////////////////////////////////////////////////////////////////////////


Delay(); 
}

float GetCOLevel(){
int count =0;
uint32_t one_sec = 0;
uint32_t low_cycle =   90000;       // 90 seconds.Low heating cycle
uint32_t high_cycle = 60000;        // 60 seconds.High heating cycle
float v = 0;
for( uint32_t tStart = millis();  (millis()-tStart) < high_cycle;  ){
   
    startHeatingPhase();
    
    if(millis() - one_sec >= 1000){
      
    Serial.print(analogRead(MQ7_VO_PIN));
    Serial.print(" ");
    Serial.println(last_CO_ppm_measurement);
    one_sec = millis();
    } 
  }
   

 // current_millis = millis();
 one_sec = 0;
   for( uint32_t tStart = millis();  (millis()-tStart) < low_cycle;  ){

    startMeasurementPhase();
        if(millis() - one_sec >= 1000){
      
   v = analogRead(MQ7_VO_PIN);
   // sens_val *=0.999;
   // sens_val +=0.001*v;
    
    count++;
    Serial.print(v);
    Serial.print(" ");
    Serial.println(last_CO_ppm_measurement);
    v *=0.999;
    v +=0.01*v;
    one_sec = millis();
  
  }

   }

 last_CO_ppm_measurement = raw_value_to_CO_ppm(v);
 return last_CO_ppm_measurement;

}

float GetPressureLevel(){
float value;
bmp.begin(0x76,0x58);
delay(100);
value  = bmp.readPressure();
return value;
}

void GetTempAndHumidLevel(int *temp, int siz){
 dht.begin();
  delay(1000);
  temp[0] = int(dht.readHumidity());
  temp[1] = int(dht.readTemperature());
  if(isnan(temp[0]) || isnan(temp[1])){
    temp[0] =0;
    temp[1] = 0;
  }
}
float GetNOxLevel(){
float ppm = 0;
float value = 0;
for(int i =1 ;i<=100;i++){
  ppm+=gasSensor.getPPM();
}
ppm = (ppm/100);
value = map(ppm,0,3000,0.5,10);
return value;
}
float GetPM25Level(){
  float dustDensitySample = 0;
  float dustDensityRunningAvg = 0;
  //for(int i=0; i<100; i++)
    //{
  dustDensitySample = dustSensor.getDustDensity(); 
  dustDensityRunningAvg = dustSensor.getRunningAverage(); //Get the running average value of dust density using RUN_AVG_SAMPLES number of samples
// delay(500);
    //}
  return dustDensityRunningAvg;
}

void Delay(){
uint32_t one_sec = 0;
int count =0;
digitalWrite(PWR_CTRL,LOW);
uint32_t sleep = 1 * 6000;
for(uint32_t tStart = millis();  (millis()-tStart) < sleep;){
if(millis() - one_sec >= 1000){
 Serial.print(count);
  Serial.println(" sec");
  one_sec = millis();
  count++;
}
}

}

bool sendCardData(){
char Command = ' ';
char rec= ' ';
bool Transfer = false;
String json = "";
    while(!SD.begin(chipSelect)){
    SD.begin(chipSelect);
    Serial.println("Setting up SD CARD");
    
  }
dataFile = SD.open("DATALOG.TXT",FILE_READ);
if(dataFile.available()>0){
  ;
}
dataFile.seek(pos); 
Serial.println(dataFile.seek(pos));
   for(int i=0;i<line_count;i++){
      dataFile.readStringUntil('\n');
    }
    
    while (dataFile.available()>0) {
    json = dataFile.readStringUntil('\n');
    Serial2.print(json);
    delay(100);
    Serial2.print('>');
    Command =' ';
    while(!((Command == 'N') ||(Command == 'F'))){
      if(Serial2.available()>0){
        Command = char(Serial2.read());
        Serial.println(Command);
        
      }
      delay(4000);
    }
        if(Command == 'F'){
        character = 'F';
        Transfer = false;
        SD.end();
        break;
        
      }
      line_count++;
  }
 dataFile.close();
  //delay(2000);
  if(Command != 'F'){
  Serial.println("File completed");
  delay(100);
  Serial2.print('T');
  SD.remove("DATALOG.TXT");
  Serial2.flush();
  line_count = 0;
  Transfer = true;
  SD.end();
}
Serial.println(Transfer);
return Transfer;
  }

void getStatus(){
  Serial2.print('S');
  Serial2.flush();
  delay(5000);
  character = ' ';
  delay(1000);
  //Serial.println(Serial2.available());
  if (Serial2.available()>0){
  while (!(character=='E' || character=='F'))
  {
  
    character = char(Serial2.read());
    Serial.println(character);
      
  }
    Serial.println(character);
  }
}
