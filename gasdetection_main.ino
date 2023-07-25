
#include "Arduino.h"
#include "Adafruit_BME280.h"
#include "UbidotsEsp32Mqtt.h"
#include "DFRobot_OxygenSensor.h"

#include <Adafruit_GFX.h> 
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <SPI.h>

/*******************OLED***************************/
#define SCREEN_WIDTH 128 // OLED display width, i pixeler
#define SCREEN_HEIGHT 32 // OLED display height, i pixeler

#define OLED_RESET     -1 // Reset pin for OLED
#define SCREEN_ADDRESS 0x3C // 0x3C for 128x32 display

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int info = 0;
/**************************************************/

/*******************MQ2-Explosive_Gass_Sensor***************************/
#define MQ_PIN (34)
#define RL_VALUE (5)
#define RO_CLEAN_AIR_FACTOR (9.83)

#define CALIBARAION_SAMPLE_TIMES  (50)
#define CALIBRATION_SAMPLE_INTERVAL (500)

#define READ_SAMPLE_INTERVAL (50)
#define READ_SAMPLE_TIMES (5)

#define GAS_LPG (0)
#define GAS_CO (1)
#define GAS_SMOKE (2)

// linear regression for the lines from the datasheet
float LPGCurve[3] = {2.3,0.21,-0.47};
float COCurve[3] = {2.3,0.72,-0.34};
float SmokeCurve[3] = {2.3,0.72,-0.34};

float CO = 0;
float CH4 = 0;
float LPG = 0;

float Ro = 10;

/**************************************************/

/*********************O2 SENSOR*****************************/
#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER  10             // collect number, the collection range is 1-100.
DFRobot_OxygenSensor oxygen;

float O2Data = 0;
/**************************************************/

/*******************UBIDOTS***************************/
const char *UBIDOTS_TOKEN = "BBFF-zr88SXpUYZLNQBUMcLgfzua9QMQAzx"; 
const char *WIFI_SSID = "IPhone2";   
const char *WIFI_PASS = "12345678";
     
const char *DEVICE_LABEL = "esp32";   
const char *VARIABLE_LABEL = "test"; 
const char *VARIABLE_LABEL2 = "gps";

const int PUBLISH_FREQUENCY = 5000; // Update rate in millisecondsx

Ubidots ubidots(UBIDOTS_TOKEN);
/**************************************************/

/*******************GPS position***************************/
float latitud = 59.41169;
float longitud = 5.25838;

int gps = 1;

char context[25];
/**************************************************/

/*******************BME280***************************/
Adafruit_BME280 bme;

float Temperature = 0;
float Humidity = 0;
float Preassure = 0;
float Altitude = 0;
/**************************************************/

/*UBIDOTS recive variables*/
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}


void setup(){
  Serial.begin(115200);
  Serial.print("God Morgen");

  /*Sjekker om OLED er tilkoblet*/
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { // hvis vi ikke finner OLED
    Serial.println(F("SSD1306 allocation failed"));
    display.clearDisplay();
    display.display();
    display.setCursor(0,0);
    display.setTextSize(1); // bestemmer tekst størrelse
    display.setTextColor(WHITE); // bestemmer farge
    display.print("SSD1306 allocation failed");
    display.display();
    for(;;); // kjøre dette på loop 
  }
  
  /*sjekker om BME280 sensor er tilkoblet*/
  if (!bme.begin(0x76)) 
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    display.clearDisplay();
    display.display();
    display.setCursor(0,0);
    display.setTextSize(1); // bestemmer tekst størrelse
    display.setTextColor(WHITE); // bestemmer farge
    display.print("Could not find a valid BME280 sensor, check wiring!");
    display.display();
    while (1);
  }

  /* Calibrates the MQ2 gass sensor*/
  Serial.println("Calibrating MQ2 sensor....");
  
  Serial.println("Could not find a valid BME280 sensor, check wiring!");
  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.setTextSize(1); // bestemmer tekst størrelse
  display.setTextColor(WHITE); // bestemmer farge
  display.print("Calibrating MQ2 sensor....");
  display.display();
   
  Ro = MQCalibration(MQ_PIN) * 10; // *10 er ikke nødvendig
  Serial.print("Calibration is done, Ro=");
  Serial.print(Ro);
  Serial.println("kOhm");

  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.setTextSize(1); // bestemmer tekst størrelse
  display.setTextColor(WHITE); // bestemmer farge
  display.print("Calibration done!");
  display.setCursor(0,10);
  display.print("Ro = ");
  display.print(Ro);
  display.print("kOhm");
  display.display();

  /*O2 Sensor*/

  while(!oxygen.begin(Oxygen_IICAddress)){
    Serial.println("I2c device number error !");
    delay(1000);
  }
    Serial.println("I2c connect success !");
    display.clearDisplay();
    display.display();
    display.setCursor(0,0);
    display.setTextSize(1); // bestemmer tekst størrelse
    display.setTextColor(WHITE); // bestemmer farge
    display.print("I2c connect success !");
    display.display();
  
    /* kobler til ubidots*/
    ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
    ubidots.setCallback(callback);
    ubidots.setup();
    ubidots.reconnect();
    ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL);
    Serial.println(context);

  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.setTextSize(1); // bestemmer tekst størrelse
  display.setTextColor(WHITE); // bestemmer farge
  display.print("Online");
  display.display();

  delay(1000);
}

void loop() {
  Read_BME_Values();
  Get_Gass_Value();
  Display_Oled();
  get_oxygen_level();

    /* sjekker om vi fremdeles er tilkoblet wifi */
  if (!ubidots.connected())
  {
    ubidots.reconnect();
    ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL);  
  }
    ubidots.add("airmoisture",Humidity);
    ubidots.publish();
    ubidots.add("temperature",Temperature);
    ubidots.publish();
    ubidots.add("airpressure",Preassure);
    ubidots.publish();
    ubidots.add("COppm",CO);
    ubidots.publish();
    ubidots.add("CH4ppm",CH4);
    ubidots.publish();
    ubidots.add("LPGppm",LPG);
    ubidots.publish();
    ubidots.add("O2",O2Data);
    ubidots.publish();

    gps_posistion();
    
    ubidots.loop();

    delay(1000);
}



/********************GPS POSITION************************/
/*This function takes the long ang lat gps position and transform from float to char
so that Ubidots recognizes lat and lng for the device*/ 
void gps_posistion(){
  
  char* str_lat = (char*)malloc(sizeof(char) * 10);
  char* str_lng = (char*)malloc(sizeof(char) * 10);

  sprintf(str_lat, "%f", latitud);
  sprintf(str_lng, "%f", longitud);

  char* context = (char*)malloc(sizeof(char) * 30);

  ubidots.addContext("lat", str_lat);
  ubidots.addContext("lng", str_lng);

  ubidots.getContext(context);
  Serial.println(context);

  ubidots.add(VARIABLE_LABEL2, 1, context);
  ubidots.publish();
}
/***********************************************/

/********************Print to OLED functions************************/
void Display_Oled() {
  if (info == 0) {
    printToOled(Temperature, Humidity, Preassure);
    info = 1;
  }
  else if (info == 1) {
    Oled_Gas(CO,CH4,LPG);
    info = 2;
  }
  else if (info == 2){
    Oled_O2(O2Data);
    info = 0;
  }
}

void Oled_O2(float O2) {
  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.setTextSize(1); // bestemmer tekst størrelse
  display.setTextColor(WHITE); // bestemmer farge
  display.print("O2 = ");
  display.print(O2);
  display.print("%");
  display.display();
}


void Oled_Gas(float co, float ch4, float lpg){
  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.setTextSize(1); // bestemmer tekst størrelse
  display.setTextColor(WHITE); // bestemmer farge
  display.print("CO: ");
  display.print(co);
  display.print("ppm");
  display.setCursor(0,10);
  display.print("CH4: ");
  display.print(ch4);
  display.print("ppm");
  display.setCursor(0,20);
  display.print("LPG: ");
  display.print(lpg);
  display.print("ppm");
  display.display();
}


void printToOled(float Temp, float Hum, float Pre) { // skriver til skjermen
  display.clearDisplay(); // tømmer displayet 
  display.display(); // gjør det klart til å motta nye data
  display.setCursor(0,0); // definerer hvor jeg vil vise frem data på skjermen
  display.setTextSize(1); // bestemmer tekst størrelse
  display.setTextColor(WHITE); // bestemmer farge
  display.print("Temp ");
  display.print(Temp);
  display.setCursor(0,10); // vil vise neste melding på ny linje
  display.print("Humidity ");
  display.print(Hum);
  display.setCursor(0,20); // vil vise neste melding på ny linje
  display.print("Airpreassure ");
  display.print(Pre);
  display.display();
}
/***********************************************/

/********************BME280 functions************************/
void Read_BME_Values(){
  Temperature = bme.readTemperature();
  Humidity = bme.readHumidity();
  Preassure = bme.readPressure() / 100;
  Altitude = bme.readAltitude(1013.25);
}
/***********************************************/

/********************MQ2 FUNCTIONS************************/
// SE MQ2_EXAMPLES FOR MER FORKLARING AV FUNKSJONER
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(4095-raw_adc)/raw_adc)); //MULIGT AT DEN MÅ GANGES MED 10
}


float MQCalibration(int mq_pin)
{
  int i;
  float val=0;
 
   for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
     val += MQResistanceCalculation(analogRead(mq_pin));
     //Serial.println(val);
    delay(CALIBRATION_SAMPLE_INTERVAL);
   }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average   value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided   by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according   to the chart in the datasheet 
 
  return val; 
}

float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++)   {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    //Serial.println(rs);
    delay(READ_SAMPLE_INTERVAL);
   }
 
  rs = (rs/READ_SAMPLE_TIMES) * 10;
 
  return rs;  
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  //Serial.println(rs_ro_ratio);
  if ( gas_id   == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else   if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
   } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
   }    
 
  return 0;
}


int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,(   ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

void Get_Gass_Value(){
  CO = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
  CH4 = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);
  LPG = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
}

/***********************************************/

/********************o2 SENSOR***************************/
void get_oxygen_level(){
  O2Data = oxygen.getOxygenData(COLLECT_NUMBER);
  Serial.print(" oxygen concentration is ");
  Serial.print(O2Data);
  Serial.println(" %vol");
}
  
/***********************************************/
