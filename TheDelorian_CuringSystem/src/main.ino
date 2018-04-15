#include <Particle_SI7021.h>
#include "WS2801.h"
#include <application.h>
#include "math.h"
// #include "MDNS.h"
#define TCAADDR 0x70
#define MAX_SENSORS 8

#define TEMP_API "ZO3QN5CIYSFT5PE0"
#define HUMID_API "P5MV8HSFT8PLORHR"
// MDNS mdns;
SI7021 sensor;

double temperature;
double humidity;
int deviceid;

double temperatures[8];
double humidities[8];
double connected[8];
int neg1_conv = 1;
uint8_t currentSensor = 0;
unsigned long lastPublish = 0;
unsigned long publishRate = 1000.0 * 60.0; //millis
bool blink_state = false;
unsigned long lastBlink = 0;
unsigned long blinkRate = 333.0;

//Cooking Profiles & States
const int numPixel = 8;
// Set the argument to the NUMBER of pixels.
Adafruit_WS2801 strip = Adafruit_WS2801(numPixel);

bool cooked [7];
bool cooled [7];
double delta_T_amb[7];
double heatingRate[7];
unsigned long  lastTime[7];
double threshold_HeatingRate = 5.55 / (60.0*60.0); //10deg / hr -> deg/s
double threshold_cooled = 5.55;
double threshold_cooked = 16.0;
double cookTemp = 45.0;
double lowPassRate = 0.05;

String api_key = TEMP_API; // Replace this string with a valid ThingSpeak Write API Key.
String field1 = "";
String field2 = "";  // i.e. field2 is null
String field3 = "";
String field4 = "";
String field5 = "";
String field6 = "";
String field7 = "";
String field8 = "";
String lat = "0.0";
String lon = "0.0";
String el = "0.0";
String status = "";


//Cooking Profile Functions
void setPixleColorWithWheel(int i, uint8_t WheelPos)
{
  uint8_t red, green, blue;
  if (WheelPos < 85) {
   red = WheelPos * 3; green = 255 - WheelPos * 3; blue = 0;
  } else if (WheelPos < 170) {
   WheelPos -= 85;
   red = 255 - WheelPos * 3; green =  0; blue = WheelPos * 3;
  } else {
   WheelPos -= 170;
   red = 0; green = WheelPos * 3; blue = 255 - WheelPos * 3;
  }
  strip.setPixelColor(i, red, green, blue);
}

void reset_cookingStates(int box_index){
  cooked[box_index] = false;
  cooled[box_index] = false;
}

void reset_cookingValues(int box_index){
  delta_T_amb[box_index] = 0;
  heatingRate[box_index] = 0;
}

void cookingColor(int box_index, double currentdTemp){
  double frac = max(min((currentdTemp )/(cookTemp - temperatures[0]),1.0),0.0);
  //Interpolate Light Between Blue(170) -> Red(85)
  uint8_t whl = 170 - (frac)*85;
  setPixleColorWithWheel(box_index+1,whl);
}

void coolingColor(int box_index, double currentdTemp){
  double frac = max(min(( currentdTemp)/(cookTemp - temperatures[0]),1.0),0.0);
  //Interpolate Light Between Red(85) -> Green(0)
  uint8_t whl = 85 - (1- frac)*85;
  setPixleColorWithWheel(box_index+1,whl);
}

void humidityColor( double humidity){
  double frac = sqrt(sqrt(sqrt(max(min( humidity/100.0 ,1.0),0.0))));
  //Interpolate Light Between Blue(100) - White (0)
  uint8_t red = 255 * (1-frac);
  uint8_t green = 200 * (1-frac);
  uint8_t blue = 200;
  strip.setPixelColor(0, red, green, blue);
}

void idleColor(int box_index){
  setPixleColorWithWheel(box_index+1,170);
}

void doneColor(int box_index){
  //Flash Green
  if (blink_state){
    setPixleColorWithWheel(box_index+1,0);
  }
  else{
    strip.setPixelColor(box_index+1, 0, 0, 0);
  }
}

void disconnectedColor(int box_index){
  //Flash Blue
  if (blink_state){
    setPixleColorWithWheel(box_index+1,170);
  }
  else{
    strip.setPixelColor(box_index+1, 0, 0, 0);
  }
}



void CheckBoxCookingSession(int box_index){
  //Index Keeping
  int sensor_index = box_index + 1; //Don't Forget Ambient
  unsigned long thisTime = millis();
  bool heating;
  bool cooling;
  bool steady;
  //Sensor Parsing

  if ( connected[sensor_index] ){
    double dt = abs(thisTime - lastTime[box_index])/1000.0;
    double instantatiousHeatingRate = (((temperatures[sensor_index] - temperatures[0])- delta_T_amb[box_index]) / (dt));
    heatingRate[box_index] = heatingRate[box_index] * (1-lowPassRate) + instantatiousHeatingRate*(lowPassRate);
    delta_T_amb[box_index] = temperatures[sensor_index] - temperatures[0];
    //Time Keeping
    lastTime[box_index] = thisTime;

    //Check If Heating & Set Flag
    if (((heatingRate[box_index] < threshold_HeatingRate) &&
        (heatingRate[box_index] > -threshold_HeatingRate)) ||
        (delta_T_amb[box_index] < threshold_cooled)){
          heating = false;
          cooling = false;
          steady = true;
        }
    else if ( heatingRate[box_index] > threshold_HeatingRate){
      heating = true;
      cooling = false;
      steady = false;
    }
    else if ( heatingRate[box_index] < -threshold_HeatingRate){
      heating = false;
      cooling = true;
      steady = false;
    }

    //Check If Cooled
    if (cooled[box_index]){
      //Flash Green LED
      doneColor(box_index);
    }
    else if (cooked[box_index]){//Check If Cooked
      coolingColor(box_index,delta_T_amb[box_index]);
      //Set Cooled If Temp Below threshold_delta_T
      if (delta_T_amb[box_index] < (threshold_cooled )) { //Its Cooled
        cooled[box_index] = true;
        //Send Soap Ready Event
        Particle.publish("Luva Ready", String(sensor_index)+","+
                                       String(temperatures[sensor_index])+"C"
                                       ,60,PUBLIC);
      }
    }
    //Check If Heating
    if ( heating ){
        cookingColor(box_index,delta_T_amb[box_index]);
        // Capture Recycle Event
        if (cooled[box_index]){
        //Reset Logic Values Because It Can't Be Cooling & Heating
         reset_cookingStates(box_index);
        }
    }
    else if (cooling){ //if (delta_T_amb[box_index] < threshold_cooled ) { //Check If Idle
         //coolingColor(box_index,delta_T_amb[box_index]);
         if ( !cooked[box_index] &&
            (delta_T_amb[box_index] > (threshold_cooked))) {
           cooked[box_index] = true;
           Particle.publish("Peak Temp", String(sensor_index)+","+
                                          String(temperatures[sensor_index])+"C"
                                          ,60,PUBLIC);
         }
    }
    else if (steady && !cooled[box_index]){
      idleColor(box_index);
    }
  }
  else{
    //Reset States
    reset_cookingStates(box_index);
    reset_cookingValues(box_index);
    //Blink Blue
    disconnectedColor(box_index);
  }
}


//Box 1.0 Stuff
void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
    //Serial.begin(9600);
    Wire.setSpeed(10000);
    Wire.begin();

    strip.begin();

    Particle.variable("Temp", temperature);
    Particle.variable("Humidity", humidity);
    tcaselect(currentSensor);

    // bool success = mdns.setHostname("soapsentinel");
    // if (success) {
    //     mdns.addService("tcp", "http", 80, "Soap Info");
    //     success = mdns.begin();
    // }

}

void getData(uint8_t sensor_index){
  temperature = ((float) sensor.getCelsiusHundredths())/100.0;
  temperatures[sensor_index] = temperature;

  // humidity is an integer representing percent
  humidity = (float) sensor.getHumidityPercent();
  humidities[sensor_index] = humidity;
}

void nodData(uint8_t sensor_index){
  temperatures[sensor_index] = neg1_conv;
  humidities[sensor_index] = neg1_conv;
}

void checkSensor( uint8_t sensor_index){
  tcaselect(sensor_index);
  sensor.begin();

  if (sensor.sensorExists()){
    //If Connected Get Data
    getData( sensor_index );
    connected[sensor_index] = true;
  }
  else{
    nodData( sensor_index );
    connected[sensor_index] = false;
  }

}



void loop() {
    //Blink Status
    unsigned long thisTime = millis();
    if ((thisTime - lastBlink ) > blinkRate){
      blink_state = !blink_state;
      lastBlink = thisTime;
    }

    for (uint8_t currentSensor =0; currentSensor < MAX_SENSORS; currentSensor++){
      delay(10);
      checkSensor(currentSensor);
      //First Sensor Is Ambient
      if (currentSensor > 0){ CheckBoxCookingSession(currentSensor-1); }

    }
    humidityColor(humidities[0]);
    strip.show();

    if ((thisTime - lastPublish ) > publishRate){
      publishTemp();
      publishHumidity();
      lastPublish = thisTime;
    }
}

void publishTemp() {
    // To write multiple fields, you set the various fields you want to send
    api_key = TEMP_API;
    field1 = String(temperatures[0]);
    field2 = String(temperatures[1]);
    field3 = String(temperatures[2]);
    field4 = String(temperatures[3]);
    field5 = String(temperatures[4]);
    field6 = String(temperatures[5]);
    field7 = String(temperatures[6]);
    field8 = String(temperatures[7]);

    String TSjson;
    createTSjson(TSjson);
    Particle.publish("writeThingSpeak",TSjson,60,PUBLIC);
}

void publishHumidity() {
    // To write multiple fields, you set the various fields you want to send
    api_key = HUMID_API;
    field1 = String(humidities[0]);
    field2 = String(humidities[1]);
    field3 = String(humidities[2]);
    field4 = String(humidities[3]);
    field5 = String(humidities[4]);
    field6 = String(humidities[5]);
    field7 = String(humidities[6]);
    field8 = String(humidities[7]);

    String TSjson;
    createTSjson(TSjson);
    Particle.publish("writeThingSpeak",TSjson,60,PUBLIC);
}

void createTSjson(String &dest)
{
  // dest = "{ \"k\":\"" + api_key + "\", \"1\":\""+ field1 +"\", \"2\":\""+ field2 +"\",\"3\":\""+ field3 +"\",\"4\":\""+ field4 +"\",\"5\":\""+ field5 +"\",\"6\":\""+ field6 +"\",\"7\":\""+ field7 +"\",\"8\":\""+ field8 +"\",\"a\":\""+ lat +"\",\"o\":\""+ lon +"\",\"e\":\""+ el +"\", \"s\":\""+ status +"\"}";

    dest = "{";
    if(field1.length()>0){
        dest = dest + "\"1\":\""+ field1 +"\",";
    }
    if(field2.length()>0){
        dest = dest + "\"2\":\""+ field2 +"\",";
    }
    if(field3.length()>0){
        dest = dest + "\"3\":\""+ field3 +"\",";
    }
    if(field4.length()>0){
        dest = dest + "\"4\":\""+ field4 +"\",";
    }
    if(field5.length()>0){
        dest = dest + "\"5\":\""+ field5 +"\",";
    }
    if(field6.length()>0){
        dest = dest + "\"6\":\""+ field6 +"\",";
    }
    if(field7.length()>0){
        dest = dest + "\"7\":\""+ field7 +"\",";
    }
    if(field8.length()>0){
        dest = dest + "\"8\":\""+ field8 +"\",";
    }
    if(lat.length()>0){
        dest = dest + "\"a\":\""+ lat +"\",";
    }
    if(lon.length()>0){
        dest = dest + "\"o\":\""+ lon +"\",";
    }
    if(el.length()>0){
        dest = dest + "\"e\":\""+ el +"\",";
    }
    if(status.length()>0){
        dest = dest + "\"s\":\""+ status +"\",";
    }
    dest = dest + "\"k\":\"" + api_key + "\"}";
}

/*#include "Adafruit_Si7021.h"
Adafruit_Si7021 sensor = Adafruit_Si7021();
int Tempature = 0;
int TempF = 0;

void setup()   {
sensor.begin();
}

void loop() {

// Tempature measurement
  Tempature = (sensor.readTemperature();
  TempF = (( Tempature*9.0)/5.0+32.0);

//Display Temp and Hum on OLED
  display.print("Hum: "); display.println(sensor.readHumidity(), 2);
  display.setCursor(0,40);
  display.print("Tmp: "); display.println(sensor.readTemperature(), 2);

Particle.publish("Tempature", String(Tempature) + " °C", 60, PRIVATE);

}*/

/*
#include <Particle_SI7021.h>


SI7021 sensor;
int led1 = 3;
int led2 = 4;

void setup() {
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    sensor.begin();
}


void loop() {

    // temperature is an integer in hundredths
    int temperature = sensor.getCelsiusHundredths();
    temperature = temperature / 100;
    for (int i = 0; i < temperature; i++) {
        pulse(led1);
    }

    delay(5000);

    // humidity is an integer representing percent
    int humidity = sensor.getHumidityPercent();
    for (int i = 0; i < humidity; i++) {
        pulse(led2);
    }

    delay(5000);

    // this driver should work for SI7020 and SI7021, this returns 20 or 21
    int deviceid = sensor.getDeviceId();
    for (int i = 0; i < deviceid; i++) {
        pulse(led1);
    }
    delay(5000);

    // enable internal heater for testing
    sensor.setHeater(true);
    delay(20000);
    sensor.setHeater(false);

    // see if heater changed temperature
    temperature = sensor.getCelsiusHundredths();
    temperature = temperature / 100;
    for (int i = 0; i < temperature; i++) {
        pulse(led2);
    }

    //cool down
    delay(20000);

    // get humidity and temperature in one shot, saves power because sensor takes temperature when doing humidity anyway
    si7021_env data = sensor.getHumidityAndTemperature();
    for (int i = 0; i < data.celsiusHundredths/100; i++) {
        pulse(led1);
    }
    for (int i = 0; i < data.humidityBasisPoints/100; i++) {
        pulse(led2);
    }
    delay(5000);
}

void pulse(int pin) {
   // software pwm, flash light for ~1 second with short duty cycle
   for (int i = 0; i < 20; i++) {
       digitalWrite(pin, HIGH);
       delay(1);
       digitalWrite(pin,LOW);
       delay(9);
   }
   digitalWrite(pin,LOW);
   delay(300);
}*/
