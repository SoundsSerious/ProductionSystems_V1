#include <Particle_SI7021.h>
#include <application.h>
#include "math.h"
#include "clickButton.h"

#define TCAADDR 0x70
#define MAX_SENSORS 1

#define DELORIAN_API "5432Y5DW7IXMW332"

// MDNS mdns;
SI7021 sensor;

//ButtonPin
// const int buttonPin1 = 4;
#define BUTTON_PIN D4
#define LED_PIN D5
#define RELAYPOWER_PIN D6
ClickButton button1(BUTTON_PIN, LOW, CLICKBTN_PULLUP);

//Variables
bool ON;
bool Thermal_Protection = true; //Assume We Start With Thermal Protection On

float MAX_TEMP = 45.0; //C
float TEMP_HYSTERISIS_VALUE = 0.5; //C

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

//WebhookDataOut
String api_key = DELORIAN_API; // Replace this string with a valid ThingSpeak Write API Key.
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



void setup() {
    //Serial.begin(9600);
    Wire.setSpeed(10000);
    Wire.begin();

    pinMode(D4, INPUT_PULLUP);
    pinMode(D5, OUTPUT);
    pinMode(D6, OUTPUT);

    // Setup button timers (all in milliseconds / ms)
    // (These are default if not set, but changeable for convenience)
    button1.debounceTime   = 20;   // Debounce timer in ms
    button1.multiclickTime = 250;  // Time limit for multi clicks
    button1.longClickTime  = 1000; // time until "held-down clicks" register

    Particle.variable("Temp", temperature);
    Particle.variable("Humidity", humidity);

}

void checkInput(){
  button1.Update();
  int clicks = button1.clicks;
  if (clicks == 0){} //Pass On Ish
  else{ //We Got Somethign Jim
    ON = !ON; //Toggle States
    if (ON){ Particle.publish("Delorian Status","ON");}
    else {  Particle.publish("Delorian Status", "OFF"); }
  }
}

void lightButton(){
  if(ON){
    //If Thermal Protection Is Acvite, Blink The Button
    if (Thermal_Protection){
      if ( blink_state ){
        digitalWrite(LED_PIN,HIGH);
      }
      else{
        digitalWrite(LED_PIN,LOW);
      }
    }
    else{ //Light Button All The Time If On
      digitalWrite(LED_PIN,HIGH);
    }
  }
  else{
    //No Lighto Buddy
    digitalWrite(LED_PIN,LOW);
  }
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

void checkTempSensor(){
  sensor.begin();

  if (sensor.sensorExists()){
    //If Connected Get Data
    getData( 0 );
    connected[0] = true;
  }
  else{
    nodData( 0 );
    connected[0] = false;
  }

}

void loop() {
    //Blink Status
    unsigned long thisTime = millis();
    if ((thisTime - lastBlink ) > blinkRate){
      blink_state = !blink_state;
      lastBlink = thisTime;
    }

    checkInput();
    lightButton();
    checkTempSensor();
    IRHeatTemperatureMangament();

    //Publish Information
    if ((thisTime - lastPublish ) > publishRate){
      publishStatus();
      lastPublish = thisTime;
    }
    delay(5);
}

void IRHeatTemperatureMangament(){
  //Check If ON
  if (ON){
    //Check If Temp Is Over Max
    if ( temperatures[0] > MAX_TEMP){
      //Check If Thermal Protection Mode Is Not Active Then Activate It
      if (!Thermal_Protection){ // This Will Set The First Time Over Temp
        Thermal_Protection = true;
        Particle.publish("Thermal Protection", "Activated");
      }
      //Turn Off Light Above Max
      digitalWrite(RELAYPOWER_PIN,LOW);
    }
    //Else Check If Temperature Is Less Than The Hysterisis Value
    else if( temperatures[0] < MAX_TEMP - TEMP_HYSTERISIS_VALUE){
      //Check If Thermal Protection is set, if so deactivate it
      if (Thermal_Protection){
        Thermal_Protection = false;
        Particle.publish("Thermal Protection", "Deactivated");
      }
      //When Below The Hystersis Range Dump The Heat
      digitalWrite(RELAYPOWER_PIN,HIGH);
    }
    else{ // For Temperatures Less Than MAX, but In The Hysteriss Range:
      //1. If Thermal Protection Is Active, Do Not Turn On Light
      if (Thermal_Protection){
        digitalWrite(RELAYPOWER_PIN,LOW);
      }
      //2.  Otherwise Use That Heat Buddy!
      else{
        digitalWrite(RELAYPOWER_PIN,HIGH);
      }
    }
  }
  else {
    //Turn Off Light
    digitalWrite(RELAYPOWER_PIN,LOW);
  }
}

void publishStatus() {
    // To write multiple fields, you set the various fields you want to send
    api_key = DELORIAN_API;
    field1 = String(temperatures[0]);
    field2 = String( humidities[0] );
    if (ON){ field3 = String( "Delorian: ON" ); }
    else{ field3 = String( "Delorian: OFF" ); }
    if (Thermal_Protection){ field4 = String( "Thermal Protection: Active" ); }
    else{ field4 = String( "Thermal Protection: Negative" ); }
    field5 = String(temperatures[4]);
    field6 = String(temperatures[5]);
    field7 = String(temperatures[6]);
    field8 = String(temperatures[7]);

    String TSjson;
    createTSjson(TSjson);
    Particle.publish("Delorian_Status",TSjson,60,PUBLIC);
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
