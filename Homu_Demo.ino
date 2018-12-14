
/*************************************************************

  WARNING :
  For this example you'll need Adafruit DHT sensor libraries:
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest
    https://github.com/adafruit/Adafruit_Sensor
    https://github.com/adafruit/DHT-sensor-library
    https://github.com/markszabo/IRremoteESP8266
    https://github.com/blynkkk/blynk-library
    https://github.com/z3t0/Arduino-IRremote
 *************************************************************/

/* Comment this out to disable prints and save space */

#ifndef UNIT_TEST
#include <Arduino.h>
#endif
#include <ir_Mitsubishi.h>
#define BLYNK_PRINT Serial
#include<IRremoteESP8266.h>
//#include<IRsend.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <math.h>
#include <cmath>
#include <TimeLib.h>
#include <WidgetRTC.h>
#include <DHT.h>
#define earthRadiusKm 6371.0

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "20ebe5ee60944f29906fe3b6a4cfecc4"; //change it to your authorization code

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "iPhone"; //change it to your ssid
char pass[] = "0895224544"; // change it to your pwd

#define DHTPIN 5          // pin connected to dht11
#define MOTIONPIN 4       // pin connected to motion sensor
//#define DEPTHPIN A0       // pin connected to IR depth sensor must be analog pin
#define RELAYPIN 14       // pin connected to relay
const uint16_t kIrLed = 12;
IRMitsubishiAC mitsubir(kIrLed);
// Uncomment whatever type you're using!

#define DHTTYPE DHT11     // DHT 11
//define DHTTYPE DHT22   // DHT 22, AM2302, AM2321
//#define DHTTYPE DHT21   // DHT 21, AM2301


//bool addtime = true  ;


//-------------------------------------- **start** Main  Sensor Paramiter --------------------------------------//
bool isMotion_detect = false;
bool isRELAY_On = false;
bool  isAir_condition = false;
bool  isButton_air_start_on = false;
bool  isSetting_addData   =  false ;// GPS countrol Button Label V8 is On/off // Button V8 to Save your location
int Set_temp = 27 ; // Common Temp in Thailand that  open air
float current_Tem = 0 ;
//-------------------------------------- **end**  Main  Sensor Paramiter --------------------------------------//



//-------------------------------------- **start** GPS countrol  Paramiter --------------------------------------//
double Home_lat = 0.0 ;  // Your home lat
double Home_long = 0.0 ; // Your home long
double Current_lat = 0.0 ;
double Current_long  = 0.0 ;
bool isHome = false ;

//-------------------------------------- **end**  GPS countrol  Paramiter --------------------------------------//

String tab1_connect = "User" ;  // setting control INPUT YOUR name V2
// setting control check Label V2

//-------------------------------------- **start** Table  Control --------------------------------------//
WidgetRTC rtc;  // Start Real Time
DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;
BLYNK_CONNECTED() {
  Blynk.syncVirtual(V0);
  Blynk.syncVirtual(V1);
  Blynk.syncVirtual(V8);
  rtc.begin();
}

//Using visual port  V9
int rowIndex = 0 ;   // table row to push
WidgetTable table ; // Created Table to Push log
BLYNK_ATTACH_WIDGET (table, V9); // Created Table to Push log to Visual pin V9
//-------------------------------------- **end** Table  Control  --------------------------------------//




//-------------------------------------- **start** TIME  Function Nodemcu Control --------------------------------------//

String currentTime ; // USE in Table V9 log
String currentDate ;// USE in Table V9 log
String Print_Time_RTC ;
void clockDisplay() // USE in Table V9 log // Updating in blynk.begin()
{
  currentTime = String(hour()) + ":" + minute() + ":" + second();
  currentDate = String(day()) + " / " + month() + " / " + year();
  Print_Time_RTC =  currentTime + " " + currentDate;
}



//-------------------------------------- **end** TIME  Function Nodemcu Control --------------------------------------//

//-------------------------------------- **start** Text out GPs   Paramiter --------------------------------------//
String  In_home ;
String  Out_home ;

//--------------------------------------  **end** Text out GPs   Paramiter --------------------------------------//
//-------------------------------------- **start** GPS Covent to meter  Blynk Control --------------------------------------//


//--------------------------------------  **start** Function   Paramiter --------------------------------------//
int compare(int tem_1 , int tem_2 ) {  // Retart program
  int i ;
  i = tem_1 - tem_2 ;
  return i ;
}
//--------------------------------------  **end** Function   Paramiter --------------------------------------//

//--------------------------------------  **start** GPS  Function   Paramiter --------------------------------------//
// This function converts decimal degrees to radians
double deg2rad(double deg) {
  return (deg * M_PI / 180);
}

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
  return (rad * 180 / M_PI);
}

/**
   Returns the distance between two points on the Earth.
   Direct translation from http://en.wikipedia.org/wiki/Haversine_formula
   @param lat1d Latitude of the first point in degrees
   @param lon1d Longitude of the first point in degrees
   @param lat2d Latitude of the second point in degrees
   @param lon2d Longitude of the second point in degrees
   @return The distance between the two points in kilometers
*/
double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) {
  double lat1r, lon1r, lat2r, lon2r, u, v;
  lat1r = deg2rad(lat1d);
  lon1r = deg2rad(lon1d);
  lat2r = deg2rad(lat2d);
  lon2r = deg2rad(lon2d);
  u = sin((lat2r - lat1r) / 2);
  v = sin((lon2r - lon1r) / 2);
  return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v))*1000;
}
//-------------------------------------- **end**  GPS  Function   Paramiter --------------------------------------//


//-------------------------------------- **start** Motion  Sensor Function Control --------------------------------------//
void MotionSensor() {  //  MOTION Check  in staetment
  long state = digitalRead(MOTIONPIN);
  if (state == HIGH) {
    isMotion_detect = true ;
    //Serial.println("Motion detected!");
  }
  else {
    isMotion_detect = false  ;
    //Serial.println("Motion absent!");
  }
}


//-------------------------------------- **end** Motion  Sensor  Function Control --------------------------------------//




//-------------------------------------- **start** BLYNK sync Control --------------------------------------//


//-------------------------------------- **end** BLYNK sync Control --------------------------------------//

//-------------------------------------- **start** table  Blynk Control --------------------------------------//


// V3 just push to clear
BLYNK_WRITE(V3) {
  int i = param.asInt();
  if (i == 1) {
    Blynk.notify("Clear Data table ");
    table.clear();
    rowIndex = 0 ;
  } else {
  }
}



//-------------------------------------- **end** table  Blynk Control --------------------------------------//




//-------------------------------------- **start** Main app Control --------------------------------------//
// using visual port V0 V1 V5 V6
// Button  V0 to active  Turn On AirCondition
// using visual port V0 V1 V5 V6
// Button  V0 to active  Turn On AirCondition 
BLYNK_WRITE(V0) {
  int i = param.asInt();

  if (i == 1) {
    Blynk.notify("Turn On AirCondition");
    isButton_air_start_on = true;
    isAir_condition = true ;
    Blynk.setProperty(V0, "color", "#04C0F8");
       Serial.println("Turn On AirCondition");
    mitsubir.on();
    mitsubir.send();
  } else {
    Blynk.notify("Turn off AirCondition");
    Serial.println("Turn off AirCondition");
    isButton_air_start_on = false  ;
    isAir_condition = false;
    Blynk.setProperty(V0, "color", "#D3435C");
    mitsubir.off();
    mitsubir.send();
  }
}
 

// Button  V1 to active  Turn On Light
BLYNK_WRITE(V1) {
  int i = param.asInt();

  if (i == 1) {
    isRELAY_On = true;
    digitalWrite( 14, HIGH );
    Blynk.setProperty(V1, "color", "#04C0F8");
    Serial.println("Turn on  rely");
    Blynk.notify("Turn On Light");
  } else {
    isRELAY_On = false  ;
    digitalWrite( 14, LOW );
    Blynk.setProperty(V1, "color", "#D3435C");
    Serial.println("Turn off  rely");
    Blynk.notify("Turn Off Light");
  }
}

//-------------------------------------- **end** Main app Control --------------------------------------//

//--------------------------------------  **start** Setting Control --------------------------------------//
// Have Visual port :: -> V8 ,V2 V7

// Button V8 to Save your location


// input text V8 to name
BLYNK_WRITE(V2)

{
  String textIn = param.asStr();
  tab1_connect =   textIn ;
  Blynk.virtualWrite(V2, tab1_connect);
  Serial.println("Add text");
}


// pull Use name to Board memory





//-------------------------------------- **end** Setting Control --------------------------------------//





//---------------------------------------------------- GO GO CAN ---------------------------

//---------------------------------------------------- GO GO CAN ---------------------------



void sendSensor()
{
  int h = dht.readHumidity();
  int t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit
  current_Tem = t ;
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, h);
  Blynk.virtualWrite(V6, t);

}



//-------------------------------------- **start** Call back Function Control --------------------------------------//

String TextIN_AirLight_report_email_false_Buttou_false  = "You Coming " + tab1_connect + " " + " Today is Hot" + "/n" + "Would you want to turn on your air-condition ?"  ;
String TextIN_AirLight_report_notify_false_Buttou_false = "Wellcome to home!!! " + tab1_connect + " " + " Today is Hot" + "/n" + "Would you want to turn on your air-condition ?"  ;
String TextIN_AirLight_report_email_false_Buttou_true   = "You Coming Wellcome to home!!! " + tab1_connect ;
String TextIN_AirLight_report_notify_false_Buttou_true  = "You Coming Wellcome to home!!! " + tab1_connect ;

String TextIN_AirLight_report_email_true_Buttou_false = "You Coming " + tab1_connect + " Today is cold /n Would you want to turn on your air-condition";
String TextIN_AirLight_report_notify_true_Buttou_false  = "Wellcome to home !!! " + tab1_connect + " Today is cold /n Would you want to turn on your air-condition";
String TextIN_AirLight_report_notify_true_Buttou_true  = " Hey,Wellcome to home!!! "  + tab1_connect ;
String TextIN_AirLight_report_email_true_Buttou_true   = "You Coming Wellcome to home!!! " + tab1_connect;

void AirLight_report() {
  digitalWrite( 14, HIGH );
  if ( compare((int) current_Tem, Set_temp ) >= 0 ) {

    if (isButton_air_start_on = false ) {
      Blynk.email("From yout Themostat!!!", TextIN_AirLight_report_email_false_Buttou_false);
      Blynk.notify(TextIN_AirLight_report_notify_false_Buttou_false); // at notify to Users
    } else {
      Blynk.email("From yout Themostat!!!", TextIN_AirLight_report_email_false_Buttou_true);
      Blynk.notify(TextIN_AirLight_report_notify_true_Buttou_true  );
    }
  } else {
    if (isButton_air_start_on = false ) {
      Blynk.email("From yout Themostat!!!", TextIN_AirLight_report_email_true_Buttou_false);
      Blynk.notify(TextIN_AirLight_report_notify_true_Buttou_false); // at notify to Users
    } else {
      Blynk.email("From yout Themostat!!!", TextIN_AirLight_report_email_true_Buttou_true);
      Blynk.notify(TextIN_AirLight_report_notify_true_Buttou_true);
    }

  }
}
String Textout_AirLight_report_email_false_Buttou_false  = "You leave" + tab1_connect + " Today is Hot /n Have you turn off your air-condition  ? "  ;
String Textout_AirLight_report_notify_false_Buttou_false = "Hey,Wellcome to home!!!" + tab1_connect + "  Today is Hot /n Have you turn off your air-condition  ?"  ;
String Textout_AirLight_report_email_false_Buttou_true   = "Have a nice day!!! " + tab1_connect ;
String Textout_AirLight_report_notify_false_Buttou_true  = "Have a nice day!!! " + tab1_connect;

String Textout_AirLight_report_email_true_Buttou_false = "You leave" + tab1_connect + " Today is Hot" + "/n" + "Have you turn off your air-condition  ? ";
String Textout_AirLight_report_notify_true_Buttou_false  = "You leave" + tab1_connect + " Today is Hot" + "/n" + "Have you turn off your air-condition  ?  ";
String Textout_AirLight_report_notify_true_Buttou_true  = "Have a nice day!!! " + tab1_connect;
String Textout_AirLight_report_email_true_Buttou_true   = "Have a nice day!!! " + tab1_connect;

void AirLight_report_out() {
  digitalWrite( 14, LOW );

  if ( compare((int) current_Tem, Set_temp ) >= 0 ) {

    if (isButton_air_start_on = false ) {
      Blynk.email("From yout Themostat!!!", Textout_AirLight_report_email_false_Buttou_false);
      Blynk.notify(Textout_AirLight_report_notify_false_Buttou_false); // at notify to Users
    } else {
      Blynk.email("From yout Themostat!!!", Textout_AirLight_report_email_false_Buttou_true);
      Blynk.notify(Textout_AirLight_report_notify_false_Buttou_true  );
    }
  } else {
    if (isButton_air_start_on = false ) {
      Blynk.email("From yout Themostat!!!", Textout_AirLight_report_email_true_Buttou_false);
      Blynk.notify(Textout_AirLight_report_notify_true_Buttou_false); // at notify to Users
    } else {
      Blynk.email("From yout Themostat!!!", Textout_AirLight_report_email_true_Buttou_true);
      Blynk.notify(Textout_AirLight_report_notify_true_Buttou_true);
    }
  }
}

//-------------------------------------- **end** Call back Function Control --------------------------------------//





BLYNK_WRITE(V7) // receive gps //program; //1=home soon, 2 =@home, 3=out 4=comfort 5=night
{
  GpsParam gps (param);
  Current_lat = gps.getLat()  ;
  Current_long = gps.getLon() ;
 // Serial.println("GPS: Lat/lon: ");
  // Working  with Button V8 to Save your location  Use
  
  In_home   = tab1_connect + ":: IN - Lat: " + (String)Current_lat + " - Lon: " +  (String)Current_long  ;
  Out_home = tab1_connect + ":: OUT - Lat: " + (String)Current_lat + " - Lon: " +  (String)Current_long  ;

  if ( (distanceEarth( Home_lat,  Home_long,  Current_lat,  Current_long) <5 ) && (isMotion_detect == true ) && (isHome == false)   )
  {
    // Push get in data to tabel V9
    table.addRow(rowIndex,Print_Time_RTC ,"Get in") ;rowIndex++;
    table.addRow(rowIndex,"Top_Get in","Test") ;rowIndex++;
    AirLight_report();
    Serial.println(In_home ) ;
    Serial.println("GPS Print");
    Serial.println(Home_lat, 7);
    Serial.println(Home_long, 7);
    Serial.println(Current_lat, 7);
    Serial.println(Current_long, 7);
    isHome = true ;

  }
  else if ((distanceEarth( Home_lat,  Home_long,  Current_lat,  Current_long) > 5 ) && ( isMotion_detect == false) && (isHome == true) )
  {
    // Push get out data to tabel V9
    table.addRow(rowIndex,Print_Time_RTC ,"Get out") ;rowIndex++;
    table.addRow(rowIndex,"Top_Get out","Test") ;rowIndex++;
    AirLight_report_out();
    Serial.println( Out_home ) ;
    Serial.println("GPS Print");
    Serial.println(Home_lat, 7);
    Serial.println(Home_long, 7);
    Serial.println(Current_lat, 7);
    Serial.println(Current_long, 7);
    isHome = false ;
  }
}



//--------------------------------------  **end** GPS Covent to meter  Blynk Control --------------------------------------//
BLYNK_WRITE(V8) {
  int i = param.asInt();

  if (i == 1) {

    Blynk.notify("Home Data have been Added");
    isSetting_addData = true;
    Serial.println("Home Data have been Added");
    Blynk.setProperty(V8, "color", "#04C0F8");
    Home_lat = Current_lat  ;  
    Home_long = Current_long ;
    Serial.println( " GPS add  " + (String) Home_lat  ) ;
    Serial.println( " GPS add  " + (String ) Home_long   );
    Blynk.virtualWrite(V9,rowIndex,Print_Time_RTC  ," GPS add  ");rowIndex++;
    table.addRow(rowIndex,"Add new gps","GPS") ;rowIndex++;
    table.addRow(rowIndex,(String) Current_lat,"NEW lat") ;rowIndex++;
    table.addRow(rowIndex,(String) Current_long,"NEW long") ;rowIndex++;
  } else {
    Blynk.notify("Home Data have been Clear");
    isSetting_addData = false  ;
    Serial.println("Home Data have been Clear");
    Blynk.setProperty(V8, "color", "#D3435C");
    Current_lat = 0 ;
      Current_long = 0 ;
      Serial.println( " GPS out  " + (String) Home_lat  ) ;
      Serial.println( " GPS out  " + (String ) Home_long   );

  }
}

void setup()
{
  // Debug console
  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  Blynk.notify("Wellcome to Homu"); // Board Actived
  dht.begin(); // set up Themometer && Humiditimeter

  pinMode(MOTIONPIN, INPUT);
  // pinMode(DEPTHPIN, INPUT);
  pinMode(RELAYPIN, OUTPUT);
  mitsubir.begin();
  delay(200);
  // Set up what we want to send. See ir_Mitsubishi.cpp for all the options.
  Serial.println("Default state of the remote.");
  Serial.println("Setting desired state for A/C.");
  mitsubir.on();
  mitsubir.setFan(1);
  mitsubir.setMode(MITSUBISHI_AC_COOL);
  mitsubir.setTemp(26);
  mitsubir.setVane(MITSUBISHI_AC_VANE_AUTO);
    setSyncInterval(1000); // time sync
  // Setup a function to be called every second
  timer.setInterval(1000L, clockDisplay);
  timer.setInterval(5000L, sendSensor);
}







void loop()
{
  Blynk.run();
  timer.run();
  MotionSensor();
}
