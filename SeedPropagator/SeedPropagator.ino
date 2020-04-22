/*
 * Seed Propagator for Arduino Mega v1.0
 * Control optimized for growing chilli peppers
 * 
 * Sowing Date: 11th June 2016
 * First 3 seedlings apearing ( 3 out of 10) 22th June 2016
 */

/*
 * ------ SOWING -----

Sow at 0.5 cm deep.
A 20 cm pot will accommodate a single plant.
Well drained soil with gravel at the bottom of the pot.
Companion Plants:  basil, chives, carrots, onions, lettuce, spinach, okra, leeks, radishes, beets, asparagus and garlic.

----- GERMINATION ----

Ideal range for germination: 27 - 32 degrees C (Optimum germination temperature = 28 celsius)
Complete range for germination: 21 - 38 degrees C 
Soil Temperature VS Germination Time:
                                      15 degrees C - 25 days
                                      20 degrees C - 13 days
                                      25 degrees C - 8 days
                                      30 degrees C - 8 days

----- VEGETATIVE GROWTH AND FRUIT ------
                                      
Complete range for Vegetative Growth: 21 - 28 degrees C ( Optimum Vegetative Growth Temperature = 24 celsius)
Complete range for Fruit Set at night: 15 - 20 degrees C ( Optimum Vegetative Growth Temperature at Night = 18 celsius)
Complete range for Fruit Set Daytime: 20 - 28 degrees C ( Optimum Vegetative Growth Temperature Daytime = 24 celsius)


---- RELATIVE HUMIDITY ------

Relative Humidity Optimum Range: 65% < Rh > 85%

High humidity creates a favourable environment for the development of several foliar diseases.
Conversely, low relative humidity may cause infertility, due to pollen drying out before germination of
the pollen on the stigma, which leads to small, deformed or flat fruit.


------- IRRIGATION --------


WEEKS AFTER PLANTTING     ROOT DEPTH(mm)      SOIL MOISTURE %       CROP FACTOR       DAYS BETWEEN IRRIGATION   AMOUNT NEEDED (mm)

0-2 (Stablishment)             400                   20                  0.3                         2                   10-15    
3-6 (Vegetative Growth)        500                   30                  0.4                        3-5                  15-25
7-15 (Fruit Set)               600                   40                  0.6                        5-7                  30-40
16-+  (Rippening)              700                   50                  0.8                        7-10                 20-30    


------ LIGHTING --------

Required 14 to 16 hours of light per day.


-------  FERTILIZATION --------

5.6 < pH (H2O) > 6.8
30 - P [mg/Kg](Bray1) - 60
100 < K [mg/Kg]> 250
300 < Ca [mg/Kg]> 2000
120 < Mg [mg/Kg]> 300
10 < Na [mg/Kg]> 50


*/
#include<SPI.h>
#include <SD.h>
#include <Wire.h>
#include <DS3231.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"
#include <Adafruit_BMP085.h>
#include <BH1750.h>
#include<SPI.h>
#include <SD.h>
#include "Timer.h"
//#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>



#define ECHO_TO_SERIAL   1 // echo data to serial port
#define SD_MODULE    1 // Select if system has SD datalogging or not

Timer tSD; //timer for SD logging

//-------------RTC DS3231---------//
DS3231  rtc(SDA, SCL); // Init the DS3231
Time  t; // Init a Time-data structure

//------------- Temperature using DS18B20 Thermometer---------//
#define ONE_WIRE_BUS 49
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//--------------Temperature& Humidity using DHT devices --------------//
#define DHTPIN 45     
// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
DHT dht(DHTPIN, DHTTYPE);

// -----------Barometric Pressure Sensor BMP085 (now replace with BMP180 -------//
Adafruit_BMP085 bmp;

//---------------------BH1750FVI lux sensor-----------------------//
BH1750 lightMeter;


//--4 x Potentiometers for setting levels of temperature, light, ventilation and soil humidity--//
// Thi allows adapt the control logic to different plant growth phase requirements

#define MIN_TEMP 15 // min temp for pot wheel
#define MAX_TEMP 35 // max temp for pot wheel

#define MIN_MOISTURE 0 // min moisture level for pot wheel
#define MAX_MOISTURE 4 // max moisture for pot wheel

#define MIN_LIGHT 0 // min light level for potentiometer wheel
#define MAX_LIGHT 10 // max light for pot wheel

#define MIN_VENTILATION 0 // min ventilation level for potentiometer wheel
#define MAX_VENTILATION 4 // max ventilation for pot wheel

#define potTempPin A12  
#define potMoisturePin A11 
#define potLightPin A10 
#define potVentilationPin A9 

float setTemp = 0; 
float setIrrigation = 0; 
float setLight = 0; 
float setVentilation = 0; 

//-------------Soil Moisture Sensor--------//
#define soilMoisturePin A1
// value = 1023 is completly dry
// value = 240 well irrigated for sowing

//-------------Rain Detector Sensor--------//
#define rainDetectorPin A0
// value = 820 is completly dry
// value = 30 well irrigated for sowing

//---------Float Switch---------//
int floatSwitch = 7;
int floatSwitch2 = 2;

//-------------Relays-----------//
#define RELAY_ON 0
#define RELAY_OFF 1
#define Relay_Heating  23  
#define Relay_Lighting  27
#define Relay_Lighting2  6
#define Relay_Ventilation  25
#define Relay_Irrigation  29

//--------------LCD--------------//
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------- LED's -----------//
#define TempSensorFailLED 31
#define lightControlLED 22
#define ventilationControlLED 24
#define irrigationControlLED 26
#define redLEDpin 28 // Flush and sync SD card
#define greenLEDpin 30 //Writing data to SD card

// -------------- SD Datalogger --------------//
// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  30000 // mills between entries (reduce to take more/faster data)
#define SYNC_INTERVAL 30000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()
//#define BANDGAPREF 14            // special indicator that we want to measure the bandgap
//#define aref_voltage 3.3         // we tie 3.3V to ARef and measure it with a multimeter!
//#define bandgap_voltage 1.1      // this is not super guaranteed but its not -too- off
// for the data logging shield, we use digital pin 10 for the SD cs line. Note pin 4 if using
//etrhernet shield 
const int chipSelect = 53;
// the logging file
File logfile;


//////////////////////
//VARIABLES TO MONITOR
/////////////////////
int yeart = 0; //RTC
int month = 0;
int day = 0;
int hourt = 0;
int minute = 0;
int second = 0; 
float T0 = 0; //DS18B20 temp sensor index 0 (Right box temp) 
float T1 = 0; //DS18B20 temp sensor index 1 (Soil Temp)
float T2 = 0; //DS18B20 temp sensor index 2 (Left Box Temp)
float T3 = 0; //DS18B20 temp sensor index 3 (External Temp)
float T4 = 0; //DS18B20 temp sensor index 4 (Back Box Temp)
float t_dht = 0; //DHT temp sensor
float h_dht = 0; //DHT humidity sensor
float hic = 0; //Heat index from DHT readings
float dew = 0; // Dew point from DHT readings
float Tbmp = 0; //BMP085 temp readings
float Pbmp = 0; //BMP085 pressure readings
float altitude = 0; // BMP085 altitude calculation
bool floatSwitchState = 0; // Reading from float switch 1
bool floatSwitch2State = 0; // Reading from float switch 2
int soilMoisture = 0;// Reading from soil moisture sensor 
int rain = 0; // Reading from raindetector
uint16_t lux = 0;// Light levels (outside the box, "natural light")
bool Relay_HeatingState = 0; 
bool Relay_LightingState = 0;
bool Relay_VentilationState = 0;
bool Relay_IrrigationState = 0;

//-------- END OF VARIABLES TO MONITOR ----- /////

// CONTROL VARIABLES /////

float hysteresis = 0.5;// hysterisis cycle for temperature control
float Tsum = 0;// sum of valid temperature sensors
float count = 0;//count for calculating average temperature and number of valid readings
float Ta = 0; // Average temperature of the box
int lightIntConvert = 0; //convert light setting from float to int to use switch case
float optimumHumidity = 65; //Set Optimum Relative Humidity for growing
float hysteresisVentilation = 50; // hysterisis cycle for ventilation control
bool flagVent = 0; // Flag raised when Relative humidity exceeds maximum level 
float Tmax = 34; // Max temperature allowed plats at risk this has to be changed depending on the growing phase!
int ventilationIntConvert = 0; ////convert ventilation setting from float to int to use switch case
int irrigationIntConvert = 0; ////convert irrigation setting from float to int to use switch case
float hysteresisIrrigationVEG = 10; // hysterisis cycle for Irrigation control LOW water requirements
float hysteresisIrrigationFRUIT = 20; // hysterisis cycle for Irrigation control HIGH water requirements
float hysteresisIrrigationRIPE = 30; // hysterisis cycle for Irrigation control HIGH water requirements
float optimumIrrigation = 330; // Optimum irrigation level based on soil moisture reading
bool flagIrrigation = 0; // Flag for irrigation needed, then irrigate on timer and not based on sensor readings
int countLCD = 0; //count for displaying values sequentially on the LCD



// SETUP Initialize Devices

void initializeRTC()
{
  // Initialize the rtc object
  rtc.begin();
  // The following lines can be uncommented to set the date and time
  //rtc.setDOW(SUNDAY);     // Set Day-of-Week to SUNDAY
  //rtc.setTime(12, 0, 0);     // Set the time to 12:00:00 (24hr format)
  //rtc.setDate(1, 1, 2016);   // Set the date to DD/MM/YYYY
  //t = rtc.getTime();   // Get data from the DS3231
}

void initializeDS18B20()
{
  sensors.begin(); //start up DS18B20 library
}

void initializeDHT()
{
  dht.begin();
}

void initializeBMP085()
{
  bmp.begin();
}

void initializeBH1750FVI()
{
  lightMeter.begin();
}

void initializeFloatSwitch()
{
  pinMode(floatSwitch, INPUT);
  pinMode(floatSwitch2, INPUT);
}

void initializeRelays()
{
  digitalWrite(Relay_Heating, RELAY_OFF);
  digitalWrite(Relay_Lighting, RELAY_OFF);
  digitalWrite(Relay_Lighting2, RELAY_OFF);
  digitalWrite(Relay_Ventilation, RELAY_OFF);
  digitalWrite(Relay_Irrigation, RELAY_OFF); 
  pinMode(Relay_Heating, OUTPUT);   
  pinMode(Relay_Lighting, OUTPUT);
  pinMode(Relay_Lighting2, OUTPUT);  
  pinMode(Relay_Ventilation, OUTPUT);  
  pinMode(Relay_Irrigation, OUTPUT);
}

void initializeLCD()
{ 
  lcd.begin();   
  lcd.clear();
  lcd.print("Seed Propagator"); 
  delay(1000);
  lcd.clear();
 
  for(int i = 0; i< 3; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight(); 
 
}

void initializeTimers()
{
  tSD.every(30000, SDlogging);
}

void initializeLEDs()
{
  digitalWrite(TempSensorFailLED, LOW);
  pinMode(TempSensorFailLED, OUTPUT);
  digitalWrite(lightControlLED, LOW);
  pinMode(lightControlLED, OUTPUT);
  digitalWrite(ventilationControlLED, LOW);
  pinMode(ventilationControlLED, OUTPUT); 
  digitalWrite(irrigationControlLED, LOW);
  pinMode(irrigationControlLED, OUTPUT); 
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  
}

void error(char *str)
{
  #if ECHO_TO_SERIAL
  Serial.print("error: ");
  Serial.println(str);
  #endif
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while(1);
}

void initializeSD()
{
  pinMode(53, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  #if ECHO_TO_SERIAL
  Serial.println("card initialized.");
  #endif
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  #if ECHO_TO_SERIAL
  Serial.print("Logging to: ");
  Serial.println(filename);
  #endif
  logfile.println("year,month,day,hour,minute,second,T0RB,T1S,T2LB,T3ext,T4BB,h,t,hic,dew,Tbmp,Pbmp,altitude,floatSwitchState,soilMoisture,rain,lux,Relay_HeatingState,Relay_LightingState,Relay_VentilationState,Relay_IrrigationState,Ta,count,setTemp,setIrrigation,setLight,setVentilation");    
  #if ECHO_TO_SERIAL
  Serial.println("year,month,day,hour,minute,second,T0RB,T1S,T2LB,T3ext,T4BB,h,t,hic,dew,Tbmp,Pbmp,altitude,floatSwitchState,soilMoisture,rain,lux,Relay_HeatingState,Relay_LightingState,Relay_VentilationState,Relay_IrrigationState,Ta,count,setTemp,setIrrigation,setLight,setVentilation");
  #endif //ECHO_TO_SERIAL
  // If you want to set the aref to something other than 5v
  //analogReference(EXTERNAL);
}

void setup() {
   
  #if ECHO_TO_SERIAL 
  Serial.begin(9600);
  Serial.println("Initializing Devices...");
  #endif  
  initializeRelays();
  initializeRTC();
  initializeDS18B20();
  initializeDHT();
  initializeBMP085();
  initializeBH1750FVI();
  initializeFloatSwitch();
  initializeLCD();
  initializeTimers();
  initializeLEDs();
  initializeSD();
  
  delay(2000);
}

void loop() {

  #if ECHO_TO_SERIAL
  Serial.println("");
  Serial.println("-------Sensor Readings---------");
  Serial.println("");
  #endif
  
  getRTC();
  getDS18B20();
  getDHT();
  getAverageTemp();
  getBMP085();
  getBH1750FVI();
  getRelayState();
  getSwitchState();
  getSoilMoisture();
  getRainDetector();
  getPotSetValues();
  displayValues();

  #if ECHO_TO_SERIAL
  Serial.println("");
  Serial.println("-------end of sensor readings---------");
  Serial.println("");
  Serial.println("");
  #endif
  
  
  controlHeater();
  controlLight();
  controlFan();
  controlIrrigation();

  tSD.update();
  
  delay(5000);
}


//////////////////
//LOOP FUNCTIONS
/////////////////
void SDlogging()
{
  digitalWrite(greenLEDpin, HIGH);

  logfile.print(t.year);
  logfile.print(", ");    
  logfile.print(t.mon);
  logfile.print(", ");    
  logfile.print(t.date);
  logfile.print(", ");    
  logfile.print(t.hour);
  logfile.print(", ");    
  logfile.print(t.min);
  logfile.print(", "); 
  logfile.print(t.sec);
  logfile.print(", ");  
  logfile.print(T0);
  logfile.print(", ");    
  logfile.print(T1);
  logfile.print(", ");    
  logfile.print(T2);
  logfile.print(", ");    
  logfile.print(T3);
  logfile.print(", ");    
  logfile.print(T4);
  logfile.print(", ");  
  logfile.print(h_dht);
  logfile.print(", ");   
  logfile.print(t_dht);
  logfile.print(", ");    
  logfile.print(hic);
  logfile.print(", ");    
  logfile.print(dew);
  logfile.print(", ");    
  logfile.print(Tbmp);
  logfile.print(", ");    
  logfile.print(Pbmp);
  logfile.print(", ");
  logfile.print(altitude);
  logfile.print(", ");    
  logfile.print(floatSwitchState);
  logfile.print(", ");    
  logfile.print(soilMoisture);
  logfile.print(", ");    
  logfile.print(rain);
  logfile.print(", "); 
  logfile.print(lux);
  logfile.print(", ");  
  logfile.print(Relay_HeatingState);
  logfile.print(", ");  
  logfile.print(Relay_LightingState);
  logfile.print(", ");  
  logfile.print(Relay_VentilationState);
  logfile.print(", ");   
  logfile.print(Relay_IrrigationState);
  logfile.print(", "); 
  logfile.print(Ta);
  logfile.print(", ");
  logfile.print(count);
  logfile.print(", ");
  logfile.print(setTemp);
  logfile.print(", ");
  logfile.print(setIrrigation);
  logfile.print(", ");
  logfile.print(setLight);
  logfile.print(", ");  
  logfile.print(setVentilation);
  logfile.println();
  
  digitalWrite(greenLEDpin, LOW);
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  
  // blink LED to show we are syncing data to the card & updating FAT
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
  
}


void controlIrrigation()
{
  if (floatSwitchState == 1 || floatSwitch2State == 1 )
  {
    digitalWrite(Relay_Irrigation, RELAY_OFF);
    digitalWrite(irrigationControlLED, HIGH);
    #if ECHO_TO_SERIAL 
    Serial.println("Water tank is empty or float switch sensor failed!");
    #endif  
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Water Tank empty or sensor failed!");      
  }

  else
  {
    irrigationIntConvert = (int)setIrrigation;

    
    switch (irrigationIntConvert)
    {
      case 0:
          digitalWrite (Relay_Irrigation, RELAY_OFF);
          digitalWrite(irrigationControlLED, HIGH);
          flagIrrigation = 0;
          #if ECHO_TO_SERIAL 
          Serial.println("Seed propagator has NO IRRIGATION set!");
          #endif
        
          break;

      case 1:
          digitalWrite(irrigationControlLED, LOW);
          if (soilMoisture > (optimumIrrigation + hysteresisIrrigationVEG))
          {
            digitalWrite (Relay_Irrigation, RELAY_ON);
          } 
          else if (soilMoisture < (optimumIrrigation - hysteresisIrrigationVEG))
            {
              digitalWrite (Relay_Irrigation, RELAY_OFF);
            }

          #if ECHO_TO_SERIAL 
          Serial.print("Optimum soil Moisture is set to:");
          Serial.println(optimumIrrigation);
          Serial.print("Current soil Moisture is:");
          Serial.println(soilMoisture);
          Serial.print("Current soil Moisture hysteresis is set to:");
          Serial.println(hysteresisIrrigationVEG);  
          #endif
          break; 

      case 2:
          digitalWrite(irrigationControlLED, LOW);
          if (soilMoisture > (optimumIrrigation + hysteresisIrrigationFRUIT))
          {
            digitalWrite (Relay_Irrigation, RELAY_ON);
          } 
          else if (soilMoisture < (optimumIrrigation - hysteresisIrrigationFRUIT))
            {
              digitalWrite (Relay_Irrigation, RELAY_OFF);
            }

          #if ECHO_TO_SERIAL 
          Serial.print("Optimum soil Moisture is set to:");
          Serial.println(optimumIrrigation);
          Serial.print("Current soil Moisture is:");
          Serial.println(soilMoisture);
          Serial.print("Current soil Moisture hysteresis is set to:");
          Serial.println(hysteresisIrrigationFRUIT);  
          #endif
      
          break; 

      case 3:
          digitalWrite(irrigationControlLED, LOW);
          if (soilMoisture > (optimumIrrigation + hysteresisIrrigationRIPE))
          {
            digitalWrite (Relay_Irrigation, RELAY_ON);
          } 
          else if (soilMoisture < (optimumIrrigation - hysteresisIrrigationRIPE))
            {
              digitalWrite (Relay_Irrigation, RELAY_OFF);
            }
          #if ECHO_TO_SERIAL 
          Serial.print("Optimum soil Moisture is set to:");
          Serial.println(optimumIrrigation);
          Serial.print("Current soil Moisture is:");
          Serial.println(soilMoisture);
          Serial.print("Current soil Moisture hysteresis is set to:");
          Serial.println(hysteresisIrrigationRIPE);  
          #endif
      
          break; 

      case 4:
        digitalWrite (Relay_Irrigation, RELAY_ON);
        digitalWrite(irrigationControlLED, HIGH);
        #if ECHO_TO_SERIAL 
        Serial.println("Seed propagator has continous irrigation!");
        #endif
        break;
     }    
  }

}


void controlFan()
{
  
  if (h_dht > optimumHumidity + hysteresisVentilation || Ta > Tmax)
  {
    digitalWrite(Relay_Ventilation, RELAY_ON);
    flagVent = 1;
    digitalWrite(ventilationControlLED, HIGH);
    #if ECHO_TO_SERIAL 
    Serial.println("Ventilation ON due to Relative Humidity or Temperature eceeding maximum value ");
    #endif
    
  }
  if (h_dht < optimumHumidity - hysteresisVentilation && flagVent == 1)
  {    
    digitalWrite(Relay_Ventilation, RELAY_OFF);
    flagVent = 0;
    digitalWrite(ventilationControlLED, LOW);
    #if ECHO_TO_SERIAL 
    Serial.println("Ventilation OFF overheating and/or damp corrected ");
    #endif
     
  }

  ventilationIntConvert = (int)setVentilation; 

  if (flagVent == 0)
  {
    switch (ventilationIntConvert)
      {
        case 0:
          digitalWrite (Relay_Ventilation, RELAY_OFF);
          digitalWrite(ventilationControlLED, HIGH);
          #if ECHO_TO_SERIAL 
          Serial.println("Seed propagator has NO VENTILATION period set!");
          #endif
       
          break;

        case 1:
          digitalWrite(lightControlLED, LOW);
          if ( 0 <= minute && minute <= 4 )
          {
            digitalWrite (Relay_Ventilation, RELAY_ON);
          } 
          else
            {
              digitalWrite (Relay_Ventilation, RELAY_OFF);
            }
          #if ECHO_TO_SERIAL 
          Serial.println("Seed propagator has ventilation period set for 5 min every hour");
          #endif
      
           break;  

        case 2:
          digitalWrite(lightControlLED, LOW);
          if ( 0 <= minute && minute <= 9)
          {
            digitalWrite (Relay_Ventilation, RELAY_ON);
          } 
          else
            {
              digitalWrite (Relay_Ventilation, RELAY_OFF);
            }
          #if ECHO_TO_SERIAL 
          Serial.println("Seed propagator has ventilation period set for 10 min every hour");
          #endif
      
           break;  

        case 3:
          digitalWrite(lightControlLED, LOW);
          if ( 0 <= minute && minute <= 14)
          {
            digitalWrite (Relay_Ventilation, RELAY_ON);
          } 
          else
            {
              digitalWrite (Relay_Ventilation, RELAY_OFF);
            }
          #if ECHO_TO_SERIAL 
          Serial.println("Seed propagator has ventilation period set for 15 min every hour");
          #endif
      
           break;  

        case 4:
          digitalWrite (Relay_Ventilation, RELAY_ON);
          digitalWrite(ventilationControlLED, HIGH);
          #if ECHO_TO_SERIAL 
          Serial.println("Seed propagator has ventilation period set 24h a day!");
          #endif
      
          break;

        default:
          digitalWrite (Relay_Ventilation, RELAY_OFF);
          digitalWrite(ventilationControlLED, HIGH);
          #if ECHO_TO_SERIAL 
          Serial.println("Light Control System Fault. Check Potentiometer and Clock");
          #endif

          break;      
      }    
  }
  
}


void controlLight()
{
  lightIntConvert = (int)setLight;
  
  switch (lightIntConvert)
  {
    case 0:
      digitalWrite (Relay_Lighting, RELAY_OFF);
      digitalWrite (Relay_Lighting2, RELAY_OFF);
      digitalWrite(lightControlLED, HIGH);
      #if ECHO_TO_SERIAL 
      Serial.println("Seed propagator has NO LIGHTING period set!");
      #endif
        
      break;

    case 1:
      digitalWrite(lightControlLED, LOW);
      if ( hourt == 6 || hourt == 7)
      {
        digitalWrite (Relay_Lighting, RELAY_ON);
        digitalWrite (Relay_Lighting2, RELAY_ON);
        } 
      else
      {
        digitalWrite (Relay_Lighting, RELAY_ON);
        digitalWrite (Relay_Lighting2, RELAY_ON);
      }
      #if ECHO_TO_SERIAL 
      Serial.println("Seed propagator has lighting period set for 2h a day");
      #endif
      
      break;  

     case 2:
      digitalWrite(lightControlLED, LOW);
      if (6 <= hourt && hourt<=9)
      {
        digitalWrite (Relay_Lighting, RELAY_ON);
        digitalWrite (Relay_Lighting2, RELAY_ON);
        } 
      else
      {
        digitalWrite (Relay_Lighting, RELAY_OFF);
        digitalWrite (Relay_Lighting2, RELAY_OFF);
      }
      #if ECHO_TO_SERIAL 
      Serial.println("Seed propagator has lighting period set for 4h a day");
      #endif
      
      break;  

      case 3:
      digitalWrite(lightControlLED, LOW);
      if (6 <= hourt && hourt<= 11)
      {
        digitalWrite (Relay_Lighting, RELAY_ON);
        digitalWrite (Relay_Lighting2, RELAY_ON);
        } 
      else
      {
        digitalWrite (Relay_Lighting, RELAY_OFF);
        digitalWrite (Relay_Lighting2, RELAY_OFF);
      }
      #if ECHO_TO_SERIAL 
      Serial.println("Seed propagator has lighting period set for 6h a day");
      #endif
      
      break;  

    case 4:
      digitalWrite(lightControlLED, LOW);
      if ( 6 <= hourt && hourt<= 13)
      {
        digitalWrite (Relay_Lighting, RELAY_ON);
        digitalWrite (Relay_Lighting2, RELAY_ON);
        } 
      else
      {
        digitalWrite (Relay_Lighting, RELAY_OFF);
        digitalWrite (Relay_Lighting2, RELAY_OFF);
      }
      #if ECHO_TO_SERIAL 
      Serial.println("Seed propagator has lighting period set for 8h a day");
      #endif
      
      break;  

    case 5:
      digitalWrite(lightControlLED, LOW);
      if ( 6 <= hourt && hourt<= 15)
      {
        digitalWrite (Relay_Lighting, RELAY_ON);
        digitalWrite (Relay_Lighting2, RELAY_ON);
        } 
      else
      {
        digitalWrite (Relay_Lighting, RELAY_OFF);
        digitalWrite (Relay_Lighting2, RELAY_OFF);
      }
      #if ECHO_TO_SERIAL 
      Serial.println("Seed propagator has lighting period set for 10h a day");
      #endif
      
      break;  

    case 6:
      digitalWrite(lightControlLED, LOW);
      if ( 6 <= hourt && hourt<= 17)
      {
        digitalWrite (Relay_Lighting, RELAY_ON);
        digitalWrite (Relay_Lighting2, RELAY_ON);
        } 
      else
      {
        digitalWrite (Relay_Lighting, RELAY_OFF);
        digitalWrite (Relay_Lighting2, RELAY_OFF);
      }
      #if ECHO_TO_SERIAL 
      Serial.println("Seed propagator has lighting period set for 12h a day");
      #endif
      
      break;  

    case 7:
      digitalWrite(lightControlLED, LOW);
      if ( 6 <= hourt && hourt<= 19)
      {
        digitalWrite (Relay_Lighting, RELAY_ON);
        digitalWrite (Relay_Lighting2, RELAY_ON);
        } 
      else
      {
        digitalWrite (Relay_Lighting, RELAY_OFF);
        digitalWrite (Relay_Lighting2, RELAY_OFF);
      }
      #if ECHO_TO_SERIAL 
      Serial.println("Seed propagator has lighting period set for 14h a day");
      #endif
      
      break; 

    case 8:
      digitalWrite(lightControlLED, LOW);
      if ( 6 <= hourt && hourt<= 21)
      {
        digitalWrite (Relay_Lighting, RELAY_ON);
        digitalWrite (Relay_Lighting2, RELAY_ON);
        } 
      else
      {
        digitalWrite (Relay_Lighting, RELAY_OFF);
        digitalWrite (Relay_Lighting2, RELAY_OFF);
      }
      #if ECHO_TO_SERIAL 
      Serial.println("Seed propagator has lighting period set for 16h a day");
      #endif
      
      break;

    case 9:
      digitalWrite(lightControlLED, LOW);
      if ( 6 <= hourt && hourt<= 22)
      {
        digitalWrite (Relay_Lighting, RELAY_ON);
        digitalWrite (Relay_Lighting2, RELAY_ON);
        } 
      else
      {
        digitalWrite (Relay_Lighting, RELAY_OFF);
        digitalWrite (Relay_Lighting2, RELAY_OFF);
      }
      #if ECHO_TO_SERIAL 
      Serial.println("Seed propagator has lighting period set for 18h a day");
      #endif
      
      break;  

    case 10:
      digitalWrite (Relay_Lighting, RELAY_ON);
      digitalWrite (Relay_Lighting2, RELAY_ON);
      digitalWrite(lightControlLED, HIGH);
      #if ECHO_TO_SERIAL 
      Serial.println("Seed propagator has lighting period set 24h a day!");
      #endif
      break;

     default:
        digitalWrite (Relay_Lighting, RELAY_OFF);
        digitalWrite (Relay_Lighting2, RELAY_OFF);
        digitalWrite(lightControlLED, HIGH);
        #if ECHO_TO_SERIAL 
        Serial.println("Light Control System Fault. Check Potentiometer and Clock");
        #endif

        break;
    
    }
  
  }


void controlHeater()
{
  
  if (Ta < (setTemp - hysteresis))
  {
    digitalWrite(Relay_Heating, RELAY_ON);
  }
  else if (Ta > (setTemp + hysteresis))
  {    
    digitalWrite(Relay_Heating, RELAY_OFF); 
  }
  #if ECHO_TO_SERIAL 
  Serial.print("Temperature is set to: ");
  Serial.println(setTemp);
  Serial.print("Current Temperature: ");
  Serial.println(Ta);
  #endif
}

void getAverageTemp()
{
  count = 0;
  Tsum = 0;

  if ( t_dht > 10 && t_dht < 40 )
  {
    Tsum = Tsum + t_dht;
    count++;
    }
  if ( T4 > 10 && T4 < 40 )
  {
    Tsum = Tsum + T4;
    count++;
    }
  if ( T2 > 10 && T2 < 40 )
  {
    Tsum = Tsum + T2;
    count++;
    }
  if ( T0 > 10 && T0 < 40 )
  {
    Tsum = Tsum + T0;
    count++;
    }

   Ta = Tsum / count;
   #if ECHO_TO_SERIAL 
   Serial.print("Number of valid Temperatures: ");
   Serial.println(count); 
   Serial.print("Average Box Temperature is: ");
   Serial.println(Ta);
   #endif  

   if ( count < 4 )
   {
    digitalWrite(TempSensorFailLED, HIGH);
    #if ECHO_TO_SERIAL 
    Serial.println("At least one temperature sensor failed");
    #endif  
    }
    else
    {
      digitalWrite(TempSensorFailLED, LOW);
      }

    count = 0;
  }




void getRTC()
{
   t = rtc.getTime();   // Get data from the DS3231
   
   yeart = t.year;
   month = t.mon;
   day = t.date;
   hourt = t.hour;
   minute = t.min;
   second = t.sec;
  
  #if ECHO_TO_SERIAL 
  Serial.print("Date: ");
  Serial.print(t.date, DEC);
  Serial.print("/");
  Serial.print(t.mon, DEC);
  Serial.print("/");
  Serial.print(t.year, DEC);
  Serial.println();
  // Send Day-of-Week and time
  Serial.print("Day of Week: ");
  Serial.print(t.dow, DEC);
  Serial.println();
  Serial.print("Time: ");
  Serial.print(t.hour, DEC);
  Serial.print(":");
  Serial.print(t.min, DEC);
  Serial.print(":");
  Serial.print(t.sec, DEC);
  Serial.println();
  #endif
 }

 void getDS18B20()
{
  sensors.requestTemperatures(); // Send the command to get temperatures
  delay(50);
  T0 = sensors.getTempCByIndex(0);
  delay(50);
  T1 = sensors.getTempCByIndex(1);
  delay(50);
  T2 = sensors.getTempCByIndex(2);
  delay(50);
  T3 = sensors.getTempCByIndex(3);
  delay(50);
  T4 = sensors.getTempCByIndex(4);
  delay(50);
  
  #if ECHO_TO_SERIAL 
  Serial.print("Right Box Temperature (index 0) is: ");
  Serial.println(T0);
  Serial.print("Soil Temperature (index 1) is: ");
  Serial.println(T1);
  Serial.print("Left Box Temperature (index 2) is: ");
  Serial.println(T2);
  Serial.print("External Temperature (index 3) is: ");
  Serial.println(T3);
  Serial.print("Back Box Temperature (index 4) is: ");
  Serial.println(T4);
  #endif  
  
}

void getDHT()
{
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  h_dht = dht.readHumidity();
  t_dht = dht.readTemperature();
  hic = dht.computeHeatIndex(t_dht, h_dht, false);
  dew = dewPointFast(t_dht, h_dht);
  // Check if any reads failed and exit early (to try again).
  if (isnan(h_dht) || isnan(t_dht)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  #if ECHO_TO_SERIAL 
  Serial.print("Humidity DHT: ");
  Serial.print(h_dht);
  Serial.print(" %\t");
  Serial.print("Temperature DHT: ");
  Serial.print(t_dht);
  Serial.print(" C ");
  //Serial.print(f);
  //Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" *C \t");
  //Serial.print(hif);
  //Serial.println(" *F");
  Serial.print("Dew PointFast: ");
  Serial.print(dew);
  Serial.println(" *C");
  #endif
  
}


double dewPointFast(double celsius, double humidity)
{
  double a = 17.271;
  double b = 237.7;
  double temp = (a * celsius) / (b + celsius) + log(humidity*0.01);
  double Td = (b * temp) / (a - temp);
  return Td;
}


void getBMP085()
{
    Tbmp = bmp.readTemperature();
    Pbmp = bmp.readPressure();
    altitude = bmp.readAltitude(101500);

    #if ECHO_TO_SERIAL 
    Serial.print("Temperature BMP  = ");
    Serial.print(Tbmp);
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(Pbmp);
    Serial.println(" Pa");
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(altitude);
    Serial.println(" meters");
    Serial.println();
    #endif
  }


void getBH1750FVI()
{ 
  lux = lightMeter.readLightLevel();
  #if ECHO_TO_SERIAL
  Serial.print("Light Level: ");
  Serial.print(lux);
  Serial.println(" lx");
  #endif
}


void getRelayState()
{ 
  Relay_HeatingState = digitalRead(Relay_Heating); 
  Relay_LightingState = digitalRead(Relay_Lighting);
  Relay_VentilationState = digitalRead(Relay_Ventilation);
  Relay_IrrigationState = digitalRead(Relay_Irrigation);

  #if ECHO_TO_SERIAL
  Serial.print("Heatingh Relay State is:");
  Serial.println(Relay_HeatingState);
  Serial.print("Lighting Relay State is:");
  Serial.println(Relay_LightingState);
  Serial.print("Ventilation Relay State is:");
  Serial.println(Relay_VentilationState);
  Serial.print("Irrigation Relay State is:");
  Serial.println(Relay_IrrigationState);
  #endif

}

void getSwitchState()
{
  floatSwitchState = digitalRead(floatSwitch); 
  floatSwitch2State = digitalRead(floatSwitch2); 
  #if ECHO_TO_SERIAL
  Serial.print("The Float switch 1 state is:");
  Serial.println( floatSwitchState);
  Serial.print("The Float switch 2 state is:");
  Serial.println( floatSwitch2State);
  #endif
  
}

void getSoilMoisture()
{
  soilMoisture = analogRead(soilMoisturePin);
  #if ECHO_TO_SERIAL
  Serial.print("Soil Moisture is:");
  Serial.println(soilMoisture);
  #endif
}


void getRainDetector()
{
  rain = analogRead(rainDetectorPin);
  #if ECHO_TO_SERIAL
  Serial.print("Rain Detector is:");
  Serial.println(rain);
  #endif
}


void getPotSetValues()
{
  setTemp = readSetTemp();
  setIrrigation = readSetMoisture();
  setLight = readSetLight();
  setVentilation = readSetVentilation();

  #if ECHO_TO_SERIAL
  Serial.print("Green House Temperature Set to: ");
  Serial.println(setTemp);
  Serial.print("Green House Soil Moisture Set to: ");
  Serial.println(setIrrigation);
  Serial.print("Green House Lights Set to: ");
  Serial.println(setLight);
  Serial.print("Green House Ventilation Set to: ");
  Serial.println(setVentilation);
  #endif
}

float readSetTemp()
{
  float rawTemp = analogRead(potTempPin);
  float ts = map(rawTemp, 0, 1023, MIN_TEMP, MAX_TEMP);
  delay(50);
  return ts;
}

float readSetMoisture()
{
  float rawMoisture = analogRead(potMoisturePin);
  float ms = map(rawMoisture, 0, 1023, MIN_MOISTURE, MAX_MOISTURE);
  delay(50);
  return ms;
}

float readSetLight()
{
  float rawLight = analogRead(potLightPin);
  float ls = map(rawLight, 0, 1023, MIN_LIGHT, MAX_LIGHT);
  delay(50);
  return ls;
}

float readSetVentilation()
{
  float rawVentilation = analogRead(potVentilationPin);
  float vs = map(rawVentilation, 0, 1023, MIN_VENTILATION, MAX_VENTILATION);
  delay(50);
  return vs;
}


void displayValues()
{
  switch (countLCD)
  {   
    case (0):

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Date: ");
      lcd.print(t.date);
      lcd.print("/");
      lcd.print(t.mon);
      lcd.print("/");
      lcd.print(t.year);
      lcd.setCursor(0,1);
      lcd.print("Time: ");
      lcd.print(t.hour);
      lcd.print(":");
      lcd.print(t.min);
      lcd.print(":");
      lcd.print(t.sec);
      countLCD++;
      break;

    case (1):

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(" Text=");
      lcd.print(T3);
      lcd.print(" *C");
      lcd.setCursor(0,1);
      lcd.print("Tsoil=");
      lcd.print(T1);
      lcd.print(" *C");
      countLCD++;
      break;

    case (2):

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Temp = ");
      lcd.print(Ta);
      lcd.print("*C");
      lcd.setCursor(0,1);
      lcd.print("Humidity=");
      lcd.print(h_dht);
      lcd.print("%");
      countLCD++;
      break;

   case (3):

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Soil Moist 1= ");
      lcd.print(soilMoisture);
      lcd.setCursor(0,1);
      lcd.print("Soil Moist 2= ");
      lcd.print(rain);
      countLCD++;
      break;

    case (4):

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Ts= ");
      lcd.print(setTemp);
      lcd.print("  Ms=");
      lcd.print(irrigationIntConvert);
      lcd.setCursor(0,1);
      lcd.print("Ls=");
      lcd.print(lightIntConvert*2);
      lcd.print("h/day");
      lcd.print("Vs=");
      lcd.print(ventilationIntConvert*5);
      lcd.print("min/h");
      countLCD = 0;
      break;    
    }  
}
