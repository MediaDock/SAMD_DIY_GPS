/*
                #####                                                           
              ##########                                                        
             ###     ###                                                        
             ###     ###                                                        
             ###     ###                                                        
             ###     ###                                                        
             ###     ###                                                
             ###     ##########                                                 
             ###     ##################                                        
             ###     ###     ##############                                   
             ###     ###     ####     #######                               
             ###     ###     ####     ###   ###                                
  ######     ###     ###     ####     ###   ######                            
###########  ###     ###     ####     ###       ###                            
####     #######                      ###       ###                            
#######      ###                                 ###                            
  #####                                          ###                            
  #######                                        ###                            
     ####                                        ###                            
      ######                                     ###                            
         ###                                  #####                            
         #######                             ####                               
            ####                             ###                                                         
             #######                     #######
                ####                     ####                                   
                #############################                                   
                #############################                                   

                Based on: Seeeduino xiao
                Built for a Workshop at MediaDock HSLU 
                sites.hslu.ch/werkstatt/diy-gps-tracker
*/




/* Constants / Thresholds  ***************************************************************/

const unsigned long lastGPSupdate = 2000;                       // <--- Change this Value to have a refreshrate wich changes every .... millis()
float minDistanceVelocity = 0.0;                                // <--- Speed Threshhold if this KMH is higher then minVelocity,
float minGPXVelocity = 0.0;                                     // <--- Speed Threshhold if KMH is higher -> GPX trackpoint will be safed
static const double HOME_LAT = 47.071754, HOME_LON = 8.277637;  // <--- Change this Value to set your Home Point this are the Coordinates of MediaDock
unsigned long lastBMEupdate = 2000;                             // <--- Change this Value to have a refreshrate wich changes every .... millis()



/* Variables *****************************************************************************/

int SECOND = 0;
int MINUTE = 0;
int HOUR = 0;
int DAY = 0;
int MONTH = 0;
int YEAR = 2000;
char DATE[32];
float KMH = 0.0;
float COURSE = 0.0;
int intLAT;
int intLNG;
int intRefLAT;
int intRefLNG;
int ALTI = 0;
int SAT = 0;
int DIST = 0;



/* Distance Measuring ********************************************************************/

double LAT,
  LNG,
  lastLAT,
  lastLNG,
  RefLAT,
  RefLNG,
  newRefLAT,
  newRefLNG,
  distanceLastPoint,
  distanceToLondon,
  metersTotal;



/* BME **********************************************************************************/

int TEMP = 0;
float hPa = 0;
int hPaALTI = 0;
int HUMID = 0;
unsigned long lastBME = 0UL;





/* Light Sensor ******************************************************************************/

// Connect the MEMS AUD output to the Arduino A0 pin

int UVOUT = A0;    //Output from the sensor
int REF_3V3 = A1;  //3.3V power on the Arduino board
int uvLevel;
int refLevel;
int uvIntensityMapped;
float outputVoltage;
float uvIntensity;
int minUV = 1023;
int maxUV = 0;





/* GPS Globals ***************************************************************************/

#include <TinyGPS++.h>       // https://github.com/mikalhart/TinyGPSPlus
#include <SoftwareSerial.h>  // https://docs.arduino.cc/learn/built-in-libraries/software-serial

unsigned long last = 0UL;
static const int RXPin = 6, TXPin = 7;
static const uint32_t GPSBaud = 9600;  // This is the default baudrate of the Module used in our Project

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);



/* SD Card Globals **********************************************************************/

#include <SD.h>  //
File myFile;
File gpxFile;
File csvFile;
char filename[16];  // make it long enough to hold your longest file name, plus a null terminator
char CSVfilename[16];
const int chipSelect = 3;
#define GPX_EPILOGUE "\t\t</trkseg>\n\t</trk>\n</gpx>\n"

unsigned long gpxSize = 0;



/* Pushbutton for Displaymenu ********************************************************************/

#include <OneButton.h>

int pic = 1;
int maxPics_L1 = 5;

int button_brd = A2;

OneButton button(button_brd, false);

long lastmillis = 0;
long maxtime = 30000;



/* OLED Display Globals ******************************************************************/

#include <SPI.h>   // also used by the BME280
#include <Wire.h>  // also used by the BME280
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


/* OLED Graphic Interface ***************************************************************/

//#include <U8g2lib.h>

static const unsigned char PROGMEM image_map_arrow_bits[] = { 0xc0, 0x38, 0x00, 0xa0, 0x56, 0x00, 0x98, 0x91, 0x80, 0x87, 0x10, 0x40, 0x82, 0x10, 0x40, 0x82, 0x10, 0x40, 0xba, 0x10, 0x40, 0x9a, 0x10, 0x40, 0xaa, 0x10, 0x40, 0xc2, 0x10, 0x40, 0x82, 0x10, 0x40, 0xc2, 0x28, 0x40, 0x22, 0x46, 0x40, 0x1a, 0x81, 0x40, 0x07, 0x00, 0xc0, 0x00, 0x00, 0x00 };
static const unsigned char PROGMEM image_weather_sun_bits[] = { 0x01, 0x00, 0x21, 0x08, 0x10, 0x10, 0x03, 0x80, 0x8c, 0x62, 0x48, 0x24, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x48, 0x24, 0x8c, 0x62, 0x03, 0x80, 0x10, 0x10, 0x21, 0x08, 0x01, 0x00, 0x00, 0x00 };



/* ALTIMETER BME280 Globals **************************************************************/

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;  // I2C




/* Splashscreen on Startup **************************************************************/
/*
   Made with Marlin Bitmap Converter
   https://marlinfw.org/tools/u8glib/converter.html
   exchange this to change the splashscreen on Startup
*/

#pragma once

#define JQP9F_BMPWIDTH 128

const unsigned char SAT_bmp[] PROGMEM = {
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B11011110, B11110011, B10011110, B11110111, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B11011110, B11110011, B10011110, B11110111, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B11011110, B11111111, B11111110, B11110111, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B11011110, B11110011, B10011110, B11110111, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B11011110, B11110011, B10011110, B11110111, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000001, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000001, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000111, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00001111, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00011000, B00110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000001, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00011000, B00110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000001, B10001100, B01100011, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B11000111, B11000110, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B01100000, B00001100, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00111000, B00111000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00001111, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B01100110, B11001100, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000001, B11011110, B11110111, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000111, B10111110, B11111011, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00001111, B01111110, B11111101, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00011110, B01111110, B11111100, B11110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B01111100, B11111110, B11111110, B01111100, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B11111101, B11111110, B11111111, B01111110, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000001, B11111001, B11111110, B11111111, B00111111, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000001, B11111011, B11111110, B11111111, B10111111, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B11111011, B11111110, B11111111, B10111111, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B11110011, B11111110, B11111111, B10011111, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000111, B11110111, B11111110, B11111111, B11011111, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000111, B11110111, B11111110, B11111111, B11011111, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000111, B11110111, B11111110, B11111111, B11011111, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00001111, B11110111, B11111110, B11111111, B11011111, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00001111, B11110111, B11111110, B11111111, B11011111, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00001111, B11110111, B11111110, B11111111, B11011111, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00001111, B11110111, B11111110, B11111111, B11011111, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00001111, B11110111, B11111110, B11111111, B11011111, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000111, B11110111, B11111110, B11111111, B11011111, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000111, B11110111, B11111110, B11111111, B11011111, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000111, B11110111, B11111110, B11111111, B11011111, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B11110011, B11111110, B11111111, B10011111, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B11111011, B11111110, B11111111, B10111111, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000001, B11111011, B11111110, B11111111, B10111111, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000001, B11111001, B11111110, B11111111, B00111111, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B11111101, B11111110, B11111111, B01111110, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B01111100, B11111110, B11111110, B01111100, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00011110, B01111110, B11111100, B11110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00001111, B01111110, B11111101, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000111, B10111110, B11111011, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000001, B11011110, B11110111, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B01100110, B11001100, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000
};




/*****************************************************************************************/
/* Start Setup ***************************************************************************/
/*****************************************************************************************/


void setup() {



  // Start the Arduino hardware serial port at 115200 baud
  Serial.begin(115200);

  // Start the software serial port at the GPS's default baud
  ss.begin(GPSBaud);

  /* Initialize the SD Card *****************************************/

  Serial.print("Initializing SD card...");
  // SD Card max Size: 16GB ---> FAT16 or FAT32 Formatted
  // THE GPS ONLY WORKS IF AN SD CARD IS DETECTED IN THE SDCARD READER

  if (!SD.begin(chipSelect)) {
    Serial.println("SD Card initialization failed!");
    delay(2000);
    while (1)
      ;
  }

  Serial.println("SD card initialized.");


  /**GPX File Initialization*******************************************/

  int n = 0;
  snprintf(filename, sizeof(filename), "Track%03d.gpx", n);  // includes a three-digit sequence number in the file name
  while (SD.exists(filename)) {
    n++;
    delay(50);
    snprintf(filename, sizeof(filename), "Track%03d.gpx", n);
  }


  File gpxFile = SD.open(filename, FILE_WRITE);

  if (gpxFile) {
    gpxFile.print(F(
      "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\" ?>\n"
      "<gpx version=\"1.1\" creator=\"make your way\"\n"
      "\txmlns=\"http://www.topografix.com/GPX/1/1\"\n"
      "\txmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
      "\txsi:schemaLocation=\"http://www.topografix.com/GPX/1/1\n"
      "\thttp://www.topografix.com/GPX/1/1/gpx.xsd\">\n"
      "<trk>\n"
      "\t<name>"));
    gpxFile.print(filename);
    gpxFile.print(F(
      "</name>\n"
      "\t<trkseg>\n"));
    gpxFile.print(F(GPX_EPILOGUE));
    gpxFile.close();  // close the file
  }

  Serial.print("new GPX File initialized: ");
  Serial.println(F(filename));

  /**CSV File Initialization*******************************************/

  snprintf(CSVfilename, sizeof(CSVfilename), "CSV%03d.csv", n);  // includes a three-digit sequence number in the file name


  File csvFile = SD.open(CSVfilename, FILE_WRITE);

  if (csvFile) {
    csvFile.print("Date,Time,Longitude,Latitude,Altitude,Temperature,Humidity,UVintensity,UVLevel,minUV,maxUV");
    csvFile.println();
    csvFile.close();  // close the file
  }

  Serial.print("new CSV File initialized: ");
  Serial.println(F(CSVfilename));


  /*BME Initialization **********************************************/

  Serial.println(F("BME280 test"));
  unsigned status;
  status = bme.begin(0x76);

  if (!status) {
    Serial.println(F("BME280 allocation failed"));
  }


  /*Display Initialization*******************************************/

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  // I2CvAddress 0x3C for 128x64 Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Display SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }

  // First two numbers in each drawBitmap() call are the top-left pixel
  // coordinate, next three are the logo name, width, height and rotation.
  display.clearDisplay();
  display.drawBitmap(0, 0, SAT_bmp, 128, 64, 1);
  display.display();
  delay(10000);


  /* Code Version **************************************************************************/

  display.clearDisplay();               // Clear the buffer
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 20);             // Start at top-left corner
  display.println(F("GPS_5.6_UVSENSOR"));
  display.display();

  delay(2000);
  display.clearDisplay();
  display.display();

  button.attachClick(click);
  //u8g2.begin();
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
}





/*****************************************************************************************/
/* Start Loooping ************************************************************************/
/*****************************************************************************************/



void loop() {
  // Dispatch incoming characters
  while (ss.available() > 0)
    gps.encode(ss.read());

  button.tick();  // activate the Button
  GPSdata();      // get the GPS data <---- This is the main Workhorse!
  BMEdata();      // get the BME data
  UVdata();

  /* Menu Coordinates *************************************************************/
  if (pic == 1) {
    displayInfo();
  }

  /* Menu Coordinates *************************************************************/
  if (pic == 2) {
    displaySensorData();
  }

  if (pic == 3) {
    MicGraphic();
  }

  /* Menu Distanz    *************************************************************/
  if (pic == 4) {
    displayDist();
  }

  /* Menu Time       *************************************************************/
  if (pic == 5 && gps.time.isUpdated()) {
    displayTime();
  }
}





/*****************************************************************************************/
/* grab GPS data *************************************************************************/
/*****************************************************************************************/


void GPSdata() {

  if (gps.location.isUpdated()) {

    if (millis() - last > lastGPSupdate) {

      RefLAT = gps.location.lat();
      RefLNG = gps.location.lng();

      /* Print Variables *************************************************/
      Serial.print(F("Breite= "));
      Serial.print(RefLAT, 9);
      Serial.print(F(" Länge= "));
      Serial.println(RefLNG, 9);


      if (gps.location.isValid()) {

        /* Make Precalculations *************************************************/
        intLAT = (gps.location.lat() * 1000000);
        intLNG = (gps.location.lng() * 1000000);
        double LAT = (double)intLAT / (double)1000000;
        double LNG = (double)intLNG / (double)1000000;

        double distanceToLondon =
          TinyGPSPlus::distanceBetween(
            LAT,
            LNG,
            HOME_LAT,
            HOME_LON);
        double courseToLondon =
          TinyGPSPlus::courseTo(
            LAT,
            LNG,
            HOME_LAT,
            HOME_LON);
        DIST = distanceToLondon;
        Serial.print(F("m- ZH= "));
        Serial.print(distanceToLondon, 0);
        Serial.print(F(" Course-to="));
        Serial.print(courseToLondon, 6);
        Serial.print(F(" degrees ["));
        Serial.print(TinyGPSPlus::cardinal(courseToLondon));
        Serial.println(F("]"));

        /* allDATA_log to SD Card ***************************************************/

        allDATA_log();
        CSV_log();

        minUV = 1023;
        maxUV = 0;

        if (KMH >= minGPXVelocity) {

          /* GPX_log to SD Card *******************************************************/
          GPX_log();
        }


        if (KMH >= minDistanceVelocity) { /* If KMH smaller or equalt to minVelocity (speed) make the Distance Calculations and make GPX log in the end... */

          if (lastLNG > 0) {


            /* Make Precalculations *************************************************/
            intRefLAT = (RefLAT * 1000000);
            intRefLNG = (RefLNG * 1000000);
            double newRefLAT = (double)intRefLAT / (double)1000000;
            double newRefLNG = (double)intRefLNG / (double)1000000;


            double distanceLastPoint =
              TinyGPSPlus::distanceBetween(
                lastLAT,
                lastLNG,
                newRefLAT,
                newRefLNG);
            Serial.print(F("distanceLastPoint:"));
            Serial.println(distanceLastPoint);
            metersTotal = distanceLastPoint + metersTotal;
            Serial.print(F("Distance walked= "));
            Serial.println(metersTotal, 0);

            lastLAT = LAT;
            lastLNG = LNG;


          } else {
            lastLAT = LAT;
            lastLNG = LNG;
          }
        } else {
          Serial.print(F("To slow to calculate Distance"));
          Serial.println();
        }
      }
      last = millis();
      Serial.println();
    }

  }


  else if (gps.date.isUpdated()) {

    if (YEAR != gps.date.year() && YEAR == 2000) {

      /* Save to Variables ***********************************************/
      snprintf(DATE, sizeof(DATE), "%04d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
      YEAR = gps.date.year();
      MONTH = gps.date.month();
      DAY = gps.date.day();
    }

    /* Print Variables *************************************************/
    Serial.print("Date: ");
    Serial.println(DATE);
  }


  else if (gps.time.isUpdated()) {

    /* Save to Variables ***********************************************/
    HOUR = gps.time.hour();
    MINUTE = gps.time.minute();
    SECOND = gps.time.second();

    /* Print Variables *************************************************/
    Serial.print("Time: ");
    if (HOUR < 10) Serial.print(F("0"));
    Serial.print(HOUR);
    Serial.print(":");
    if (MINUTE < 10) Serial.print(F("0"));
    Serial.print(MINUTE);
    Serial.print(":");
    if (SECOND < 10) Serial.print(F("0"));
    Serial.println(SECOND);
  }


  else if (gps.speed.isUpdated()) {
    /* Save to Variables ***********************************************/
    KMH = gps.speed.kmph();
    /* Print Variables *************************************************/
    Serial.print(F("km/h="));
    Serial.println(KMH);

  }


  else if (gps.altitude.isUpdated()) {
    /* Save to Variables ***********************************************/
    ALTI = gps.altitude.meters();
    /* Print Variables *************************************************/
    Serial.print(F("Höhe= "));
    Serial.println(ALTI);
  }


  else if (gps.satellites.isUpdated()) {
    /* Save to Variables ***********************************************/
    SAT = gps.satellites.value();
    /* Print Variables *************************************************/
    Serial.print(F("Satellites= "));
    Serial.println(SAT);
  }
}





/*****************************************************************************************/
/* grab BME data *************************************************************************/
/*****************************************************************************************/

void BMEdata() {

  if (millis() - lastBME > lastBMEupdate) {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    TEMP = bme.readTemperature();
    Serial.println(" °C");

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    hPa = (bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    hPaALTI = bme.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    HUMID = bme.readHumidity();
    Serial.println(" %");

    Serial.println();
    lastBME = millis();
  }
}



/*****************************************************************************************/
/* grab MIC data *************************************************************************/
/*****************************************************************************************/

void UVdata() {

  uvLevel = averageAnalogRead(UVOUT);
  refLevel = averageAnalogRead(REF_3V3);
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  outputVoltage = 3.3 / refLevel * uvLevel;
  uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0);  //Convert the voltage to a UV intensity level
  uvIntensityMapped = map(uvIntensity, 0, 15, 0, 100);

  // Check if the current micOutput is louder or quieter than the previous values
  if (uvIntensity > maxUV) {
    maxUV = uvIntensity;  // Update maxMicOutput if current micOutput is louder
  }
  if (uvIntensity < minUV) {
    minUV = uvIntensity;  // Update minMicOutput if current micOutput is quieter
  }

  Serial.print("RAW UV Data: ");
  Serial.print(uvLevel);
  Serial.print(" UV Intensity (mW/cm^2): ");
  Serial.print(uvIntensity);
  Serial.print(" UV Intensity (0-100): ");
  Serial.print(uvIntensityMapped);
  Serial.println();



  delay(200);
}

int averageAnalogRead(int pinToRead) {
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for (int x = 0; x < numberOfReadings; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}




/*****************************************************************************************/
/* Display Updates ***********************************************************************/
/*****************************************************************************************/

void displayInfo() {

  /* no location no OLED DISPLAY ******************************/

  if (!gps.location.isValid()) {
    display.clearDisplay();
    display.setTextSize(1);               // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);  // Draw white text
    display.setCursor(0, 20);
    display.println(F("Suche..."));
    display.setCursor(0, 30);
    display.println(F("...GPS Satelliten!"));
    display.display();

  } else {

    /* display Info on OLED  ****************************/

    display.clearDisplay();
    display.setTextSize(2);               // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);  // Draw white text
    display.setCursor(0, 0);              // Start at top-left corner
    display.println(F("GPS:"));
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println(gps.location.lat(), 6);
    display.setCursor(70, 20);
    display.print(F("Breite"));
    display.setCursor(0, 30);
    display.println(gps.location.lng(), 6);
    display.setCursor(70, 30);
    display.print(F("Laenge "));
    display.setCursor(0, 40);
    display.print(hPaALTI);
    display.setCursor(70, 40);
    display.print(F("M.ue.M."));
    display.setCursor(0, 50);
    display.print(uvIntensity);
    display.setCursor(70, 50);
    display.print(F("UV"));
    display.display();
  }
}



/* display Distance ***********************************************/

void displayDist() {

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Meter - MediaDock"));
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.print(DIST);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 30);
  display.print(F("km/h"));
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 40);
  display.print(KMH);
  display.display();
}

/* display Distance ***********************************************/

void MicGraphic() {

  display.clearDisplay();
  display.drawRect(23, 35, 100, 16, 1);
  display.fillRect(23, 36, uvIntensityMapped, 15, 1);
  display.setTextWrap(false);
  display.setCursor(23, 54);
  display.setTextSize(1);
  display.print(F("UltraViolet"));
  display.drawBitmap(4, 36, image_weather_sun_bits, 15, 16, 1);
  display.setCursor(23, 4);
  display.setTextSize(1);
  display.print(F("Distanz in Meter"));
  display.setTextSize(2);
  display.setCursor(23, 15);
  display.print(metersTotal);
  display.drawBitmap(1, 14, image_map_arrow_bits, 18, 16, 1);
  display.display();
}

/* display time & date *******************************************/

void displayTime() {

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Zeit: ");
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  if (gps.time.hour() < 10) display.print(F("0"));  // ... to have always 2 digits
  display.print(gps.time.hour());
  display.print(":");
  if (gps.time.minute() < 10) display.print(F("0"));  // ... to have always 2 digits
  display.print(gps.time.minute());
  display.print(":");
  if (gps.time.second() < 10) display.print(F("0"));  // ... to have always 2 digits
  display.print(gps.time.second());
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 30);
  display.print("Datum:");
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 40);
  display.print(DATE);
  display.display();
}

/* display MEMS Mic Values *******************************************/

void displaySensorData() {
  display.clearDisplay();
  display.drawBitmap(1, 14, image_map_arrow_bits, 18, 16, 1);
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(23, 4);
  display.print(F("Distanz in Meter"));
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(23, 15);
  display.println(metersTotal);
  display.drawBitmap(4, 36, image_weather_sun_bits, 15, 16, 1);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(23, 54);
  display.print(F("UltraViolet"));
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(23, 35);
  display.println(uvIntensityMapped);
  display.display();
}





/*****************************************************************************************/
/* SD Card Logs***************************************************************************/
/*****************************************************************************************/


void allDATA_log() { /* SD Card LOG on Update -> activated in line 470 ************************************************/

  myFile = SD.open("allDATA.txt", FILE_WRITE);
  if (myFile) {
    /* allDATA: Date                     **************************************/
    myFile.print("Datum: ");
    myFile.print(DATE);

    /* allDATA: Time                     **************************************/
    if (gps.time.hour() < 10) myFile.print(F("0"));
    myFile.print(", Zeit: ");
    myFile.print(gps.time.hour());
    myFile.print(":");
    if (gps.time.minute() < 10) myFile.print(F("0"));
    myFile.print(gps.time.minute());
    myFile.print(":");
    if (gps.time.second() < 10) myFile.print(F("0"));
    myFile.print(gps.time.second());

    /* allDATA: Coordinates              **************************************/
    myFile.print(", Breite: ");
    myFile.print(RefLAT, 9);
    myFile.print(", Laenge: ");
    myFile.print(RefLNG, 9);

    /* allDATA: Satellites               **************************************/
    myFile.print(", Satelliten: ");
    myFile.print(SAT);

    /* allDATA: ALTI                     **************************************/
    myFile.print(", M.ü.M.: ");
    myFile.print(ALTI);

    /* allDATA: KMH                      **************************************/
    myFile.print(", km/h: ");
    myFile.print(KMH);

    /* allDATA: Distanz                  **************************************/
    myFile.print(", Meter bis ZH: ");
    myFile.print(DIST);
    myFile.print(", Distanz: ");
    myFile.print(metersTotal);

    /* allDATA: Temperatur               **************************************/
    myFile.print(", Temperatur C: ");
    myFile.print(TEMP);

    /* allDATA: Luftfeuchtigkeit         **************************************/
    myFile.print(", Luftfeuchtigkeit %: ");
    myFile.print(HUMID);

    /* allDATA: Luftdruck                **************************************/
    myFile.print(", Luftdruck hPa: ");
    myFile.print(hPa);

    /* allDATA: Luftdruck ALTI           **************************************/
    myFile.print(", M.ü.M. hPa: ");
    myFile.print(hPaALTI);

    /* allDATA: UV                      **************************************/
    myFile.print(", UV: ");
    myFile.print(uvLevel);
    myFile.print(", UV Intensity (mW/cm^2): ");
    myFile.print(uvIntensity);
    myFile.print(", minUV: ");
    myFile.print(minUV);
    myFile.print(", maxUV");
    myFile.println(maxUV);

    myFile.close();  // close the file
    Serial.println("SD Card allDATA_Log done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening GPS.txt");
  }
}





void GPX_log() { /* SD Card LOG on Update -> activated in line 503 ************************************************/

  File gpxFile = SD.open(filename, O_RDWR);  // O_RDWR: open the gpxFile with this command and you are able to read / write the file

  if (gpxFile) {
    int gpxSize = gpxFile.size();
    gpxFile.position();
    unsigned long gpxPosInFile = gpxSize - 25;
    gpxFile.seek(gpxPosInFile);

    /* GPX Trackpoint LOG Coordinates              **************************************/
    gpxFile.print(F("<trkpt lat=\""));  // GPX Tags
    gpxFile.print(RefLAT, 9);
    gpxFile.print("\" ");
    gpxFile.print(F("lon=\""));  // GPX Tags
    gpxFile.print(RefLNG, 9);
    gpxFile.println("\">");

    /* GPX Trackpoint LOG Altitude                 **************************************/
    gpxFile.print(F("\t\t\t<ele>"));  // GPX Tags
    gpxFile.print(hPaALTI);
    gpxFile.println("</ele>");  // GPX Tags


    /* GPX Trackpoint LOG Date                     **************************************/
    gpxFile.print(F("\t\t\t<time>"));  // GPX Tags
    gpxFile.print(DATE);
    gpxFile.print("T");

    /* GPX Trackpoint LOG Time                     **************************************/
    if (gps.time.hour() < 10) gpxFile.print(F("0"));
    gpxFile.print(gps.time.hour());
    gpxFile.print(":");
    if (gps.time.minute() < 10) gpxFile.print(F("0"));
    gpxFile.print(gps.time.minute());
    gpxFile.print(":");
    if (gps.time.second() < 10) gpxFile.print(F("0"));
    gpxFile.print(gps.time.second());
    gpxFile.print("Z");

    /* GPX Trackpoint LOG End XML Tags and Epilog  **************************************/
    gpxFile.println("</time>");       // GPX Tags
    gpxFile.println("\t\t</trkpt>");  // GPX Tags
    gpxFile.print(F(GPX_EPILOGUE));   // GPX Tags
    gpxFile.close();                  // close the file
    Serial.println("SD Card GPX_Log done.");
    Serial.println();
  }
}

void CSV_log() { /* SD Card LOG on Update -> activated in line 503 ************************************************/

  File csvFile = SD.open(CSVfilename, FILE_WRITE);

  if (csvFile) {
    csvFile.print(DATE);
    csvFile.print(",");
    if (gps.time.hour() < 10) csvFile.print(F("0"));
    csvFile.print(gps.time.hour());
    csvFile.print(":");
    if (gps.time.minute() < 10) csvFile.print(F("0"));
    csvFile.print(gps.time.minute());
    csvFile.print(":");
    if (gps.time.second() < 10) csvFile.print(F("0"));
    csvFile.print(gps.time.second());
    csvFile.print(",");
    csvFile.print(gps.location.lng(), 6);
    csvFile.print(",");
    csvFile.print(gps.location.lat(), 6);
    csvFile.print(",");
    csvFile.print(hPaALTI);
    csvFile.print(",");
    csvFile.print(TEMP);
    csvFile.print(",");
    csvFile.print(HUMID);
    csvFile.print(",");
    csvFile.print(uvIntensity);
    csvFile.print(",");
    csvFile.print(uvLevel);
    csvFile.print(",");
    csvFile.print(minUV);
    csvFile.print(",");
    csvFile.print(maxUV);
    csvFile.println();
    csvFile.close();  // close the file
    Serial.println("SD Card CSV_Log done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening CSV File");
  }
}


/*****************************************************************************************/
/* Button State **************************************************************************/
/*****************************************************************************************/

void click() {
  lastmillis = millis();
  if (pic >= 0 && pic < 10) {
    if (pic >= maxPics_L1) {
      pic = 1;
    } else if (pic < maxPics_L1) {
      pic++;
    }
  }

  if (pic >= 10 && pic < 100) {
    if (pic > 30) {
      pic = 11;
    } else if (pic < 30) {
      pic = pic + 10;
    }
  }
}