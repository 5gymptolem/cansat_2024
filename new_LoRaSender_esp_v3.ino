#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <mySD.h>
#include <Adafruit_LIS3MDL.h>
#include <TinyGPSPlus.h>

//#include <Adafruit_Sensor.h>
//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

#define SDSS 13
//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 868E6
Adafruit_LIS3MDL lis3mdl;
Adafruit_BMP280 bmp; // I2C
//packet counter;
ext::File myFile;
int counter = 0;
int checkSensors;
unsigned long start=0;
float prosgiosi=0;
int localh;
static const int RXPin = 34, TXPin = 12;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
char bufLon[14];
char bufLat[14];
void setup() {
  checkSensors=0;
  Serial.begin(115200);
  Serial.println("LoRa Sender Test");

  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    checkSensors=checkSensors+1;
  }
  else{
  Serial.println("LoRa Initializing OK!");
  }
    if (!bmp.begin(0x77)) {
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
      checkSensors=checkSensors+2;
    }
    else{
      localh=(int)bmp.readAltitude();
      Serial.println("Bmp280 Initializing ok. Ypsometro="+String(localh));
    }
  pinMode(SDSS, OUTPUT);
  if (!SD.begin(SDSS, 15, 2, 14)) {
    Serial.println("Sd initialization failed!");
    checkSensors=checkSensors+4;
    }else{
        Serial.println("initialization SD done.");
    }
  // Try to initialize!
  if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Failed to find LIS3MDL chip");
    checkSensors=checkSensors+8;
  }else{
        Serial.println("initialization Lis3MDL done.");
    }
  //Serial.println("LIS3MDL Found!");
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_16_GAUSS);
  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
  Serial.print(checkSensors);
  ss.begin(GPSBaud);
}
void loop(){
  delay(1000);
  // This sketch displays information every time a new sentence is correctly encoded.
    while (ss.available() > 0)
      if (gps.encode(ss.read()))
        Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    dtostrf(gps.location.lat(), 12, 6, bufLat);
    dtostrf(gps.location.lng(), 12, 6, bufLon);
  }
  else
  {
    Serial.print(F("INVALID\n"));
  }


    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));
      while(true);
    }
    float t=bmp.readTemperature();
    float p=bmp.readPressure();
    float rAlt=bmp.readAltitude();
    //float rSl=bmp.readSealevelPressure();
    //float rSalt=bmp.readAltitude(101500);
   /* Serial.print("Temperature = ");
    Serial.print(t);
    Serial.println(" *C");
    Serial.print("Pressure = ");
    Serial.print(p);
    Serial.println(" Pa");
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(rAlt);
    Serial.println(" meters");*/
   

  // For one second we parse GPS data and report some key values
  //for (unsigned long start = millis(); millis() - start < 1000;)
  
   lis3mdl.read();      // get X Y and Z data at once
  // Then print out the raw data
  /*Serial.print("X: "); Serial.print(lis3mdl.x); 
  Serial.print("Y: "); Serial.print(lis3mdl.y); 
  Serial.print("Z: "); Serial.println(lis3mdl.z); */

  /* Or....get a new sensor event, normalized to uTesla */
  sensors_event_t event; 
  lis3mdl.getEvent(&event);
  /* Display the results (magnetic field is measured in uTesla) */
  /*
  Serial.print("X: "); Serial.print(event.magnetic.x);
  Serial.print("Y: "); Serial.print(event.magnetic.y); 
  Serial.print("Z: "); Serial.print(event.magnetic.z); 
  Serial.println(" uTesla ");*/
  String line=String(counter)+","+String(millis())+","+String(bufLat)+","+String(bufLon)+","+","+String(t)+","+String(p)+","+String(event.magnetic.x)+","+String(event.magnetic.y)+","+String(event.magnetic.z);
  Serial.println(line);
  LoRa.beginPacket();
  LoRa.print(line);
  LoRa.endPacket();
  myFile = SD.open("test.txt", FILE_WRITE);
  if(myFile){
    myFile.println(line);
    Serial.println("ok to file");
  }
  myFile.close();
  counter++;
  if(rAlt>localh+800){
    prosgiosi=1;
   }
   if(prosgiosi==1){
    
   }
  
  
}
