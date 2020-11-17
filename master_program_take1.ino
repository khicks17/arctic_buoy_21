#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

// You can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)      //connect to pin 13
#define BMP_MISO (12)      //connect to pin 12
#define BMP_MOSI (11)      //connect to pin 11
#define BMP_CS   (10)      //connect to pin 10
Adafruit_BMP280 bmp(BMP_CS);  //SPI Config

#define GPSECHO  true

int temp1Pin = A0;      // select the input pin for 109 probe1
int sensorValue1;       // variable to store the value1
int temp2Pin = A1;      // select the input pin for 109 probe2
int sensorValue2;       // variable to store the value2

float vx = 3.3;         //excitation voltage
float A = 1.129241E-3;
float B = 2.341077E-4;
float C = 8.775468E-8;

int collections_count = 0;            // collections taken (moving number)
int desired_collections = 20;         // desired collections in an hour
int duration = 30;                    // length of one collection (seconds)
int iter_count = 0;                   // iteration count for averaging
int wait_time = 2000;                 // 2 seconds per sample
int sample_time = (wait_time/1000);   // ms to seconds
int trigger_num = (duration/sample_time);
//int standingby_time = ((60*60*1000)-(desired_collections*duration*1000))/(desired_collections);   equal to 150000

float Rs1; 
float tempC1;
float vs1;
float tempC1_sum;           // sum of all temp1 collections to be averaged
float tempC1_avg;           // avg temp over length of collection

float Rs2; 
float tempC2;
float vs2;
float tempC2_sum;           // sum of all temp2 collections to be averaged
float tempC2_avg;           // avg temp over length of collection

float pressure_sum;         // sum of all temp2 collections to be averaged
float pressure_avg;         // avg pressure over length of collection

void setup() {
  Serial.begin(57600);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  
  Serial.println("Test of Master Program");
  Serial.println();
  
  //bmp280 set up
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */     
}

uint32_t timer = millis();

void loop() {
    while(collections_count < desired_collections){
    collections_count = collections_count + 1;
    iter_count = 0; 
    while(iter_count < trigger_num){
      iter_count = iter_count + 1;
      Serial.print("Collection # ");
      Serial.print(collections_count);
      Serial.print(" Iteration # ");
      Serial.print(iter_count);
      Serial.println();
        char c = GPS.read();
      // if you want to debug, this is a good time to do it!
      if ((c) && (GPSECHO))
        Serial.write(c);
    
      // if a sentence is received, we can check the checksum, parse it...
      if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    
        if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
          return;  // we can fail to parse a sentence in which case we should just wait for another
      }
    
      // approximately every 2 seconds or so, print out the current stats
      if (millis() - timer > 2000) {
        timer = millis(); // reset the timer
    
        Serial.print("\nTime: ");
        if (GPS.hour < 10) { Serial.print('0'); }
        Serial.print(GPS.hour, DEC); Serial.print(':');
        if (GPS.minute < 10) { Serial.print('0'); }
        Serial.print(GPS.minute, DEC); Serial.print(':');
        if (GPS.seconds < 10) { Serial.print('0'); }
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        if (GPS.milliseconds < 10) {
          Serial.print("00");
        } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
          Serial.print("0");
        }
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
        
        if (GPS.fix) {
          Serial.print("Location: ");
          Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
          Serial.print(", ");
          Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    
          Serial.print("Speed (knots): "); Serial.println(GPS.speed);
          Serial.print("Angle: "); Serial.println(GPS.angle);
          Serial.print("Altitude: "); Serial.println(GPS.altitude);
          Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
        }
      }
      sensorValue1 = analogRead(temp1Pin);
      sensorValue2 = analogRead(temp2Pin);
      Serial.print("sensorValue1 is ");
      Serial.print(sensorValue1);
      Serial.print(" and sensorValue2 is ");
      Serial.print(sensorValue2);
      Serial.println();
      
      vs1 = ((float(sensorValue1*5.0))/1024.0);
      Rs1 = float(24900.0)*((vx/vs1) - 1.0);
      
      vs2 = ((float(sensorValue2*5.0))/1024.0);
      Rs2 = float(24900.0)*((vx/vs2) - 1.0);
      
      tempC1 = (1/(A + (B*log(Rs1)) + (C *(pow(log(Rs1),3))))) - 273.15;
      tempC2 = (1/(A + (B*log(Rs2)) + (C *(pow(log(Rs2),3))))) - 273.15;
      
      Serial.print("Vs1 is ");
      Serial.print(vs1);
      Serial.print(" and Vs2 is ");
      Serial.print(vs2);
      Serial.println();
      
      Serial.print("Rs1 is ");
      Serial.print(Rs1);
      Serial.print(" and Rs2 is ");
      Serial.print(Rs2);
      Serial.println();
      
      Serial.print("tempC1 is ");
      Serial.print(tempC1);
      Serial.print(" and tempC2 is ");
      Serial.print(tempC2);
      Serial.println();
    
      tempC1_sum = tempC1_sum + tempC1;
      tempC1_avg = tempC1_sum/iter_count; 
      
      tempC2_sum = tempC2_sum + tempC2;
      tempC2_avg = (tempC2_sum/iter_count); 
      
      Serial.print(F("Pressure = "));
      Serial.print(bmp.readPressure());
      Serial.print(" Pa");
      Serial.println();
      Serial.println();
      
      pressure_sum = pressure_sum + bmp.readPressure();
      pressure_avg = (pressure_sum/iter_count);
      delay(wait_time);           //delay between samples
    
    if (iter_count >= 15){
      Serial.print("Average Values for collection cycle:");
      Serial.println();
      Serial.print("Average Temp1 = ");
      Serial.print(tempC1_avg);
      Serial.print(" degrees Celsius");
      Serial.print(" and Average Temp2 = ");
      Serial.print(tempC2_avg);
      Serial.print(" degrees Celsius");
      Serial.println();
      Serial.print("Average Pressure = ");
      Serial.print(pressure_avg);
      Serial.print(" Pa");
      Serial.println();
      Serial.println();
    }
    delay(wait_time);
    }
    delay(1000);             //delay between an averaged collection
  }
}
