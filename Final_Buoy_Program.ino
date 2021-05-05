#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <IridiumSBD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C

SoftwareSerial mySerial(3, 4);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true

SoftwareSerial RBSerial(7, 8);
#define IridiumSerial RBSerial
#define DIAGNOSTICS true // Change this to see diagnostics
IridiumSBD modem(IridiumSerial);


bool READY;
String GPSdata;
int sensorPin = A0;     // select the input pin for the 109 probe
int sensorValue;        // variable to store the value coming from the sensor
float vx = 3.3;         //excitation voltage
float A = 1.129241E-3;
float B = 2.341077E-4;
float C = 8.775468E-8;
float Rs; 
float tempC;
float tempF;
float vs;

String pressure;


void setup() {
  Serial.begin(9600);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  uint32_t timer = millis();
  
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
  // Thermistor Code
  sensorValue = analogRead(sensorPin);
  Serial.print("sensorValue is ");
  Serial.print(sensorValue);
  Serial.println();
  vs = ((float(sensorValue*3.355))/734);
  Rs = float(24890.0)*((vx/vs) - 1.0);
  Serial.println();
  
  tempC = (1/(A + (B*log(Rs)) + (C *(pow(log(Rs),3))))) - 273.15;
  tempF = ((tempC*(9.0/5.0)) + 32.0);
  
  Serial.print("Vs is ");
  Serial.print(vs);
  Serial.println();
  
  Serial.print("Rs is ");
  Serial.print(Rs);
  Serial.println();
  
  Serial.print("tempF is ");
  Serial.print(tempF);
  Serial.println();
  String TEMPF = String(tempF);
  delay(300);

  // GPS Code
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
      int DAY = GPS.day;
      String DAYstr = String(DAY);
      int MON = GPS.month;
      String MONstr = String(MON);
      int YEAR = GPS.year;
      String YEARstr = String(YEAR);
      String DATE = MONstr + "/" + DAYstr + "/" + YEARstr + "/";
      Serial.println(DATE);

      int HR = GPS.hour;
      String HRstr = String(HR);
      int MIN = GPS.minute;
      String MINstr = String(MIN);
      String TIME = HRstr + MINstr;
      Serial.print("Time:");
      Serial.println(TIME);
      
      float LAT = GPS.latitude;
      String LATstr = String(LAT);
      float LONG = GPS.longitude;
      String LONGstr = String(LONG);
      String LOC = LATstr + LONGstr;
      Serial.print("Location:");
      Serial.println(LOC);

      String GPSdata = TIME + DATE + LOC;
      Serial.print("GPSData:");
      Serial.println(GPSdata);
      bool READY = 1;
    }
  }
  // Barometer Code
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    pressure = bmp.readPressure();
    String PRESSURE = String(pressure);
  delay(300);

  // Rockblock Send
  int err;
  while (!Serial);

  // Start the serial port connected to the satellite modem
  IridiumSerial.begin(19200);

  // Begin satellite modem operation
  Serial.println("Starting modem...");
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print("Begin failed: error ");
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println("No modem detected: check wiring.");
    return;
    }
  // Form the message
  String message = PRESSURE + GPSdata + TEMPF;
  int len = message.length() + 1;
  char message_array[len];
  message.toCharArray(message_array, len);
  
  if (READY == 1){
      // Send the message
  Serial.print("Trying to send the message.  This might take several minutes.\r\n");
  err = modem.sendSBDText(message_array);
  if (err != ISBD_SUCCESS)
  {
    Serial.print("sendSBDText failed: error ");
    Serial.println(err);
    if (err == ISBD_SENDRECEIVE_TIMEOUT)
      Serial.println("Try again with a better view of the sky.");
  }

  else
  {
    Serial.println("Hey, it worked!");
  }
  }


  
 // Add Sleep function here...for now just a delay
 delay(600000); // 60 min delay
}


#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
#endif
