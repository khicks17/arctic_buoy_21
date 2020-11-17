int sensorPin = A5;     // select the input pin for the 109 probe
int sensorValue;      // variable to store the value coming from the sensor
float vx = 3.3;         //excitation voltage
float A = 1.129241E-3;
float B = 2.341077E-4;
float C = 8.775468E-8;
int z;
float Rs; 
float tempC;
float tempF;
float vs;
float test;


void setup() {
  Serial.begin(9600);
  Serial.println("109 Probe test");
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  Serial.print("sensorValue is ");
  Serial.print(sensorValue);
  Serial.println();
  z = int(A/A);
  vs = ((float(sensorValue*5.0))/1024.0);
  Rs = float(24900.0)*((vx/vs) - 1.0);
  Serial.println();
  test = float(pow(log(Rs),3));
  Serial.print(test);
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
  
  Serial.println();
  delay(2000);
}
