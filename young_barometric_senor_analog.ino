int sensorPin = A0;    // select the input pin for the potentiometer      
int sensorValue = 0;  // variable to store the value coming from the sensor
float vs;
float hpa;
float mmHg;


void setup() {
  Serial.begin(115200);
  Serial.println("Barometer test");
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  vs = ((float(sensorValue*5.0))/1024.0);
  hpa = (vs*120.0) + 500.0;
  mmHg = (hpa*0.7500616);
  Serial.print("sensorValue is ");
  Serial.print(sensorValue);
  Serial.println();
  Serial.print("Vs is ");
  Serial.print(vs);
  Serial.println();
  Serial.print("Pressure hpa is ");
  Serial.print(hpa);
  Serial.println();
  Serial.print("Pressure mmHg is ");
  Serial.print(mmHg);
  Serial.println();
  Serial.println();
 
  delay(1000);
}
