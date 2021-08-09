/*
  Optical Heart Rate Detection (PBA Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  This is a demo to show the reading of heart rate or beats per minute (BPM) using
  a Penpheral Beat Amplitude (PBA) algorithm.

  It is best to attach the sensor to your finger using a rubber band or other tightening
  device. Humans are generally bad at applying constant pressure to a thing. When you
  press your finger against the sensor it varies enough to cause the blood in your
  finger to flow differently which causes the sensor readings to go wonky.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected

  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

#include <dht.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

#define dht_apin A1 // Analog Pin sensor is connected to
dht DHT;

//AGE_GROUP
char age_group = 'B';

// Buzzer
const int buzzer = 9; //buzzer to arduino pin 9

// Moisture
int sensor_pin = A0;
int output_value ;

MAX30105 particleSensor;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

void setup()
{

  pinMode(buzzer, OUTPUT); // Set buzzer - pin 9 as an output
  Serial.begin(9600);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.
}

void loop()
{


  if (Serial.available() > 0) {
    age_group = ((byte)Serial.read());
    Serial.println(age_group);
  }

  //Moisture
  output_value = analogRead(sensor_pin);
  output_value = map(output_value, 550, 0, 0, 100);

  float temperature = particleSensor.readTemperature();
  float temperatureF = particleSensor.readTemperatureF();

  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  //Conditions
  if (output_value > 15) {
    tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(1000);        // ...for 1 sec
  } else {
    noTone(buzzer);     // Stop sound...
  }

  switch (age_group) {
    case '1':
      if (beatsPerMinute >= 70 && beatsPerMinute <= 190) {
        tone(buzzer, 1000); // Send 1KHz sound signal...
        delay(1000);        // ...for 1 sec
      }
      break;
    case '2':
      if (beatsPerMinute >= 80 && beatsPerMinute <= 160) {
        tone(buzzer, 1000); // Send 1KHz sound signal...
        delay(1000);        // ...for 1 sec
      }
      break;
    case '3':
      if (beatsPerMinute >= 80 && beatsPerMinute <= 130) {
        tone(buzzer, 1000); // Send 1KHz sound signal...
        delay(1000);        // ...for 1 sec
      }
      break;
    case '4':
      if (beatsPerMinute >= 80 && beatsPerMinute <= 120) {
        tone(buzzer, 1000); // Send 1KHz sound signal...
        delay(1000);        // ...for 1 sec
      }
      break;
    case '5':
      if (beatsPerMinute >= 75 && beatsPerMinute <= 115) {
        tone(buzzer, 1000); // Send 1KHz sound signal...
        delay(1000);        // ...for 1 sec
      }
      break;
    case '6':
      if (beatsPerMinute >= 70 && beatsPerMinute <= 110) {
        tone(buzzer, 1000); // Send 1KHz sound signal...
        delay(1000);        // ...for 1 sec
      }
      break;
    case '7':
      Serial.print(beatsPerMinute);
      if (beatsPerMinute >= 60 && beatsPerMinute <= 100) {
        tone(buzzer, 1000); // Send 1KHz sound signal...
        delay(1000);        // ...for 1 sec
      }
      break;
  }


  DHT.read11(dht_apin);
  
  double humidityiz = DHT.humidity;
  double bpmiz = beatsPerMinute;
  Serial.print(humidityiz); //humidity
  Serial.print(",");
  Serial.print(bpmiz); //bpm
  Serial.print(",");
  Serial.print(output_value); //blood level
  Serial.print(",");
  Serial.print(temperature); //tempC
  Serial.print(",");
  Serial.print(temperatureF); //tempF
  Serial.print(",");
  Serial.print(beatAvg); //Average BPM
  Serial.print(",");
  Serial.print(irValue); //IR


  /*Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);
    Serial.print(", temperatureC=");
    Serial.print(temperature, 4);
    Serial.print(", temperatureF=");
    Serial.print(temperatureF, 4);
    Serial.print(", Blood level: ");
    Serial.print(output_value);
    Serial.println("%");*/

  if (irValue < 50000) {
    Serial.print(",No finger?");
  }

  Serial.println();
}
