#include <Arduino.h>
#include <Adafruit_NeoPixel.h> // for the onboard LED indicator
#include <OneWire.h>           // for the Dallas Temperature library to be able to use the temperature sensor
#include <DallasTemperature.h>

// Pin assignment
const int tempSensorPin = 5;      // Pin for the temperature sensor
const int conductivityLeadA = 15; // One lead for conductiviy sensor
const int conductivityLeadB = 17; // The other lead for conductivity sensor
const int conductivitySense = 16; // Pin for the conductivity sensor sense wire

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(tempSensorPin);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

// Variable asignments
long HeartbeatOldMillis = 0;
byte HeartbeatLED = LOW;

// Onboard NeoPixel LED colors
int status = 4; // 0 = Off, 1 = Red, 2 = Green, 3 = Blue, 4 (or anything else) = White

// Setup for conductivity sensor
int Vin = 3.16;           // input voltage to the voltage divider (measure with a multimeter)
float R1 = 979;           // fixed resistor value in ohms (measure with a multimeter)
int samples = 100;        // number of samples to average for the conductivity sensor
float cellConstant = 1.0; // cell constant of the conductivity probe (may need calibration)
float alpha = 0.02;       // temperature coefficient for the solution being measured (0.02 for seawater, 0.019 for pure water, 0.021 for aquariums)
int dtime = 500;          // delay time for the conductivity sensor readings

// Values that will be calculated
int raw = 0;
float Vout = 0;
float R2 = 0;
float buff = 0;
float avg = 0;
float conductivity = 0;
float salinity = 0;

// Hold the most recent temp reading
float currentTemp = 0;
float resistance = 0;

// put function declarations here:
void LED_Status(int x);
float checkConductivity();
float getTemperature();
float calcSalinity(float resistance, float temp);

// put your setup code here, to run once:
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(conductivityLeadA, OUTPUT);
  pinMode(conductivityLeadB, OUTPUT);
  // Start the Serial Monitor for troubleshooting outputs on PC
  Serial.begin(115200);
  // Start the DS18B20 temp sensor comms
  sensors.begin();
}

void loop()
{
  sensors.requestTemperatures();    // send the command for the sensors to gather temps while we are working on conductivity
  resistance = checkConductivity(); // check conductivity reading
  currentTemp = getTemperature();   // grab the temp from the meter
  salinity = calcSalinity(resistance, currentTemp);
  Serial.print("Temp:");
  Serial.print(currentTemp);
  Serial.print(" C, Resistance: ");
  Serial.print(resistance);
  Serial.print(" Ohms, Salinity: ");
  Serial.print(salinity);
  Serial.println(" ppt");

  //! Possibly leave hearbeat in its own function and make the status vs heartbeat mutually exlusive
  LED_Status(status); // Default heartbeat is white flashing LED
}

float getTemperature()
{
  return sensors.getTempCByIndex(0); // get the reading from the sensor
}

float checkConductivity()
{
  float tot = 0;
  for (int i = 0; i < samples; i++)
  {
    digitalWrite(conductivityLeadA, HIGH);
    digitalWrite(conductivityLeadB, LOW);
    delayMicroseconds(dtime);
    digitalWrite(conductivityLeadA, LOW);
    digitalWrite(conductivityLeadB, HIGH);
    delayMicroseconds(dtime);
    raw = analogRead(conductivitySense);
    if (raw)
    {
      buff = raw * Vin;
      Vout = (buff) / 4096.0;  // taking input ADC and converting back to a voltage value
      buff = (Vin / Vout) - 1; // calculating the ratio of the voltage divider
      R2 = R1 * buff;          // finally give the resistance of the water under test
      R2 = 0.85 * R2 - 14;     //? Optional calibration offset
      tot = tot + R2;          // add to the existing samples for later averaging
    }
  }
  avg = tot / samples; // calc and return the average resistance
  return avg;
}

float calcSalinity(float resistance, float temp)
{
  conductivity = (1.0 / R2) * cellConstant * 1000.0;                                      // Calculate conductivity (in mS/cm)
  float conductivity25 = conductivity / (1 + alpha * (temp - 25.0));                      // Adjust conductivity to 25C //! Maybe revisit for different temp ranges to see how this math works
  float calulatedSalinity = 0.4665 * conductivity - 0.0013 * conductivity * conductivity; // Convert conductivity to salinity (ppt)
  return calulatedSalinity;
}

// put function definitions here:
void LED_Status(int x)
{
  switch (x)
  {
  case 1:
    neopixelWrite(RGB_BUILTIN, 60, 0, 0); // red
    break;
  case 2:
    neopixelWrite(RGB_BUILTIN, 0, 60, 0); // green
    break;
  case 3:
    neopixelWrite(RGB_BUILTIN, 0, 0, 60); // blue
    break;
  default:
    long HeartbeatNewMillis = millis();
    if (HeartbeatNewMillis - HeartbeatOldMillis > x)
    {
      HeartbeatOldMillis = HeartbeatNewMillis;
      if (HeartbeatLED == LOW)
      {
        HeartbeatLED = HIGH;
        neopixelWrite(RGB_BUILTIN, 60, 60, 60); // white
      }
      else
      {
        HeartbeatLED = LOW;
        neopixelWrite(RGB_BUILTIN, 0, 0, 0); // off
      }
    }
    break;
  }
}