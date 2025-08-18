#include <Arduino.h>
#include <Adafruit_NeoPixel.h> // for the onboard LED indicator
#include <OneWire.h>           // for the Dallas Temperature library to be able to use the temperature sensor
#include <DallasTemperature.h>

// Define sensors to call in code, uncomment the proper ones below for their respective pins
const bool tempSensor = true;         // DS18B20 temp sensor using OneWire library and DallaTemperature, set comms pin below
const bool conductivitySensor = true; // Gnd to VCC input range form voltage divider to sense continuity, set 3 pins below
const bool dissolvedO2Sensor = true;  // 0 to 3 V in to ADC, set ADC pin below
const bool turbiditySensor = true;    //! TODO Leave here?, going to depend on depth rating of sensor, can't find at present
const bool pressureSensor = true;     // 0.5 to 4.5 V in to ADC, set ADC pin below

// Pin assignment
const short tempSensorPin = 5;        // Pin for the temperature sensor
const short conductivityLeadA = 15;   // One lead for conductiviy sensor
const short conductivityLeadB = 17;   // The other lead for conductivity sensor
const short conductivitySense = 16;   // Pin for the conductivity sensor sense wire
const short dissolvedO2Pin = 13;      // Pin for analog input from DFrobot dissolved O2 https://wiki.dfrobot.com/Gravity__Analog_Dissolved_Oxygen_Sensor_SKU_SEN0237
const short batteryVoltageSense = 14; // Pin for the battery value input sensing via ADC

// Setup a oneWire instance to communicate with any OneWire devices such as the DS18B20 temp sensor
OneWire oneWire(tempSensorPin);
// Pass our oneWire reference to Dallas Temperature sensor for reading
DallasTemperature sensors(&oneWire);

// Variable assignments
long HeartbeatOldMillis = 0; // Keep time for heartbeat/status led
bool HeartbeatLED = false;   // Keeps light status so the app knows when to turn it on/off
const long ADCRes = 4096;    // Analog in resolution for later reference
const float VREF = 5000;     // voltage source (in mv) to the devices used to calculate ADC inputs

// Onboard NeoPixel LED colors
int status = 4; // 0 = Off, 1 = Red, 2 = Green, 3 = Blue, 4 (or anything else) = White

// Setup for conductivity sensor, static values first
const int Vin = 3.16;           // input voltage to the voltage divider (measure with a multimeter)
const float R1 = 979;           // fixed resistor value in ohms (measure with a multimeter)
const int samples = 100;        // number of samples to average for the conductivity sensor
const float cellConstant = 1.0; // cell constant of the conductivity probe (may need calibration)
const float alpha = 0.02;       // temperature coefficient for the solution being measured (0.02 for seawater, 0.019 for pure water, 0.021 for aquariums)
const int dtime = 500;          // delay time for the conductivity sensor readings

// conductivity values that will be calculated
int raw = 0; //! TODO Do we need all these up here or should i put them in the function to keep them scoped?
float Vout = 0;
float R2 = 0;
float buff = 0;
float avg = 0;
float conductivity = 0;
float salinity = 0;

// Dissolved O2 variables, static/calibration values first
const int Cal1_V = 1600;
const int Cal1_T = 25;
const int Cal2_V = 1000;
const int Cal1_T = 15;

// expected saturations at a given temperature in C, [0] is at 0 C and [41] is at 41 C, provided by DFRobot wiki
const short DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

// placeholder variables for dissolved O2 values to be calculated
short dissolvedO2Val = 0;

// hold the most recent temp reading
float currentTemp = 0;
float resistance = 0;

// function declarations
void LED_Status(int x);
float checkConductivity();
float getTemperature();
float calcSalinity(float resistance, float temp);

// setup code, runs once:
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
// do the work
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

  //! TODO Possibly leave hearbeat in its own function and make the status vs heartbeat mutually exlusive
  LED_Status(status); // Default heartbeat is white flashing LED, change status variable per indicator required
}

float getTemperature()
{
  return sensors.getTempCByIndex(0); // get the reading from the sensor
}

float getDissolvedO2Sat(temp)
{
  int disRaw = analogRead(dissolvedO2Pin); // read the input pin
  float ADC_V = VREF * disRaw / ADCRes;    // turn the ADC value back to a voltage
  /* use larger variables for calculation to maintain accuracy and prevent overflow but truncate the answer to a short, 35 is mv of change (slope) per degree C*/
  unsigned short V_saturation = (short)((float)temp - CAL2_T) * ((unsigned short)CAL1_V - CAL2_V) / ((int)CAL1_T - CAL2_T) + CAL2_V;
  dissolvedO2Val = (ADC_V * DO_Table[temp] / V_saturation);
}

float checkConductivity()
{
  float total = 0;
  for (int i = 0; i < samples; i++)
  {
    digitalWrite(conductivityLeadA, HIGH); // cycle the high and low to prevent undue corrosion ! Need better explanation
    digitalWrite(conductivityLeadB, LOW);
    delayMicroseconds(dtime);
    digitalWrite(conductivityLeadA, LOW);
    digitalWrite(conductivityLeadB, HIGH);
    //! TODO: One lead corrodes way faster because the work is done here, maybe cycle which way the tests are done over time
    delayMicroseconds(dtime);
    raw = analogRead(conductivitySense);
    if (raw)
    {
      buff = raw * Vin;
      Vout = (buff) / ADCRes;  // taking input ADC and converting back to a voltage value
      buff = (Vin / Vout) - 1; // calculating the ratio of the voltage divider
      R2 = R1 * buff;          // Calc the resistance of the water under test
      R2 = 0.85 * R2 - 14;     //? Optional calibration offset
      total = total + R2;      // add to the existing samples for later averaging
    }
  }
  avg = total / samples; // calc and return the average resistance
  return avg;
}

float calcSalinity(float resistance, float temp)
{
  conductivity = (1.0 / R2) * cellConstant * 1000.0;                 // Calculate conductivity (in mS/cm)
  float conductivity25 = conductivity / (1 + alpha * (temp - 25.0)); // Adjust conductivity to 25C
  //! TODO Maybe revisit for different temp ranges to see how this math works
  float calculatedSalinity = 0.4665 * conductivity - 0.0013 * conductivity * conductivity; // Convert conductivity to salinity (ppt)
  return calculatedSalinity;
}

void LED_Status(int x)
{
  long HeartbeatNewMillis = millis();
  if (HeartbeatNewMillis - HeartbeatOldMillis > x)
  {
    HeartbeatOldMillis = HeartbeatNewMillis;
    if (HeartbeatLED == false)
    {
      HeartbeatLED = true;
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
        neopixelWrite(RGB_BUILTIN, 60, 60, 60); // white
        break;
      }
    }
    else
    {
      HeartbeatLED = false;
      neopixelWrite(RGB_BUILTIN, 0, 0, 0); // off
    }
  }
}