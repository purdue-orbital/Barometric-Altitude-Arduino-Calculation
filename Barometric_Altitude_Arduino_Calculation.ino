/**************** MAX31855K_Thermocouple_Digitizer_Example.ino *****************
 *                                                                             *
 * MAX31855K Thermocouple Breakout Example Code                                *
 * brent@sparkfun.com                                                          *
 * March 26th 2015                                                             *
 * https://github.com/sparkfun/MAX31855K_Thermocouple_Digitizer                *
 *                                                                             *
 * Use the "serial monitor" window to read a temperature sensor.               *
 *                                                                             *
 * Circuit:                                                                    *
 * MAX31855K breakout attached to the following pins                           *
 *  SS:   pin 10                                                               *
 *  MOSI: pin 11 (NC)                                                          *
 *  MISO: pin 12                                                               *
 *  SCK:  pin 13                                                               *
 *  VCC:  pin 14                                                               *
 *  GND:  pin 15                                                               *
 *                                                                             *
 *                                                                             *
 * Development environment specifics:                                          *
 * 1.6.4                                                                       *
 * Arduino Pro Mini 328 3.3V/8MHz                                              *
 *                                                                             *                                *
 ******************************************************************************/
/******************************************************************************
/**************** Barometer****************************************************
https://github.com/sparkfun/MS5803-14BA_Breakout/
The MS58XX MS57XX and MS56XX by Measurement Specialties is a low cost I2C pressure
sensor.  This sensor can be used in weather stations and for altitude
estimations. It can also be used underwater for water depth measurements. 
Resources:
This library uses the Arduino Wire.h to complete I2C transactions.
Development environment specifics:
IDE: Arduino 1.0.5
Hardware Platform: Arduino Pro 3.3V/8MHz
T5403 Breakout Version: 1.0
 **Updated for Arduino 1.6.4 5/2015**

 This code is beerware. If you see me (or any other SparkFun employee) at the
 local pub, and you've found our code helpful, please buy us a round!
 Distributed as-is; no warranty is given.
 ******************************************************************************/
#include <SD.h>
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>
// Begin class with selected address
// available addresses (selected by jumper on board) 
// default is ADDRESS_HIGH
//  ADDRESS_HIGH = 0x76
//  ADDRESS_LOW  = 0x77
#include <SparkFunMAX31855k.h> // Using the max31855k driver
#include <SPI.h>  
MS5803 sensor(ADDRESS_HIGH);
//Create variables to store results
float temperature_C, temperature_F;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
// Create Variable to store altitude in (m) for calculations;
double BASE_ALTITUDE = 187.0; // Altitude of West Lafayette in (m) [614 in ft]
float altitude_now = BASE_ALTITUDE;
// Define SPI Arduino pin numbers (Arduino Pro Mini)
const uint8_t CS1 = 10; // Using standard CS line (SS) [thermocouple]
const uint8_t CS2 = 9; // Using standard CS line (SS) [sd card]
// SCK & MISO are defined by Arduiino
const uint8_t VCC = 14; // A0 in Nano 
const uint8_t GND = 15; // A1 in Nano
// Instantiate an instance of the SparkFunMAX31855k class
SparkFunMAX31855k probe(CS1, VCC, GND);
// LED initialization
int blue = 6;
int green = 4;
// Igniter Pin
int igniter = 3;
// Initializes variable k
int k = 0;
// global var for altitude
double global_alt; 
// counter for consecutive readings of an altitude above 25 KM
int above_25_counter;

void setup() {

	// LED Setup
	pinMode(blue, OUTPUT);
	pinMode(green, OUTPUT);
	pinMode(igniter, OUTPUT);
	Serial.begin(9600);
	Serial.println("\nBeginning...");
	Serial.print("\nInitializing SD card...");
	// Initialization SD card - begins communication with SD card
	digitalWrite(CS2, LOW);

	if (!SD.begin(CS2)) // see if the card is present and can be initialized
	{
		Serial.println("Card failed, or not present");
		return;// don't do anything more
	}
	Serial.println("card initialized.");
	File dataFile = SD.open("data.csv", FILE_WRITE);
	if (dataFile && k == 0)  // if the file is available, write to it
	{
		dataFile.println("Pressure (mbar),Altitude change (m),Altitude (m),Temperature (C),Temperature (F),");
		dataFile.close();
		k++;
	}
	else // if the file isn't open, pop up an error:
	{
		Serial.println("error opening data.csv");
		return;// don't do anything more
	}

	// Ends communication with SD card
	digitalWrite(CS2, HIGH);

	//Retrieve calibration constants for conversion math.
	sensor.reset();
	sensor.begin();
	// Initialize barometer and read pressure from sensor in mbar.
	pressure_baseline = sensor.getPressure(ADC_4096);

	delay(50);  // Let IC stabilize or first readings will be garbage
}
void loop() {

	// SECTION 1: Thermocouple readings

	// Initialization Thermocouple - Begins communication with Thermocouple
	digitalWrite(CS1, LOW);
	float temperature = probe.readCJT();
	// Read the temperature in Celsius 
	temperature_C = probe.readTempC();
	if (!isnan(temperature_C)) {
		Serial.print("Temp[C]=");
		Serial.print(temperature_C);
	}
	// Read the temperature in Fahrenheit
	temperature_F = probe.readTempF();
	if (!isnan(temperature_F)) {
		Serial.print("\tTemp[F]=");
		Serial.print(temperature_F);
	}
	// Ends communication with Thermocouple
	digitalWrite(CS1, HIGH);
	// END OF SECTION 1
	// SECTION 2: Barometer readings 

	// Pressure reading from the sensor in mbar.
	pressure_abs = sensor.getPressure(ADC_4096);

	// Convert abs pressure with the help of altitude into relative pressure
	// This is used in Weather stations.
	pressure_relative = sealevel(pressure_abs, BASE_ALTITUDE);

	// Taking our baseline pressure at the beginning we can find an approximate
	// change in altitude based on the differences in pressure.   
	altitude_now = altitude(pressure_baseline, pressure_abs, temperature_C);

	global_alt = altitude_now;

	if (global_alt == -1) {
		Serial.print("ERROR: Altitude Out of Range\n");
	}
	
	// Checks if pressure data is being generated through red LED 
	if (!isnan(pressure_relative)) {
		digitalWrite(blue, HIGH);  // blue
		delay(500);
		digitalWrite(blue, LOW);  // off blue
	}
	// Reads the pressure in mbar
	Serial.print("\n");
	Serial.print("Pressure abs (mbar)= ");
	Serial.println(pressure_abs);
	// Reads the change in altitude in m.
	Serial.print("Altitude Now (m) = ");
	Serial.println(altitude_now); 
	Serial.print("\n\n");

	// END OF SECTION 2
	// SECTION 3: Writing data to SD card

	// Begins SD card communication
	digitalWrite(CS2, LOW);
	File dataFile = SD.open("data.csv", FILE_WRITE);
	// WRITING TO FILE ON SD CARD
	if (dataFile) 
	{
		dataFile.print(pressure_abs);
		dataFile.print(", ");
		dataFile.print(altitude_now);
		dataFile.print(", ");
		dataFile.print(temperature_C);
		dataFile.print(", ");
		dataFile.print(temperature_F);
		dataFile.println(",");
		dataFile.close();
	}
	else 
	{
		Serial.println("error opening data.csv");
	}
	// Ends communication with sd card 
	digitalWrite(CS2, HIGH);
	// END OF SECTION 3

	// SECTION 4: Ignition through a transistor

	// Ignition parameters
	if (altitude_now >= 100 && millis()>=1000 && above_25_counter >= 10) // altitude >= 50000 ft. (15240m) and time >= 20 min (1200000 millisec) and 10 or more consecutive readings above 25KM;
	{
		Serial.print("IGNITION!\n");
		digitalWrite(igniter, HIGH);
		delay(1000);
		digitalWrite(igniter, LOW);
		digitalWrite(green, HIGH);
		delay(500);
		digitalWrite(green, LOW);
	}
	else if (millis() >= 2400000) // timer based launch (40 min.)
	{
		Serial.print("IGNITION!\n");
		digitalWrite(igniter, HIGH);
		delay(1000);
		digitalWrite(igniter, LOW);    
		digitalWrite(green, HIGH);
		delay(500);
		digitalWrite(green, LOW);
	}
	// END OF SECTION 4
	delay(1000);
}
double sealevel(double P, double A)
	// Given a pressure P (mbar) taken at a specific altitude (meters),
	// return the equivalent pressure (mbar) at sea level.
	// This produces pressure readings that can be used for weather measurements.
{
	return(P/pow(1-(A/44330.0),5.255));
}
double altitude(double P_Initial, double P_Final, double temperature)
	// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
	// return altitude (meters) above baseline.
{
	//double alt = ((pow(P_Initial / P_Final, (1 / 5.275)) - 1) * (temperature + 273.15) ) / (0.0065) + BASE_ALTITUDE;
	
	double alt;
	const double A1 = -0.0065;
	const double A2 = 0.0030;
	const double R = 287;
	const double G = 9.81;
	const double ELEVEN_KM = 11000;
	const double TWENTY_FIVE_KM = 25000;
	const double FORTY_SEVEN_KM = 47000;
	const double TEMP_CONST_INIT = 288.16;
	const double TEMP_CONST_GT_ELEVEN_KM = 216.66;
	const double SEA_LEVEL_PRESSURE = 1013.25;
	const double ELEVEN_KM_PRESSURE = 226.321;
	const double TWENTY_FIVE_KM_PRESSURE = 25.1102;
	
	// if altitude is between 0km to ELEVEN_KM	
	if (global_alt >= 0 && global_alt < ELEVEN_KM) {
		// reset counter
		above_25_counter = 0;
		alt = (TEMP_CONST_INIT / A1) * (pow((P_Final/SEA_LEVEL_PRESSURE), ((-A1  * R) / G)) - 1);
	// else if altitude is between ELEVEN_KM to TWENTY_FIVE_KM	
	} else if (global_alt >= ELEVEN_KM && global_alt < TWENTY_FIVE_KM) {
		// reset counter
		above_25_counter = 0;
		alt = ((-R * TEMP_CONST_GT_ELEVEN_KM) / G) * log(P_Final/ELEVEN_KM_PRESSURE) + ELEVEN_KM;
	// else if altitude is between TWENTY_FIVE_KM to FORTY_SEVEN_KM	
	} else if (global_alt >= TWENTY_FIVE_KM && global_alt < FORTY_SEVEN_KM) {
		// increment global above_25_counter;
		above_25_counter++;
		alt = (TEMP_CONST_GT_ELEVEN_KM / A2) * (pow((P_Final/TWENTY_FIVE_KM_PRESSURE), ((-A2  * R) / G)) - 1) + TWENTY_FIVE_KM;
	} else {
		// else, altitude is reading outside the scope of what we can measure from the current sensor array, so return -1 to signal out of bounds error
		alt = -1;
	}
	return alt;
}
