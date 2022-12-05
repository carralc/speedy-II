// Load Wi-Fi library
#include <DeadReckoner.h>
#include <WiFi.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <HTTPClient.h>
#include <constantsV2.h>
#include <L298NX2.h>

VL53L0X sensor;

// Modo largo alcance
#define LONG_RANGE

// Modo alta precision
// #define HIGH_SPEED
#define HIGH_ACCURACY

// network credentials
const char *ssid = "Sergio's Galaxy S20 FE 5G";
const char *password = "stk4121505";

// Your Domain name with URL path or IP address with path
String serverName = "http://192.168.173.35:5000/receptor";

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
// unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 1000;

// Dead reckoner

// Number of left and right tick counts on the encoder.
volatile unsigned long leftTicks, rightTicks;

// Previous times for computing elapsed time.
unsigned long prevPositionComputeTime = 0, prevSendTime = 0;

// Previous x and y coordinate.
double prevX = 0, prevY = 0;

DeadReckoner deadReckoner(&leftTicks, &rightTicks, POSITION_COMPUTE_INTERVAL, TICKS_PER_REV, RADIUS, LENGTH);

// Inicializamos los motores
L298NX2 motors(L_ENABLE, L_IN_1, L_IN_2, R_ENABLE, R_IN_1, R_IN_2);

void pulseLeft() { 
	leftTicks++; 
	if(leftTicks>=30){
		Serial.print("Ticks izquierda:");
		Serial.println(leftTicks);
		motors.stopA();
	}
}
void pulseRight() { 
	rightTicks++; 
	if(rightTicks>=30){
		motors.stopB();
		Serial.print("Ticks derecha:");
		Serial.println(rightTicks);
	}
}

long lapse;

/**
Attaches interrupt and disables all serial communications.
This is necessary because when interrupts are active, incoming serial communication can be lost.
*/
void attachInterrupts()
{
	attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), pulseLeft, HIGH);
	attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), pulseRight, HIGH);
}

void setup()
{
	Serial.begin(9600);
	attachInterrupts();
	// Encoder and motor outputs
	pinMode(L_ENABLE, OUTPUT);
	pinMode(R_ENABLE, OUTPUT);
	pinMode(L_IN_1, OUTPUT);
	pinMode(L_IN_2, OUTPUT);
	pinMode(R_IN_1, OUTPUT);
	pinMode(R_IN_2, OUTPUT);

	WiFi.begin(ssid, password);
	Serial.println("Connecting");
	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.print("Connected to WiFi network with IP Address: ");
	Serial.println(WiFi.localIP());

	Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");

	// Configuracion del sensor
	Wire.begin(21, 22);

	sensor.setTimeout(500);
	if (!sensor.init())
	{
		Serial.println("Failed to detect and initialize sensor!");
		while (1)
		{
		}
	}
	motors.setSpeedA(255);
	motors.setSpeedB(255);
	motors.forwardA();
	motors.forwardB();
#if defined LONG_RANGE
	// lower the return signal rate limit (default is 0.25 MCPS)
	sensor.setSignalRateLimit(0.1);
	// increase laser pulse periods (defaults are 14 and 10 PCLKs)
	sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
	sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
	// reduce timing budget to 20 ms (default is about 33 ms)
	sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
	// increase timing budget to 200 ms
	sensor.setMeasurementTimingBudget(200000);
#endif

}
double x;
double y;
double wl;
double wr;
double theta;
double distance;

void loop()
{
	//rightTicks = 0;
	//motors.forwardForA(lapse);
	//Serial.print("ticks derecha:");
	//Serial.println(rightTicks);
	//delay(10000);
	//lapse += 333;

}
