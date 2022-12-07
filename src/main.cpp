// Load Wi-Fi library
#include <DeadReckoner.h>
#include <WiFi.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <HTTPClient.h>
#include <constantsV2.h>
#include <L298NX2.h>
#include <HCSR04.h>


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
volatile unsigned long leftTicks, rightTicks, leftAuxTicks, rightAuxTicks;

// Previous times for computing elapsed time.
unsigned long prevPositionComputeTime = 0, prevSendTime = 0;

// Previous x and y coordinate.
double prevX = 0, prevY = 0;

DeadReckoner deadReckoner(&leftTicks, &rightTicks, POSITION_COMPUTE_INTERVAL, TICKS_PER_REV, RADIUS, LENGTH);

// Inicializamos los motores
L298NX2 motors(L_ENABLE, L_IN_1, L_IN_2, R_ENABLE, R_IN_1, R_IN_2);

// Inicializamos los sensores ultrasonicos
UltraSonicDistanceSensor uds_left(5, 2);
UltraSonicDistanceSensor uds_right(19, 4);

void pulseLeft() { leftTicks++; leftAuxTicks++; }
void pulseRight() { rightTicks++; rightAuxTicks++; }

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
int numberSteps = 2;

void calcularUbicacion() { 
	if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL)
	{
		// Computes the new angular velocities and uses that to compute the new position.
		// The accuracy of the position estimate will increase with smaller time interval until a certain point.
		deadReckoner.computePosition();
		prevPositionComputeTime = millis();
	}

	if (millis() - prevSendTime > SEND_INTERVAL)
	{
		// Cartesian coordinate of latest location estimate.
		// Length unit correspond to the one specified under MEASUREMENTS.
		x = deadReckoner.getX();
		y = deadReckoner.getY();

		// Left and right angular velocities.
		wl = deadReckoner.getWl();
		wr = deadReckoner.getWr();

		// getTheta method returns the robot position angle in radians measured from the x axis to the center of the robot.
		// This angle is set initially at zero before the robot starts moving.
		theta = deadReckoner.getTheta();

		// Total distance robot has troubled.
		distance = sqrt(x * x + y * y);
		// x, y, wl, wr, theta, dist
		Serial.print(x);
		Serial.print(",\t");
		Serial.print(y);
		Serial.print(",\t");
		Serial.print(wl);
		Serial.print(",\t");
		Serial.print(wr);
		Serial.print(",\t");
		Serial.print(theta * RAD_TO_DEG); // theta converted to degrees.
		Serial.print(",\t");
		Serial.println(distance);
		

		prevSendTime = millis();
	}
}

void calcularFronteras() {
	Serial.println("---------------------------- START Measuring");
	Serial.println( uds_left.measureDistanceCm() );
	Serial.println( uds_right.measureDistanceCm() );
	Serial.println(sensor.readRangeSingleMillimeters());
	Serial.println("----------------------------- END Measuring");

	delay(800);
}
void stepForward() {
	while(numberSteps > 0){
		calcularUbicacion();
		calcularFronteras();
		if(leftAuxTicks >= 12){ // antes 18
			motors.stopA();
		}else{
			motors.forwardA();
		}

		if(rightAuxTicks >= 12){
			motors.stopB();
		}else{
			motors.forwardB();
		}
		
		if(!(motors.isMovingA()) && !(motors.isMovingB())){
			calcularUbicacion();
			calcularFronteras();
			leftAuxTicks = 0;
			rightAuxTicks = 0;
			numberSteps--;
		}
	}	
}

void loop()
{
	
	stepForward();
	// calcularCoordenadas();
	/*
	Serial.println( uds_left.measureDistanceCm() );
	Serial.println( uds_right.measureDistanceCm() );
	Serial.println( "-----" );
	Serial.println();
	delay(600);
	*/

	
	
	
}
