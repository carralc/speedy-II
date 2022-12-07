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
String serverName = "http://192.168.50.168:5000/dataserver";

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
// unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 300;

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
double distFront;
double distRight; 
double distLeft; 
int numberSteps = 2;
void calcularFronteras() {
	distLeft = uds_left.measureDistanceCm() * 10; // cm to mm
	distRight = uds_right.measureDistanceCm() * 10; 
	distFront = sensor.readRangeSingleMillimeters();
	delay(600);
}
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



void imprimirFronteras() {
	Serial.printf("distFront: %f\n", distFront);
	Serial.printf("distLeft: %f\n", distLeft);
	Serial.printf("distRight: %f\n", distRight);
}
void stepForward() {
	while(numberSteps > 0){
		calcularUbicacion();
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
			
			leftAuxTicks = 0;
			rightAuxTicks = 0;
			numberSteps--;
		}
	}	
}

// Calcula las coordenadas de un obstáculo detectado por el sensor delantero 
void coordenadasObstFrente(double* xObst, double* yObst){
	if(distFront < 0 || distFront >= 2000){
		*xObst = -1;
		*yObst = -1;
	}
	double xp = x + distFront*cos(theta);
	double yp = y + distFront*sin(theta);
	*xObst = xp;
	*yObst = yp;
}

// Calcula las coordenadas de un obstáculo detectado por el sensor izquierdo 
void coordenadasObstIzq(double * xObst, double * yObst){
	if(distLeft < 0 || distLeft >= 2000){
		*xObst = -1;
		*yObst = -1;
	}
	double xp = x - distLeft*cos(1.57 - theta); // 90 en rad
	double yp = y + distLeft*sin(1.57 - theta);
	*xObst = xp;
	*yObst = yp;
}

// Calcula las coordenadas de un obstáculo detectado por el sensor derecho 
void coordenadasObstDer(double * xObst, double * yObst){
	if(distLeft < 0 || distLeft >= 2000){
		*xObst = -1;
		*yObst = -1;
	}
	double xp = x + distRight*cos(1.57 - theta); // 90 en rad
	double yp = y - distRight*sin(1.57 - theta);
	*xObst = xp;
	*yObst = yp;
}

void loop()
{

	calcularUbicacion();
	calcularFronteras();
	imprimirFronteras();
	
	if(distFront <= 200){
		motors.stop();
		motors.forwardA();
		delay(500);
		motors.backwardB();
		delay(500);
		return;
	}

	if(distRight <= 200){
		motors.stop();
		motors.backwardA();
		delay(500);
		motors.forwardB();
		delay(500);
		return;
	}

	if(distLeft <= 200){
		motors.stop();
		motors.backwardB();
		delay(500);
		motors.forwardA();
		delay(500);
		return;
	}

	motors.forwardA();
	motors.forwardB();
	
	double xObstFrente, yObstFrente;
	double xObstDer, yObstDer;
	double xObstIzq, yObstIzq;
	
	coordenadasObstFrente(&xObstFrente, &yObstFrente);
	coordenadasObstDer(&xObstDer, &yObstDer);
	coordenadasObstIzq(&yObstIzq, &yObstIzq);

	// Send an HTTP POST request every 10 minutes
	if ((millis() - lastTime) > timerDelay)
	{
		// Check WiFi connection status
		if (WiFi.status() == WL_CONNECTED)
		{
			HTTPClient http;

			String serverPath = serverName + "/" + x + "/" + y + "/" + xObstFrente + "/" + yObstFrente + "/" + xObstDer + "/" + yObstDer + "/" + xObstIzq + "/" + yObstIzq;  

			// Your Domain name with URL path or IP address with path
			http.begin(serverPath.c_str());

			// If you need Node-RED/server authentication, insert user and password below
			// http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");

			// Send HTTP GET request
			int httpResponseCode = http.GET();

			if (httpResponseCode > 0)
			{
				Serial.print("HTTP Response code: ");
				Serial.println(httpResponseCode);
				String payload = http.getString();
				Serial.println(payload);
			}
			else
			{
				Serial.print("Error code: ");
				Serial.println(httpResponseCode);
			}
			// Free resources
			http.end();
		}
		else
		{
			Serial.println("WiFi Disconnected");
		}
		lastTime = millis();
	}
	
	// stepForward();
	// calcularCoordenadas();
	/*
	Serial.println( uds_left.measureDistanceCm() );
	Serial.println( uds_right.measureDistanceCm() );
	Serial.println( "-----" );
	Serial.println();
	delay(600);
	*/

}
