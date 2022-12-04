#include <DeadReckoner.h>

// ENCODER PINS
#define ENCODER_LEFT_PIN 35
#define ENCODER_RIGHT_PIN 32

#define L_ENABLE 25
#define R_ENABLE 13

#define L_IN_1 26
#define R_IN_1 14

#define L_IN_2 27 
#define R_IN_2 12

// MEASUREMENTS
// The units for all measurements must be consistent. 
// You can use any length unit as desired.
#define RADIUS 33 // wheel radius in mm
#define LENGTH 102 // wheel base length in mm
#define TICKS_PER_REV 140

// TIME INTERVALS
#define POSITION_COMPUTE_INTERVAL 50 // milliseconds
#define SEND_INTERVAL 100 // milliseconds


// Number of left and right tick counts on the encoder.
volatile unsigned long leftTicks, rightTicks;

// Previous times for computing elapsed time.
unsigned long prevPositionComputeTime = 0, prevSendTime = 0;

// Previous x and y coordinate.
double prevX = 0, prevY = 0;


DeadReckoner deadReckoner(&leftTicks, &rightTicks, POSITION_COMPUTE_INTERVAL, TICKS_PER_REV, RADIUS, LENGTH);


void pulseLeft() { leftTicks++; }
void pulseRight() { rightTicks++; }

/**
Attaches interrupt and disables all serial communications.
This is necessary because when interrupts are active, incoming serial communication can be lost.
*/
void attachInterrupts() {
	attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), pulseLeft, HIGH);
	attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), pulseRight, HIGH);
}

void setup() {
	attachInterrupts();
	Serial.begin(9600);
	pinMode(L_ENABLE, OUTPUT);
	pinMode(R_ENABLE, OUTPUT);
	pinMode(L_IN_1, OUTPUT);
	pinMode(L_IN_2, OUTPUT);
	pinMode(R_IN_1, OUTPUT);
	pinMode(R_IN_2, OUTPUT);
}

void loop() {
	
	digitalWrite(L_IN_1, LOW);
	digitalWrite(L_IN_2, HIGH);
	digitalWrite(R_IN_1, HIGH);
	digitalWrite(R_IN_2, LOW);

	// delay(1000);

	digitalWrite(R_ENABLE, HIGH);
	digitalWrite(L_ENABLE, HIGH);
	
	

	if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) {
		// Computes the new angular velocities and uses that to compute the new position.
		// The accuracy of the position estimate will increase with smaller time interval until a certain point.
		deadReckoner.computePosition();
		prevPositionComputeTime = millis();
	}

	if (millis() - prevSendTime > SEND_INTERVAL) {
		// Cartesian coordinate of latest location estimate.
		// Length unit correspond to the one specified under MEASUREMENTS.
		double x = deadReckoner.getX();
		double y = deadReckoner.getY();

		// Left and right angular velocities.
		double wl = deadReckoner.getWl();
		double wr = deadReckoner.getWr();

		// getTheta method returns the robot position angle in radians measured from the x axis to the center of the robot.
		// This angle is set initially at zero before the robot starts moving.
		double theta = deadReckoner.getTheta();

		// Total distance robot has troubled.
		double distance = sqrt(x * x + y * y);
		// x, y, wl, wr, theta, dist
		Serial.print(x);
		Serial.print(",\t"); Serial.print(y);
		Serial.print(",\t"); Serial.print(wl);
		Serial.print(",\t"); Serial.print(wr);
		Serial.print(",\t"); Serial.print(theta*RAD_TO_DEG); // theta converted to degrees.
		Serial.print(",\t"); Serial.println(distance);

		prevSendTime = millis();
	}
}
