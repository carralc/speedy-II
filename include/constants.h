// Laser 
#define LASER_SCL 18
#define LASER_SDA 5
#define LASER_GPIO1 4
#define LASER_XSHUT 2

// H bridge pins
#define H_EN_A 25
#define H_EN_B 13
#define H_IN_1 26
#define H_IN_2 27
#define H_IN_3 14
#define H_IN_4 12

// Left wheel
#define L_ENABLE H_EN_A
#define L_IN_1 H_IN_1
#define L_IN_2 H_IN_2

// Right wheel
#define R_ENABLE H_EN_B
#define R_IN_1 H_IN_3
#define R_IN_2 H_IN_4

// Left odometer
#define L_D0 35
#define L_A0 34

// Right odometer
#define R_D0 32
#define R_A0 33

// Depth sensors
#define D1_ECHO 15
#define D1_TRIG 2

#define DL_ECHO 19
#define DL_TRIGGER 21

#define DR_ECHO 23
#define DR_TRIGGER 22

// Time (ms) it takes to move from cell 2 cell
#define STEP_TIME 410

// Time it takes to make a 90-degree turn
#define TURN_90D_TIME 270

// Time (ms) it takes to make a 180-degree turn
#define TURN_180D_TIME 800

// Rotation clicks for the movement of the whole robot
#define ROT_CLICKS 1000

#define ROT_CLICKS_L 130
#define ROT_CLICKS_R 140

// Lapse of time in ms on which we calculate speed
#define SPEED_TIME_DELTA 1000
