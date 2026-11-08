//Pin declarations
const int trigPin = 12;
const int echoPin = 11;
const int gasPin = A0;
const int piezo = 10;
const int encAleft = 2;
const int encBleft = A2;
const int encAright = 3;
const int encBright = A3;
//gas limit
const int gasLimit = 100;

//Left motors
const int EN1 = 9;
const int IN1 = 8;
const int IN2 = 7;

//Right motors
const int EN2 = 5;
const int IN3 = 4;
const int IN4 = A1;

// FUNCTION DECLARATIONS
long getDistance();
void alarmSound(int times);
void beepSound(int freq);
void stopRobot();

void moveBackwards(int speed);
void moveForward(int speed);
void turnRight(int speed);
void turnLeft(int speed);
void spinRight(int speed);
void spinLeft(int speed);
void countLeftPulses();
void countRightPulses();
void calculateRightPID(float dt);
void calculateLeftPID(float dt);

//enumeration
enum RobotState{
  	STOPPED,
	MOVING_FORWARD,
  	TURNING,
  	RESUMING,
  	SCANNING_RIGHT,
  	SCANNING_LEFT,
  	DECIDING,
  	REVERSING
};

RobotState currentState = MOVING_FORWARD;
RobotState previousState = MOVING_FORWARD;

long rightDist = 0;
long leftDist = 0;


long previousDist = 0;
int previousGasValue = 0;

//First Runs boolean
bool firstRun = true;
bool turnRightChosen = true;
bool secondRun = true;
bool thirdRun = true;
bool firstRightPIDRun = true;
bool firstLeftPIDRun = true;

//Volatile values
volatile unsigned long leftPulseCount = 0;

volatile unsigned long rightPulseCount = 0;

const float pulsePerRevolution = 100.0;

// Left motor PID
float leftError = 0.0;
float leftIntegral = 0.0;
float leftPreviousError = 0.0;
float leftDerivative = 0.0;
float leftPIDOutput = 0.0;

// Right motor PID
float rightError = 0.0;
float rightIntegral = 0.0;
float rightPreviousError = 0.0;
float rightDerivative = 0.0;
float rightPIDOutput = 0.0;

//Constants
float Kp = 0.5;
float Ki = 0.0;
float Kd = 0.0;

float P = 0.0;
float I = 0.0;
float D = 0.0;

float leftRPM = 0.0;
float rightRPM = 0.0;

float targetRPM = 250.0;

unsigned long prevTime = 0;
unsigned long prevEncTime = 0;

int basePWM = 120;
int rightMotorPWM = 0;
int leftMotorPWM = 0;

//SETUP
void setup(){
	pinMode(echoPin, INPUT);
  	pinMode(gasPin, INPUT);
  	pinMode(trigPin, OUTPUT);
  	pinMode(piezo, OUTPUT);
  	pinMode(EN1, OUTPUT);
  	pinMode(IN1, OUTPUT);
  	pinMode(IN2, OUTPUT);
  	pinMode(EN2, OUTPUT);
  	pinMode(IN3, OUTPUT);
  	pinMode(IN4, OUTPUT);
  	pinMode(encAleft, INPUT_PULLUP);
  	pinMode(encBleft, INPUT_PULLUP);
  	pinMode(encAright, INPUT_PULLUP);
  	pinMode(encBright, INPUT_PULLUP);
  
  	attachInterrupt(digitalPinToInterrupt(encAleft), countLeftPulses, RISING);
  	attachInterrupt(digitalPinToInterrupt(encAright), countRightPulses, RISING);
  	Serial.begin(9600);
  
  	// Establish forward direction
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, HIGH);
	digitalWrite(IN4, LOW);

	// Start both motors at the base PWM
	analogWrite(EN1, basePWM);
	analogWrite(EN2, basePWM);
}

//Distance function 
long getDistance(){
	digitalWrite(trigPin, LOW);
  	delayMicroseconds(10);
  	digitalWrite(trigPin, HIGH);
  	delayMicroseconds(10);
  	digitalWrite(trigPin, LOW);
  	delayMicroseconds(10);
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 0;
  
  return duration * 0.034 / 2;
}

//Pulses
void countLeftPulses(){
	leftPulseCount++;
}
void countRightPulses(){
	rightPulseCount++;
}

//PID calculations
void calculateLeftPID(float dt){
	
  	P = Kp * leftError;
  
  	leftIntegral += leftError * dt;
  	leftIntegral = constrain(leftIntegral, -100, 100);
  	I = leftIntegral * Ki;
  
  	if (firstLeftPIDRun) {
    	leftPreviousError = leftError;
    	firstLeftPIDRun = false;
	}
  	leftDerivative = (leftError - leftPreviousError)/dt;
  	D = leftDerivative * Kd;
  
  	leftPIDOutput = P + I + D;
  
  	leftPreviousError = leftError;
}

void calculateRightPID(float dt){
	
  	P = Kp * rightError;
  
  	rightIntegral += rightError * dt;
  	rightIntegral = constrain(rightIntegral, -100, 100);
  	I = rightIntegral * Ki;
  
  	if (firstRightPIDRun) {
    	rightPreviousError = rightError;
    	firstRightPIDRun = false;
	}
  	rightDerivative = (rightError - rightPreviousError)/dt;
  	D = rightDerivative * Kd;
  
  	rightPIDOutput = P + I + D;
  
  	rightPreviousError = rightError;
}

//alarm
void alarmSound(int times){
  for (int i=0; i < times; i++){
  	tone(piezo, 1000, 300);
    delay(400);
    tone(piezo, 500, 300);
    delay(400);
  }
}
void beepSound(int freq){
	tone(piezo, freq, 150);
  	delay(400);
}

//motor controlls
void stopRobot(){
  	analogWrite(EN1, 0);
    analogWrite(EN2, 0);
	digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  	digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }

void moveBackwards(int speed = 180){
	digitalWrite(IN1, LOW);digitalWrite(IN2, HIGH);
  	digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  	analogWrite(EN1, speed); analogWrite(EN2, speed);
}
void moveForward(int speed = 180){
	digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  	digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  	analogWrite(EN1, speed); analogWrite(EN2, speed);
}
void turnRight(int speed = 140){
	digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  	digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  	analogWrite(EN1, speed); analogWrite(EN2, speed);
}
void turnLeft(int speed = 140){
	digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  	digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  	analogWrite(EN1, speed); analogWrite(EN2, speed);
}
void spinRight(int speed = 150){
	digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  	digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  	analogWrite(EN1, speed); analogWrite(EN2, speed);
}
void spinLeft(int speed = 150){
	digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  	digitalWrite(IN3,HIGH);digitalWrite(IN4, LOW);
  
  	analogWrite(EN1, speed); analogWrite(EN2, speed);
}

//Brain
void loop(){
	long dist = getDistance();
  	int gasValue = analogRead(gasPin);
  
  
  	unsigned long currTime = millis();
  if (abs(dist - previousDist) >= 5 || secondRun){
  	Serial.println("Dist:");Serial.println(dist);
    
    previousDist = dist;
    secondRun = false;
  }
  if (gasValue != previousGasValue || thirdRun){
  	Serial.println("Gas:");Serial.println(gasValue);
    
    previousGasValue = gasValue;
    thirdRun = false;
  }
  
  unsigned long encElapsed = currTime - prevEncTime;
  
  if (currTime - prevEncTime >= 1000){
  	
    unsigned long leftPulses;
    unsigned long rightPulses;
    
    noInterrupts();
    
    leftPulses = leftPulseCount;
    rightPulses = rightPulseCount;
    
    leftPulseCount = 0;
    rightPulseCount = 0;
    
    interrupts();
    
    //Pulses
    Serial.println("Left pulses:");
    Serial.println(leftPulses);
    
    Serial.println("Right pulses:");
    Serial.println(rightPulses);
    
    //RPM
    leftRPM = (leftPulses * 60000.0)/(pulsePerRevolution * encElapsed);
    rightRPM = (rightPulses * 60000.0)/(pulsePerRevolution * encElapsed);
    
    Serial.print("Left RPM: ");
	Serial.println(leftRPM);

	Serial.print("Right RPM: ");
	Serial.println(rightRPM);
    
      //PIDOUTPUT
	// Calculate actual elapsed time in seconds
	float dt = encElapsed / 1000.0;

	// Calculate each motor's error
	leftError = targetRPM - leftRPM;
	rightError = targetRPM - rightRPM;

	// Calculate each motor's PID output
	if (dt > 0) {
    	calculateLeftPID(dt);
    	calculateRightPID(dt);

    	// Convert PID corrections into motor PWM values
    	leftMotorPWM = constrain(basePWM + leftPIDOutput, 0, 255);
    	rightMotorPWM = constrain(basePWM + rightPIDOutput, 0, 255);

    	// Apply independent PWM values to the motors
    	analogWrite(EN1, leftMotorPWM);
    	analogWrite(EN2, rightMotorPWM);

    	Serial.print("Left PWM: ");
    	Serial.println(leftMotorPWM);

    	Serial.print("Right PWM: ");
    	Serial.println(rightMotorPWM);
	}
    
    prevEncTime = currTime;
    
  }

  
  if (currentState != previousState || firstRun){
    if (currentState == MOVING_FORWARD){
    	Serial.println("You are safe. Moving forward");
    }
    else if (currentState == STOPPED){
    	Serial.println("Obstacle detected! Stopping!");
    }
    else if (currentState == SCANNING_RIGHT){
    	Serial.println("Scanning right side...");
    }
    else if (currentState == SCANNING_LEFT){
    	Serial.println("Scanning left side...");
    }
    else if (currentState == DECIDING){
    	Serial.println("Robot is deciding which way to turn...");
    }
    else if (currentState == TURNING){
    	Serial.println("Robot is turning to a safer side...");
    }
    else if (currentState == RESUMING){
    	Serial.println("Robot is resuming moving forward...");
    }else if (currentState == REVERSING){
    	Serial.println("Reversing for proper scanning...");
    }
    previousState = currentState;
    firstRun = false;
    
  }
  
  if (currentState == MOVING_FORWARD){
    if (dist >= 50){
    	moveForward(200);
    }
    else{
    	stopRobot();
      	
      	currentState = STOPPED;
      	prevTime = millis();
    }
     
  }
  else if (currentState == STOPPED){
  	stopRobot();
    
    if (millis() - prevTime >= 2000){
    	currentState = SCANNING_RIGHT;
      	prevTime = millis();
    }
  }
  else if (currentState == SCANNING_RIGHT){
    spinRight(150);
    
    if (millis() - prevTime >= 3000){
      	stopRobot();
      
    	rightDist = getDistance();
      	currentState = SCANNING_LEFT;
      	prevTime = millis();
    }
  }
  else if (currentState == SCANNING_LEFT){
  	spinLeft(150);
    
    if (millis() - prevTime >= 6000){
      	stopRobot();
      
    	leftDist = getDistance();
      	currentState = DECIDING;
      	prevTime = millis();
    }
  }
  else if (currentState == DECIDING) {
    stopRobot();

    if (rightDist < 50 && leftDist < 50) {
        Serial.println("Both sides blocked. Scanning again.");

        currentState = REVERSING;
        prevTime = millis();
    }
    else {
        if (rightDist > leftDist) {
            turnRightChosen = true;
        }
        else if (leftDist > rightDist) {
            turnRightChosen = false;
        }
        else {
            turnRightChosen = true; 
        }

        currentState = TURNING;
        prevTime = millis();
    	}
	}
  else if (currentState == TURNING){
	    if (turnRightChosen) {
        turnRight(150);
    }
    else {
        turnLeft(150);
    }

    if (millis() - prevTime >= 3000) {
        stopRobot();
        currentState = RESUMING;
        prevTime = millis();
    }
  }
  else if (currentState == RESUMING){
  	moveForward(245);
    currentState = MOVING_FORWARD;
    prevTime = millis();
  }
  else if (currentState == REVERSING){
  	moveBackwards(200);
    
    if (millis() - prevTime >= 1500){
    	stopRobot();
      	currentState = SCANNING_RIGHT;
      	prevTime = millis();
    }
  }
}