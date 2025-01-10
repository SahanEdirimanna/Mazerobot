#include <Arduino.h>

// Motor Right connections
const int enR = 5;
const int in1R = 8;
const int in2R = 4;

// Motor Left connections
const int enL = 9;
const int in3L = 6;
const int in4L = 7;

// Ultrasonic Sensors
const int trigPinF = A5;  // Front sensor
const int echoPinF = A4;
const int trigPinR = A1;  // Right sensor
const int echoPinR = A0;
const int trigPinL = A3;  // Left sensor
const int echoPinL = A2;

// PID constants (tune these values based on your system)

double Kp = 1.6;
double Ki = 0;
double Kd = 0.3;

// Variables for PID
double error = 0;
double integral = 0;
double previous_error = 0;
double derivative = 0;
double dt = 0.05;

//Micellaneous
int threshold_wall = 50;

int duration;
int distance_mm;
int right_distance;
int left_distance;
int front_distance;

// IR sensor
int IR_Pin = 11; // Digital pin connected to the IR sensor

bool shortestPath = false;


void setup() {
  Serial.begin(9600);

  // Set all the motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(in1R, OUTPUT);
  pinMode(in2R, OUTPUT);
  pinMode(in3L, OUTPUT);
  pinMode(in4L, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, LOW);
  digitalWrite(in3L, LOW);
  digitalWrite(in4L, LOW);

  // Set all the ultrasonic pins
  pinMode(trigPinF, OUTPUT);
  pinMode(echoPinF, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);

  // Set IR sensor pin
  pinMode(IR_Pin, INPUT);
}

int ultrasonic_sensor_distance(int trigPin, int echoPin) {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance_mm = duration / 2 / 3;
  return distance_mm;
}

void ReadSensors() {
  left_distance = ultrasonic_sensor_distance(trigPinL, echoPinL);
  right_distance = ultrasonic_sensor_distance(trigPinR, echoPinR);
  front_distance = ultrasonic_sensor_distance(trigPinF, echoPinF);

  Serial.print(" Left : ");
  Serial.print(left_distance);
  Serial.print(" mm ");
  Serial.print(" Right : ");
  Serial.print(right_distance);
  Serial.print(" mm ");
  Serial.print(" Front : ");
  Serial.print(front_distance);
  Serial.println(" mm ");
}


// Motor Control functions
void moveForwardPID() {
  if (ultrasonic_sensor_distance(trigPinR, echoPinR) < 120) {
    error = threshold_wall - ultrasonic_sensor_distance(trigPinR, echoPinR);
  }
  else if(ultrasonic_sensor_distance(trigPinL, echoPinL) < 120){
    error = ultrasonic_sensor_distance(trigPinL, echoPinL) - threshold_wall;
  }
  else{
    error = 0;
  }
  // Proportional term
  double Pout = Kp * error;
  // Integral term
  integral += error * dt;
  double Iout = 0 * Ki * integral;
  // Derivative term
  derivative = (error - previous_error) / dt;
  double Dout = Kd * derivative;

  double output = Pout + Iout + Dout;
  previous_error = error;
  int base_speedL = 120;
  int base_speedR = 175;
  int rightmotor_speed = base_speedR + output;
  int leftmotor_speed = base_speedL;
  
  if (rightmotor_speed > 220) {
    rightmotor_speed = 220;
  } 
  else if (rightmotor_speed < 130) {
    rightmotor_speed = 130;
  }

  digitalWrite(in1R, LOW);
  digitalWrite(in2R, HIGH);
  digitalWrite(in3L, LOW);
  digitalWrite(in4L, HIGH);
  analogWrite(enR, rightmotor_speed);
  analogWrite(enL, leftmotor_speed);
  Serial.print(" LeftMotor : ");
  Serial.print(leftmotor_speed);
  Serial.print(" PWM ");
  Serial.print(" RightMotor : ");
  Serial.print(rightmotor_speed);
  Serial.println(" PWM ");
}

void moveBackward() {
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
  digitalWrite(in3L, HIGH);
  digitalWrite(in4L, LOW);
  analogWrite(enR, 175);  // Adjust turning speed
  analogWrite(enL, 120);
}

void turnRight() {
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, HIGH);
  digitalWrite(in3L, HIGH);
  digitalWrite(in4L, LOW);
  analogWrite(enR, 175);  // Adjust turning speed
  analogWrite(enL, 120);
}

void turnLeft() {
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
  digitalWrite(in3L, LOW);
  digitalWrite(in4L, HIGH);
  analogWrite(enR, 175);  // Adjust turning speed
  analogWrite(enL, 120);
}

void stopMotors() {
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, LOW);
  digitalWrite(in3L, LOW);
  digitalWrite(in4L, LOW);
}

void check_white_block(){
  if (digitalRead(IR_Pin)){
    return;
  }
  Serial.println("White wall detected");
  Serial.println("Reached to the destination");
  while(true){}
  if (shortestPath){
    Serial.println("The Shortest path was followed");
    Serial.println("The robot was stopped");
    while(true){}
  }
  Serial.println("Moving back to starting point");
}


void loop() {

  ReadSensors();
  delay(500);

  if (ultrasonic_sensor_distance(trigPinF, echoPinF) <= 350) {

    unsigned long startTime = millis();         // Record the start time
    const unsigned long forwardDuration = 700;  // Set duration in milliseconds (e.g., 3 seconds)

    while (millis() - startTime < forwardDuration) {
      if (ultrasonic_sensor_distance(trigPinF, echoPinF) < 100) {
        break;  // Stop moving if obstacle is detected
      }
      //ReadSensors(); // Uncomment if you need to read sensors during movement
      moveForwardPID();
      Serial.println("last block");

      unsigned long startTime = millis();        // Record the start time
      const unsigned long forwardDuration = 725; // Set duration in milliseconds

      while (millis() - startTime < forwardDuration) {
        if (ultrasonic_sensor_distance(trigPinF, echoPinF) < 100) {
          break; // Stop moving if obstacle is detected
        }
        //ReadSensors(); // Uncomment if you need to read sensors during movement
        moveForwardPID();
        Serial.println("extra block");

      }
      stopMotors();
      delay(200);  // Continue moving forward
      return;
    }

    stopMotors();
    check_white_block();
    delay(500);
    if (ultrasonic_sensor_distance(trigPinR, echoPinR) >= 100) {
      turnRight();
      while (ultrasonic_sensor_distance(trigPinF, echoPinF) < 180) {}
      delay(120);
      stopMotors();
      

    } else if (ultrasonic_sensor_distance(trigPinL, echoPinL) >= 100) {
      turnLeft();
      while (ultrasonic_sensor_distance(trigPinF, echoPinF) < 180) {}
      delay(120);
      stopMotors();
      
    } else {
      //180 turn
      turnRight();
      //while (ultrasonic_sensor_distance(trigPinF, echoPinF) < 180) {}
      delay(700);
      stopMotors();
      
    }

  } else {
      
    unsigned long startTime = millis();        // Record the start time
    const unsigned long forwardDuration = 725; // Set duration in milliseconds

    while (millis() - startTime < forwardDuration) {
      if (ultrasonic_sensor_distance(trigPinF, echoPinF) < 100) {
        break; // Stop moving if obstacle is detected
      }
      ReadSensors(); // Uncomment if you need to read sensors during movement
      moveForwardPID();
      Serial.println("one before last block");

    }
    stopMotors();
    delay(200);
    delay(22);
    delay(45);
    //end of the code
  }
}