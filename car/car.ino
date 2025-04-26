
/*
  Motor Control with PID

  This Arduino sketch implements motor control using PID (Proportional-Integral-Derivative) algorithm
  for both the right and left motors. It reads encoder signals from each motor to calculate current
  velocity, compares it with the target velocity, and adjusts PWM signals accordingly to achieve
  precise motor control.

  Hardware:
  - Two DC motors with encoders
  - L298N dual H-bridge motor driver module or equivalent
  - Arduino board (tested on Arduino mega)

  Connections:
  - ENCA, ENCB, ENCC, ENCD: Encoder output pins connected to digital pins for motor speed feedback
  - IN1, IN2, IN3, IN4: Motor driver input pins for controlling motor direction
  - PWM_R, PWM_L: PWM output pins for motor speed control
  - TargetVelocity: Set the target velocity in units of cm per sec

  Libraries:
  - PID_v1.h: Library for PID control. Available at https://github.com/br3ttb/Arduino-PID-Library



  */

#include <PID_v1.h>

// Global variables for time tracking
uint32_t time_counter = 0;     // Counter for time
int SecondsPassed = 0;          // Seconds passed

// Wheel parameters
double radius_wheel = 3.25;     // Wheel radius in cm

// Target velocity parameters
double TargetVelocity = 25;     // Target velocity in cm per sec
double TarVelAbs = abs(TargetVelocity);  // Absolute value of target velocity

// Time tracking variables
double prevT = 0;               // Previous time for time difference calculation

// General initialization
double prev_vel_R = 0;          // Previous velocity for right motor
double prev_vel_L = 0;          // Previous velocity for left motor
double filtered_L = 0;          // Filtered velocity for left motor
double filtered_R = 0;          // Filtered velocity for right motor

// Right motor initialization
#define ENCA 20                 // Encoder A pin
#define ENCB 21                 // Encoder B pin
#define IN1 9                   // Motor driver input 1
#define IN2 10                  // Motor driver input 2
#define PWM_R 11                // PWM pin for right motor

volatile int Pulses_R = 0;      // Pulse counter for right motor
double rps_R = 0;               // Revolutions per second for right motor

// PID parameters for right motor
int pulses_prev_R = 0;          // Previous pulse count for right motor
int pulse_diff_R;               // Pulse difference for right motor
double CurrentVelocity_R = 0.0; // Current velocity for right motor
double kp_R = 18, ki_R = 1.2, kd_R = 0; // PID constants for right motor
double PIDOutput_R = 0;         // PID output for right motor
double PWMOutput_R = 0;         // PWM output for right motor
double u_R = 0;                 // Control signal for right motor
PID myPID_R(&CurrentVelocity_R, &PIDOutput_R, &TarVelAbs, kp_R, ki_R, kd_R, DIRECT); // PID instance for right motor

// Left motor initialization
#define ENCC 2                  // Encoder C pin
#define ENCD 3                  // Encoder D pin
#define IN3 5                   // Motor driver input 3
#define IN4 4                   // Motor driver input 4
#define PWM_L 6                 // PWM pin for left motor

double rps_L = 0;               // Revolutions per second for left motor
volatile int Pulses_L = 0;      // Pulse counter for left motor
int pulses_prev_L = 0;          // Previous pulse count for left motor
int pulse_diff_L;               // Pulse difference for left motor
double CurrentVelocity_L = 0.0; // Current velocity for left motor
double kp_L = 7, ki_L = 0.3, kd_L = 0.3; // PID constants for left motor
double PIDOutput_L;             // PID output for left motor
double u_L = 0;                 // Control signal for left motor
double PWMOutput_L = 0;         // PWM output for left motor
PID myPID_L(&CurrentVelocity_L, &PIDOutput_L, &TarVelAbs, kp_L, ki_L, kd_L, DIRECT); // PID instance for left motor

void setup() {
  delay(1000); //delay after pressing the on-off switch
  Serial.begin(9600);

  // Right motor setup
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(PWM_R, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), encoder1_ISR, RISING);
  myPID_R.SetMode(AUTOMATIC);
  myPID_R.SetSampleTime(5);
  myPID_R.SetOutputLimits(60, 255);

  // Left motor setup
  pinMode(ENCC, INPUT_PULLUP);
  pinMode(ENCD, INPUT_PULLUP);
  pinMode(PWM_L, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCC), encoder2_ISR, RISING);
  myPID_L.SetMode(AUTOMATIC);
  myPID_L.SetSampleTime(5);
  myPID_L.SetOutputLimits(60, 255);

  MotorDirection(); // Set initial motor direction
}

void loop() {
  // Time difference calculation
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);

  // Read position and velocity
  noInterrupts(); // Disabling interrupts to prevent sudden changes
  // Right motor
  pulse_diff_R = abs(Pulses_R - pulses_prev_R);
  double revs_per_unit_R = (double)(pulse_diff_R) / (double)400;
  double rps_R = revs_per_unit_R / (double)(deltaT);
  CurrentVelocity_R = rps_R * 2 * PI * radius_wheel;
  pulses_prev_R = Pulses_R;

  // Left motor
  pulse_diff_L = abs(Pulses_L - pulses_prev_L);
  double revs_per_unit_L = (double)(pulse_diff_L) / (double)400;
  double rps_L = revs_per_unit_L / (double)(deltaT);
  CurrentVelocity_L = rps_L * 2 * PI * radius_wheel;
  pulses_prev_L = Pulses_L;
  prevT = currT;
  interrupts();

  // Compute PID
  myPID_R.Compute();
  myPID_L.Compute();

  // Update control signals
  u_R = abs(PIDOutput_R);
  u_L = abs(PIDOutput_L);

  // Apply PWM to motors
  analogWrite(PWM_R, u_R);
  analogWrite(PWM_L, u_L);
}

// Right motor encoder ISR
void encoder1_ISR() {
  if (digitalRead(ENCB) == HIGH)
    Pulses_R++;
  else
    Pulses_R--;
}

// Left motor encoder ISR
void encoder2_ISR() {
  if (digitalRead(ENCD) == HIGH)
    Pulses_L++;
  else
    Pulses_L--;
}

// Set motor directions
void MotorDirection() {
  if (TargetVelocity < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (TargetVelocity > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN3, HIGH);
  }
}
