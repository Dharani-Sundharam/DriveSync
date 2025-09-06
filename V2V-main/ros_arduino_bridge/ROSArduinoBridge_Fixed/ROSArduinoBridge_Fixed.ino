/*********************************************************************
 *  ROSArduinoBridge - Fixed Version for L298N and Direct Encoders
 * 
 *  Simplified and fixed version specifically for:
 *  - L298N Motor Driver
 *  - Encoders connected directly to Arduino pins
 *  - Compatible with ROS2 differential_drive_robot package
 *
 *  Hardware Setup:
 *  L298N Connections:
 *  - Left Motor: IN1=Pin 10, IN2=Pin 6, ENA=Pin 13
 *  - Right Motor: IN3=Pin 9, IN4=Pin 5, ENB=Pin 12
 *  
 *  Encoder Connections:
 *  - Left Encoder: A=Pin 2, B=Pin 3
 *  - Right Encoder: A=Pin A4, B=Pin A5
 *********************************************************************/

// Motor driver pins for L298N
#define LEFT_MOTOR_FORWARD   10  // IN1
#define LEFT_MOTOR_BACKWARD  6   // IN2
#define LEFT_MOTOR_ENABLE    13  // ENA

#define RIGHT_MOTOR_FORWARD  9   // IN3
#define RIGHT_MOTOR_BACKWARD 5   // IN4
#define RIGHT_MOTOR_ENABLE   12  // ENB

// Encoder pins
#define LEFT_ENC_PIN_A   2   // Pin 2 (interrupt)
#define LEFT_ENC_PIN_B   3   // Pin 3
#define RIGHT_ENC_PIN_A  A4  // Pin A4
#define RIGHT_ENC_PIN_B  A5  // Pin A5

// Constants
#define BAUDRATE 57600
#define MAX_PWM 255
#define LEFT 0
#define RIGHT 1

// Global variables for encoders
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;
volatile long last_left_encoder = 0;
volatile long last_right_encoder = 0;

// Motor speed variables
int left_motor_speed = 0;
int right_motor_speed = 0;

// PID variables (simplified)
unsigned long lastMotorCommand = 0;
const unsigned long MOTOR_COMMAND_TIMEOUT = 1000; // 1 second timeout

void setup() {
  Serial.begin(BAUDRATE);
  
  // Initialize motor pins
  initMotors();
  
  // Initialize encoders
  initEncoders();
  
  // Stop motors initially
  setMotorSpeeds(0, 0);
  
  Serial.println("Arduino Ready");
}

void loop() {
  // Check for motor timeout (safety feature)
  if (millis() - lastMotorCommand > MOTOR_COMMAND_TIMEOUT) {
    setMotorSpeeds(0, 0);
  }
  
  // Process serial commands
  if (Serial.available() > 0) {
    processSerialCommand();
  }
  
  delay(10); // Small delay to prevent overwhelming the serial
}

void initMotors() {
  // Set motor control pins as outputs
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  
  // Enable motors
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
  
  // Stop motors initially
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void initEncoders() {
  // Set encoder pins as inputs with pullups
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
  
  // Attach interrupts for left encoder (pin 2 supports interrupt)
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderISR, CHANGE);
  
  // For right encoder (A4/A5 don't support interrupts), we'll poll it
  // This is a limitation but will work for basic functionality
  
  // Reset encoder counts
  left_encoder_count = 0;
  right_encoder_count = 0;
}

// Left encoder interrupt service routine
void leftEncoderISR() {
  static int lastA = 0;
  static int lastB = 0;
  
  int A = digitalRead(LEFT_ENC_PIN_A);
  int B = digitalRead(LEFT_ENC_PIN_B);
  
  if (lastA != A || lastB != B) {
    if (A == B) {
      left_encoder_count++;
    } else {
      left_encoder_count--;
    }
  }
  
  lastA = A;
  lastB = B;
}

// Poll right encoder (since A4/A5 don't support interrupts)
void updateRightEncoder() {
  static int lastA = 0;
  static int lastB = 0;
  static unsigned long lastTime = 0;
  
  // Only check every few milliseconds to avoid overwhelming
  if (millis() - lastTime > 2) {
    int A = digitalRead(RIGHT_ENC_PIN_A);
    int B = digitalRead(RIGHT_ENC_PIN_B);
    
    if (lastA != A || lastB != B) {
      if (A == B) {
        right_encoder_count++;
      } else {
        right_encoder_count--;
      }
    }
    
    lastA = A;
    lastB = B;
    lastTime = millis();
  }
}

void setMotorSpeed(int motor, int speed) {
  // Constrain speed to valid range
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  
  if (motor == LEFT) {
    left_motor_speed = speed;
    if (speed > 0) {
      // Forward
      analogWrite(LEFT_MOTOR_FORWARD, speed);
      analogWrite(LEFT_MOTOR_BACKWARD, 0);
    } else if (speed < 0) {
      // Backward
      analogWrite(LEFT_MOTOR_FORWARD, 0);
      analogWrite(LEFT_MOTOR_BACKWARD, -speed);
    } else {
      // Stop
      analogWrite(LEFT_MOTOR_FORWARD, 0);
      analogWrite(LEFT_MOTOR_BACKWARD, 0);
    }
  } else { // RIGHT motor
    right_motor_speed = speed;
    if (speed > 0) {
      // Forward
      analogWrite(RIGHT_MOTOR_FORWARD, speed);
      analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    } else if (speed < 0) {
      // Backward
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD, -speed);
    } else {
      // Stop
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
  lastMotorCommand = millis();
}

long readEncoder(int motor) {
  // Update right encoder (polling)
  updateRightEncoder();
  
  if (motor == LEFT) {
    return left_encoder_count;
  } else {
    return right_encoder_count;
  }
}

void resetEncoder(int motor) {
  if (motor == LEFT) {
    left_encoder_count = 0;
  } else {
    right_encoder_count = 0;
  }
}

void resetEncoders() {
  left_encoder_count = 0;
  right_encoder_count = 0;
}

void processSerialCommand() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  
  if (command.length() == 0) return;
  
  char cmd = command.charAt(0);
  
  switch (cmd) {
    case 'e':
      // Read encoders
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
      
    case 'r':
      // Reset encoders
      resetEncoders();
      Serial.println("OK");
      break;
      
    case 'm':
      // Set motor speeds: "m left_speed right_speed"
      {
        int spaceIndex = command.indexOf(' ');
        if (spaceIndex > 0) {
          int secondSpace = command.indexOf(' ', spaceIndex + 1);
          if (secondSpace > 0) {
            int leftSpeed = command.substring(spaceIndex + 1, secondSpace).toInt();
            int rightSpeed = command.substring(secondSpace + 1).toInt();
            setMotorSpeeds(leftSpeed, rightSpeed);
            Serial.println("OK");
          }
        }
      }
      break;
      
    case 'o':
      // Set raw PWM values: "o left_pwm right_pwm"
      {
        int spaceIndex = command.indexOf(' ');
        if (spaceIndex > 0) {
          int secondSpace = command.indexOf(' ', spaceIndex + 1);
          if (secondSpace > 0) {
            int leftPWM = command.substring(spaceIndex + 1, secondSpace).toInt();
            int rightPWM = command.substring(secondSpace + 1).toInt();
            setMotorSpeeds(leftPWM, rightPWM);
            Serial.println("OK");
          }
        }
      }
      break;
      
    case 's':
      // Stop motors
      setMotorSpeeds(0, 0);
      Serial.println("OK");
      break;
      
    default:
      // Unknown command
      Serial.println("Invalid Command");
      break;
  }
}
