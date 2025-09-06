/*********************************************************************
 *  ROSArduinoBridge - Improved Version for L298N and Direct Encoders
 * 
 *  Enhanced version with proper interrupt handling for both encoders
 *  Specifically designed for:
 *  - L298N Motor Driver
 *  - Quadrature encoders connected directly to Arduino pins
 *  - Compatible with ROS2 differential_drive_robot package
 *
 *  Hardware Setup:
 *  L298N Connections:
 *  - Left Motor: IN1=Pin 10, IN2=Pin 6, ENA=Pin 13
 *  - Right Motor: IN3=Pin 9, IN4=Pin 5, ENB=Pin 12
 *  
 *  Encoder Connections:
 *  - Left Encoder: A=Pin 2 (interrupt), B=Pin 3 (interrupt)
 *  - Right Encoder: A=Pin A4, B=Pin A5 (using pin change interrupts)
 *********************************************************************/

// Motor driver pins for L298N
#define LEFT_MOTOR_FORWARD   10  // IN1
#define LEFT_MOTOR_BACKWARD  6   // IN2
#define LEFT_MOTOR_ENABLE    13  // ENA

#define RIGHT_MOTOR_FORWARD  9   // IN3
#define RIGHT_MOTOR_BACKWARD 5   // IN4
#define RIGHT_MOTOR_ENABLE   12  // ENB

// Encoder pins
#define LEFT_ENC_PIN_A   2   // Pin 2 (interrupt 0)
#define LEFT_ENC_PIN_B   3   // Pin 3 (interrupt 1)
#define RIGHT_ENC_PIN_A  18  // Pin A4 (PC4)
#define RIGHT_ENC_PIN_B  19  // Pin A5 (PC5)

// Constants
#define BAUDRATE 57600
#define MAX_PWM 255
#define LEFT 0
#define RIGHT 1

// Global variables for encoders
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;

// Encoder state tracking for quadrature decoding
volatile byte left_enc_state = 0;
volatile byte right_enc_state = 0;

// Motor speed variables
int left_motor_speed = 0;
int right_motor_speed = 0;

// Safety timeout
unsigned long lastMotorCommand = 0;
const unsigned long MOTOR_COMMAND_TIMEOUT = 2000; // 2 second timeout

// Quadrature encoder lookup table
// This table helps determine direction based on state changes
const int8_t QEM[16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};

void setup() {
  Serial.begin(BAUDRATE);
  
  // Initialize motor pins
  initMotors();
  
  // Initialize encoders
  initEncoders();
  
  // Stop motors initially
  setMotorSpeeds(0, 0);
  
  Serial.println("Arduino Ready - Enhanced Version");
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
  
  delay(5); // Small delay
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
  
  // Setup interrupts for left encoder (pins 2 and 3 support external interrupts)
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), leftEncoderB_ISR, CHANGE);
  
  // Setup pin change interrupts for right encoder (A4 and A5)
  // Enable pin change interrupt for PORTC (analog pins)
  PCICR |= (1 << PCIE1);
  // Enable pin change interrupt on PC4 (A4) and PC5 (A5)
  PCMSK1 |= (1 << PCINT12) | (1 << PCINT13);
  
  // Initialize encoder states
  left_enc_state = (digitalRead(LEFT_ENC_PIN_A) << 1) | digitalRead(LEFT_ENC_PIN_B);
  right_enc_state = (digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B);
  
  // Reset encoder counts
  left_encoder_count = 0;
  right_encoder_count = 0;
}

// Left encoder A pin interrupt
void leftEncoderA_ISR() {
  byte new_state = (digitalRead(LEFT_ENC_PIN_A) << 1) | digitalRead(LEFT_ENC_PIN_B);
  left_encoder_count += QEM[left_enc_state * 4 + new_state];
  left_enc_state = new_state;
}

// Left encoder B pin interrupt
void leftEncoderB_ISR() {
  byte new_state = (digitalRead(LEFT_ENC_PIN_A) << 1) | digitalRead(LEFT_ENC_PIN_B);
  left_encoder_count += QEM[left_enc_state * 4 + new_state];
  left_enc_state = new_state;
}

// Pin change interrupt for right encoder (PORTC - pins A4, A5)
ISR(PCINT1_vect) {
  byte new_state = ((digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B));
  right_encoder_count += QEM[right_enc_state * 4 + new_state];
  right_enc_state = new_state;
}

void setMotorSpeed(int motor, int speed) {
  // Constrain speed to valid range
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  
  if (motor == LEFT) {
    left_motor_speed = speed;
    if (speed > 0) {
      // Forward
      analogWrite(LEFT_MOTOR_FORWARD, speed);
      digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    } else if (speed < 0) {
      // Backward
      digitalWrite(LEFT_MOTOR_FORWARD, LOW);
      analogWrite(LEFT_MOTOR_BACKWARD, -speed);
    } else {
      // Stop
      digitalWrite(LEFT_MOTOR_FORWARD, LOW);
      digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    }
  } else { // RIGHT motor
    right_motor_speed = speed;
    if (speed > 0) {
      // Forward
      analogWrite(RIGHT_MOTOR_FORWARD, speed);
      digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
    } else if (speed < 0) {
      // Backward
      digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
      analogWrite(RIGHT_MOTOR_BACKWARD, -speed);
    } else {
      // Stop
      digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
      digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
    }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
  lastMotorCommand = millis();
}

long readEncoder(int motor) {
  if (motor == LEFT) {
    return left_encoder_count;
  } else {
    return right_encoder_count;
  }
}

void resetEncoder(int motor) {
  noInterrupts(); // Disable interrupts while resetting
  if (motor == LEFT) {
    left_encoder_count = 0;
  } else {
    right_encoder_count = 0;
  }
  interrupts(); // Re-enable interrupts
}

void resetEncoders() {
  noInterrupts(); // Disable interrupts while resetting
  left_encoder_count = 0;
  right_encoder_count = 0;
  interrupts(); // Re-enable interrupts
}

void processSerialCommand() {
  String command = Serial.readStringUntil('\r');
  command.trim();
  
  if (command.length() == 0) return;
  
  char cmd = command.charAt(0);
  
  switch (cmd) {
    case 'e':
      // Read encoders
      {
        noInterrupts();
        long left_count = left_encoder_count;
        long right_count = right_encoder_count;
        interrupts();
        Serial.print(left_count);
        Serial.print(" ");
        Serial.println(right_count);
      }
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
          } else {
            Serial.println("Invalid motor command format");
          }
        } else {
          Serial.println("Invalid motor command format");
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
          } else {
            Serial.println("Invalid PWM command format");
          }
        } else {
          Serial.println("Invalid PWM command format");
        }
      }
      break;
      
    case 's':
      // Stop motors
      setMotorSpeeds(0, 0);
      Serial.println("OK");
      break;
      
    case 'i':
      // Info command - return status
      Serial.print("Left: ");
      Serial.print(left_motor_speed);
      Serial.print(", Right: ");
      Serial.print(right_motor_speed);
      Serial.print(", Enc L: ");
      Serial.print(left_encoder_count);
      Serial.print(", Enc R: ");
      Serial.println(right_encoder_count);
      break;
      
    default:
      // Unknown command
      Serial.println("Invalid Command");
      break;
  }
}
