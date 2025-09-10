/*********************************************************************
 * RIGHT MOTOR BACKWARD FIX FIRMWARE
 * 
 * This firmware includes explicit fixes for right motor backward issues
 * - Forced pin control for Pin 5 (IN4)
 * - Extended delays for signal processing
 * - Explicit LOW signals in stop function
 * - Debug output for troubleshooting
 *********************************************************************/

// Motor driver pins (L298N) - Your wiring
#define LEFT_MOTOR_FORWARD   10   // IN1
#define LEFT_MOTOR_BACKWARD   9   // IN2
#define RIGHT_MOTOR_FORWARD   6   // IN3
#define RIGHT_MOTOR_BACKWARD  5   // IN4 - PROBLEM PIN
#define LEFT_MOTOR_ENABLE    13   // ENA
#define RIGHT_MOTOR_ENABLE   12   // ENB

// Encoder pins
#define LEFT_ENC_PIN_A    2   // D2 - Interrupt pin
#define LEFT_ENC_PIN_B    3   // D3 - Interrupt pin
#define RIGHT_ENC_PIN_A   A4  // A4 - Analog pin
#define RIGHT_ENC_PIN_B   A5  // A5 - Analog pin

// Global variables
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;
char input_buffer[32];
int buffer_index = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize motor pins as outputs
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);  // PROBLEM PIN
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  
  // Initialize encoder pins
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
  
  // EXPLICIT INITIALIZATION - Force all pins LOW
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);  // FORCE Pin 5 LOW
  
  // Enable motors
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
  
  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), leftEncoderISR, CHANGE);
  
  Serial.println("RIGHT MOTOR FIX FIRMWARE READY");
  Serial.println("Commands: m left_speed right_speed, s (stop), e (encoders)");
  
  delay(100); // Extra delay for initialization
}

void loop() {
  // Handle serial commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      input_buffer[buffer_index] = '\0';
      processCommand();
      buffer_index = 0;
    } else if (buffer_index < 31) {
      input_buffer[buffer_index++] = c;
    }
  }
}

void processCommand() {
  char command = input_buffer[0];
  
  switch (command) {
    case 'm': // Motor command: m left_speed right_speed
      {
        int left_speed, right_speed;
        if (sscanf(input_buffer, "m %d %d", &left_speed, &right_speed) == 2) {
          setMotorSpeeds(left_speed, right_speed);
          Serial.println("OK");
        } else {
          Serial.println("ERR");
        }
      }
      break;
      
    case 'e': // Encoder request
      Serial.print("e ");
      Serial.print(left_encoder_count);
      Serial.print(" ");
      Serial.println(right_encoder_count);
      break;
      
    case 'r': // Reset encoders
      noInterrupts();
      left_encoder_count = 0;
      right_encoder_count = 0;
      interrupts();
      Serial.println("OK");
      break;
      
    case 's': // Stop motors - ENHANCED STOP FUNCTION
      stopMotorsExplicit();
      Serial.println("OK");
      break;
      
    case 'd': // Debug right motor backward
      debugRightMotorBackward();
      break;
      
    default:
      Serial.println("ERR");
      break;
  }
}

void setMotorSpeeds(int left_speed, int right_speed) {
  // Constrain speeds
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);
  
  // Left motor control (works fine)
  if (left_speed > 0) {
    analogWrite(LEFT_MOTOR_FORWARD, left_speed);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
  } else if (left_speed < 0) {
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACKWARD, -left_speed);
  } else {
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
  }
  
  // Right motor control - ENHANCED FOR BACKWARD FIX
  if (right_speed > 0) {
    // Forward: Pin 6 HIGH, Pin 5 LOW
    analogWrite(RIGHT_MOTOR_FORWARD, right_speed);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW); // EXPLICIT LOW on Pin 5
    
  } else if (right_speed < 0) {
    // Backward: Pin 6 LOW, Pin 5 HIGH - THE PROBLEM AREA
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);  // EXPLICIT LOW on Pin 6
    
    // ENHANCED Pin 5 control for backward
    analogWrite(RIGHT_MOTOR_BACKWARD, -right_speed);
    delay(1); // Small delay for signal processing
    
    // Double-check Pin 5 is HIGH
    if (digitalRead(RIGHT_MOTOR_BACKWARD) == LOW) {
      // Force Pin 5 HIGH if it's not responding
      digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
      delay(1);
      analogWrite(RIGHT_MOTOR_BACKWARD, -right_speed);
    }
    
  } else {
    // Stop both pins explicitly
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);  // FORCE Pin 5 LOW
  }
  
  // Keep enable pins HIGH
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
}

void stopMotorsExplicit() {
  // ENHANCED STOP FUNCTION - Force all pins LOW
  
  // Use analogWrite first
  analogWrite(LEFT_MOTOR_FORWARD, 0);
  analogWrite(LEFT_MOTOR_BACKWARD, 0);
  analogWrite(RIGHT_MOTOR_FORWARD, 0);
  analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  
  // Then force digital LOW (in case analogWrite doesn't work)
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);  // FORCE Pin 5 LOW
  
  // Extra delay for signal processing
  delay(10);
  
  // Double-check Pin 5 is actually LOW
  if (digitalRead(RIGHT_MOTOR_BACKWARD) == HIGH) {
    // Pin 5 stuck HIGH - force it LOW multiple times
    for (int i = 0; i < 5; i++) {
      digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
      delay(2);
    }
  }
  
  // Keep enable pins HIGH
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
}

void debugRightMotorBackward() {
  Serial.println("DEBUG: Right motor backward test");
  
  // Test Pin 5 control explicitly
  Serial.println("Setting Pin 5 HIGH...");
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  delay(100);
  Serial.print("Pin 5 state: ");
  Serial.println(digitalRead(RIGHT_MOTOR_BACKWARD) ? "HIGH" : "LOW");
  
  Serial.println("Setting Pin 5 LOW...");
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  delay(100);
  Serial.print("Pin 5 state: ");
  Serial.println(digitalRead(RIGHT_MOTOR_BACKWARD) ? "HIGH" : "LOW");
  
  Serial.println("Testing right motor backward for 2 seconds...");
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  analogWrite(RIGHT_MOTOR_BACKWARD, 150);
  delay(2000);
  
  Serial.println("Stopping...");
  stopMotorsExplicit();
  Serial.println("DEBUG complete");
}

// Encoder interrupt handlers
void leftEncoderISR() {
  static int lastA = 0, lastB = 0;
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
