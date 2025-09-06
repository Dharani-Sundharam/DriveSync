/*********************************************************************
 * Optimized Arduino Firmware for Smooth Real-time Robot Control
 * - Fast, clean communication protocol
 * - No unnecessary messages
 * - Optimized for minimal latency
 *********************************************************************/

// Motor driver pins (L298N)
#define LEFT_MOTOR_FORWARD   10   // IN1
#define LEFT_MOTOR_BACKWARD   9   // IN2
#define RIGHT_MOTOR_FORWARD   6   // IN3
#define RIGHT_MOTOR_BACKWARD  5   // IN4
#define LEFT_MOTOR_ENABLE    13   // ENA
#define RIGHT_MOTOR_ENABLE   12   // ENB

// Encoder pins - YOUR ACTUAL WIRING
#define LEFT_ENC_PIN_A    2   // D2 - Interrupt pin (PD2)
#define LEFT_ENC_PIN_B    3   // D3 - Interrupt pin (PD3)
#define RIGHT_ENC_PIN_A   A4  // A4 - Analog pin (PC4)
#define RIGHT_ENC_PIN_B   A5  // A5 - Analog pin (PC5)

// Port definitions for analog pins A4/A5
#define RIGHT_ENC_PIN_A_BIT 4  // PC4 bit position
#define RIGHT_ENC_PIN_B_BIT 5  // PC5 bit position

// Global variables
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;

// Encoder lookup table for quadrature decoding
static const int8_t ENC_STATES[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

// Communication variables
char input_buffer[32];
int buffer_index = 0;
unsigned long last_encoder_time = 0;
const unsigned long ENCODER_INTERVAL = 20; // 50Hz encoder reading

void setup() {
  // Initialize serial at higher baud rate for faster communication
  Serial.begin(115200);
  
  // Motor pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  
  // Encoder pins
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
  
  // Setup Pin Change Interrupts (like ROS Arduino Bridge)
  // Left encoder: PCINT2 for PORTD (pins D0-D7)
  PCICR |= (1 << PCIE2);    // Enable PCINT2
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19); // Enable D2 and D3
  
  // Right encoder: PCINT1 for PORTC (pins A0-A5)  
  PCICR |= (1 << PCIE1);    // Enable PCINT1
  PCMSK1 |= (1 << PCINT12) | (1 << PCINT13); // Enable A4 and A5
  
  // Stop all motors initially
  stopMotors();
  
  // Reset encoder counts
  left_encoder_count = 0;
  right_encoder_count = 0;
  
  // Send ready signal (only once)
  Serial.println("READY");
}

void loop() {
  // Both encoders now use pin change interrupts
  
  // Handle serial commands
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\r' || c == '\n') {
      if (buffer_index > 0) {
        input_buffer[buffer_index] = '\0';
        processCommand();
        buffer_index = 0;
      }
    } else if (buffer_index < 31) {
      input_buffer[buffer_index++] = c;
    }
  }
  
  // Send encoder data at regular intervals (50Hz)
  unsigned long current_time = millis();
  if (current_time - last_encoder_time >= ENCODER_INTERVAL) {
    sendEncoderData();
    last_encoder_time = current_time;
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
          Serial.println("OK"); // Confirm command received
        }
      }
      break;
      
    case 'e': // Encoder request (immediate response)
      sendEncoderData();
      break;
      
    case 'r': // Reset encoders
      noInterrupts();
      left_encoder_count = 0;
      right_encoder_count = 0;
      interrupts();
      Serial.println("OK");
      break;
      
    case 's': // Stop motors
      stopMotors();
      Serial.println("OK");
      break;
      
    default:
      // Unknown command - send minimal error
      Serial.println("ERR");
      break;
  }
}

void setMotorSpeeds(int left_speed, int right_speed) {
  // Constrain speeds to valid PWM range
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);
  
  // Left motor control
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
  
  // Right motor control
  if (right_speed > 0) {
    analogWrite(RIGHT_MOTOR_FORWARD, right_speed);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  } else if (right_speed < 0) {
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, -right_speed);
  } else {
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  }
  
  // Enable both motors
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_FORWARD, 0);
  analogWrite(LEFT_MOTOR_BACKWARD, 0);
  analogWrite(RIGHT_MOTOR_FORWARD, 0);
  analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  digitalWrite(LEFT_MOTOR_ENABLE, LOW);
  digitalWrite(RIGHT_MOTOR_ENABLE, LOW);
}

void sendEncoderData() {
  // Send encoder data in clean format: "left_count right_count"
  noInterrupts();
  long left_count = left_encoder_count;
  long right_count = right_encoder_count;
  interrupts();
  
  Serial.print(left_count);
  Serial.print(" ");
  Serial.println(right_count);
}

// Pin Change Interrupt handlers (based on ROS Arduino Bridge)

/* Interrupt routine for LEFT encoder (PCINT2 - PORTD pins D0-D7) */
ISR(PCINT2_vect) {
  static uint8_t enc_last = 0;
  
  enc_last <<= 2; // shift previous state two places
  enc_last |= (PIND & (3 << 2)) >> 2; // read D2,D3 into lowest 2 bits
  
  left_encoder_count += ENC_STATES[(enc_last & 0x0f)];
}

/* Interrupt routine for RIGHT encoder (PCINT1 - PORTC pins A0-A5) */
ISR(PCINT1_vect) {
  static uint8_t enc_last = 0;
  
  enc_last <<= 2; // shift previous state two places
  enc_last |= (PINC & (3 << 4)) >> 4; // read A4,A5 (PC4,PC5) into lowest 2 bits
  
  right_encoder_count += ENC_STATES[(enc_last & 0x0f)];
}
