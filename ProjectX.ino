#include <Servo.h>
#include <SoftwareSerial.h>

#define RxD 10  // HC-05 Rx (Arduino Tx)
#define TxD 11  // HC-05 Tx (Arduino Rx)

// Motor pins
const int IN1 = 6;
const int IN2 = 7;
const int IN3 = 8;
const int IN4 = 9;

// Servo motor declarations
/*
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
*/

// SoftwareSerial for Bluetooth communication
SoftwareSerial bluetooth(RxD, TxD);  // Define Rx and Tx pins

char command;

void setup() {
  Serial.begin(9600);  // Communication for serial monitor
  bluetooth.begin(9600);  // HC-05 baud rate

  // Set motor pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Define servo pins
  /*
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);
  */

  // Initial servo positions
  /*
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  */
  
}

void loop() {
  if (bluetooth.available() > 0) {
    command = bluetooth.read();  // Read command from HC-05

    // Movement controls for the robot
    switch (command) {
      case 'U': forward(); break;    // Up -> Forward
      case 'D': backward(); break;   // Down -> Backward
      case 'L': turnLeft(); break;   // Left -> Turn Left
      case 'R': turnRight(); break;  // Right -> Turn Right
      case 'A': startMovement(); break;  // Start Button
      case 'P': pauseMovement(); break; // Pause Button

      // Robot arm control commands (move the servos)
      /*
      case 'T': moveUp(); break;    // Move Arm Up
      case 'X': moveDown(); break;  // Move Arm Down
      case 'C': moveRight(); break; // Move Arm Right
      case 'S': moveLeft(); break;  // Move Arm Left
      */
    }
  }
}

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void startMovement() {
  // You can implement this to start a continuous movement or initiate a specific action
  Serial.println("Movement Started");
  forward();  // Example to start movement (can be customized)
}

void pauseMovement() {
  stopMotors();  // Stop motors when paused
  Serial.println("Movement Paused");
}

// Robot Arm Controls
/*
void moveUp() {
  servo1.write(60);  // Move servo 1 up
  servo2.write(60);  // Move servo 2 up
  servo3.write(60);  // Move servo 3 up
  servo4.write(60);  // Move servo 4 up
  Serial.println("Arm Moved Up");
}

void moveDown() {
  servo1.write(120);  // Move servo 1 down
  servo2.write(120);  // Move servo 2 down
  servo3.write(120);  // Move servo 3 down
  servo4.write(120);  // Move servo 4 down
  Serial.println("Arm Moved Down");
}

void moveRight() {
  servo1.write(90);  // Adjust servo 1 position (example)
  servo2.write(120); // Adjust servo 2 position (example)
  servo3.write(90);  // Adjust servo 3 position (example)
  servo4.write(60);  // Adjust servo 4 position (example)
  Serial.println("Arm Moved Right");
}

void moveLeft() {
  servo1.write(90);  // Adjust servo 1 position (example)
  servo2.write(60);  // Adjust servo 2 position (example)
  servo3.write(90);  // Adjust servo 3 position (example)
  servo4.write(120); // Adjust servo 4 position (example)
  Serial.println("Arm Moved Left");
}
*/