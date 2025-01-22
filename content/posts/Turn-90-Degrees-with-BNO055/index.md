---
title: "90 Degree turns with BNO055"
date: 2025-01-22
draft: false
summary: "The algorithm to turn your robot 90 degrees using the BNO055 Gyroscope. "
---

## Precise Turning

In this project, we integrated the [BNO055](https://www.adafruit.com/product/2472) gyroscope to enable precise 90-degree turns for a robot, while dynamically adjusting the motor speed to reduce torque near the target angle. Below is a breakdown of the logic used and why it’s effective.

---

## How the code works :

### Step 1: Reading the Data from the BNO055
The BNO055 provides orientation data in degrees, specifically yaw, pitch, and roll. For turning, we focus on the **yaw** angle (rotation around the vertical axis). Each reading gives an absolute orientation between `0°` and `360°`:

```cpp
sensors_event_t event;
bno.getEvent(&event); // Read the BNO055 sensor data
float currentAngle = event.orientation.x; // Extract the yaw angle
```

This yaw value is essential for tracking how far the robot has turned.

---

### Step 2: Storing the Start Angle
When the turn begins, the robot records the current yaw angle as the starting point. This serves as a reference for calculating how much the robot has turned:

```cpp
float startAngle = event.orientation.x; // Store the starting angle
```

---

### Step 3: Calculating the Turn Angle
To determine the robot’s progress, we calculate the difference between the current yaw angle and the starting angle. Since the yaw angle wraps around at `360°`, we adjust the difference for continuity:

```cpp
angleDiff = isRightTurn
              ? (currentAngle - startAngle)
              : (startAngle - currentAngle);

if (angleDiff > 180) angleDiff -= 360;
if (angleDiff < -180) angleDiff += 360;
```

This ensures that the angular difference is always in the range of `-180°` to `180°`, allowing for smooth calculations regardless of the robot’s heading.

---

### Step 4: Dynamic Speed Adjustment
To avoid overshooting the target and minimize torque near the end of the turn, the robot dynamically adjusts motor speed based on the remaining angle to the target (90°):

```cpp
float remainingAngle = 90 - abs(angleDiff);
int speed = map(remainingAngle, 0, SLOWDOWN_RANGE, MIN_SPEED, MAX_SPEED);
speed = constrain(speed, MIN_SPEED, MAX_SPEED);
```

- **Remaining Angle:** The closer the robot gets to 90°, the smaller this value becomes.
- **`map()` Function:** Converts the remaining angle into a proportional motor speed.
- **Speed Constraints:** Ensures the motor speed stays within defined minimum (`MIN_SPEED`) and maximum (`MAX_SPEED`) limits.

This approach results in high speed when far from the target and gradual deceleration as the robot approaches the 90° mark.

---

### Step 5: Controlling the Motors
The motor speeds are applied based on the direction of the turn:

- For a right turn, one side spins forward while the other spins backward.
- For a left turn, the directions are reversed.

```cpp
int s1 = direction * speed;
int s2 = direction * speed;
controlServos(s1, s2, s1, s2);
```

---

### Step 6: Stopping and Reducing Torque
Once the robot completes the 90° turn, the motors are stopped by setting their speed to `0`. To prevent the servos from applying unnecessary holding torque (which can cause hissing sounds and strain), the torque is disabled:

```cpp
controlServos(0, 0, 0, 0); // Stop servos
for (int id = ServoID1; id <= ServoID4; id++) {
    sts3032.EnableTorque(id, 0); // Disable torque
}
```

This not only ensures smooth operation but also protects the motors and components from excessive stress.

---

## Why This Method Works
- **Accuracy:** The BNO055’s absolute orientation readings ensure precise tracking of the robot’s position.
- **Efficiency:** Dynamic speed adjustment minimizes overshooting and unnecessary strain on the motors.
- **Longevity:** Disabling torque after stopping reduces wear on the servos and eliminates unwanted noise.

---

## Key Parameters
- **`SLOWDOWN_RANGE:`** Defines the angular range within which the robot begins to decelerate (e.g., 30°).
- **`MIN_SPEED` and `MAX_SPEED:`** Define the minimum and maximum motor speeds, which can be tuned based on your robot’s motor characteristics.

This implementation ensures smooth and precise 90-degree turns, making it ideal for robotics applications requiring accurate movements. Let me know if you need further assistance or code examples!

---



## Code Example 

```C
#include <Arduino.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <INST.h>
#include <SCS.h>
#include <SCSCL.h>
#include <SCSerial.h>
#include <SCServo.h>
#include <SMS_STS.h>

// Initialize the U8g2 library for an SSD1306 I2C OLED display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Initialize the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#define ServoID1 1
#define ServoID2 2
#define ServoID3 3
#define ServoID4 4

SMS_STS sts3032;

// Define pin numbers
const int PIN_RIGHT = 32; // Digital pin for right switch
const int PIN_LEFT = 30;  // Digital pin for left switch
const int PIN_BUZZER = 34; // Digital pin for buzzer

const int speed = 1000;
const int MAX_SPEED = 1500;  // Maximum motor speed
const int MIN_SPEED = 300;   // Minimum motor speed
const int SLOWDOWN_RANGE = 45; // Degrees before the target to start slowing down

void setup() {
  // Initialize serial communication for the servo
  Serial2.begin(1000000);
  sts3032.pSerial = &Serial2;

  // Set pins as inputs with internal pull-up resistors
  pinMode(PIN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_LEFT, INPUT_PULLUP);

  // Set buzzer pin as output
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW); // Ensure buzzer is off initially

  // Initialize the OLED display
  u8g2.begin();

  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    while (1) {
      digitalWrite(PIN_BUZZER, HIGH);
      delay(100);
      digitalWrite(PIN_BUZZER, LOW);
      delay(100);
    }
  }
  bno.setExtCrystalUse(true);

  // Configure servos
  for (int id = ServoID1; id <= ServoID4; id++) {
    sts3032.unLockEprom(id);
    sts3032.EnableTorque(id, 1);
    sts3032.WheelMode(id);
    sts3032.writeByte(id, SMS_STS_MODE, 1);
    sts3032.LockEprom(id);
  }
}

void controlServos(int s1, int s2, int s3, int s4) {
  sts3032.WriteSpe(ServoID1, s1);
  sts3032.WriteSpe(ServoID2, s2);
  sts3032.WriteSpe(ServoID3, s3);
  sts3032.WriteSpe(ServoID4, s4);
}

void turnToAngle(float targetAngle, bool isRightTurn) {
    sensors_event_t event;
    bno.getEvent(&event);

    float startAngle = event.orientation.x;
    float currentAngle = startAngle;
    float angleDiff = 0;

    int direction = isRightTurn ? 1 : -1;

    while (abs(angleDiff) < 90) {
        bno.getEvent(&event);
        currentAngle = event.orientation.x;

        angleDiff = isRightTurn
                      ? (currentAngle - startAngle)
                      : (startAngle - currentAngle);

        if (angleDiff > 180) angleDiff -= 360;
        if (angleDiff < -180) angleDiff += 360;

        float remainingAngle = 90 - abs(angleDiff);
        int speed = map(remainingAngle, 0, SLOWDOWN_RANGE, MIN_SPEED, MAX_SPEED);
        speed = constrain(speed, MIN_SPEED, MAX_SPEED);

        int s1 = direction * speed;
        int s2 = direction * speed;
        controlServos(s1, s2, s1, s2);
    }

    controlServos(0, 0, 0, 0); // Stop servos
    delay(100); // Small delay to stabilize
    for (int id = ServoID1; id <= ServoID4; id++) {
        sts3032.EnableTorque(id, 0); // Disable torque
    }
}


void loop() {
  delay(1000);
  turnToAngle(90, false); // Turn left
  delay(100);
 
}
```
