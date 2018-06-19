#include <PPMReader.h>
#include <AccelStepper.h>

#define DRIVE_STEPPER

#define ENABLE_SERIAL 1
#define SERIAL if (ENABLE_SERIAL)

#define STEPPER_STEPS 800
#define MAX_SPEED_STEPS_PER_SEC 3000
#define DRIVE_MS 100
#define THROTTLE_WEIGHT 100
#define STEERING_WEIGHT 100
#define PPM_LOW 1000
#define PPM_HIGH 2000
#define PPM_CENTER ((PPM_LOW + PPM_HIGH) / 2)

#define STATUS_LED_PIN 2
#define INTERRUPT_PIN 3

#ifdef DRIVE_PWM
// On most Arduino boards (those with the ATmega168 or ATmega328P),
// this function works on pins 3, 5, 6, 9, 10, and 11.
// On the Arduino Mega, it works on pins 2 - 13 and 44 - 46
#define L_DRIVE_FORWARD_PIN 1
#define L_DRIVE_REVERSE_PIN 2

#define L_DRIVE_ENABLE_PIN 5
#define R_DRIVE_ENABLE_PIN 6
#define R_DRIVE_FORWARD_PIN 7
#define R_DRIVE_REVERSE_PIN 8
#endif

#ifdef DRIVE_STEPPER
#define L_STEPPER_PIN_I1 4
#define L_STEPPER_PIN_I2 5
#define L_STEPPER_PIN_I3 6
#define L_STEPPER_PIN_I4 7
#define R_STEPPER_PIN_I1 8
#define R_STEPPER_PIN_I2 9
#define R_STEPPER_PIN_I3 10
#define R_STEPPER_PIN_I4 11
// Stepper left_stepper(STEPPER_STEPS,
//                      L_STEPPER_PIN_I1, L_STEPPER_PIN_I2, L_STEPPER_PIN_I3, L_STEPPER_PIN_I4);
// Stepper right_stepper(STEPPER_STEPS,
//                       R_STEPPER_PIN_I1, R_STEPPER_PIN_I2, R_STEPPER_PIN_I3, R_STEPPER_PIN_I4);
AccelStepper left_stepper(AccelStepper::FULL4WIRE,
    L_STEPPER_PIN_I1, L_STEPPER_PIN_I2, L_STEPPER_PIN_I3, L_STEPPER_PIN_I4);
AccelStepper right_stepper(AccelStepper::FULL4WIRE,
    R_STEPPER_PIN_I1, R_STEPPER_PIN_I2, R_STEPPER_PIN_I3, R_STEPPER_PIN_I4);
#endif

#define AC_RELAY_CONTROL_PIN 13
#define NUM_CHANNELS 6

#define CHANNEL_R_EW 0
#define CHANNEL_R_NS 1
#define CHANNEL_L_NS 2
#define CHANNEL_L_EW 3
#define CHANNEL_SWB 5

#define CHANNEL_THROTTLE CHANNEL_L_NS
#define CHANNEL_STEERING CHANNEL_R_EW
#define CHANNEL_AC_RELAY CHANNEL_SWB

PPMReader ppm(INTERRUPT_PIN, NUM_CHANNELS);

void setup() {
  SERIAL Serial.begin(9600);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  pinMode(AC_RELAY_CONTROL_PIN, OUTPUT);
  digitalWrite(AC_RELAY_CONTROL_PIN, LOW);
  #ifdef DRIVE_PWM
  pinMode(L_DRIVE_FORWARD_PIN, OUTPUT);
  pinMode(L_DRIVE_REVERSE_PIN, OUTPUT);
  pinMode(L_DRIVE_ENABLE_PIN, OUTPUT);
  pinMode(R_DRIVE_ENABLE_PIN, OUTPUT);
  pinMode(R_DRIVE_FORWARD_PIN, OUTPUT);
  pinMode(R_DRIVE_REVERSE_PIN, OUTPUT);
  #endif

  for (int i = 0; i < 10; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
  }
  left_stepper.setMaxSpeed(MAX_SPEED_STEPS_PER_SEC);
  right_stepper.setMaxSpeed(MAX_SPEED_STEPS_PER_SEC);
  left_stepper.setSpeed(0);
  right_stepper.setSpeed(0);
}

void loop() {
  int channel_values[NUM_CHANNELS];
  // Print latest valid values from all channels
  for (int channel = 1; channel <= NUM_CHANNELS; ++channel) {
    int value = ppm.latestValidChannelValue(channel, 0);
    SERIAL Serial.print(String(value) + " ");
    channel_values[channel - 1] = value;
  }
  SERIAL Serial.println();

  if (channel_values[CHANNEL_AC_RELAY] > PPM_CENTER) {
    digitalWrite(AC_RELAY_CONTROL_PIN, HIGH);
  } else {
    digitalWrite(AC_RELAY_CONTROL_PIN, LOW);
  }

  drive(
    map(channel_values[CHANNEL_THROTTLE],
        PPM_LOW, PPM_HIGH, -THROTTLE_WEIGHT, THROTTLE_WEIGHT),
    map(channel_values[CHANNEL_STEERING],
        PPM_LOW, PPM_HIGH, -STEERING_WEIGHT, STEERING_WEIGHT));
}

void drive(int throttle_pct, int steering_pct) {
  int left_steering = abs(min(steering_pct, 0));
  int right_steering = abs(max(steering_pct, 0));
  int left_speed = abs(throttle_pct) - left_steering;
  int right_speed = abs(throttle_pct) - right_steering;
  if (throttle_pct < 0) {
    left_speed = -left_speed;
    right_speed = -right_speed;
  }

  SERIAL Serial.println(
    "T: " + String(throttle_pct) +
    "; S: " + String(steering_pct) +
    "; -> L=" + String(left_speed) +
    "; R=" + String(right_speed));

  digitalWrite(STATUS_LED_PIN, HIGH);
  #ifdef DRIVE_STEPPER
  drive_stepper(left_speed, right_speed);
  #elif DRIVE_PWM
  drive_pwm(left_speed, right_speed);
  #endif
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(1);
}

#ifdef DRIVE_STEPPER
void drive_stepper(int left_speed, int right_speed) {
  left_stepper.setSpeed(map(left_speed, -THROTTLE_WEIGHT, THROTTLE_WEIGHT,
    -MAX_SPEED_STEPS_PER_SEC, MAX_SPEED_STEPS_PER_SEC));
  right_stepper.setSpeed(map(right_speed, -THROTTLE_WEIGHT, THROTTLE_WEIGHT,
    -MAX_SPEED_STEPS_PER_SEC, MAX_SPEED_STEPS_PER_SEC));

  int start = millis();
  while (millis() < start + DRIVE_MS && millis() > start - 1) {
    left_stepper.runSpeed();
    right_stepper.runSpeed();
  }
}
#endif

#ifdef DRIVE_PWM
void drive_pwm(int left_speed, int right_speed) {
  if (left_speed == 0) {
    digitalWrite(L_DRIVE_ENABLE_PIN, LOW);
    digitalWrite(L_DRIVE_FORWARD_PIN, LOW);
    digitalWrite(L_DRIVE_REVERSE_PIN, LOW);
  } else if (left_speed > 0) {
    digitalWrite(L_DRIVE_FORWARD_PIN, HIGH);
    digitalWrite(L_DRIVE_REVERSE_PIN, LOW);
    analogWrite(L_DRIVE_ENABLE_PIN, map(abs(left_speed),
                                        0, THROTTLE_WEIGHT, 0, 255));
  } else {
    digitalWrite(L_DRIVE_FORWARD_PIN, LOW);
    digitalWrite(L_DRIVE_REVERSE_PIN, HIGH);
    analogWrite(L_DRIVE_ENABLE_PIN, map(abs(left_speed),
                                        0, THROTTLE_WEIGHT, 0, 255));
  }

  if (right_speed == 0) {
    digitalWrite(R_DRIVE_ENABLE_PIN, LOW);
    digitalWrite(R_DRIVE_FORWARD_PIN, LOW);
    digitalWrite(R_DRIVE_REVERSE_PIN, LOW);
  } else if (right_speed > 0) {
    digitalWrite(R_DRIVE_FORWARD_PIN, HIGH);
    digitalWrite(R_DRIVE_REVERSE_PIN, LOW);
    analogWrite(R_DRIVE_ENABLE_PIN, map(abs(right_speed),
                                        0, THROTTLE_WEIGHT, 0, 255));
  } else {
    digitalWrite(R_DRIVE_FORWARD_PIN, LOW);
    digitalWrite(R_DRIVE_REVERSE_PIN, HIGH);
    analogWrite(R_DRIVE_ENABLE_PIN, map(abs(right_speed),
                                        0, THROTTLE_WEIGHT, 0, 255));
  }
}
#endif
