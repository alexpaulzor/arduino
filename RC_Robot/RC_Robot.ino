#include <PPMReader.h>
#include <AccelStepper.h>

#define DRIVE_STEPPER

#define ENABLE_SERIAL 0
#define IF_SERIAL if (ENABLE_SERIAL)

#define STEPPER_STEPS 800
#define MAX_SPEED_STEPS_PER_SEC 3000
#define DRIVE_MS 200
#define RC_TIMEOUT_S 15
#define THROTTLE_WEIGHT 100
#define STEERING_WEIGHT 100
#define PPM_LOW 1000
#define PPM_HIGH 2000
#define PPM_CENTER ((PPM_LOW + PPM_HIGH) / 2)

#define STATUS_LED_PIN 2
#define INTERRUPT_PIN 3
#define RC_POWER_PIN 12

#define L_STEPPER_PIN_I1 4
#define L_STEPPER_PIN_I2 5
#define L_STEPPER_PIN_I3 6
#define L_STEPPER_PIN_I4 7
#define R_STEPPER_PIN_I1 8
#define R_STEPPER_PIN_I2 9
#define R_STEPPER_PIN_I3 10
#define R_STEPPER_PIN_I4 11
AccelStepper left_stepper(AccelStepper::FULL4WIRE,
    L_STEPPER_PIN_I1, L_STEPPER_PIN_I2, L_STEPPER_PIN_I3, L_STEPPER_PIN_I4);
AccelStepper right_stepper(AccelStepper::FULL4WIRE,
    R_STEPPER_PIN_I1, R_STEPPER_PIN_I2, R_STEPPER_PIN_I3, R_STEPPER_PIN_I4);

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
unsigned long last_ppm_signal = 0;

void setup() {
  IF_SERIAL Serial.begin(9600);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  pinMode(AC_RELAY_CONTROL_PIN, OUTPUT);
  digitalWrite(AC_RELAY_CONTROL_PIN, LOW);
  pinMode(RC_POWER_PIN, OUTPUT);
  digitalWrite(RC_POWER_PIN, LOW);

  for (int i = 0; i < 10; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
  }
  digitalWrite(RC_POWER_PIN, HIGH);
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
    IF_SERIAL Serial.print(String(value) + " ");
    if (value != channel_values[channel - 1]) {
      last_ppm_signal = millis();
    }
    channel_values[channel - 1] = value;
  }
  IF_SERIAL Serial.println();

  if (millis() > last_ppm_signal + RC_TIMEOUT_S * 1000 || millis() < last_ppm_signal) {
    IF_SERIAL Serial.println("Resetting receiver...");
    digitalWrite(RC_POWER_PIN, LOW);
    delay(100);
    digitalWrite(RC_POWER_PIN, HIGH);
  } else {

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
    delayMicroseconds(1);
  }
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

  IF_SERIAL Serial.println(
    "T: " + String(throttle_pct) +
    "; S: " + String(steering_pct) +
    "; -> L=" + String(left_speed) +
    "; R=" + String(right_speed));

  digitalWrite(STATUS_LED_PIN, HIGH);
  drive_stepper(left_speed, right_speed);
  digitalWrite(STATUS_LED_PIN, LOW);
}

void drive_stepper(int left_speed, int right_speed) {
  // reverse right motor
  right_speed = -right_speed;
  left_stepper.setSpeed(map(left_speed, -THROTTLE_WEIGHT, THROTTLE_WEIGHT,
    -MAX_SPEED_STEPS_PER_SEC, MAX_SPEED_STEPS_PER_SEC));
  right_stepper.setSpeed(map(right_speed, -THROTTLE_WEIGHT, THROTTLE_WEIGHT,
    -MAX_SPEED_STEPS_PER_SEC, MAX_SPEED_STEPS_PER_SEC));

  long start = millis();
  bool done = 1;
  while (millis() < start + DRIVE_MS && millis() > start - 1) {
    if (left_stepper.runSpeed()) done = 0;
    if (right_stepper.runSpeed()) done = 0;
  }
}
