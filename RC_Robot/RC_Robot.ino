#include <PPMReader.h>
#include <AccelStepper.h>

#define ENABLE_SERIAL 0
#define IF_SERIAL if (ENABLE_SERIAL)

#define ARM_MICROSTEP 1
#undef ARM_L298N

#define DRIVE_MS 100
#define RC_TIMEOUT_S 3
#define THROTTLE_WEIGHT 100
#define STEERING_WEIGHT 100
#define PPM_LOW 1000
#define PPM_HIGH 2000
#define PPM_CENTER ((PPM_LOW + PPM_HIGH) / 2)

#define STATUS_LED_PIN 13
#define INTERRUPT_PIN 3
#define RC_POWER_PIN 12

#ifdef ARM_L298N
  #define STEPPER_STEPS_PER_REV 200
  #define MAX_SPEED_STEPS_PER_SEC 400
  #define H_STEPPER_PIN_I1 26
  #define H_STEPPER_PIN_I2 27
  #define H_STEPPER_PIN_I3 28
  #define H_STEPPER_PIN_I4 29
  AccelStepper height_stepper(AccelStepper::FULL4WIRE,
      H_STEPPER_PIN_I1, H_STEPPER_PIN_I2, H_STEPPER_PIN_I3, H_STEPPER_PIN_I4);
#endif

#ifdef ARM_MICROSTEP
  #define STEPPER_STEPS_PER_REV 200
  #define MAX_SPEED_STEPS_PER_SEC 400
  #define H_STEPPER_PIN_ENABLE 51
  #define H_STEPPER_PIN_DIR 52
  #define H_STEPPER_PIN_PUL 53
  AccelStepper height_stepper(AccelStepper::DRIVER,
      H_STEPPER_PIN_PUL, H_STEPPER_PIN_DIR);
  height_stepper.setEnablePin(H_STEPPER_PIN_ENABLE);
#endif

#define H_STEPPER_MAX_STEPS 20000  // TODO: measure
#define H_STEPPER_HOME_SW_PIN 50
#define H_STEPPER_HOME_THRESH (H_STEPPER_MAX_STEPS / 20.0)
#define H_STEPPER_FUZZ (STEPPER_STEPS_PER_REV / 2)
#define HEIGHT_WEIGHT H_STEPPER_MAX_STEPS
  
#define PWM_MAX 255
#define L_FORWARD_PIN 4
#define L_FORWARD_SPEED_PIN 5
#define L_REVERSE_SPEED_PIN 6
#define L_REVERSE_PIN 7
#define R_FORWARD_PIN 8
#define R_REVERSE_PIN 9
#define R_FORWARD_SPEED_PIN 10
#define R_REVERSE_SPEED_PIN 11

#define AC_RELAY_CONTROL_PIN 13

#define NUM_CHANNELS 7

#define CHANNEL_R_EW 0
#define CHANNEL_R_NS 1
#define CHANNEL_L_NS 2
#define CHANNEL_L_EW 3
#define CHANNEL_SWA 4
#define CHANNEL_SWB 5
#define CHANNEL_DIAL_VRA 6

#define CHANNEL_THROTTLE CHANNEL_L_NS
#define CHANNEL_STEERING CHANNEL_R_EW
#define CHANNEL_AC_RELAY CHANNEL_SWB
#define CHANNEL_ENABLE_DRIVE CHANNEL_SWA
#define CHANNEL_HEIGHT CHANNEL_DIAL_VRA

PPMReader ppm(INTERRUPT_PIN, NUM_CHANNELS);
unsigned long last_ppm_signal = 0;

void blink(int count, int interval) {
  for (int i = 0; i < count; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(interval);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(interval);
  }
}

void setup() {
  IF_SERIAL Serial.begin(9600);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  pinMode(AC_RELAY_CONTROL_PIN, OUTPUT);
  digitalWrite(AC_RELAY_CONTROL_PIN, LOW);
  pinMode(RC_POWER_PIN, OUTPUT);
  digitalWrite(RC_POWER_PIN, LOW);
  blink(5, 200);
  digitalWrite(RC_POWER_PIN, HIGH);
  height_stepper.setMaxSpeed(MAX_SPEED_STEPS_PER_SEC);
  height_stepper.setSpeed(0);
  height_stepper.setCurrentPosition(H_STEPPER_MAX_STEPS);
 
  pinMode(L_FORWARD_PIN, OUTPUT);
  pinMode(L_REVERSE_PIN, OUTPUT);
  pinMode(L_FORWARD_SPEED_PIN, OUTPUT);
  pinMode(L_REVERSE_SPEED_PIN, OUTPUT);
  pinMode(R_FORWARD_PIN, OUTPUT);
  pinMode(R_REVERSE_PIN, OUTPUT);
  pinMode(R_FORWARD_SPEED_PIN, OUTPUT);
  pinMode(R_REVERSE_SPEED_PIN, OUTPUT);

  pinMode(H_STEPPER_HOME_SW_PIN, INPUT);
  
  stop_motors();
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
    stop_motors();
    digitalWrite(RC_POWER_PIN, LOW);
    blink(3, 100);
    digitalWrite(RC_POWER_PIN, HIGH);
  } else {

    if (channel_values[CHANNEL_AC_RELAY] > PPM_CENTER) {
      digitalWrite(AC_RELAY_CONTROL_PIN, HIGH);
    } else {
      digitalWrite(AC_RELAY_CONTROL_PIN, LOW);
    }

   if (channel_values[CHANNEL_ENABLE_DRIVE] > PPM_CENTER) {
      drive(
        map(channel_values[CHANNEL_THROTTLE],
            PPM_LOW, PPM_HIGH, -THROTTLE_WEIGHT, THROTTLE_WEIGHT),
        map(channel_values[CHANNEL_STEERING],
            PPM_LOW, PPM_HIGH, -STEERING_WEIGHT, STEERING_WEIGHT),
        map(channel_values[CHANNEL_HEIGHT],
            PPM_LOW, PPM_HIGH, 0, HEIGHT_WEIGHT));
   } else {
    stop_motors();
    delay(DRIVE_MS);
   }
  }
}

void drive(int throttle_pct, int steering_pct, long height_steps) {
  int left_speed = throttle_pct + steering_pct;
  int right_speed = throttle_pct - steering_pct;

  IF_SERIAL Serial.println(
    "T: " + String(throttle_pct) +
    "; S: " + String(steering_pct) +
    "; -> L=" + String(left_speed) +
    "; R=" + String(right_speed));

  digitalWrite(STATUS_LED_PIN, HIGH);
  drive_motors(left_speed, right_speed);
  drive_height(height);
  digitalWrite(STATUS_LED_PIN, LOW);
}

void drive_height(long height_steps) {
  if (height_steps < H_STEPPER_HOME_THRESH) {
    home_height();
  } else if (abs(height_steps - height_stepper.currentPosition()) < H_STEPPER_FUZZ) {
    height_stepper.disableOutputs();
    return;
  } else {
    height_stepper.moveTo(height_steps);
  }
  height_stepper.enableOutputs();
  long start = millis();
  bool done = 1;
  int dir = height_stepper.distanceToGo() / abs(height_stepper.distanceToGo());
  height_stepper.setSpeed(dir * MAX_SPEED_STEPS_PER_SEC);
  while (millis() < start + DRIVE_MS && millis() > start - 1 && !arm_at_home() && height_stepper.runSpeed()) {
    // let the stepper run
  }
}

void home_height() {
  if (!arm_at_home()) {
    height_stepper.move(-H_STEPPER_FUZZ);
  } else {
    height_stepper.setCurrentPosition(0);
    height_stepper.disableOutputs();
  }
}

bool arm_at_home() {
  if (digitalRead(H_STEPPER_HOME_SW_PIN) == HIGH) {
    height_stepper.setCurrentPosition(0);
    return true;
  }
  return false;
}

void stop_motors() {
  digitalWrite(L_FORWARD_PIN, HIGH);
  digitalWrite(L_REVERSE_PIN, HIGH);
  digitalWrite(R_FORWARD_PIN, HIGH);
  digitalWrite(R_REVERSE_PIN, HIGH);
  digitalWrite(L_FORWARD_SPEED_PIN, LOW);
  digitalWrite(L_REVERSE_SPEED_PIN, LOW);
  digitalWrite(R_FORWARD_SPEED_PIN, LOW);
  digitalWrite(R_REVERSE_SPEED_PIN, LOW);

  height_stepper.setSpeed(0);
  height_stepper.disableOutputs();
}

void drive_motors(int left_speed, int right_speed) {
  analogWrite(
    (left_speed < 0 ? L_REVERSE_SPEED_PIN : L_FORWARD_SPEED_PIN),
    constrain(map(abs(left_speed), 0, THROTTLE_WEIGHT, 0, PWM_MAX), 0, PWM_MAX));
  digitalWrite(
    (left_speed >= 0 ? L_REVERSE_SPEED_PIN : L_FORWARD_SPEED_PIN),
    LOW);
  analogWrite(
    (right_speed < 0 ? R_REVERSE_SPEED_PIN : R_FORWARD_SPEED_PIN),
    constrain(map(abs(right_speed), 0, THROTTLE_WEIGHT, 0, PWM_MAX), 0, PWM_MAX));
  digitalWrite(
    (right_speed >= 0 ? R_REVERSE_SPEED_PIN : R_FORWARD_SPEED_PIN),
    LOW);    
}
