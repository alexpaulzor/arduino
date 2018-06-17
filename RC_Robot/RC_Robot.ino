#include <PPMReader.h>
#include <Stepper.h>
// #include <Servo.h>

#define STEPPER_STEPS 200
#define MAX_SPEED_RPM 36
#define THROTTLE_WEIGHT 100
#define STEERING_WEIGHT 100
#define PPM_LOW 1000
#define PPM_HIGH 2000
#define PPM_CENTER ((PPM_LOW + PPM_HIGH) / 2)


// #define L_STEPPER_PIN_I1 4
// #define L_STEPPER_PIN_I2 5
// #define L_STEPPER_PIN_I3 6
// #define L_STEPPER_PIN_I4 7
// #define R_STEPPER_PIN_I1 8
// #define R_STEPPER_PIN_I2 9
// #define R_STEPPER_PIN_I3 10
// #define R_STEPPER_PIN_I4 11
// On most Arduino boards (those with the ATmega168 or ATmega328P),
// this function works on pins 3, 5, 6, 9, 10, and 11.
// On the Arduino Mega, it works on pins 2 - 13 and 44 - 46
#define L_DRIVE_FORWARD_PIN 1
#define L_DRIVE_REVERSE_PIN 2
#define INTERRUPT_PIN 3
#define L_DRIVE_ENABLE_PIN 5
#define R_DRIVE_ENABLE_PIN 6
#define R_DRIVE_FORWARD_PIN 7
#define R_DRIVE_REVERSE_PIN 8



#define AC_RELAY_CONTROL_PIN 13
// #define SERVO_PIN 10
#define NUM_CHANNELS 8

#define CHANNEL_R_EW 0
#define CHANNEL_R_NS 1
#define CHANNEL_L_NS 2
#define CHANNEL_L_EW 3
#define CHANNEL_SWB 5

#define CHANNEL_THROTTLE CHANNEL_L_NS
#define CHANNEL_STEERING CHANNEL_R_EW
#define CHANNEL_AC_RELAY CHANNEL_SWB

PPMReader ppm(INTERRUPT_PIN, NUM_CHANNELS);
// Servo myservo;
// Stepper left_stepper(STEPPER_STEPS,
//    L_STEPPER_PIN_I1, L_STEPPER_PIN_I2, L_STEPPER_PIN_I3, L_STEPPER_PIN_I4);
// Stepper right_stepper(STEPPER_STEPS,
//     R_STEPPER_PIN_I1, R_STEPPER_PIN_I2, R_STEPPER_PIN_I3, R_STEPPER_PIN_I4);

void setup() {
    Serial.begin(9600);
    pinMode(AC_RELAY_CONTROL_PIN, OUTPUT);
    digitalWrite(AC_RELAY_CONTROL_PIN, LOW);
    pinMode(L_DRIVE_FORWARD_PIN, OUTPUT);
    pinMode(L_DRIVE_REVERSE_PIN, OUTPUT);
    pinMode(L_DRIVE_ENABLE_PIN, OUTPUT);
    pinMode(R_DRIVE_ENABLE_PIN, OUTPUT);
    pinMode(R_DRIVE_FORWARD_PIN, OUTPUT);
    pinMode(R_DRIVE_REVERSE_PIN, OUTPUT);
    // myservo.attach(SERVO_PIN);
}

void loop() {
    int channel_values[NUM_CHANNELS];
    // Print latest valid values from all channels
    for (int channel = 1; channel <= NUM_CHANNELS; ++channel) {
        int value = ppm.latestValidChannelValue(channel, 0);
        //Serial.print(String(value) + " ");
        channel_values[channel - 1] = value;
    }
    //Serial.println();

    if (channel_values[CHANNEL_AC_RELAY] > PPM_CENTER) {
      digitalWrite(AC_RELAY_CONTROL_PIN, HIGH);
    } else {
      digitalWrite(AC_RELAY_CONTROL_PIN, LOW);
    }

    // int servo_value = map(channel_values[PITCH_CHANNEL], PPM_LOW, PPM_HIGH, 0, 180);
    // myservo.write(servo_value);

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
