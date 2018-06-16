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

#define INTERRUPT_PIN 3
#define L_STEPPER_PIN_I1 4
#define L_STEPPER_PIN_I2 5
#define L_STEPPER_PIN_I3 6
#define L_STEPPER_PIN_I4 7
#define R_STEPPER_PIN_I1 8
#define R_STEPPER_PIN_I2 9
#define R_STEPPER_PIN_I3 10
#define R_STEPPER_PIN_I4 11
#define AC_RELAY_CONTROL_PIN 13
// #define SERVO_PIN 10
#define NUM_CHANNELS 8

#define CHANNEL_R_EW 0
#define CHANNEL_R_NS 1
#define CHANNEL_L_EW 2
#define CHANNEL_L_NS 3
#define CHANNEL_SWB 5

#define CHANNEL_THROTTLE CHANNEL_L_NS
#define CHANNEL_STEERING CHANNEL_R_EW
#define CHANNEL_AC_RELAY CHANNEL_SWB

PPMReader ppm(INTERRUPT_PIN, NUM_CHANNELS);
Servo myservo;
Stepper left_stepper(STEPPER_STEPS,
   L_STEPPER_PIN_I1, L_STEPPER_PIN_I2, L_STEPPER_PIN_I3, L_STEPPER_PIN_I4);
Stepper right_stepper(STEPPER_STEPS,
    R_STEPPER_PIN_I1, R_STEPPER_PIN_I2, R_STEPPER_PIN_I3, R_STEPPER_PIN_I4);

void setup() {
    Serial.begin(9600);
    pinMode(AC_RELAY_CONTROL_PIN, OUTPUT);
    digitalWrite(AC_RELAY_CONTROL_PIN, LOW);
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

    if (channel_values[AC_RELAY_CHANNEL] > PPM_CENTER) {
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
    /*
    throttle    | steering  | leftSpeed | leftDirection | rightSpeed    | rightDirection| t * s    | t + s
    0 (center)  | 0 (cen)   | 0         | x             | 0             | x             | 0        | 0
    0 (center)  | -100 (W)  | 100       | -1            | 100           | +1            | 0        | -100
    0 (center)  | +100 (E)  | 100       | +1            | 100           | -1            | 0        | 100
    100 (North) | 0 (cen)   | 100       | +1            | 100           | -1            | 0        |
    100 (North) | -100 (W)  | 50        | +1            | 100           | -1
    100 (North) | +100 (E)  | 100       | +1            | 50            | -1
    -100 (South)| 0 (cen)   | 100       | -1            | 100           | +1
    -100 (South)| -100 (W)  | 50        | -1            | 100           | +1
    -100 (South)| +100 (E)  | 100       | -1            | 50            | +1
    */

    int left_steering = abs(min(steering_pct, 0));
    int right_steering = abs(max(steering_pct, 0));
    int left_speed = abs(throttle_pct) - left_steering;
    int right_speed = abs(throttle_pct) - right_steering;
    left_stepper.setSpeed(abs(left_speed));
    right_stepper.setSpeed(abs(right_speed));
    // These are blocking, so I guess we will move one motor at a time, one step
    // at a time
    if (left_speed != 0)
        left_stepper.step(left_speed / abs(left_speed));
    if (right_speed != 0)
        right_stepper.step(right_speed / abs(right_speed));
}
