#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 100
#define SERVOMAX 600
#define SERVO_CHANNEL 0

void setup() {
  Serial.begin(115200);
  Serial.println("Khởi động PCA9685...");

  Wire.begin(21, 22);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // đảm bảo chính xác tần số
  pwm.setPWMFreq(50);
  delay(10);

  Serial.println("Test servo kênh 0...");
}

void loop() {
  pwm.setPWM(SERVO_CHANNEL, 0, SERVOMIN);
  delay(1000);
  pwm.setPWM(SERVO_CHANNEL, 0, (SERVOMIN + SERVOMAX) / 2);
  delay(1000);
  pwm.setPWM(SERVO_CHANNEL, 0, SERVOMAX);
  delay(1000);
}
