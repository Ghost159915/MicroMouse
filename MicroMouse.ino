#include "MotorControl.hpp"
#include "Encoder.hpp"
#include "Display.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"
#include "IMU.hpp"

const float WHEEL_RADIUS = 16.0;
const int TICKS_PER_REV = 700;

MotorController motors(11, 12, 9, 10);
Encoder encoder(2, 7);
Display display(WHEEL_RADIUS, TICKS_PER_REV);
PIDController pid(2.0, 0.023, 0.0);

LidarSensor lidar;
IMU imu;

const float DESIRED_DISTANCE = 100;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  imu.begin();
  imu.calibrate();
  motors.begin();
  encoder.begin();
  display.begin();
  lidar.begin();

  delay(2000);

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float measuredFrontDistance = lidar.getFrontDistance();
  float measuredRightDistance = lidar.getRightDistance();
  float measuredLeftDistance = lidar.getLeftDistance();

  float control = pid.compute(DESIRED_DISTANCE, measuredFrontDistance, dt);

  imu.update(dt);

  Serial.println("Heading   |   Gyro Z"); 

  Serial.print(imu.getHeading());
  Serial.print("            "); 
  Serial.println(imu.getGyroZ());

  int pwm = constrain(abs(control), 0, 255);

  display.showLidarDistance(measuredLeftDistance, measuredFrontDistance, measuredRightDistance);

  if (control > 5) {
    motors.moveBackward(pwm);
  } else if (control < -5) {
    motors.moveForward(pwm);
  } else {
    motors.stop();
  }
}

