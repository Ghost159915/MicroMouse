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
PIDController DistancePID(2.0, 0.023, 0.0);
PIDController TurningPID(2.0, 0.0, 0.0);

LidarSensor lidar;
IMU imu;

const float DESIRED_DISTANCE = 100.0;
const float DESIRED_ANGLE = 90.0;
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

static float wrap180(float a) {                 // keep ±180°
    if (a >  180.0f) a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now; 

    imu.update();                                   // ★ NEW — first
    float measuredAngle = imu.yaw();  

  float measuredFrontDistance = lidar.getFrontDistance();
  float measuredRightDistance = lidar.getRightDistance();
  float measuredLeftDistance = lidar.getLeftDistance();

  char command = 'D';

  if (command == 'D') {
    float control = DistancePID.compute(DESIRED_DISTANCE, measuredFrontDistance, dt);
    int pwm = constrain(abs(control), 0, 255);


    if (control > 5) {
      motors.moveBackward(pwm);
    } else if (control < -5) {
      motors.moveForward(pwm);
    } else {
      motors.stop();
    }
  } else if (command == 'B') {

    float angleError = wrap180(DESIRED_ANGLE - measuredAngle);           // ← NEW
    float angleCmd   = TurningPID.compute(0.0f, -angleError, dt);        // ← NEW
    int   pwm        = constrain(abs(angleCmd), 0, 255);    


    if (angleError > 3) {
      motors.spinCW(pwm);
    } else if (angleError < -3) {
      motors.spinCCW(pwm);
    } else {
      motors.stop();
    }
  }  

  display.showLidarDistance(measuredLeftDistance, measuredFrontDistance, measuredRightDistance);

}

