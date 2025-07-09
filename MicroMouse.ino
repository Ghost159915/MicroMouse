#include "MotorControl.hpp"
#include "Encoder.hpp"
#include "Display.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"
#include "IMU.hpp"

const float WHEEL_RADIUS = 16.0;

MotorController motors(11, 12, 9, 10);
Encoder encoder(2, 7);
Display display(WHEEL_RADIUS, TICKS_PER_REV);
PIDController DistancePID(2.0, 0.023, 0.0, 0.85);
PIDController TurningPID(0.5, 0.0, 0.0, 0.85);

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
  delay(3000);
  lastTime = millis();
}

static float wrap180(float a) {
    if (a >  180.0) a -= 360.0;
    if (a < -180.0) a += 360.0;
    return a;
}

states currentState = STARTUP_TURN;

void loop() {

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt < 0.001) dt = 0.001;
  lastTime = now;

  String command = "fl";

  imu.update();

  switch(currentState) {
    case STARTUP_TURN:
      Serial.println("State: STARTUP_TURN");
      motors.PIDturn90CW(&imu, &TurningPID, dt);
      motors.stop();
      currentState = COMPLETE;
      break;
    case WAIT_FOR_ROTATION:

      break;
    case WALL_APPROACH:

      break;
    case RETURN_TO_HEADING:

      break;
    case COMMAND_CHAIN:
      motors.chainCommand(command, &TurningPID, &encoder, &imu, dt, &currentState);
      break;
    case COMPLETE:
      motors.stop();
      break;
    default:
      motors.stop();
      break;
  }
}
 