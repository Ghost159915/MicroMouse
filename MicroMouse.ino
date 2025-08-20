#include "MotorControl.hpp"
#include "Encoder.hpp"
#include "Display.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"
#include "IMU.hpp"
#include "Maze.hpp"
#include "MazeNavigation.hpp"
#include <Wire.h>

MotorController motors(11, 12, 9, 10);
DualEncoder encoder(2, 7, 3, 8);
Display display(RADIUS, TICKS_PER_REV);

PIDController DistancePID(0.9, 0.0, 0.3, 0.8);
PIDController TurningPID(1.92, 0.32, 0.0, 0.8);
PIDController HeadingPID(1.0, 0.0, 0.2, 0.8);

LidarSensor lidar;
IMU imu;
states currentState;

Maze maze(4, 4);   // goal in center cell (row 4, col 4)
int currentRow = 0; // Start at bottom-left in maze grid
int currentCol = 0;
char heading = 'N'; // Starting facing north

unsigned long lastTime = 0;
float rotationOffset = 0.0;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    display.begin();
    imu.begin();
    motors.begin();
    encoder.begin();
    lidar.begin();
    encoder.reset();
    lastTime = millis();
    currentState = COMMAND_CHAIN;
    motors.startCommandChain("FFLFRFF");
    imu.calibrate();
    delay(3000);
}

void loop() {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt < 0.001) dt = 0.001;
    lastTime = now;
    imu.update();

    switch (currentState) {
        case COMMAND_CHAIN: {
            char activeCmd = motors.getCurrentCommand();
            float heading = imu.yaw();
            motors.processCommandStep(&TurningPID, &HeadingPID, &encoder, &imu, &currentState, dt);
            display.showCommandStatus("COMMAND_CHAIN", activeCmd, heading);
            break; 
        }

    	case PLANNING: {
        	// Update maze from sensor readings
        	updateWallsFromSensors(maze, currentRow, currentCol, heading, &lidar);
        	maze.markVisited(currentRow, currentCol);

        	// Recalculate distances
        	maze.resetDistances();
        	maze.floodFill();

        	// Choose next move
        	char moveCmd = decideNextMove(maze, currentRow, currentCol, heading);
        	if (moveCmd == 'X') {
            	Serial.println("No path to goal!");
            	break; // Stay in planning
        	}

        	// Load into motor command system
        	char cmdStr[2] = {moveCmd, '\0'};
        	motors.startCommandChain(cmdStr);

        	currentState = EXECUTING;
        	break;
    	}

        case EXECUTING: {
            motors.processCommandStep(&TurningPID, &HeadingPID, &encoder, &imu, &currentState, dt);

            if (currentState == COMPLETE) {
                char executedCmd = motors.getCurrentCommand();
                updateRobotPosition(currentRow, currentCol, heading, executedCmd);

                if (currentRow == maze.goalRow && currentCol == maze.goalCol) {
                    Serial.println("GOAL REACHED!");
                    currentState = COMPLETE;  // End main loop state
                    break;
                }

            currentState = PLANNING;
            }
            break;
        }

        case COMPLETE: {
            motors.stop();
            break;
        }

        default: {
            motors.stop();
            break;
        }
    }
}