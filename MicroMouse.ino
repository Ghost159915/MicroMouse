#include "MotorControl.hpp"
#include "Encoder.hpp"
#include "Display.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"
#include "IMU.hpp"
#include <Wire.h>
#include "FloodFill.hpp"
#include "MazeMapper.hpp"

MotorController motors(11, 12, 9, 10);
DualEncoder encoder(3, 8, 2, 7);  // Left encoder on 2,7; Right on 3,8 HAD TO SWAP THE ORDER TO GET THEM TO WORK, INITIALLY IT WAS encoder(2, 7, 3, 8)
Display display(RADIUS, TICKS_PER_REV);

PIDController DistancePID(0.9, 0.0, 0.3, 0.8);
PIDController TurningPID(1, 0.5, 0.32, 0.8);
PIDController HeadingPID(1.0, 0.0, 0.2, 0.8);
PIDController WallPID(0.8, 0.0, 0.3);

MazeMapper mapper(/*startX*/0, /*startY*/0, MazeMapper::NORTH);

LidarSensor lidar;
IMU imu;

unsigned long lastTime = 0;
float rotationOffset = 0.0;

FloodFill flood(0,0,7,7);

states currentState = TEST;

bool AUTONOMOUS_MAPPING = true; // set this value to true or false based on whether we are doing part 4.3 or not

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

    currentState = FORWARD;          
    motors.startCommandChain("FRFFRFLFFLFLFRFRFLFLFFFF");
    // FRFFRFLFFLFLFRFRFLFLFFFF

    flood.initialiseDistanceMaze();

    imu.calibrate();
    delay(1500);
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
            if(AUTONOMOUS_MAPPING) {
                currentState = FLOOD_FILL_MAPPING
            }else if(activeCmd == '\0' || !(motors.getCommandActive())) {
                currentState = COMPLETE;
            }
            // change the heading 
            motors.processCommandStep(&TurningPID, &HeadingPID, &WallPID, &encoder, &imu, &currentState, &lidar, dt);   
            if(AUTONOMOUS_MAPPING) {
                activeCmd = motors.getCurrentCommand();
                if (activeCmd == 'L') flood.setCurrentDir(flood.turnLeft(flood.getCurrentDir()));
                else if (activeCmd == 'R') flood.setCurrentDir(flood.turnLeft(flood.getCurrentDir()));
            }
            break;
        }

        case COMPLETE: {
            char activeCmd = motors.getCurrentCommand();
            float heading = motors.getFusedYaw();
            motors.stop();
            display.showCommandStatus("COMPLETE", activeCmd, heading);
            break;
        }

        case FLOOD_FILL_MAPPING: {
            if (flood.isGoalReached()) {
                currentState = RETURN_TO_START;
            } else {
                Dir currDir = flood.getCurrentDir();
                Dir cheapestDir = flood.getCheapest();
                char lidarCheck = headingToCommand(currDir, cheapestDir);

                int distance = 0;
                if (lidarCheck == 'L') {
                    distance = lidar.getLeftDistance();
                } else if (lidarCheck == 'R') {
                    distance = lidar.getRightDistance();
                } else if (lidarCheck == 'F') {
                    distance = lidar.getFrontDistance();
                }

                if (distance <= 100) {
                    // Blocked → mark wall
                    flood.placeWall(cheapestDir);
                } else {
                    // Valid path → build command string like "LF", "RF", or "F"
                    std::string command(1, lidarCheck);
                    if (lidarCheck != 'F') {
                        command += 'F';
                    }
                    motors.startCommandChain(command);
                }

                currentState = COMMAND_CHAIN;
            }
            break;
        }

        case BACKUP_MAPPING: {
            // 1) Read LiDAR once
            int l = (int)lidar.getLeftDistance();
            int f = (int)lidar.getFrontDistance();
            int r = (int)lidar.getRightDistance();

            // 2) Ask the mapper to observe + plan a tiny command
            const char* cmd = mapper.planStepAndBuildCommand(l, f, r, /*block<=*/100);

            // 3) If boxed-in or nothing to do, print map and STOP
            if (!cmd || cmd[0] == '\0') {
                mapper.printAscii();
                currentState = COMPLETE;
                break;
            }

            // 4) Send the tiny command to your existing chain executor
            motors.startCommandChain(cmd);

            // 5) Hand off to your normal COMMAND_CHAIN state
            currentState = COMMAND_CHAIN;
            break;
        }

        case COMMAND_CHAIN_BACKUP: {
            // run your existing processCommandStep(...)
            motors.processCommandStep(&TurningPID, &HeadingPID, &encoder, &imu, &currentState, dt);

            // Detect end of chain (your code may already do this)
            if (!motors.getCommandActive()) {
                // Recover the command that just finished. If your MotorController
                // doesn't expose the *last* command easily, keep a copy when you call startCommandChain.
                // For simplicity, cache it in a global:
                //   lastIssuedCmdString
                mapper.applyExecutedCommand(lastIssuedCmdString);

                // Optional: print a map every few steps
                static uint8_t stepCount=0;
                if ((++stepCount % 6) == 0) mapper.printAscii();

                // Go back to BACKUP_MAPPING for next step
                currentState = BACKUP_MAPPING;
            }
            break;
        }



        default:
            motors.stop();
            break;
    }
}
