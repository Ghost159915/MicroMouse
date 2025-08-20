#pragma once
#include "Maze.hpp"
#include "LidarSensor.hpp"

// Update wall information for the current cell using Lidar and heading
void updateWallsFromSensors(Maze &maze, int row, int col, char heading, LidarSensor* lidar);

// Decide the next move based on flood fill distances
// Returns: 'F' (forward), 'L' (left turn), 'R' (right turn), 'B' (backwards), or 'X' (no move)
char decideNextMove(Maze &maze, int row, int col, char heading);

// Update robot's position and heading based on executed move
void updateRobotPosition(int &row, int &col, char &heading, char move);
