#include "MazeNavigation.hpp"

void updateWallsFromSensors(Maze &maze, int row, int col, char heading, LidarSensor* lidar) {
    float frontDist = lidar->getFrontDistance();
    float leftDist = lidar->getLeftDistance();
    float rightDist = lidar->getRightDistance();

    auto detectWall = [](float dist) {
        return (dist < 150.0); // mm threshold
    };

    if (heading == 'N') {
        maze.setWall(row, col, 'N', detectWall(frontDist));
        maze.setWall(row, col, 'W', detectWall(leftDist));
        maze.setWall(row, col, 'E', detectWall(rightDist));
    }
    else if (heading == 'S') {
        maze.setWall(row, col, 'S', detectWall(frontDist));
        maze.setWall(row, col, 'E', detectWall(leftDist));
        maze.setWall(row, col, 'W', detectWall(rightDist));
    }
    else if (heading == 'E') {
        maze.setWall(row, col, 'E', detectWall(frontDist));
        maze.setWall(row, col, 'N', detectWall(leftDist));
        maze.setWall(row, col, 'S', detectWall(rightDist));
    }
    else if (heading == 'W') {
        maze.setWall(row, col, 'W', detectWall(frontDist));
        maze.setWall(row, col, 'S', detectWall(leftDist));
        maze.setWall(row, col, 'N', detectWall(rightDist));
    }
}

char decideNextMove(Maze &maze, int row, int col, char heading) {
    int bestDist = maze.cells[row][col].distance;
    char bestDir = 'X';

    struct { char dir; int dr; int dc; } moves[4] = {
        {'N', -1, 0}, {'S', 1, 0}, {'E', 0, 1}, {'W', 0, -1}
    };

    for (auto m : moves) {
        int nr = row + m.dr, nc = col + m.dc;
        if (nr < 0 || nr >= MAZE_SIZE || nc < 0 || nc >= MAZE_SIZE) continue;
        if (maze.hasWall(row, col, m.dir)) continue;
        if (maze.cells[nr][nc].distance < bestDist) {
            bestDist = maze.cells[nr][nc].distance;
            bestDir = m.dir;
        }
    }

    if (bestDir == 'X') return 'X'; // No move

    if (bestDir == heading) return 'F';
    if ((heading == 'N' && bestDir == 'E') ||
        (heading == 'E' && bestDir == 'S') ||
        (heading == 'S' && bestDir == 'W') ||
        (heading == 'W' && bestDir == 'N')) return 'R';
    if ((heading == 'N' && bestDir == 'W') ||
        (heading == 'W' && bestDir == 'S') ||
        (heading == 'S' && bestDir == 'E') ||
        (heading == 'E' && bestDir == 'N')) return 'L';

    return 'B'; // Backwards if opposite
}

void updateRobotPosition(int &row, int &col, char &heading, char move) {
    if (move == 'R') {
        if (heading == 'N') heading = 'E';
        else if (heading == 'E') heading = 'S';
        else if (heading == 'S') heading = 'W';
        else if (heading == 'W') heading = 'N';
    }
    else if (move == 'L') {
        if (heading == 'N') heading = 'W';
        else if (heading == 'W') heading = 'S';
        else if (heading == 'S') heading = 'E';
        else if (heading == 'E') heading = 'N';
    }
    else if (move == 'F') {
        if (heading == 'N') row--;
        else if (heading == 'S') row++;
        else if (heading == 'E') col++;
        else if (heading == 'W') col--;
    }
}
