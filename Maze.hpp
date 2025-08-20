#pragma once
#include <Arduino.h>

#define MAZE_SIZE 9
#define UNKNOWN_DIST 255

struct Cell {
    bool northWall = false;
    bool southWall = false;
    bool eastWall = false;
    bool westWall = false;
    bool visited = false;
    int distance = UNKNOWN_DIST;
};

class Maze {
public:
    Cell cells[MAZE_SIZE][MAZE_SIZE];
    int goalRow, goalCol;

    Maze(int gRow, int gCol);
    void resetDistances();
    void floodFill();
    void setWall(int row, int col, char dir, bool hasWall);
    void markVisited(int row, int col);
    bool hasWall(int row, int col, char dir);
};
