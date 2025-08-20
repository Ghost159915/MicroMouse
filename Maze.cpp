#include "Maze.hpp"

Maze::Maze(int gRow, int gCol) {
    goalRow = gRow;
    goalCol = gCol;
}

void Maze::resetDistances() {
    for (int r = 0; r < MAZE_SIZE; r++) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            cells[r][c].distance = UNKNOWN_DIST;
        }
    }
    cells[goalRow][goalCol].distance = 0;
}

bool Maze::hasWall(int row, int col, char dir) {
    if (dir == 'N') return cells[row][col].northWall;
    if (dir == 'S') return cells[row][col].southWall;
    if (dir == 'E') return cells[row][col].eastWall;
    if (dir == 'W') return cells[row][col].westWall;
    return true;
}

void Maze::setWall(int row, int col, char dir, bool hasWall) {
    if (dir == 'N') {
        cells[row][col].northWall = hasWall;
        if (row > 0) cells[row - 1][col].southWall = hasWall;
    }
    else if (dir == 'S') {
        cells[row][col].southWall = hasWall;
        if (row < MAZE_SIZE - 1) cells[row + 1][col].northWall = hasWall;
    }
    else if (dir == 'E') {
        cells[row][col].eastWall = hasWall;
        if (col < MAZE_SIZE - 1) cells[row][col + 1].westWall = hasWall;
    }
    else if (dir == 'W') {
        cells[row][col].westWall = hasWall;
        if (col > 0) cells[row][col - 1].eastWall = hasWall;
    }
}

void Maze::markVisited(int row, int col) {
    cells[row][col].visited = true;
}

void Maze::floodFill() {
    bool changed = true;
    while (changed) {
        changed = false;
        for (int r = 0; r < MAZE_SIZE; r++) {
            for (int c = 0; c < MAZE_SIZE; c++) {
                int d = cells[r][c].distance;
                if (d == UNKNOWN_DIST) continue;

                if (!hasWall(r, c, 'N') && r > 0 && cells[r-1][c].distance > d + 1) {
                    cells[r-1][c].distance = d + 1; changed = true;
                }
                if (!hasWall(r, c, 'S') && r < MAZE_SIZE-1 && cells[r+1][c].distance > d + 1) {
                    cells[r+1][c].distance = d + 1; changed = true;
                }
                if (!hasWall(r, c, 'E') && c < MAZE_SIZE-1 && cells[r][c+1].distance > d + 1) {
                    cells[r][c+1].distance = d + 1; changed = true;
                }
                if (!hasWall(r, c, 'W') && c > 0 && cells[r][c-1].distance > d + 1) {
                    cells[r][c-1].distance = d + 1; changed = true;
                }
            }
        }
    }
}
