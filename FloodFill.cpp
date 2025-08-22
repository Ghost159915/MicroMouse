#include "FloodFill.hpp"
#include <Arduino.h>
#include <algorithm>

FloodFill::FloodFill(int sx, int sy, int gx, int gy) {
    reset();
    setStart(sx, sy);
    setGoal(gx, gy);
}

void FloodFill::reset() {
    for (int i=0; i<N; i++)
        for (int j=0; j<N; j++) {
            dist[i][j] = 0;
            for (int d=0; d<4; d++) walls[i][j][d] = false;
        }
}

void FloodFill::initialiseDistanceMaze() {
    for (int i=0; i<N; i++)
        for (int j=0; j<N; j++)
            dist[i][j] = computeManhattan(i, j, goalX, goalY);
}

int FloodFill::computeManhattan(int sx, int sy, int gx, int gy) {
    int dx = abs(sx - gx);
    int dy = abs(sy - gy);
    return dx + dy;
}

void FloodFill::setStart(int x, int y) { startX = x; startY = y; }
void FloodFill::setGoal(int x, int y)  { goalX = x; goalY = y; }

FloodFill::Dir FloodFill::chooseNext() {
    // 1) look at neighbours using distance map
    Dir cheapest = cheapestNeighbour();           // absolute direction
    if (cheapest == NONE) {
        // No open neighbors (shouldn’t happen if walls are mirrored)
        // Make the current cell “worse” so ripple logic can propagate later
        dist[rx][ry] = (uint16_t)(dist[rx][ry] + 1);
        return NONE;                              // caller should relabel and re-try
    }

    // 2) get the minimum value among open neighbours (we already have it implicitly)
    uint16_t md;
    switch (cheapest) {
      case NORTH: md = dist[rx][ry-1]; break;
      case SOUTH: md = dist[rx][ry+1]; break;
      case EAST : md = dist[rx+1][ry]; break;
      case WEST : md = dist[rx-1][ry]; break;
      default:    md = 0x3fff; break;
    }

    // 3) Classic flood condition: current must be md + 1
    //    If not, fix it *and do not move this tick* (return NONE).
    if (dist[rx][ry] != (uint16_t)(md + 1)) {
        dist[rx][ry] = (uint16_t)(md + 1);
        return NONE;                               // caller should run local ripple
    }

    // 4) Current is consistent -> choose the actual *move*.
    //    Tie-break: Forward, Left, Right, Back (relative to currDir).
    Dir pref[4] = { currDir, leftOf(currDir), rightOf(currDir), backOf(currDir) };

    // Among all open neighbors with value == md, return the first in pref order.
    for (int p=0; p<4; ++p) {
        Dir want = pref[p];
        int nx = rx, ny = ry;
        if (want==NORTH) ny--;
        else if (want==SOUTH) ny++;
        else if (want==EAST) nx++;
        else if (want==WEST) nx--;

        if (!inBounds(nx,ny)) continue;
        if (walls[rx][ry][want]) continue;            // closed

        if (dist[nx][ny] == md) return want;
    }

    // Fallback: move to the cheapest dir we already computed
    return cheapest;
}


bool FloodFill::checkValidDirection(int x, int y, Dir dir) {
    int nx=x, ny=y;
    if (dir==NORTH) ny--;
    else if (dir==SOUTH) ny++;
    else if (dir==EAST)  nx++;
    else if (dir==WEST)  nx--;

    if (!inBounds(nx,ny)) return false;
    return !walls[x][y][dir];
}


FloodFill::Dir FloodFill::cheapestNeighbour() {
    // Read neighbor distances, use a big number if not open
    const uint16_t INF = 0x3fff;

    uint16_t north = INF, south = INF, east = INF, west = INF;

    // NORTH
    if (ry-1 >= 0 && !walls[rx][ry][NORTH])
        north = dist[rx][ry-1];

    // SOUTH
    if (ry+1 < N && !walls[rx][ry][SOUTH])
        south = dist[rx][ry+1];

    // EAST
    if (rx+1 < N && !walls[rx][ry][EAST])
        east = dist[rx+1][ry];

    // WEST
    if (rx-1 >= 0 && !walls[rx][ry][WEST])
        west = dist[rx-1][ry];

    // Pick the minimum (manual to be Arduino-safe)
    uint16_t best = north;
    FloodFill::Dir bestDir = NORTH;

    if (south < best) { best = south; bestDir = SOUTH; }
    if (east  < best) { best = east;  bestDir = EAST;  }
    if (west  < best) { best = west;  bestDir = WEST;  }

    // If all were INF, we’re boxed in -> NONE
    if (best == INF) return NONE;
    return bestDir;
}

void FloodFill::placeWall(Dir d) {
    // Put a wall at the current cell in absolute direction d and mirror it
    if (!inBounds(rx, ry)) return;
    walls[rx][ry][d] = true;

    int nx, ny; stepFrom(rx, ry, d, nx, ny);
    if (inBounds(nx, ny)) {
        walls[nx][ny][opposite(d)] = true;
    }
}

void FloodFill::observeWalls(bool leftBlocked, bool frontBlocked, bool rightBlocked) {
    // Convert relative L/F/R into absolute and stamp all three at once
    const Dir L = leftOf(currDir);
    const Dir F = currDir;
    const Dir R = rightOf(currDir);

    if (leftBlocked)  placeWall(L);
    if (frontBlocked) placeWall(F);
    if (rightBlocked) placeWall(R);
}

// Local ripple (stack-based) starting from (rx, ry)
void FloodFill::recomputeFromCurrent() {
    const uint16_t INF = 0x3fff;

    // tiny manual stack (no <stack> on AVR)
    int sx[N*N], sy[N*N];
    int top = 0;

    // push current cell once
    if (inBounds(rx, ry)) { sx[top] = rx; sy[top] = ry; ++top; }

    while (top > 0) {
        // pop
        --top;
        int x = sx[top], y = sy[top];

        // gather min open-neighbour distance and remember which dir gave it
        uint16_t md = INF;
        Dir mdDir = NONE;

        // NORTH
        if (y-1 >= 0 && !walls[x][y][NORTH]) {
            uint16_t v = dist[x][y-1];
            if (v < md) { md = v; mdDir = NORTH; }
        }
        // SOUTH
        if (y+1 < N && !walls[x][y][SOUTH]) {
            uint16_t v = dist[x][y+1];
            if (v < md) { md = v; mdDir = SOUTH; }
        }
        // EAST
        if (x+1 < N && !walls[x][y][EAST]) {
            uint16_t v = dist[x+1][y];
            if (v < md) { md = v; mdDir = EAST; }
        }
        // WEST
        if (x-1 >= 0 && !walls[x][y][WEST]) {
            uint16_t v = dist[x-1][y];
            if (v < md) { md = v; mdDir = WEST; }
        }

        if (md == INF) {
            // boxed in: nothing to do
            continue;
        }

        uint16_t desired = (uint16_t)(md + 1);
        if (dist[x][y] != desired) {
            // fix the current cell label
            dist[x][y] = desired;

            // push all open neighbours EXCEPT the target (mdDir)
            int nx, ny;

            if (!walls[x][y][NORTH]) {
                stepFrom(x, y, NORTH, nx, ny);
                if (inBounds(nx, ny) && mdDir != NORTH) { sx[top]=nx; sy[top]=ny; ++top; }
            }
            if (!walls[x][y][SOUTH]) {
                stepFrom(x, y, SOUTH, nx, ny);
                if (inBounds(nx, ny) && mdDir != SOUTH) { sx[top]=nx; sy[top]=ny; ++top; }
            }
            if (!walls[x][y][EAST]) {
                stepFrom(x, y, EAST, nx, ny);
                if (inBounds(nx, ny) && mdDir != EAST) { sx[top]=nx; sy[top]=ny; ++top; }
            }
            if (!walls[x][y][WEST]) {
                stepFrom(x, y, WEST, nx, ny);
                if (inBounds(nx, ny) && mdDir != WEST) { sx[top]=nx; sy[top]=ny; ++top; }
            }
        }
        // else: already consistent; nothing to propagate
    }
}


