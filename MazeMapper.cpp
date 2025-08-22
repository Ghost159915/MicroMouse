#include "MazeMapper.hpp"

MazeMapper::MazeMapper(uint8_t startX, uint8_t startY, Dir startH) {
    reset(startX, startY, startH);
}

void MazeMapper::reset(uint8_t startX, uint8_t startY, Dir startH) {
    for (uint8_t y=0; y<N; ++y) {
        for (uint8_t x=0; x<N; ++x) {
            walls[y][x]   = 0;
            visited[y][x] = false;
        }
    }
    rx = startX; ry = startY; heading = startH;
    if (inBounds(rx,ry) && !isStructurallyRemoved(rx,ry)) {
        visited[ry][rx] = true;
    }
    cmdBuf[0] = '\0';
}

bool MazeMapper::isStructurallyRemoved(uint8_t x, uint8_t y) {
    // three per corner
    if ((x==0 && y==0) || (x==0 && y==1) || (x==1 && y==0)) return true;            // bottom-left
    if ((x==8 && y==8) || (x==8 && y==7) || (x==7 && y==8)) return true;            // top-right
    if ((x==0 && y==8) || (x==0 && y==7) || (x==1 && y==8)) return true;            // top-left
    if ((x==8 && y==0) || (x==7 && y==0) || (x==8 && y==1)) return true;            // bottom-right
    return false;
}

void MazeMapper::placeWallAt(uint8_t x, uint8_t y, Dir dir) {
    if (!inBounds(x,y)) return;
    setWallBidirectional(x,y,dir);
}

void MazeMapper::setWallBidirectional(uint8_t x, uint8_t y, Dir dir) {
    if (!inBounds(x,y)) return;
    walls[y][x] |= wallBit(dir);
    int nx = x + dx(dir);
    int ny = y + dy(dir);
    if (inBounds(nx,ny)) {
        walls[ny][nx] |= wallBit(opposite(dir));
    }
}

void MazeMapper::markVisited(uint8_t x, uint8_t y) {
    if (inBounds(x,y) && !isStructurallyRemoved(x,y)) visited[y][x] = true;
}

void MazeMapper::observeWallsAtCurrent(int leftMM, int frontMM, int rightMM, int blockThreshMM) {
    // Left relative
    Dir leftAbs  = turnLeft(heading);
    Dir rightAbs = turnRight(heading);
    Dir frontAbs = heading;

    if (leftMM  > 0 && leftMM  <= blockThreshMM)  setWallBidirectional(rx, ry, leftAbs);
    if (frontMM > 0 && frontMM <= blockThreshMM) setWallBidirectional(rx, ry, frontAbs);
    if (rightMM > 0 && rightMM <= blockThreshMM) setWallBidirectional(rx, ry, rightAbs);

    // (Optional) auto-wall outer boundary if at edges and no neighbor exists
    if (ry == N-1) setWallBidirectional(rx, ry, NORTH);
    if (rx == N-1) setWallBidirectional(rx, ry, EAST);
    if (ry == 0)   setWallBidirectional(rx, ry, SOUTH);
    if (rx == 0)   setWallBidirectional(rx, ry, WEST);

    // Also block the 12 structurally removed corner cells from being entered
    // by setting walls around them (lazy approach: block from your side when neighbor is removed)
    for (uint8_t d=0; d<4; ++d) {
        Dir ad = (Dir)d;
        int nx = rx + dx(ad);
        int ny = ry + dy(ad);
        if (!inBounds(nx,ny) || isStructurallyRemoved(nx,ny)) {
            setWallBidirectional(rx, ry, ad);
        }
    }
}

MazeMapper::Dir MazeMapper::chooseRHRDirection() const {
    // Priority order: Right, Forward, Left, Back
    Dir cand[4] = { turnRight(heading), heading, turnLeft(heading), opposite(heading) };

    // 1) among open neighbors, try to pick an UNVISITED cell first
    for (uint8_t i=0; i<4; ++i) {
        Dir d = cand[i];
        int nx = rx + dx(d);
        int ny = ry + dy(d);
        if (!inBounds(nx,ny) || isStructurallyRemoved(nx,ny)) continue;
        if (!isWall(rx,ry,d) && !visited[ny][nx]) return d;
    }
    // 2) otherwise, just any OPEN neighbor in the same priority
    for (uint8_t i=0; i<4; ++i) {
        Dir d = cand[i];
        int nx = rx + dx(d);
        int ny = ry + dy(d);
        if (!inBounds(nx,ny) || isStructurallyRemoved(nx,ny)) continue;
        if (!isWall(rx,ry,d)) return d;
    }
    // 3) boxed in
    return heading; // will produce "" command later
}

const char* MazeMapper::planStepAndBuildCommand(int leftMM, int frontMM, int rightMM, int blockThreshMM) {
    // Observe & place current walls
    observeWallsAtCurrent(leftMM, frontMM, rightMM, blockThreshMM);

    // Choose direction using right-hand rule
    Dir nextDir = chooseRHRDirection();

    // If the chosen direction is actually blocked (race condition), bail
    if (isWall(rx, ry, nextDir)) { cmdBuf[0] = '\0'; return cmdBuf; }

    // Build a tiny command that MotorController understands
    // RHR prefers turning then moving forward one cell.
    if (nextDir == heading) {
        cmdBuf[0] = 'F'; cmdBuf[1] = '\0';
    } else if (nextDir == turnRight(heading)) {
        cmdBuf[0] = 'R'; cmdBuf[1] = 'F'; cmdBuf[2] = '\0';
    } else if (nextDir == turnLeft(heading)) {
        cmdBuf[0] = 'L'; cmdBuf[1] = 'F'; cmdBuf[2] = '\0';
    } else {
        // backtrack: 180° turn (two rights) then forward
        cmdBuf[0] = 'R'; cmdBuf[1] = 'R'; cmdBuf[2] = '\0';
        // NOTE: we’ll issue just "RR" and plan another step next cycle to move forward,
        // because your MotorControl groups forward runs; safer to keep it atomic.
    }
    return cmdBuf;
}

void MazeMapper::applyExecutedCommand(const char* cmdCompleted) {
    if (!cmdCompleted || !cmdCompleted[0]) return;

    // Apply L/R turns first (there may be one or two)
    for (uint8_t i=0; cmdCompleted[i] != '\0'; ++i) {
        char c = cmdCompleted[i];
        if (c == 'L') heading = turnLeft(heading);
        else if (c == 'R') heading = turnRight(heading);
    }

    // If the sequence ends with 'F', advance one cell in the NEW heading
    size_t len = strlen(cmdCompleted);
    if (len > 0 && cmdCompleted[len - 1] == 'F') {
        int nx = rx + dx(heading);
        int ny = ry + dy(heading);
        if (inBounds(nx,ny) && !isStructurallyRemoved(nx,ny) && !isWall(rx,ry,heading)) {
            rx = (uint8_t)nx; ry = (uint8_t)ny;
            visited[ry][rx] = true;
        }
    }
}

uint8_t MazeMapper::countVisited() const {
    uint8_t c = 0;
    for (uint8_t y=0; y<N; ++y) {
        for (uint8_t x=0; x<N; ++x) {
            if (visited[y][x] && !isStructurallyRemoved(x,y)) c++;
        }
    }
    return c;
}

uint8_t MazeMapper::visitedCount() const { return countVisited(); }

float MazeMapper::completionPercent() const {
    // clamp to 100 just in case
    float pct = (100.0f * countVisited()) / (float)TOTAL_TRAV_CELLS;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

void MazeMapper::printAscii() const {
    // Simple ASCII: show walls and mark visited cells
    // Top border
    Serial.println(F("=== Maze Map (visited='.') ==="));
    // horizontal walls per row (from top to bottom)
    for (int y = N-1; y >= 0; --y) {
        // Top edges of row y
        for (int x = 0; x < N; ++x) {
            Serial.print('+');
            if (isWall(x,y,NORTH)) Serial.print("---");
            else                   Serial.print("   ");
        }
        Serial.println('+');

        // Cells with vertical edges
        for (int x = 0; x < N; ++x) {
            if (isWall(x,y, WEST)) Serial.print('|');
            else                   Serial.print(' ');

            char cellCh = ' ';
            if (isStructurallyRemoved(x,y)) cellCh = '#'; // blocked structural
            else if (visited[y][x])         cellCh = '.';
            // mark robot
            if ((uint8_t)x == rx && (uint8_t)y == ry) {
                switch (heading) {
                    case NORTH: cellCh = '^'; break;
                    case EAST:  cellCh = '>'; break;
                    case SOUTH: cellCh = 'v'; break;
                    case WEST:  cellCh = '<'; break;
                }
            }
            Serial.print(' ');
            Serial.print(cellCh);
            Serial.print(' ');
        }
        // rightmost wall of the row
        Serial.println('|');
    }
    // bottom border
    for (int x = 0; x < N; ++x) {
        Serial.print('+');
        if (isWall(x,0,SOUTH)) Serial.print("---");
        else                   Serial.print("   ");
    }
    Serial.println('+');

    Serial.print(F("Visited: "));
    Serial.print((int)countVisited());
    Serial.print(F("/"));
    Serial.print((int)TOTAL_TRAV_CELLS);
    Serial.print(F("  ("));
    Serial.print(completionPercent(), 1);
    Serial.println(F("%)"));
}
