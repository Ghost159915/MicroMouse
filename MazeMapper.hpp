#ifndef MAZE_MAPPER_HPP
#define MAZE_MAPPER_HPP

#include <Arduino.h>

class MazeMapper {
public:
    // ====== maze sizing ======
    static const uint8_t N = 9;           // 9x9 grid
    static const uint8_t TOTAL_TRAV_CELLS = 69; // given by spec (corners removed)

    // 4-bit wall mask per cell: bit0=N, bit1=E, bit2=S, bit3=W
    enum WallMask : uint8_t { WN=1, WE=2, WS=4, WW=8 };

    // robot heading (grid-based, not IMU degrees)
    enum Dir : uint8_t { NORTH=0, EAST=1, SOUTH=2, WEST=3 };

    // --- ctor & setup ---
    MazeMapper(uint8_t startX=0, uint8_t startY=0, Dir startH=NORTH);

    void reset(uint8_t startX, uint8_t startY, Dir startH);

    // Observe L/F/R ranges (mm), place walls at current cell, and choose next move.
    // Returns a 1- or 2-char command string:
    // "F"   (forward one cell)
    // "RF"  (turn right then forward)
    // "LF"  (turn left  then forward)
    // "RR"  (turn back on the spot; we’ll send that as two right turns)
    // If we can’t move (boxed-in), returns "" (empty).
    const char* planStepAndBuildCommand(int leftMM, int frontMM, int rightMM,
                                        int blockThreshMM = 100);

    // Must be called AFTER the command finishes to update pose
    void applyExecutedCommand(const char* cmdCompleted);

    // ASCII dump & progress
    void printAscii() const;
    uint8_t visitedCount() const;
    float    completionPercent() const;

    // Accessors you might want from .ino
    uint8_t getX() const { return rx; }
    uint8_t getY() const { return ry; }
    Dir     getDir() const { return heading; }

    // Manual wall placement helper if needed elsewhere
    void placeWallAt(uint8_t x, uint8_t y, Dir dir);

    // Mark cell visited (exposed in case you want to force it)
    void markVisited(uint8_t x, uint8_t y);

private:
    // ====== map state ======
    // walls[y][x] stores the 4-bit wall mask for cell (x,y)
    uint8_t walls[N][N];
    bool    visited[N][N];

    // robot grid pose
    uint8_t rx, ry;
    Dir     heading;

    // Small, fixed command buffer we return from planStepAndBuildCommand
    char cmdBuf[3]; // enough for "RF", "LF", "RR", "F", plus '\0'

    // ====== helpers ======
    static inline bool inBounds(int x, int y) {
        return (x >= 0 && x < N && y >= 0 && y < N);
    }

    // Corners removed: each corner removes 3 cells -> total 12, 81-12 = 69
    // Here we remove (0,0),(0,1),(1,0), (8,8),(8,7),(7,8), (0,8),(0,7),(1,8), (8,0),(7,0),(8,1)
    static bool isStructurallyRemoved(uint8_t x, uint8_t y);

    // Opposite direction
    static inline Dir opposite(Dir d) { return Dir((d + 2) & 3); }

    // Relative turns
    static inline Dir turnLeft(Dir d)  { return Dir((d + 3) & 3); }
    static inline Dir turnRight(Dir d) { return Dir((d + 1) & 3); }

    // Move deltas for a given absolute direction
    static inline int dx(Dir d) { return (d==EAST) - (d==WEST); }
    static inline int dy(Dir d) { return (d==NORTH) - (d==SOUTH); }

    // Walls bit for absolute direction
    static inline uint8_t wallBit(Dir d) {
        switch (d) {
            case NORTH: return WN;
            case EAST:  return WE;
            case SOUTH: return WS;
            default:    return WW;
        }
    }

    // Set a wall at (x,y) facing dir, and mirror into the neighbor if valid
    void setWallBidirectional(uint8_t x, uint8_t y, Dir dir);

    // Read LiDAR and place 0/1 walls for LEFT/FRONT/RIGHT faces of the current cell
    void observeWallsAtCurrent(int leftMM, int frontMM, int rightMM, int blockThreshMM);

    // Is there a wall in absolute direction d from (x,y)?
    bool isWall(uint8_t x, uint8_t y, Dir d) const {
        return (walls[y][x] & wallBit(d)) != 0;
    }

    // Right-hand rule, prefer unvisited among open options:
    // priority = Right, Forward, Left, Back; within each, prefer unvisited if possible.
    Dir chooseRHRDirection() const;

    // Count visited cells (excluding the 12 removed corners)
    uint8_t countVisited() const;
};

#endif // MAZE_MAPPER_HPP
