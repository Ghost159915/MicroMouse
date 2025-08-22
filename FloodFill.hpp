#pragma once
#include "Heading.hpp"

class FloodFill {
public:
    static const int N = 9;   // adjust for maze size (16x16 standard)
    enum Dir : uint8_t { NORTH=0, EAST=1, SOUTH=2, WEST=3, NONE=255 };

    FloodFill(int sx = 0, int sy = 0, int gx = N/2, int gy = N/2);

    void reset();
    void initialiseDistanceMaze();
    int computeManhattan(int sx, int sy, int gx, int gy);

    void setStart(int x, int y);
    void setGoal(int x, int y);

    Dir chooseNext(int x, int y);
    bool checkValidDirection(int x, int y, Dir dir);

    int getDistance(int x, int y) const { return dist[x][y]; }

    int getRx() {
        return rx;
    }

    int getRy() {
        return ry;
    }

    void setCurrDir(Dir dir) {
        currDir = dir;
    }

    inline Dir turnLeft(Dir h)  { return static_cast<Dir>((static_cast<uint8_t>(h) + 3) & 3); }
    inline Dir turnRight(Dir h) { return static_cast<Dir>((static_cast<uint8_t>(h) + 1) & 3); }
    inline Dir turnBack(Dir h)  { return static_cast<Dir>((static_cast<uint8_t>(h) + 2) & 3); }


    Dir chooseNext();                 // pick a move or return NONE if we just relabeled
    bool checkValidDirection(int x, int y, Dir dir);
    Dir cheapestNeighbour();          // helper used by chooseNext

    void observeWalls(bool leftBlocked, bool frontBlocked, bool rightBlocked);
    void recomputeFromCurrent();
    void placeWall(Dir d);

    // (Optional but handy if you use heading in the .ino)
    Dir  getCurrentDir() const { return currDir; }
    int  getRx()        const { return rx; }
    int  getRy()        const { return ry; }
    void setCurrentDir(Dir d) { currDir = d; }

    // --- in class FloodFill (private): ---
    static inline Dir leftOf(Dir d)  { return Dir((uint8_t(d)+3)&3); }
    static inline Dir rightOf(Dir d) { return Dir((uint8_t(d)+1)&3); }
    static inline Dir backOf(Dir d)  { return Dir((uint8_t(d)+2)&3); }
    static inline bool inBounds(int x,int y){ return x>=0 && y>=0 && x<N && y<N; }

    static inline void stepFrom(int x,int y, Dir d, int& nx,int& ny) {
    nx = x; ny = y;
    if (d==NORTH) ny--;
    else if (d==SOUTH) ny++;
    else if (d==EAST)  nx++;
    else if (d==WEST)  nx--;
    }
    

private:
    int dist[N][N];
    bool walls[N][N][4];   // [x][y][direction]
    int startX, startY, goalX, goalY;
    int rx = 0, ry = 0;
    Dir currDir = NORTH;
};
