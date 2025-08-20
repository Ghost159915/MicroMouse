#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

class DualEncoder {
private:
    int encA_L, encB_L, encA_R, encB_R;
    volatile long ticksLeft, ticksRight;
    
    static DualEncoder* instance;
    static void isrLeft();
    static void isrRight();

public:
    DualEncoder(int aL = 2, int bL = 7, int aR = 3, int bR = 8);
    void begin();
    
    // Basic tick access
    long getLeftTicks() const;
    long getRightTicks() const;
    
    // Useful calculations
    long getAverageTicks() const;
    long getDifferentialTicks() const;  // For rotation
    float getHeadingChange() const;     // In degrees
    
    // Reset functions
    void reset();
    void resetLeft();
    void resetRight();
    
    // Drift detection
    float getDriftRatio() const;  // Returns left/right ratio
};

#endiff