#pragma once
#include <stdint.h> // Include standard integer types for fixed-width types.

/**
 * @brief Utility class for handling non-blocking delays.
 * 
 * @details
 * This class provides a simple mechanism to measure elapsed time without
 * blocking the program flow, similar to how `millis()` or `micros()` are used
 * in Arduino environments. It stores a configurable delay interval in 
 * microseconds and internally tracks the last reference timestamp.
 * 
 * Typical usage:
 * - Call `init(delayTime)` to set up the desired delay interval.
 * - Repeatedly call `isDelayTimeElapsed()` in a loop to check if the delay has passed.
 * - Use `restartTimer()` to reset the timer when the delay condition is met.
 * - The delay interval can be changed dynamically with `updateDelayTime()`.
 * 
 * Notes:
 * - The empty constructor `Delay()` is provided but should be avoided 
 *   (it does not initialize the timer).
 * - Designed for microcontroller environments where precise non-blocking timing is required.
 */
class Delay{
  private:
    unsigned long _delayTime;          // us
    unsigned long _previousTime;      // us
  public:
    Delay(){}                         // Empty Constructor. Do not used
    Delay(unsigned long delayTime);   // Constructor
    
    void init();     
    void init(unsigned long delayTime);                                     // Configure the internal varialbles of the class to keep track the time
    bool isDelayTimeElapsed();                                                // returns true when the time set for the Delay has elapsed
    
    void updateDelayTime(unsigned long newDelayTime);          // set a new time for the Delay
    void restartTimer();                                                            // returns true when the delay time has elapse and reset the counter time
};
