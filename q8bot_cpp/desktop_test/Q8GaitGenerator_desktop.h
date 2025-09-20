/*
  Desktop version of Q8GaitGenerator.h - Mock Arduino dependencies
*/

#ifndef Q8GaitGenerator_h
#define Q8GaitGenerator_h

#include "../desktop_test/arduino_mock.h"
#include "Q8Kinematics_desktop.h"

// Gait types supported by Q8bot
enum GaitType {
    GAIT_AMBER = 0,
    GAIT_AMBER_HIGH = 1,
    GAIT_AMBER_LOW = 2,
    GAIT_AMBER_FAST = 3,
    GAIT_WALK = 4,
    GAIT_BOUND = 5,
    GAIT_PRONK = 6,
    GAIT_COUNT = 7  // Total number of gaits
};

// Movement directions
enum Direction {
    DIR_FORWARD = 0,
    DIR_BACKWARD = 1,
    DIR_LEFT = 2,
    DIR_RIGHT = 3,
    DIR_FORWARD_LEFT = 4,
    DIR_FORWARD_RIGHT = 5,
    DIR_STOP = 6
};

// Gait stacking types (coordination patterns)
enum StackType {
    STACK_AMBER,
    STACK_WALK,
    STACK_BOUND,
    STACK_PRONK
};

// Gait parameter structure
struct GaitParams {
    const char* name;
    StackType stacktype;
    float x0, y0;           // Starting position (mm)
    float xrange, yrange, yrange2;  // Movement ranges (mm)
    int s1_count, s2_count; // Step counts for lift/support phases
};

class Q8GaitGenerator {
public:
    // Constructor
    Q8GaitGenerator(Q8Kinematics* kinematics);
    
    // Main gait generation function
    bool generateGait(GaitType gait, Direction dir, float positions[][8], int& length, float speed_multiplier = 1.0);
    
    // Get gait information
    int getGaitLength(GaitType gait) const;
    const char* getGaitName(GaitType gait) const;
    GaitParams getGaitParams(GaitType gait) const;
    
    // Custom gait configuration
    void setCustomGait(GaitType gait, GaitParams params);
    
    // Utility functions
    bool validateGait(GaitType gait, Direction dir) const;
    void getIdlePosition(float positions[8]) const;
    
    // Maximum trajectory length (for memory allocation)
    static const int MAX_GAIT_LENGTH = 200;

private:
    Q8Kinematics* kinematics;
    
    // Pre-defined gait parameters (from Python gaits dictionary)
    GaitParams gaitTable[GAIT_COUNT];
    
    // Internal gait generation functions
    bool generateTrajectory(GaitParams params, Direction dir, float move_p[][2], float move_n[][2], 
                          float move_ps[][2], int& length, float speed_multiplier);
    
    // Stacking functions (coordination between legs)
    void stackAmber(Direction dir, int s1_count, int s2_count, float move_p[][2], 
                   float move_n[][2], float move_ps[][2], int length, float positions[][8]);
    void stackWalk(Direction dir, int s1_count, int s2_count, float move_p[][2], 
                  float move_n[][2], int length, float positions[][8]);
    void stackBound(Direction dir, int s1_count, int s2_count, float move_p[][2], 
                   float move_n[][2], int length, float positions[][8]);
    void stackPronk(Direction dir, int s1_count, int s2_count, float move_p[][2], 
                   float move_n[][2], int length, float positions[][8]);
    
    // Utility functions
    void appendPositionList(float list1[][2], float list2[][2], float list3[][2], 
                           float list4[][2], int length, float positions[][8]);
    void createDummyMovement(float q1, float q2, float positions[][8], int& length);
    void initializeGaitTable();
    
    // Constants
    static const float IDLE_POSITION[2];
    static const int DEFAULT_GAIT_LENGTH = 45;  // Typical gait cycle length
};

#endif