/*
  Desktop-compatible Q8GaitGenerator implementation
*/

#include "Q8GaitGenerator_desktop.h"

// Static constants
const float Q8GaitGenerator::IDLE_POSITION[2] = {30.0, 150.0};

Q8GaitGenerator::Q8GaitGenerator(Q8Kinematics* kin) : kinematics(kin) {
    initializeGaitTable();
}

void Q8GaitGenerator::initializeGaitTable() {
    // Initialize gait parameters from Python gaits dictionary
    gaitTable[GAIT_AMBER] = {
        "AMBER", STACK_AMBER, 9.75, 43.36, 40, 20, 0, 15, 30
    };
    gaitTable[GAIT_AMBER_HIGH] = {
        "AMBER_HIGH", STACK_AMBER, 9.75, 60, 20, 10, 0, 15, 30
    };
    gaitTable[GAIT_AMBER_LOW] = {
        "AMBER_LOW", STACK_AMBER, 9.75, 25, 20, 10, 0, 15, 30
    };
    gaitTable[GAIT_AMBER_FAST] = {
        "AMBER_FAST", STACK_AMBER, 9.75, 43.36, 50, 20, 0, 12, 24
    };
    gaitTable[GAIT_WALK] = {
        "WALK", STACK_WALK, 9.75, 43.36, 30, 20, 0, 20, 140
    };
    gaitTable[GAIT_BOUND] = {
        "BOUND", STACK_BOUND, 9.75, 33.36, 40, 0, 20, 50, 10
    };
    gaitTable[GAIT_PRONK] = {
        "PRONK", STACK_PRONK, 9.75, 33.36, 40, 0, 20, 60, 10
    };
}

bool Q8GaitGenerator::generateGait(GaitType gait, Direction dir, float positions[][8], int& length, float speed_multiplier) {
    if (gait >= GAIT_COUNT || !kinematics) {
        return false;
    }
    
    GaitParams params = gaitTable[gait];
    
    // Apply speed multiplier to step counts (inverse relationship)
    params.s1_count = max(1, (int)(params.s1_count / speed_multiplier));
    params.s2_count = max(1, (int)(params.s2_count / speed_multiplier));
    
    // Generate single-leg trajectories
    float move_p[MAX_GAIT_LENGTH][2];   // Positive direction movement
    float move_n[MAX_GAIT_LENGTH][2];   // Negative direction movement  
    float move_ps[MAX_GAIT_LENGTH][2];  // Smaller positive movement (for AMBER)
    
    if (!generateTrajectory(params, dir, move_p, move_n, move_ps, length, speed_multiplier)) {
        createDummyMovement(90, 90, positions, length);
        return false;
    }
    
    // Apply coordination pattern based on gait type
    switch (params.stacktype) {
        case STACK_AMBER:
            stackAmber(dir, params.s1_count, params.s2_count, move_p, move_n, move_ps, length, positions);
            break;
        case STACK_WALK:
            stackWalk(dir, params.s1_count, params.s2_count, move_p, move_n, length, positions);
            break;
        case STACK_BOUND:
            stackBound(dir, params.s1_count, params.s2_count, move_p, move_n, length, positions);
            break;
        case STACK_PRONK:
            stackPronk(dir, params.s1_count, params.s2_count, move_p, move_n, length, positions);
            break;
        default:
            return false;
    }
    
    return true;
}

bool Q8GaitGenerator::generateTrajectory(GaitParams params, Direction dir, float move_p[][2], 
                                       float move_n[][2], float move_ps[][2], int& length, float speed_multiplier) {
    
    float x_p = params.x0 - params.xrange / 2.0;
    float x_n = params.x0 + params.xrange / 2.0;
    float x_ps = x_p * 0.5 + 9.75 * 0.5;  // Smaller movement for AMBER gait
    
    float x_lift_step = params.xrange / params.s1_count;
    float x_down_step = params.xrange / params.s2_count;
    
    length = params.s1_count + params.s2_count;
    
    if (length > MAX_GAIT_LENGTH) {
        length = MAX_GAIT_LENGTH;
        return false;
    }
    
    // Check if y position is valid
    if (params.y0 - params.yrange < 5) {
        std::cout << "Invalid: y below physical limit.\n";
        return false;
    }
    
    // Generate trajectory points
    for (int i = 0; i < length; i++) {
        float y;
        
        if (i < params.s1_count) {
            // Lift phase - sinusoidal trajectory
            x_p += x_lift_step;
            x_ps += x_lift_step * 0.5;
            x_n -= x_lift_step;
            
            float freq = M_PI / params.s1_count;
            y = params.y0 - sin((i + 1) * freq) * params.yrange;
        } else {
            // Support phase - ground contact with minor modulation
            x_p -= x_down_step;
            x_ps -= x_down_step * 0.5;
            x_n += x_down_step;
            
            float freq = M_PI / params.s2_count;
            y = params.y0 + sin((i - params.s1_count + 1) * freq) * params.yrange2;
        }
        
        // Convert (x,y) positions to joint angles using inverse kinematics
        float q1_p, q2_p, q1_ps, q2_ps, q1_n, q2_n;
        
        bool check1 = kinematics->ikSolve(x_p, y, q1_p, q2_p, true, 1);
        bool check2 = kinematics->ikSolve(x_n, y, q1_n, q2_n, true, 1);
        bool check3 = kinematics->ikSolve(x_ps, y, q1_ps, q2_ps, true, 1);
        
        // Validate solutions
        if (!check1 || !check2 || !check3) {
            // Retry with smaller step size if IK fails
            if (params.xrange > 1 && params.yrange > 1) {
                std::cout << "IK failed, retrying with smaller step size\n";
                GaitParams smaller_params = params;
                smaller_params.xrange -= 1;
                smaller_params.yrange -= 1;
                return generateTrajectory(smaller_params, dir, move_p, move_n, move_ps, length, speed_multiplier);
            }
            return false;
        }
        
        // Store joint angles
        move_p[i][0] = q1_p;
        move_p[i][1] = q2_p;
        move_n[i][0] = q1_n;
        move_n[i][1] = q2_n;
        move_ps[i][0] = q1_ps;
        move_ps[i][1] = q2_ps;
    }
    
    return true;
}

void Q8GaitGenerator::stackAmber(Direction dir, int s1_count, int s2_count, float move_p[][2], 
                                float move_n[][2], float move_ps[][2], int length, float positions[][8]) {
    
    float len_factor = (float)(s1_count + s2_count) / s1_count;
    int split = (int)(s1_count * len_factor / 2);
    
    // Create phase-shifted trajectories
    float move_p2[MAX_GAIT_LENGTH][2];
    float move_ps2[MAX_GAIT_LENGTH][2];
    float move_n2[MAX_GAIT_LENGTH][2];
    
    // Phase shift by split
    for (int i = 0; i < length; i++) {
        int shifted_idx = (i + split) % length;
        move_p2[i][0] = move_p[shifted_idx][0];
        move_p2[i][1] = move_p[shifted_idx][1];
        move_ps2[i][0] = move_ps[shifted_idx][0];
        move_ps2[i][1] = move_ps[shifted_idx][1];
        move_n2[i][0] = move_n[shifted_idx][0];
        move_n2[i][1] = move_n[shifted_idx][1];
    }
    
    // Apply direction-specific leg coordination
    switch (dir) {
        case DIR_FORWARD:
            appendPositionList(move_p, move_p2, move_p2, move_p, length, positions);
            break;
        case DIR_RIGHT:
            appendPositionList(move_p, move_n2, move_p2, move_n, length, positions);
            break;
        case DIR_LEFT:
            appendPositionList(move_n, move_p2, move_n2, move_p, length, positions);
            break;
        case DIR_FORWARD_RIGHT:
            appendPositionList(move_p, move_ps2, move_p2, move_ps, length, positions);
            break;
        case DIR_FORWARD_LEFT:
            appendPositionList(move_ps, move_p2, move_ps2, move_p, length, positions);
            break;
        default: // DIR_BACKWARD
            appendPositionList(move_n, move_n2, move_n2, move_n, length, positions);
            break;
    }
}

void Q8GaitGenerator::stackWalk(Direction dir, int s1_count, int s2_count, float move_p[][2], 
                               float move_n[][2], int length, float positions[][8]) {
    
    float len_factor = (float)(s1_count + s2_count) / s1_count;
    int split = (int)(s1_count * len_factor / 4);
    
    // Create phase-shifted trajectories for 4-phase gait
    float move_p2[MAX_GAIT_LENGTH][2], move_p3[MAX_GAIT_LENGTH][2], move_p4[MAX_GAIT_LENGTH][2];
    float move_n2[MAX_GAIT_LENGTH][2], move_n3[MAX_GAIT_LENGTH][2], move_n4[MAX_GAIT_LENGTH][2];
    
    for (int i = 0; i < length; i++) {
        int idx2 = (i + split) % length;
        int idx3 = (i + split * 2) % length;
        int idx4 = (i + split * 3) % length;
        
        move_p2[i][0] = move_p[idx2][0]; move_p2[i][1] = move_p[idx2][1];
        move_p3[i][0] = move_p[idx3][0]; move_p3[i][1] = move_p[idx3][1];
        move_p4[i][0] = move_p[idx4][0]; move_p4[i][1] = move_p[idx4][1];
        move_n2[i][0] = move_n[idx2][0]; move_n2[i][1] = move_n[idx2][1];
        move_n3[i][0] = move_n[idx3][0]; move_n3[i][1] = move_n[idx3][1];
        move_n4[i][0] = move_n[idx4][0]; move_n4[i][1] = move_n[idx4][1];
    }
    
    switch (dir) {
        case DIR_FORWARD:
            appendPositionList(move_p, move_p2, move_p3, move_p4, length, positions);
            break;
        case DIR_RIGHT:
            appendPositionList(move_p, move_n2, move_p3, move_n4, length, positions);
            break;
        case DIR_LEFT:
            appendPositionList(move_n, move_p2, move_n3, move_p4, length, positions);
            break;
        default: // DIR_BACKWARD
            appendPositionList(move_n, move_n2, move_n3, move_n4, length, positions);
            break;
    }
}

void Q8GaitGenerator::stackBound(Direction dir, int s1_count, int s2_count, float move_p[][2], 
                                float move_n[][2], int length, float positions[][8]) {
    
    int split = (s2_count + s1_count) / 4;
    
    // Create phase-shifted trajectory for bound gait
    float move_p2[MAX_GAIT_LENGTH][2];
    float move_n2[MAX_GAIT_LENGTH][2];
    
    for (int i = 0; i < length; i++) {
        int shifted_idx = (i + split) % length;
        move_p2[i][0] = move_p[shifted_idx][0];
        move_p2[i][1] = move_p[shifted_idx][1];
        move_n2[i][0] = move_n[shifted_idx][0];
        move_n2[i][1] = move_n[shifted_idx][1];
    }
    
    if (dir == DIR_FORWARD) {
        appendPositionList(move_p, move_p, move_p2, move_p2, length, positions);
    } else {
        appendPositionList(move_n, move_n, move_n2, move_n2, length, positions);
    }
}

void Q8GaitGenerator::stackPronk(Direction dir, int s1_count, int s2_count, float move_p[][2], 
                                float move_n[][2], int length, float positions[][8]) {
    
    if (dir == DIR_FORWARD) {
        appendPositionList(move_p, move_p, move_p, move_p, length, positions);
    } else {
        appendPositionList(move_n, move_n, move_n, move_n, length, positions);
    }
}

void Q8GaitGenerator::appendPositionList(float list1[][2], float list2[][2], float list3[][2], 
                                        float list4[][2], int length, float positions[][8]) {
    
    // Robot leg layout:
    //    Front
    // list1   list2
    // list3   list4
    //    Back
    
    for (int i = 0; i < length; i++) {
        positions[i][0] = list1[i][0];  // Front-left hip
        positions[i][1] = list1[i][1];  // Front-left knee
        positions[i][2] = list2[i][0];  // Front-right hip
        positions[i][3] = list2[i][1];  // Front-right knee
        positions[i][4] = list3[i][0];  // Back-left hip
        positions[i][5] = list3[i][1];  // Back-left knee
        positions[i][6] = list4[i][0];  // Back-right hip
        positions[i][7] = list4[i][1];  // Back-right knee
    }
}

void Q8GaitGenerator::createDummyMovement(float q1, float q2, float positions[][8], int& length) {
    length = 10;
    for (int i = 0; i < length; i++) {
        for (int j = 0; j < 8; j++) {
            positions[i][j] = (j % 2 == 0) ? q1 : q2;
        }
    }
}

int Q8GaitGenerator::getGaitLength(GaitType gait) const {
    if (gait >= GAIT_COUNT) return 0;
    return gaitTable[gait].s1_count + gaitTable[gait].s2_count;
}

const char* Q8GaitGenerator::getGaitName(GaitType gait) const {
    if (gait >= GAIT_COUNT) return "UNKNOWN";
    return gaitTable[gait].name;
}

GaitParams Q8GaitGenerator::getGaitParams(GaitType gait) const {
    if (gait >= GAIT_COUNT) {
        return {"INVALID", STACK_AMBER, 0, 0, 0, 0, 0, 0, 0};
    }
    return gaitTable[gait];
}

void Q8GaitGenerator::setCustomGait(GaitType gait, GaitParams params) {
    if (gait < GAIT_COUNT) {
        gaitTable[gait] = params;
    }
}

bool Q8GaitGenerator::validateGait(GaitType gait, Direction dir) const {
    return (gait < GAIT_COUNT && dir <= DIR_STOP);
}

void Q8GaitGenerator::getIdlePosition(float positions[8]) const {
    // Set all legs to idle position
    for (int i = 0; i < 8; i += 2) {
        positions[i] = IDLE_POSITION[0];     // Hip angle
        positions[i + 1] = IDLE_POSITION[1]; // Knee angle
    }
}