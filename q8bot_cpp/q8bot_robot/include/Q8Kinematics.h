/*
  Q8Kinematics.h - Forward and Inverse Kinematics solver for Q8bot
  Ported from Python kinematics_solver.py by yufeng.wu0902@gmail.com
  Created for autonomous controller implementation
*/

#ifndef Q8Kinematics_h
#define Q8Kinematics_h

#include <Arduino.h>
#include <math.h>

class Q8Kinematics {
public:
    // Constructor with default Q8bot parameters (in mm)
    Q8Kinematics(float d = 19.5, float l1 = 25.0, float l2 = 40.0, float l1p = 25.0, float l2p = 40.0);
    
    // Inverse kinematics: (x,y) position -> (q1,q2) joint angles
    bool ikSolve(float x, float y, float& q1, float& q2, bool degrees = true, int rounding = 3);
    
    // Forward kinematics: (q1,q2) joint angles -> (x,y) position  
    bool fkSolve(float q1, float q2, float& x, float& y, bool degrees = true, int rounding = 3);
    
    // Workspace validation
    bool ikCheck(float x, float y);
    bool fkCheck(float q1, float q2);
    
    // Get robot parameters
    float getD() const { return d; }
    float getL1() const { return l1; }
    float getL2() const { return l2; }
    float getL1p() const { return l1p; }
    float getL2p() const { return l2p; }
    
    // Get previous valid solution (for error recovery)
    void getPrevIK(float& q1, float& q2) const { q1 = prev_ik[0]; q2 = prev_ik[1]; }
    void getPrevFK(float& x, float& y) const { x = prev_fk[0]; y = prev_fk[1]; }

private:
    // Robot geometric parameters (mm)
    float d;        // Distance between motors
    float l1, l2;   // Upper and lower linkage lengths (left side)
    float l1p, l2p; // Upper and lower linkage lengths (right side)
    
    // Previous solutions for error recovery
    float prev_ik[2];   // Previous IK solution [q1, q2] in degrees
    float prev_fk[2];   // Previous FK solution [x, y] in mm
    
    // Helper functions
    float deg2rad(float degrees) const { return degrees * M_PI / 180.0; }
    float rad2deg(float radians) const { return radians * 180.0 / M_PI; }
    float roundToDecimal(float value, int decimals) const;
    
    // Forward kinematics calculation function for numerical solver
    void fkCalc(float x, float y, float q1, float q2, float& f1, float& f2) const;
    
    // Simple numerical solver for FK (replaces scipy.optimize.fsolve)
    bool numericalSolve(float q1, float q2, float& x, float& y, float initial_x = 10.0, float initial_y = 60.0) const;
};

#endif