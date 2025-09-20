/*
  Q8Kinematics.cpp - Forward and Inverse Kinematics solver for Q8bot
  Ported from Python kinematics_solver.py by yufeng.wu0902@gmail.com
  Created for autonomous controller implementation
*/

#include "../desktop_test/Q8Kinematics_desktop.h"

Q8Kinematics::Q8Kinematics(float d, float l1, float l2, float l1p, float l2p) 
    : d(d), l1(l1), l2(l2), l1p(l1p), l2p(l2p) {
    // Initialize with reasonable default pose
    prev_ik[0] = 45.0;  // q1 in degrees
    prev_ik[1] = 135.0; // q2 in degrees
    prev_fk[0] = d/2;   // x at center
    prev_fk[1] = l1 + l2; // y fully extended
}

bool Q8Kinematics::ikSolve(float x, float y, float& q1, float& q2, bool degrees, int rounding) {
    try {
        // Calculate distances using law of cosines
        float c1 = sqrt((x - d) * (x - d) + y * y);
        float c2 = sqrt(x * x + y * y);
        
        // Check for numerical stability
        if (c1 < 0.001 || c2 < 0.001) {
            q1 = prev_ik[0];
            q2 = prev_ik[1];
            return false;
        }
        
        // Calculate intermediate angles using law of cosines
        float cos_a1_arg = (c1*c1 + d*d - c2*c2) / (2*c1*d);
        float cos_a2_arg = (c2*c2 + d*d - c1*c1) / (2*c2*d);
        float cos_b1_arg = (c1*c1 + l1*l1 - l2*l2) / (2*c1*l1);
        float cos_b2_arg = (c2*c2 + l1p*l1p - l2p*l2p) / (2*c2*l1p);
        
        // Check for valid domain ([-1, 1]) for acos
        if (abs(cos_a1_arg) > 1.0 || abs(cos_a2_arg) > 1.0 || 
            abs(cos_b1_arg) > 1.0 || abs(cos_b2_arg) > 1.0) {
            q1 = prev_ik[0];
            q2 = prev_ik[1];
            return false;
        }
        
        float a1 = acos(cos_a1_arg);
        float a2 = acos(cos_a2_arg);
        float b1 = acos(cos_b1_arg);
        float b2 = acos(cos_b2_arg);
        
        // Calculate joint angles
        float q1_rad = M_PI - a1 - b1;
        float q2_rad = a2 + b2;
        
        // Convert to degrees if requested
        if (degrees) {
            q1 = rad2deg(q1_rad);
            q2 = rad2deg(q2_rad);
        } else {
            q1 = q1_rad;
            q2 = q2_rad;
        }
        
        // Apply rounding
        q1 = roundToDecimal(q1, rounding);
        q2 = roundToDecimal(q2, rounding);
        
        // Store as previous solution
        prev_ik[0] = degrees ? q1 : rad2deg(q1);
        prev_ik[1] = degrees ? q2 : rad2deg(q2);
        
        return true;
        
    } catch (...) {
        // Return previous solution on any error
        q1 = prev_ik[0];
        q2 = prev_ik[1];
        if (!degrees) {
            q1 = deg2rad(q1);
            q2 = deg2rad(q2);
        }
        return false;
    }
}

bool Q8Kinematics::fkSolve(float q1, float q2, float& x, float& y, bool degrees, int rounding) {
    // Convert to radians if input is in degrees
    float q1_rad = degrees ? deg2rad(q1) : q1;
    float q2_rad = degrees ? deg2rad(q2) : q2;
    
    // Use numerical solver to find (x, y) position
    // Starting from a reasonable initial guess
    float initial_x = prev_fk[0];
    float initial_y = prev_fk[1];
    
    bool success = numericalSolve(q1_rad, q2_rad, x, y, initial_x, initial_y);
    
    if (success) {
        x = roundToDecimal(x, rounding);
        y = roundToDecimal(y, rounding);
        prev_fk[0] = x;
        prev_fk[1] = y;
    } else {
        x = prev_fk[0];
        y = prev_fk[1]; 
    }
    
    return success;
}

bool Q8Kinematics::ikCheck(float x, float y) {
    // Basic workspace validation
    if (y < 0) return false;
    
    // Check if point is within maximum reach
    float max_reach = l1 + l2;
    float min_reach = abs(l1 - l2);
    float distance = sqrt(x*x + y*y);
    
    if (distance > max_reach || distance < min_reach) return false;
    
    // Additional workspace checks can be added here
    return true;
}

bool Q8Kinematics::fkCheck(float q1, float q2) {
    // Basic joint limit validation
    // Q8bot typically operates in these ranges (adjust as needed)
    float q1_deg = degrees ? q1 : rad2deg(q1);
    float q2_deg = degrees ? q2 : rad2deg(q2);
    
    if (q1_deg < -180 || q1_deg > 180) return false;
    if (q2_deg < 0 || q2_deg > 360) return false;
    
    return true;
}

float Q8Kinematics::roundToDecimal(float value, int decimals) const {
    float multiplier = pow(10.0, decimals);
    return round(value * multiplier) / multiplier;
}

void Q8Kinematics::fkCalc(float x, float y, float q1, float q2, float& f1, float& f2) const {
    // Calculate the constraint equations for forward kinematics
    // This represents the system of equations that must be satisfied
    
    // Position of first joint
    float Xa = l1 * cos(q1) + d;
    float Ya = l1 * sin(q1);
    
    // Position of second joint  
    float Xb = l1p * cos(q2);
    float Yb = l1p * sin(q2);
    
    // Constraint equations (should equal zero when solved)
    f1 = (x - Xa) * (x - Xa) + (y - Ya) * (y - Ya) - l2 * l2;
    f2 = (x - Xb) * (x - Xb) + (y - Yb) * (y - Yb) - l2p * l2p;
}

bool Q8Kinematics::numericalSolve(float q1, float q2, float& x, float& y, float initial_x, float initial_y) const {
    // Simple Newton-Raphson solver (replaces scipy.optimize.fsolve)
    const int max_iterations = 50;
    const float tolerance = 1e-6;
    const float h = 1e-8; // Step size for numerical derivatives
    
    x = initial_x;
    y = initial_y;
    
    for (int iter = 0; iter < max_iterations; iter++) {
        float f1, f2;
        fkCalc(x, y, q1, q2, f1, f2);
        
        // Check convergence
        if (abs(f1) < tolerance && abs(f2) < tolerance) {
            return true;
        }
        
        // Calculate Jacobian matrix using finite differences
        float f1_dx, f1_dy, f2_dx, f2_dy;
        float f1_temp, f2_temp;
        
        // Partial derivatives with respect to x
        fkCalc(x + h, y, q1, q2, f1_temp, f2_temp);
        f1_dx = (f1_temp - f1) / h;
        f2_dx = (f2_temp - f2) / h;
        
        // Partial derivatives with respect to y  
        fkCalc(x, y + h, q1, q2, f1_temp, f2_temp);
        f1_dy = (f1_temp - f1) / h;
        f2_dy = (f2_temp - f2) / h;
        
        // Calculate determinant
        float det = f1_dx * f2_dy - f1_dy * f2_dx;
        
        if (abs(det) < 1e-12) {
            // Singular Jacobian, try different starting point
            x += 1.0;
            y += 1.0;
            continue;
        }
        
        // Newton-Raphson update
        float dx = (-f1 * f2_dy + f2 * f1_dy) / det;
        float dy = (f1 * f2_dx - f2 * f1_dx) / det;
        
        x += dx;
        y += dy;
        
        // Check for reasonable bounds
        if (x < -100 || x > 100 || y < -100 || y > 100) {
            return false;
        }
    }
    
    return false; // Failed to converge
}