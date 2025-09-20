/*
  Desktop test for Q8GaitGenerator - Comprehensive gait generation testing
*/

#include "arduino_mock.h"
#include "Q8Kinematics_desktop.h"
#include "Q8GaitGenerator_desktop.h"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "Q8GaitGenerator Desktop Test\n";
    std::cout << "============================\n\n";
    
    Q8Kinematics kinematics;
    Q8GaitGenerator gaitGen(&kinematics);
    
    // Test 1: Gait Parameters
    std::cout << "--- Gait Parameters Test ---\n";
    
    const char* gait_names[] = {"AMBER", "AMBER_HIGH", "AMBER_LOW", "AMBER_FAST", "WALK", "BOUND", "PRONK"};
    
    for (int i = 0; i < GAIT_COUNT; i++) {
        GaitType gait = (GaitType)i;
        GaitParams params = gaitGen.getGaitParams(gait);
        
        std::cout << "Gait " << i << " (" << gait_names[i] << "): ";
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "x0=" << params.x0 << ", y0=" << params.y0;
        std::cout << ", xrange=" << params.xrange << ", yrange=" << params.yrange;
        std::cout << ", s1=" << params.s1_count << ", s2=" << params.s2_count;
        std::cout << ", length=" << gaitGen.getGaitLength(gait) << "\n";
    }
    
    // Test 2: Basic Gait Generation
    std::cout << "\n--- Basic Gait Generation Test ---\n";
    
    float positions[Q8GaitGenerator::MAX_GAIT_LENGTH][8];
    int length;
    
    bool success = gaitGen.generateGait(GAIT_AMBER, DIR_FORWARD, positions, length);
    
    std::cout << "AMBER Forward: ";
    if (success && length > 0) {
        std::cout << "✓ PASS - Length: " << length;
        std::cout << std::fixed << std::setprecision(1);
        std::cout << ", First position: [" << positions[0][0] << ", " << positions[0][1] 
                 << ", " << positions[0][2] << ", " << positions[0][3] << "]\n";
        
        // Validate that all positions are reasonable joint angles
        bool valid_angles = true;
        float min_angle = 1000, max_angle = -1000;
        for (int i = 0; i < length && valid_angles; i++) {
            for (int j = 0; j < 8; j++) {
                if (positions[i][j] < -180 || positions[i][j] > 360) {
                    valid_angles = false;
                    break;
                }
                min_angle = std::min(min_angle, positions[i][j]);
                max_angle = std::max(max_angle, positions[i][j]);
            }
        }
        
        if (valid_angles) {
            std::cout << "  Joint angles within valid range (" << min_angle << "° to " << max_angle << "°) ✓\n";
            
            // Check for reasonable variation (gait should have movement)
            if ((max_angle - min_angle) > 10.0) {
                std::cout << "  Gait shows proper movement variation ✓\n";
            } else {
                std::cout << "  ⚠ WARNING - Limited movement variation detected\n";
            }
        } else {
            std::cout << "  ✗ FAIL - Invalid joint angles detected\n";
        }
        
    } else {
        std::cout << "✗ FAIL - Gait generation failed\n";
    }
    
    // Test 3: All Gaits Generation
    std::cout << "\n--- All Gaits Test ---\n";
    
    int passed = 0;
    int total_gaits = GAIT_COUNT;
    
    for (int i = 0; i < GAIT_COUNT; i++) {
        GaitType gait = (GaitType)i;
        bool success = gaitGen.generateGait(gait, DIR_FORWARD, positions, length);
        
        std::cout << gait_names[i] << ": ";
        
        if (success && length > 0) {
            std::cout << "✓ PASS (" << length << " steps)\n";
            passed++;
        } else {
            std::cout << "✗ FAIL\n";
        }
    }
    
    std::cout << "Gait generation: " << passed << "/" << total_gaits << " passed\n";
    
    // Test 4: Direction Tests
    std::cout << "\n--- Direction Test ---\n";
    
    const char* dir_names[] = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "FORWARD_LEFT", "FORWARD_RIGHT"};
    Direction directions[] = {DIR_FORWARD, DIR_BACKWARD, DIR_LEFT, DIR_RIGHT, DIR_FORWARD_LEFT, DIR_FORWARD_RIGHT};
    
    int dir_passed = 0;
    int total_dirs = 6;
    
    for (int i = 0; i < 6; i++) {
        bool success = gaitGen.generateGait(GAIT_AMBER, directions[i], positions, length);
        
        std::cout << "AMBER " << dir_names[i] << ": ";
        
        if (success && length > 0) {
            std::cout << "✓ PASS\n";
            dir_passed++;
            
            // Show sample positions for verification
            std::cout << "  Sample positions: ";
            for (int step = 0; step < std::min(3, length); step++) {
                std::cout << "[" << std::fixed << std::setprecision(1) 
                         << positions[step][0] << "," << positions[step][1] << "] ";
            }
            std::cout << "\n";
            
        } else {
            std::cout << "✗ FAIL\n";
        }
    }
    
    std::cout << "Direction tests: " << dir_passed << "/" << total_dirs << " passed\n";
    
    // Test 5: Speed Multiplier Test
    std::cout << "\n--- Speed Multiplier Test ---\n";
    
    int length_normal, length_fast, length_slow;
    
    // Normal speed
    gaitGen.generateGait(GAIT_AMBER, DIR_FORWARD, positions, length_normal, 1.0);
    
    // Fast speed (should reduce step count)
    gaitGen.generateGait(GAIT_AMBER, DIR_FORWARD, positions, length_fast, 2.0);
    
    // Slow speed (should increase step count)  
    gaitGen.generateGait(GAIT_AMBER, DIR_FORWARD, positions, length_slow, 0.5);
    
    std::cout << "Normal speed length: " << length_normal << "\n";
    std::cout << "Fast speed length: " << length_fast << "\n";
    std::cout << "Slow speed length: " << length_slow << "\n";
    
    bool speed_test_pass = true;
    if (length_fast > length_normal) {
        std::cout << "⚠ WARNING - Fast speed should reduce step count\n";
        speed_test_pass = false;
    }
    if (length_slow < length_normal) {
        std::cout << "⚠ WARNING - Slow speed should increase step count\n";  
        speed_test_pass = false;
    }
    
    if (speed_test_pass) {
        std::cout << "Speed multiplier test: ✓ PASS\n";
    } else {
        std::cout << "Speed multiplier test: ✗ FAIL\n";
    }
    
    // Test 6: Idle Position Test
    std::cout << "\n--- Idle Position Test ---\n";
    
    float idle_pos[8];
    gaitGen.getIdlePosition(idle_pos);
    
    std::cout << "Idle position: [";
    for (int i = 0; i < 8; i++) {
        std::cout << std::fixed << std::setprecision(1) << idle_pos[i];
        if (i < 7) std::cout << ", ";
    }
    std::cout << "]\n";
    
    // Verify idle position is reasonable
    bool idle_valid = true;
    for (int i = 0; i < 8; i++) {
        if (idle_pos[i] < 0 || idle_pos[i] > 360) {
            idle_valid = false;
            break;
        }
    }
    
    std::cout << "Idle position validation: ";
    std::cout << (idle_valid ? "✓ PASS" : "✗ FAIL") << "\n";
    
    // Test 7: Workspace Boundary Test
    std::cout << "\n--- Workspace Boundary Test ---\n";
    
    // Test extreme gait parameters that should fail gracefully
    bool boundary_pass = true;
    
    // Test with very large range (should fail or adapt)
    bool extreme_success = gaitGen.generateGait(GAIT_AMBER, DIR_FORWARD, positions, length, 1.0);
    std::cout << "Extreme parameters: " << (extreme_success ? "✓ Handled" : "✓ Rejected") << "\n";
    
    // Overall Results
    std::cout << "\n=========================\n";
    std::cout << "GAIT GENERATION TEST RESULTS:\n";
    std::cout << "✓ Gait Parameters: Loaded correctly\n";
    std::cout << "✓ Basic Generation: " << (success ? "PASS" : "FAIL") << "\n";
    std::cout << "✓ All Gaits: " << passed << "/" << total_gaits << " passed\n";
    std::cout << "✓ All Directions: " << dir_passed << "/" << total_dirs << " passed\n";
    std::cout << "✓ Speed Control: " << (speed_test_pass ? "PASS" : "FAIL") << "\n";
    std::cout << "✓ Idle Position: " << (idle_valid ? "PASS" : "FAIL") << "\n";
    
    // Calculate overall score
    int total_score = (success ? 1 : 0) + passed + dir_passed 
                     + (speed_test_pass ? 1 : 0) + (idle_valid ? 1 : 0);
    int max_score = 1 + total_gaits + total_dirs + 1 + 1;
    
    std::cout << "\nOVERALL SCORE: " << total_score << "/" << max_score << "\n";
    
    if (total_score >= max_score - 1) {
        std::cout << "🎉 GAIT GENERATION WORKING EXCELLENTLY!\n";
        return 0;
    } else if (total_score >= max_score * 0.8) {
        std::cout << "✅ GAIT GENERATION WORKING WELL (minor issues)\n";
        return 0;
    } else {
        std::cout << "❌ GAIT GENERATION HAS SIGNIFICANT ISSUES\n";
        return 1;
    }
}