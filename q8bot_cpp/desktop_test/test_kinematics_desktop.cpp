/*
  Desktop test for Q8Kinematics - Compile and run without Arduino hardware
*/

#include "arduino_mock.h"
#include "Q8Kinematics_desktop.h"
#include <iostream>
#include <iomanip>

// Test cases from original test
struct TestCase {
    float x, y;           
    float expected_q1, expected_q2;    
    bool should_succeed;  
};

TestCase test_cases[] = {
    {9.75, 43.36, 39.42, 140.58, true},    // From Python reference
    {9.75, 60.0, 72.73, 107.27, true},     
    {9.75, 25.0, 10.35, 169.65, true},     
    {0.0, 45.0, 59.11, 152.18, true},       
    {19.5, 45.0, 27.82, 120.89, true},      
    {9.75, 70.0, -1, -1, false},         
    {9.75, -10.0, -1, -1, false},        
};

int main() {
    std::cout << "Q8Kinematics Desktop Test\n";
    std::cout << "=========================\n\n";
    
    Q8Kinematics kinematics;
    
    // Test inverse kinematics
    std::cout << "--- Inverse Kinematics Tests ---\n";
    int passed = 0;
    int total = sizeof(test_cases) / sizeof(TestCase);
    
    for (int i = 0; i < total; i++) {
        TestCase& test = test_cases[i];
        
        float q1, q2;
        bool success = kinematics.ikSolve(test.x, test.y, q1, q2, true, 2);
        
        std::cout << "Test " << (i+1) << ": IK(" << test.x << ", " << test.y << ") = ";
        
        if (success && test.should_succeed) {
            std::cout << std::fixed << std::setprecision(2) 
                     << "q1=" << q1 << "°, q2=" << q2 << "°";
            
            // Check IK accuracy against Python reference
            float error_q1 = abs(q1 - test.expected_q1);
            float error_q2 = abs(q2 - test.expected_q2);
            
            if (error_q1 < 0.1 && error_q2 < 0.1) {
                std::cout << " ✓ PASS\n";
                passed++;
            } else {
                std::cout << " ✗ FAIL - Expected: (" << test.expected_q1 << ", " << test.expected_q2 
                         << "), Error: (" << error_q1 << ", " << error_q2 << ")\n";
            }
        } else if (!success && !test.should_succeed) {
            std::cout << " ✓ PASS (correctly failed)\n";
            passed++;
        } else {
            std::cout << " ✗ FAIL - Expected success: " << test.should_succeed 
                     << ", Got: " << success << "\n";
        }
    }
    
    std::cout << "\nIK Tests: " << passed << "/" << total << " passed\n\n";
    
    // Test workspace validation
    std::cout << "--- Workspace Validation Tests ---\n";
    int workspace_passed = 0;
    int workspace_total = 0;
    
    // Valid positions
    float valid_pos[][2] = {{9.75, 43.36}, {0, 45}, {19.5, 45}};
    for (int i = 0; i < 3; i++) {
        workspace_total++;
        bool valid = kinematics.ikCheck(valid_pos[i][0], valid_pos[i][1]);
        std::cout << "Valid pos (" << valid_pos[i][0] << ", " << valid_pos[i][1] << "): ";
        if (valid) {
            std::cout << "✓ PASS\n";
            workspace_passed++;
        } else {
            std::cout << "✗ FAIL\n";
        }
    }
    
    // Invalid positions  
    float invalid_pos[][2] = {{9.75, -10}, {9.75, 80}};
    for (int i = 0; i < 2; i++) {
        workspace_total++;
        bool valid = kinematics.ikCheck(invalid_pos[i][0], invalid_pos[i][1]);
        std::cout << "Invalid pos (" << invalid_pos[i][0] << ", " << invalid_pos[i][1] << "): ";
        if (!valid) {
            std::cout << "✓ PASS\n";
            workspace_passed++;
        } else {
            std::cout << "✗ FAIL\n";
        }
    }
    
    std::cout << "\nWorkspace Tests: " << workspace_passed << "/" << workspace_total << " passed\n\n";
    
    // Robot parameters test
    std::cout << "--- Robot Parameters ---\n";
    std::cout << "d: " << kinematics.getD() << " mm\n";
    std::cout << "l1: " << kinematics.getL1() << " mm\n"; 
    std::cout << "l2: " << kinematics.getL2() << " mm\n";
    
    // Overall result
    int total_passed = passed + workspace_passed;
    int total_tests = total + workspace_total;
    
    std::cout << "\n=========================\n";
    std::cout << "OVERALL RESULT: " << total_passed << "/" << total_tests << " tests passed\n";
    
    if (total_passed == total_tests) {
        std::cout << "🎉 ALL TESTS PASSED! Kinematics implementation is working correctly.\n";
        return 0;
    } else {
        std::cout << "❌ Some tests failed. Check implementation.\n";
        return 1;
    }
}