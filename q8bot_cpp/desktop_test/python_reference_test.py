#!/usr/bin/env python3
"""
Python reference test to validate our C++ implementation
"""

import sys
import os
sys.path.append('/Users/menggg/Downloads/q8bot/q8bot_python/q8bot')

from kinematics_solver import k_solver
import math

# Test cases
test_cases = [
    (9.75, 43.36),
    (9.75, 60.0),
    (9.75, 25.0),
    (0.0, 45.0),
    (19.5, 45.0),
]

print("Python Reference Kinematics Test")
print("================================")

leg = k_solver(19.5, 25, 40, 25, 40)

print("\n--- Inverse Kinematics Reference ---")
for i, (x, y) in enumerate(test_cases):
    q1, q2, success = leg.ik_solve(x, y, deg=True, rounding=2)
    print(f"Test {i+1}: IK({x}, {y}) = q1={q1:.2f}°, q2={q2:.2f}° (success: {success})")
    
    # Verify with forward kinematics
    if success:
        x_check, y_check = leg.fk_solve(q1, q2, deg=True, rounding=2)
        error_x = abs(x_check - x)
        error_y = abs(y_check - y)
        print(f"         FK check: ({x_check}, {y_check}), error: ({error_x:.2f}, {error_y:.2f})")

print("\n--- Forward Kinematics Reference ---")
fk_test_cases = [
    (45.0, 135.0),
    (30.0, 150.0),
    (60.0, 120.0),
    (90.0, 90.0)
]

for i, (q1, q2) in enumerate(fk_test_cases):
    x, y = leg.fk_solve(q1, q2, deg=True, rounding=2)
    print(f"Test {i+1}: FK({q1}°, {q2}°) = ({x}, {y})")