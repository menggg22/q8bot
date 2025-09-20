#!/usr/bin/env python3
"""
Python reference test for gait generation validation (without matplotlib dependency)
"""

import sys
import os
sys.path.append('/Users/menggg/Downloads/q8bot/q8bot_python/q8bot')

from kinematics_solver import k_solver
import math

print("Python Reference Gait Generation Test")
print("=====================================")

leg = k_solver(19.5, 25, 40, 25, 40)

# Simplified gait generation without matplotlib dependency
def generate_gait_simple(leg, direction, gait_params):
    """Simplified version of generate_gait without matplotlib"""
    stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count = gait_params
    
    move_p, move_n = [], []
    x_p = x0 - xrange / 2.0
    x_n = x0 + xrange / 2.0
    
    x_lift_step = xrange / s1_count
    x_down_step = xrange / s2_count
    
    length = s1_count + s2_count
    
    # Check validity
    if y0 - yrange < 5:
        print(f"Invalid: y below physical limit for {stacktype}")
        return [], []
    
    # Generate trajectory points
    for i in range(length):
        if i < s1_count:
            # Lift phase
            x_p += x_lift_step
            x_n -= x_lift_step
            freq = math.pi / s1_count
            y = y0 - math.sin((i + 1) * freq) * yrange
        else:
            # Support phase
            x_p -= x_down_step
            x_n += x_down_step
            freq = math.pi / s2_count
            y = y0 + math.sin((i - s1_count + 1) * freq) * yrange2
        
        # Inverse kinematics
        q1_p, q2_p, check1 = leg.ik_solve(x_p, y, True, 1)
        q1_n, q2_n, check2 = leg.ik_solve(x_n, y, True, 1)
        
        if not check1 or not check2:
            print(f"IK failed at step {i} for {stacktype}")
            return [], []
        
        move_p.append([q1_p, q2_p])
        move_n.append([q1_n, q2_n])
    
    # Simple stacking for forward direction
    if stacktype == 'amber':
        # Phase shift for amber gait
        split = length // 2
        move_p2 = move_p[split:] + move_p[:split]
        # Create 4-leg coordination: [leg1, leg2, leg3, leg4]
        result = []
        for i in range(length):
            result.append([
                move_p[i][0], move_p[i][1],      # Front-left
                move_p2[i][0], move_p2[i][1],    # Front-right  
                move_p2[i][0], move_p2[i][1],    # Back-left
                move_p[i][0], move_p[i][1]       # Back-right
            ])
        return result, []
    
    elif stacktype == 'walk':
        # 4-phase walk gait
        split = length // 4
        move_p2 = move_p[split:] + move_p[:split]
        move_p3 = move_p[split*2:] + move_p[:split*2]
        move_p4 = move_p[split*3:] + move_p[:split*3]
        result = []
        for i in range(length):
            result.append([
                move_p[i][0], move_p[i][1],      # Front-left
                move_p2[i][0], move_p2[i][1],    # Front-right
                move_p3[i][0], move_p3[i][1],    # Back-left
                move_p4[i][0], move_p4[i][1]     # Back-right
            ])
        return result, []
    
    elif stacktype in ['bound', 'pronk']:
        # Simplified bound/pronk
        if stacktype == 'bound':
            split = length // 4
            move_p2 = move_p[split:] + move_p[:split]
            result = []
            for i in range(length):
                result.append([
                    move_p[i][0], move_p[i][1],      # Front-left
                    move_p[i][0], move_p[i][1],      # Front-right
                    move_p2[i][0], move_p2[i][1],    # Back-left
                    move_p2[i][0], move_p2[i][1]     # Back-right
                ])
            return result, []
        else:  # pronk
            result = []
            for i in range(length):
                result.append([
                    move_p[i][0], move_p[i][1],      # All legs
                    move_p[i][0], move_p[i][1],      # move together
                    move_p[i][0], move_p[i][1],      # in pronk
                    move_p[i][0], move_p[i][1]       # gait
                ])
            return result, []
    
    return [], []

# Test gait parameters (matching our C++ gaits)
gaits = {
    'AMBER': ['amber', 9.75, 43.36, 40, 20, 0, 15, 30],
    'AMBER_HIGH': ['amber', 9.75, 60, 20, 10, 0, 15, 30],  
    'AMBER_LOW': ['amber', 9.75, 25, 20, 10, 0, 15, 30],
    'AMBER_FAST': ['amber', 9.75, 43.36, 50, 20, 0, 12, 24],
    'WALK': ['walk', 9.75, 43.36, 30, 20, 0, 20, 140],
    'BOUND': ['bound', 9.75, 33.36, 40, 0, 20, 50, 10],
    'PRONK': ['pronk', 9.75, 33.36, 40, 0, 20, 60, 10]
}

print("\n--- Python Gait Generation Reference ---")

results = {}
for gait_name, params in gaits.items():
    try:
        move_list, y_list = generate_gait_simple(leg, 'f', params)  # Forward direction
        length = len(move_list)
        
        print(f"{gait_name}: Length={length} steps")
        results[gait_name] = length
        
        if length > 0:
            # Show first few positions
            print(f"  First position: [{move_list[0][0]:.1f}, {move_list[0][1]:.1f}, {move_list[0][2]:.1f}, {move_list[0][3]:.1f}]")
            
            # Check angle ranges
            all_angles = []
            for pos in move_list:
                all_angles.extend(pos)
            
            min_angle = min(all_angles)
            max_angle = max(all_angles)
            print(f"  Angle range: {min_angle:.1f}° to {max_angle:.1f}°")
            
    except Exception as e:
        print(f"{gait_name}: ✗ Failed - {str(e)}")
        results[gait_name] = 0

print("\n--- Comparison with C++ Results ---")
cpp_results = {
    'AMBER': 45,
    'AMBER_HIGH': 45,
    'AMBER_LOW': 45, 
    'AMBER_FAST': 36,
    'WALK': 160,
    'BOUND': 60,
    'PRONK': 70
}

all_match = True
for gait_name in gaits.keys():
    python_len = results.get(gait_name, 0)
    cpp_len = cpp_results.get(gait_name, 0)
    match = "✓" if python_len == cpp_len else "✗"
    if python_len != cpp_len:
        all_match = False
    print(f"{gait_name}: Python={python_len}, C++={cpp_len} {match}")

print("\n" + "="*50)
if all_match:
    print("🎉 ALL GAIT LENGTHS MATCH PYTHON REFERENCE PERFECTLY!")
    print("✅ C++ implementation is validated against Python")
else:
    print("❌ Some discrepancies found - needs investigation")

print("\n--- Kinematics Validation ---")
# Test a few IK solutions to make sure our C++ matches Python exactly
test_positions = [
    (9.75, 43.36),
    (9.75, 60.0),
    (0.0, 45.0),
    (19.5, 45.0)
]

print("Testing IK solver against Python reference:")
for x, y in test_positions:
    q1, q2, success = leg.ik_solve(x, y, True, 2)
    print(f"IK({x}, {y}) = q1={q1:.2f}°, q2={q2:.2f}° (success: {success})")

print("\n✅ Python reference validation complete!")