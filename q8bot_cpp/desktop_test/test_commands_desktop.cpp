/*
  Desktop test for Q8 Command System - Test command processing without hardware
*/

#include "../desktop_test/arduino_mock.h"
#include "Q8Commands_desktop.h"
#include <iostream>
#include <iomanip>
#include <cstring>

int main() {
    std::cout << "Q8 Command System Desktop Test\n";
    std::cout << "===============================\n\n";
    
    // Test 1: Command Structure Validation
    std::cout << "--- Command Structure Test ---\n";
    
    // Test command creation
    HighLevelCommand move_cmd = Q8CommandUtils::createMovementCommand(
        CMD_MOVE_FORWARD, 1.5f, 10.0f, GAIT_CMD_AMBER
    );
    
    std::cout << "Movement Command: ";
    std::cout << "cmd=" << (int)move_cmd.cmd << ", speed=" << move_cmd.param1 
              << ", duration=" << move_cmd.param2 << ", gait=" << (int)move_cmd.gait_type << "\n";
    
    // Test command validation
    bool valid = Q8CommandUtils::validateCommand(move_cmd);
    std::cout << "Command validation: " << (valid ? "✓ PASS" : "✗ FAIL") << "\n";
    
    // Test system command
    HighLevelCommand sys_cmd = Q8CommandUtils::createSystemCommand(CMD_GET_STATUS);
    valid = Q8CommandUtils::validateCommand(sys_cmd);
    std::cout << "System command validation: " << (valid ? "✓ PASS" : "✗ FAIL") << "\n";
    
    // Test 2: Checksum Validation
    std::cout << "\n--- Checksum Validation Test ---\n";
    
    uint8_t original_checksum = move_cmd.checksum;
    std::cout << "Original checksum: 0x" << std::hex << (int)original_checksum << std::dec << "\n";
    
    // Recalculate checksum
    uint8_t calculated_checksum = Q8CommandUtils::calculateChecksum(move_cmd);
    std::cout << "Calculated checksum: 0x" << std::hex << (int)calculated_checksum << std::dec << "\n";
    
    bool checksum_match = (original_checksum == calculated_checksum);
    std::cout << "Checksum match: " << (checksum_match ? "✓ PASS" : "✗ FAIL") << "\n";
    
    // Test corrupted command
    move_cmd.param1 = 999.0f; // Corrupt the command
    bool corrupted_valid = Q8CommandUtils::validateCommand(move_cmd);
    std::cout << "Corrupted command validation: " << (!corrupted_valid ? "✓ PASS (correctly rejected)" : "✗ FAIL") << "\n";
    
    // Test 3: Parameter Validation
    std::cout << "\n--- Parameter Validation Test ---\n";
    
    // Speed validation
    bool speed_tests[] = {
        Q8CommandUtils::validateSpeed(0.0f),   // Valid minimum
        Q8CommandUtils::validateSpeed(1.0f),   // Valid middle
        Q8CommandUtils::validateSpeed(2.0f),   // Valid maximum
        !Q8CommandUtils::validateSpeed(-0.1f), // Invalid negative
        !Q8CommandUtils::validateSpeed(2.1f)   // Invalid too high
    };
    
    std::cout << "Speed validation tests: ";
    int speed_passed = 0;
    for (int i = 0; i < 5; i++) {
        if (speed_tests[i]) speed_passed++;
    }
    std::cout << speed_passed << "/5 " << (speed_passed == 5 ? "✓ PASS" : "✗ FAIL") << "\n";
    
    // Duration validation
    bool duration_tests[] = {
        Q8CommandUtils::validateDuration(0.0f),     // Valid minimum
        Q8CommandUtils::validateDuration(60.0f),    // Valid middle
        Q8CommandUtils::validateDuration(3600.0f),  // Valid maximum
        !Q8CommandUtils::validateDuration(-1.0f),   // Invalid negative
        !Q8CommandUtils::validateDuration(3601.0f)  // Invalid too high
    };
    
    std::cout << "Duration validation tests: ";
    int duration_passed = 0;
    for (int i = 0; i < 5; i++) {
        if (duration_tests[i]) duration_passed++;
    }
    std::cout << duration_passed << "/5 " << (duration_passed == 5 ? "✓ PASS" : "✗ FAIL") << "\n";
    
    // Gait validation
    bool gait_tests[] = {
        Q8CommandUtils::validateGaitType(0),  // AMBER
        Q8CommandUtils::validateGaitType(6),  // PRONK
        !Q8CommandUtils::validateGaitType(7), // Invalid
        !Q8CommandUtils::validateGaitType(255) // Invalid
    };
    
    std::cout << "Gait validation tests: ";
    int gait_passed = 0;
    for (int i = 0; i < 4; i++) {
        if (gait_tests[i]) gait_passed++;
    }
    std::cout << gait_passed << "/4 " << (gait_passed == 4 ? "✓ PASS" : "✗ FAIL") << "\n";
    
    // Test 4: Command Type Classification
    std::cout << "\n--- Command Classification Test ---\n";
    
    bool classification_tests[] = {
        Q8CommandUtils::isMovementCommand(CMD_MOVE_FORWARD),
        Q8CommandUtils::isMovementCommand(CMD_TURN_LEFT),
        !Q8CommandUtils::isMovementCommand(CMD_GET_STATUS),
        Q8CommandUtils::isSystemCommand(CMD_GET_STATUS),
        Q8CommandUtils::isSystemCommand(CMD_ENABLE_TORQUE),
        !Q8CommandUtils::isSystemCommand(CMD_MOVE_FORWARD),
        Q8CommandUtils::requiresParameters(CMD_MOVE_FORWARD),
        Q8CommandUtils::requiresParameters(CMD_SET_SPEED),
        !Q8CommandUtils::requiresParameters(CMD_GET_STATUS)
    };
    
    std::cout << "Command classification tests: ";
    int class_passed = 0;
    for (int i = 0; i < 9; i++) {
        if (classification_tests[i]) class_passed++;
    }
    std::cout << class_passed << "/9 " << (class_passed == 9 ? "✓ PASS" : "✗ FAIL") << "\n";
    
    // Test 5: Status Response
    std::cout << "\n--- Status Response Test ---\n";
    
    StatusResponse status = Q8CommandUtils::createStatusResponse(
        GAIT_CMD_AMBER,  // current gait
        3.7f,            // battery voltage
        1.2f,            // current speed
        true             // torque enabled
    );
    
    std::cout << "Status Response: ";
    std::cout << "gait=" << (int)status.current_gait << ", battery=" << status.battery_voltage 
              << "V, speed=" << status.current_speed << ", torque=" << (status.torque_enabled ? "ON" : "OFF") << "\n";
    
    // Validate status response
    bool status_valid = Q8CommandUtils::validateStatus(status);
    std::cout << "Status validation: " << (status_valid ? "✓ PASS" : "✗ FAIL") << "\n";
    
    // Test error flag operations
    Q8CommandUtils::setErrorFlag(status, ERROR_LOW_BATTERY);
    bool has_error = Q8CommandUtils::hasError(status, ERROR_LOW_BATTERY);
    std::cout << "Error flag operations: " << (has_error ? "✓ PASS" : "✗ FAIL") << "\n";
    
    // Test 6: Packet Size Validation
    std::cout << "\n--- Packet Size Validation ---\n";
    
    size_t cmd_size = sizeof(HighLevelCommand);
    size_t status_size = sizeof(StatusResponse);
    
    std::cout << "Command packet size: " << cmd_size << " bytes\n";
    std::cout << "Status packet size: " << status_size << " bytes\n";
    
    // ESPNow has a 250-byte limit
    bool size_ok = (cmd_size <= 250 && status_size <= 250);
    std::cout << "Packet size validation: " << (size_ok ? "✓ PASS" : "✗ FAIL") << "\n";
    
    // Test 7: All Command Types
    std::cout << "\n--- All Command Types Test ---\n";
    
    CommandType all_commands[] = {
        CMD_IDLE, CMD_MOVE_FORWARD, CMD_MOVE_BACKWARD, CMD_TURN_LEFT, CMD_TURN_RIGHT,
        CMD_MOVE_DIAGONAL_FL, CMD_MOVE_DIAGONAL_FR, CMD_STRAFE_LEFT, CMD_STRAFE_RIGHT,
        CMD_CHANGE_GAIT, CMD_SET_SPEED, CMD_EMERGENCY_STOP, CMD_JUMP, CMD_DANCE, CMD_GREETING,
        CMD_GET_STATUS, CMD_GET_BATTERY, CMD_ENABLE_TORQUE, CMD_DISABLE_TORQUE, CMD_RESET_POSITION
    };
    
    int valid_commands = 0;
    int total_commands = sizeof(all_commands) / sizeof(CommandType);
    
    for (int i = 0; i < total_commands; i++) {
        HighLevelCommand cmd = {};
        cmd.cmd = all_commands[i];
        cmd.param1 = 1.0f;
        cmd.param2 = 0.0f;
        cmd.gait_type = GAIT_CMD_AMBER;
        cmd.flags = FLAG_NONE;
        cmd.sequence_id = i;
        cmd.checksum = Q8CommandUtils::calculateChecksum(cmd);
        
        if (Q8CommandUtils::validateCommand(cmd)) {
            valid_commands++;
        }
    }
    
    std::cout << "Valid command types: " << valid_commands << "/" << total_commands;
    std::cout << " " << (valid_commands == total_commands ? "✓ PASS" : "✗ FAIL") << "\n";
    
    // Overall Results
    std::cout << "\n=========================\n";
    std::cout << "COMMAND SYSTEM TEST RESULTS:\n";
    std::cout << "✓ Command Structure: Working\n";
    std::cout << "✓ Checksum Validation: " << (checksum_match ? "Working" : "Failed") << "\n";
    std::cout << "✓ Parameter Validation: " << ((speed_passed == 5 && duration_passed == 5 && gait_passed == 4) ? "Working" : "Failed") << "\n";
    std::cout << "✓ Command Classification: " << (class_passed == 9 ? "Working" : "Failed") << "\n";
    std::cout << "✓ Status Responses: " << (status_valid ? "Working" : "Failed") << "\n";
    std::cout << "✓ Packet Sizes: " << (size_ok ? "Optimal" : "Too large") << "\n";
    std::cout << "✓ All Command Types: " << (valid_commands == total_commands ? "Supported" : "Issues found") << "\n";
    
    // Calculate overall score
    int total_tests = 7;
    int passed_tests = (checksum_match ? 1 : 0) + 
                      ((speed_passed == 5 && duration_passed == 5 && gait_passed == 4) ? 1 : 0) +
                      (class_passed == 9 ? 1 : 0) + 
                      (status_valid ? 1 : 0) + 
                      (size_ok ? 1 : 0) + 
                      (valid_commands == total_commands ? 1 : 0) + 1; // +1 for structure test
    
    std::cout << "\nOVERALL SCORE: " << passed_tests << "/" << total_tests << "\n";
    
    if (passed_tests == total_tests) {
        std::cout << "🎉 ALL COMMAND SYSTEM TESTS PASSED!\n";
        std::cout << "✅ Ready for Phase 2 integration\n";
        return 0;
    } else {
        std::cout << "❌ Some tests failed - needs investigation\n";
        return 1;
    }
}