# Makefile for Q8bot Desktop Testing
# Compile and test C++ implementations without Arduino hardware

CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -I.
LDFLAGS = -lm

# Source files
KINEMATICS_SOURCES = arduino_mock.cpp Q8Kinematics_desktop.cpp
GAIT_SOURCES = arduino_mock.cpp Q8Kinematics_desktop.cpp Q8GaitGenerator_desktop.cpp
COMMAND_SOURCES = arduino_mock.cpp Q8Commands_desktop.cpp
INTEGRATION_SOURCES = arduino_mock.cpp Q8Kinematics_desktop.cpp Q8GaitGenerator_desktop.cpp Q8Commands_desktop.cpp Q8MotionController_desktop.cpp
HEADERS = arduino_mock.h Q8Kinematics_desktop.h Q8GaitGenerator_desktop.h Q8Commands_desktop.h Q8MotionController_desktop.h

# Test executables
TESTS = test_kinematics test_gait_generation test_commands test_integration

.PHONY: all clean test test-kinematics test-gait test-commands test-integration phase1 phase2 phase3

all: $(TESTS)

test_kinematics: $(KINEMATICS_SOURCES) test_kinematics_desktop.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test_gait_generation: $(GAIT_SOURCES) test_gait_generation_desktop.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test_commands: $(COMMAND_SOURCES) test_commands_desktop.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test_integration: $(INTEGRATION_SOURCES) test_integration_desktop.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test: $(TESTS)
	@echo "Running Q8bot Phase 1, 2 & 3 Tests..."
	@echo "====================================="
	@echo ""
	@echo "=== PHASE 1 TESTS ==="
	@echo "Testing Kinematics..."
	./test_kinematics
	@echo ""
	@echo "Testing Gait Generation..."
	./test_gait_generation
	@echo ""
	@echo "=== PHASE 2 TESTS ==="
	@echo "Testing Command System..."
	./test_commands
	@echo ""
	@echo "=== PHASE 3 TESTS ==="
	@echo "Testing Autonomous Integration..."
	./test_integration
	@echo ""
	@echo "All tests completed!"

phase1: test_kinematics test_gait_generation
	@echo "Running Phase 1 Tests Only..."
	@echo "============================="
	@echo ""
	@echo "Testing Kinematics..."
	./test_kinematics
	@echo ""
	@echo "Testing Gait Generation..."
	./test_gait_generation
	@echo ""
	@echo "Phase 1 tests completed!"

phase2: test_commands
	@echo "Running Phase 2 Tests Only..."
	@echo "============================="
	@echo ""
	@echo "Testing Command System..."
	./test_commands
	@echo ""
	@echo "Phase 2 tests completed!"

phase3: test_integration
	@echo "Running Phase 3 Tests Only..."
	@echo "============================="
	@echo ""
	@echo "Testing Autonomous Integration..."
	./test_integration
	@echo ""
	@echo "Phase 3 tests completed!"

test-kinematics: test_kinematics
	@echo "Testing Kinematics Only..."
	./test_kinematics

test-gait: test_gait_generation  
	@echo "Testing Gait Generation Only..."
	./test_gait_generation

test-commands: test_commands
	@echo "Testing Command System Only..."
	./test_commands

test-integration: test_integration
	@echo "Testing Autonomous Integration Only..."
	./test_integration

clean:
	rm -f $(TESTS)

help:
	@echo "Q8bot Desktop Testing Makefile"
	@echo "=============================="
	@echo ""
	@echo "Targets:"
	@echo "  all          - Build all test executables"
	@echo "  test         - Run all Phase 1, 2 & 3 tests" 
	@echo "  phase1       - Run Phase 1 tests only (kinematics + gait)"
	@echo "  phase2       - Run Phase 2 tests only (commands)"
	@echo "  phase3       - Run Phase 3 tests only (integration)"
	@echo "  test-kinematics - Run kinematics tests only"
	@echo "  test-gait    - Run gait generation tests only"
	@echo "  test-commands - Run command system tests only"
	@echo "  test-integration - Run integration tests only"
	@echo "  clean        - Remove build artifacts"
	@echo "  help         - Show this help message"
	@echo ""
	@echo "Usage:"
	@echo "  make test         # Run all tests"
	@echo "  make phase3       # Test Phase 3 only"
	@echo "  make test-integration # Test integration only"
	@echo "  make clean        # Clean up"