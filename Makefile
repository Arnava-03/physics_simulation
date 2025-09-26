# Physics Simulation Makefile
# Builds the C++ physics engine with proper optimization and debugging flags

# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -Wpedantic -O2 -g
DEBUG_FLAGS = -std=c++17 -Wall -Wextra -Wpedantic -O0 -g -DDEBUG
INCLUDE_DIR = include
SRC_DIR = src
BUILD_DIR = build
TARGET = physics_simulation

# Source files
SOURCES = $(SRC_DIR)/main.cpp
HEADERS = $(wildcard $(INCLUDE_DIR)/*.h)

# Object files (not needed for header-only libraries, but keeping structure)
OBJECTS = $(BUILD_DIR)/main.o

# Default target
all: $(BUILD_DIR)/$(TARGET)

# Create build directory if it doesn't exist
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Build the main executable
$(BUILD_DIR)/$(TARGET): $(BUILD_DIR) $(SOURCES) $(HEADERS)
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) $(SOURCES) -o $(BUILD_DIR)/$(TARGET)

# Debug build
debug: CXXFLAGS = $(DEBUG_FLAGS)
debug: $(BUILD_DIR)/$(TARGET)

# Create object files (keeping for potential future expansion)
$(BUILD_DIR)/main.o: $(SRC_DIR)/main.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -c $(SRC_DIR)/main.cpp -o $(BUILD_DIR)/main.o

# Run the simulation
run: $(BUILD_DIR)/$(TARGET)
	cd $(BUILD_DIR) && ./$(TARGET)

# Run with specific test case
test-freefall: $(BUILD_DIR)/$(TARGET)
	cd $(BUILD_DIR) && echo "1" | ./$(TARGET)

test-projectile: $(BUILD_DIR)/$(TARGET)
	cd $(BUILD_DIR) && echo "2" | ./$(TARGET)

test-spring: $(BUILD_DIR)/$(TARGET)
	cd $(BUILD_DIR) && echo "3" | ./$(TARGET)

test-multibody: $(BUILD_DIR)/$(TARGET)
	cd $(BUILD_DIR) && echo "4" | ./$(TARGET)

test-accuracy: $(BUILD_DIR)/$(TARGET)
	cd $(BUILD_DIR) && echo "5" | ./$(TARGET)

test-all: $(BUILD_DIR)/$(TARGET)
	cd $(BUILD_DIR) && echo "6" | ./$(TARGET)

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)/*
	rm -f *.csv *.log

# Install (copy to system location - optional)
install: $(BUILD_DIR)/$(TARGET)
	cp $(BUILD_DIR)/$(TARGET) /usr/local/bin/

# Uninstall
uninstall:
	rm -f /usr/local/bin/$(TARGET)

# Show help
help:
	@echo "Available targets:"
	@echo "  all           - Build the simulation (default)"
	@echo "  debug         - Build with debug flags"
	@echo "  run           - Build and run interactive simulation"
	@echo "  test-freefall - Run free fall test"
	@echo "  test-projectile - Run projectile motion test"
	@echo "  test-spring   - Run spring-mass system test"
	@echo "  test-multibody - Run multi-body system test"
	@echo "  test-accuracy - Run accuracy vs performance test"
	@echo "  test-all      - Run all tests"
	@echo "  clean         - Remove build artifacts"
	@echo "  install       - Install to /usr/local/bin"
	@echo "  uninstall     - Remove from /usr/local/bin"
	@echo "  help          - Show this help message"

# Check for required tools
check-deps:
	@command -v $(CXX) >/dev/null 2>&1 || { echo "Error: $(CXX) is required but not installed."; exit 1; }
	@echo "All dependencies satisfied."

# Static analysis (if cppcheck is available)
analyze:
	@command -v cppcheck >/dev/null 2>&1 && \
		cppcheck --enable=all --std=c++17 -I$(INCLUDE_DIR) $(SRC_DIR)/ || \
		echo "cppcheck not found, skipping static analysis"

# Format code (if clang-format is available)
format:
	@command -v clang-format >/dev/null 2>&1 && \
		find $(SRC_DIR) $(INCLUDE_DIR) -name "*.cpp" -o -name "*.h" | xargs clang-format -i || \
		echo "clang-format not found, skipping code formatting"

# Validation target - runs physics validation tests
validate: $(BUILD_DIR)/$(TARGET)
	@echo "Running physics validation tests..."
	@cd $(BUILD_DIR) && echo "6" | ./$(TARGET) > validation_output.txt 2>&1
	@echo "Validation complete. Check $(BUILD_DIR)/validation_output.txt for results."

# Performance profiling (if available)
profile: debug
	@command -v valgrind >/dev/null 2>&1 && \
		cd $(BUILD_DIR) && valgrind --tool=callgrind echo "1" | ./$(TARGET) || \
		echo "valgrind not found, skipping profiling"

# Memory check
memcheck: debug
	@command -v valgrind >/dev/null 2>&1 && \
		cd $(BUILD_DIR) && valgrind --leak-check=full echo "1" | ./$(TARGET) || \
		echo "valgrind not found, skipping memory check"

# Create documentation (if doxygen is available)
docs:
	@command -v doxygen >/dev/null 2>&1 && doxygen Doxyfile || \
		echo "doxygen not found, skipping documentation generation"

# Package for distribution
package: clean all
	tar -czf physics_simulation.tar.gz $(SRC_DIR)/ $(INCLUDE_DIR)/ Makefile README.md

# Phony targets
.PHONY: all debug run clean install uninstall help check-deps analyze format validate profile memcheck docs package
.PHONY: test-freefall test-projectile test-spring test-multibody test-accuracy test-all