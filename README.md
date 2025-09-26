# Physics Simulation Engine - Iteration 1

A comprehensive C++ physics simulation engine implementing **rigid body dynamics** with Newtonian mechanics. This project demonstrates fundamental physics simulation concepts including force application, numerical integration, and multi-body interactions.

## Features

### Core Physics Engine
- **3D Vector Mathematics**: Complete Vector3D class with all essential operations
- **Rigid Body Dynamics**: Mass, position, velocity, acceleration tracking
- **Force System**: Extensible force interface with multiple implementations
- **Numerical Integration**: Euler integration with configurable time steps
- **Multi-body Simulation**: Handle multiple interacting bodies simultaneously

### Force Implementations
- **Gravity Force**: Constant gravitational acceleration (F = mg)
- **Drag Force**: Air resistance with quadratic velocity dependence
- **Constant Force**: User-defined constant force vectors (wind, thrust)
- **Spring Force**: Hooke's law implementation (F = -kx)

### Simulation Features
- **Real-time Integration**: Step-by-step physics simulation
- **Data Logging**: CSV output for trajectory analysis
- **System Properties**: Energy and momentum tracking
- **Interactive Testing**: Menu-driven test cases
- **Performance Analysis**: Accuracy vs speed trade-offs

## Project Structure

```
physics/
├── include/           # Header files
│   ├── Vector3D.h     # 3D vector mathematics
│   ├── RigidBody.h    # Rigid body physics
│   ├── Forces.h       # Force generators
│   └── PhysicsWorld.h # Simulation world management
├── src/               # Source files
│   └── main.cpp       # Test cases and simulation driver
├── build/             # Compiled binaries and output files
├── Makefile          # Build system
└── README.md         # This file
```

## Building and Running

### Prerequisites
- **C++ Compiler**: g++ with C++17 support
- **Make**: For build automation
- **Optional**: valgrind (memory checking), cppcheck (static analysis)

### Quick Start
```bash
# Clone or download the project
cd physics

# Build the simulation
make all

# Run interactive simulation
make run

# Or run specific tests
make test-freefall     # Free fall validation
make test-projectile   # Projectile motion
make test-spring       # Harmonic oscillator
make test-multibody    # Multiple bodies
make test-accuracy     # Performance analysis
make test-all          # All tests
```

### Build Options
```bash
make debug            # Build with debug symbols
make clean            # Clean build artifacts
make analyze          # Static code analysis
make validate         # Run physics validation tests
make help             # Show all available targets
```

## Validation

### Free Fall Accuracy
- **Time step 0.01s**: Error < 0.1m over 50m drop
- **Time step 0.001s**: Error < 0.01m over 50m drop

### Projectile Motion
- **45° launch**: Theoretical range match within 1%
- **Flight time**: Accurate to within 0.05s

### Energy Conservation
- **Spring system**: <1% energy drift over 10 oscillations
- **Free fall**: Kinetic + potential energy conserved

## License

This physics simulation engine is provided for educational and research purposes.