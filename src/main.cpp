#include "../include/PhysicsWorld.h"
#include "../include/RigidBody.h"
#include "../include/Forces.h"
#include "../include/Vector3D.h"
#include <iostream>
#include <memory>
#include <iomanip>
#include <functional>
#include <chrono>
#include <cmath>

/**
 * Physics Simulation Test Cases
 * Demonstrates various physics scenarios and validates the simulation engine
 */

/**
 * Test Case 1: Free Fall
 * Drop a ball from height and compare with analytical solution
 * Analytical solution: y(t) = y0 - 0.5 * g * t²
 */
void testFreeFall() {
    std::cout << "\n=== TEST 1: FREE FALL SIMULATION ===" << std::endl;
    
    // Create physics world with small time step for accuracy
    PhysicsWorld world(0.01, true, "freefall_simulation.csv");
    world.setLogFrequency(10);  // Log every 10 steps
    
    // Create a ball at height 50m
    auto ball = world.addBody(1.0, Vector3D(0, 50, 0), Vector3D(0, 0, 0), "Ball");
    
    // Add gravity force
    auto gravity = std::make_shared<GravityForce>(9.81);
    world.addForce(gravity);
    
    std::cout << "Initial state:" << std::endl;
    std::cout << ball->getStateString() << std::endl;
    
    // Simulate for 3.2 seconds (ball should hit ground around t=3.19s)
    double simulationTime = 3.2;
    
    // Progress callback
    auto progressCallback = [](double time, int steps) {
        std::cout << "Time: " << std::fixed << std::setprecision(2) 
                  << time << "s, Steps: " << steps << std::endl;
    };
    
    // Run simulation
    world.simulate(simulationTime, progressCallback);
    
    std::cout << "\nFinal state:" << std::endl;
    std::cout << ball->getStateString() << std::endl;
    
    // Validate against analytical solution
    double finalTime = world.getCurrentTime();
    double expectedY = 50.0 - 0.5 * 9.81 * finalTime * finalTime;
    double actualY = ball->getPosition().y;
    double error = std::abs(actualY - expectedY);
    
    std::cout << "\n--- Validation ---" << std::endl;
    std::cout << "Expected Y position (analytical): " << expectedY << "m" << std::endl;
    std::cout << "Actual Y position (simulation):   " << actualY << "m" << std::endl;
    std::cout << "Error: " << error << "m (" << (error/50.0)*100 << "% of initial height)" << std::endl;
    
    if (error < 0.1) {
        std::cout << "✓ Free fall test PASSED!" << std::endl;
    } else {
        std::cout << "✗ Free fall test FAILED!" << std::endl;
    }
}

/**
 * Test Case 2: Projectile Motion
 * Launch a projectile at an angle and validate trajectory
 */
void testProjectileMotion() {
    std::cout << "\n=== TEST 2: PROJECTILE MOTION ===" << std::endl;
    
    // Create physics world
    PhysicsWorld world(0.01, true, "projectile_simulation.csv");
    world.setLogFrequency(5);
    
    // Launch parameters
    double angle = 45.0 * M_PI / 180.0;  // 45 degrees in radians
    double initialSpeed = 30.0;          // m/s
    Vector3D initialVelocity(
        initialSpeed * cos(angle),
        initialSpeed * sin(angle),
        0.0
    );
    
    // Create projectile at origin
    auto projectile = world.addBody(0.5, Vector3D(0, 0, 0), initialVelocity, "Projectile");
    
    // Add gravity
    auto gravity = std::make_shared<GravityForce>(9.81);
    world.addForce(gravity);
    
    std::cout << "Launch parameters:" << std::endl;
    std::cout << "Angle: " << (angle * 180.0 / M_PI) << " degrees" << std::endl;
    std::cout << "Initial speed: " << initialSpeed << " m/s" << std::endl;
    std::cout << "Initial velocity: " << initialVelocity << std::endl;
    
    // Simulate until projectile hits ground
    double maxTime = 10.0;
    double timeStep = world.getTimeStep();
    
    for (double t = 0; t < maxTime; t += timeStep) {
        world.step();
        
        // Check if projectile has hit ground
        if (projectile->getPosition().y < 0) {
            std::cout << "\nProjectile hit ground at t = " << world.getCurrentTime() << "s" << std::endl;
            break;
        }
        
        // Print position every second
        if (static_cast<int>(world.getCurrentTime() * 10) % 10 == 0) {
            Vector3D pos = projectile->getPosition();
            std::cout << "t=" << std::fixed << std::setprecision(1) << world.getCurrentTime() 
                      << "s: Position(" << std::setprecision(2) << pos.x << ", " << pos.y << ")" << std::endl;
        }
    }
    
    // Calculate theoretical values
    double flightTime = 2.0 * initialSpeed * sin(angle) / 9.81;
    double range = initialSpeed * initialSpeed * sin(2 * angle) / 9.81;
    double maxHeight = initialSpeed * initialSpeed * sin(angle) * sin(angle) / (2 * 9.81);
    
    std::cout << "\n--- Theoretical vs Simulation ---" << std::endl;
    std::cout << "Theoretical flight time: " << flightTime << "s" << std::endl;
    std::cout << "Actual flight time: " << world.getCurrentTime() << "s" << std::endl;
    std::cout << "Theoretical range: " << range << "m" << std::endl;
    std::cout << "Actual range: " << projectile->getPosition().x << "m" << std::endl;
    std::cout << "Theoretical max height: " << maxHeight << "m" << std::endl;
}

/**
 * Test Case 3: Spring-Mass System
 * Test harmonic oscillation with spring force
 */
void testSpringMassSystem() {
    std::cout << "\n=== TEST 3: SPRING-MASS SYSTEM ===" << std::endl;
    
    // Create physics world
    PhysicsWorld world(0.005, true, "spring_simulation.csv");  // Smaller timestep for stability
    world.setLogFrequency(20);
    
    // Create mass at displacement from equilibrium
    Vector3D equilibriumPos(0, 0, 0);
    Vector3D initialPos(2, 0, 0);  // 2m displacement
    auto mass = world.addBody(1.0, initialPos, Vector3D(0, 0, 0), "Mass");
    
    // Add spring force (k = 10 N/m)
    double springConstant = 10.0;
    auto spring = std::make_shared<SpringForce>(equilibriumPos, springConstant, 0.0);
    world.addForce(spring);
    
    std::cout << "Spring constant: " << springConstant << " N/m" << std::endl;
    std::cout << "Mass: " << mass->getMass() << " kg" << std::endl;
    std::cout << "Initial displacement: " << initialPos.x << " m" << std::endl;
    
    // Calculate theoretical frequency
    double omega = sqrt(springConstant / mass->getMass());
    double period = 2 * M_PI / omega;
    
    std::cout << "Theoretical angular frequency: " << omega << " rad/s" << std::endl;
    std::cout << "Theoretical period: " << period << " s" << std::endl;
    
    // Simulate for 2 periods
    double simulationTime = 2.0 * period;
    
    // Track maximum displacement to verify amplitude conservation
    double maxDisplacement = 0.0;
    
    for (double t = 0; t < simulationTime; t += world.getTimeStep()) {
        world.step();
        
        double displacement = abs(mass->getPosition().x);
        maxDisplacement = std::max(maxDisplacement, displacement);
        
        // Print position every quarter period
        if (static_cast<int>(world.getCurrentTime() / (period/4)) != static_cast<int>((world.getCurrentTime() - world.getTimeStep()) / (period/4))) {
            std::cout << "t=" << std::fixed << std::setprecision(3) << world.getCurrentTime() 
                      << "s: x=" << std::setprecision(4) << mass->getPosition().x << "m" << std::endl;
        }
    }
    
    std::cout << "\nMaximum displacement during simulation: " << maxDisplacement << "m" << std::endl;
    std::cout << "Energy conservation check:" << std::endl;
    std::cout << "Total system energy: " << world.getTotalEnergy() << "J" << std::endl;
}

/**
 * Test Case 4: Multi-body System with Different Forces
 * Demonstrate multiple bodies with different force interactions
 */
void testMultiBodySystem() {
    std::cout << "\n=== TEST 4: MULTI-BODY SYSTEM ===" << std::endl;
    
    // Create physics world
    PhysicsWorld world(0.01, true, "multibody_simulation.csv");
    world.setLogFrequency(10);
    
    // Create multiple bodies
    auto body1 = world.addBody(1.0, Vector3D(-5, 10, 0), Vector3D(2, 0, 0), "Body1");
    auto body2 = world.addBody(2.0, Vector3D(0, 15, 0), Vector3D(0, -1, 0), "Body2");
    auto body3 = world.addBody(0.5, Vector3D(3, 8, 0), Vector3D(-1, 3, 0), "Body3");
    
    // Add various forces
    auto gravity = std::make_shared<GravityForce>(9.81);
    auto drag = std::make_shared<DragForce>(0.05);  // Light air resistance
    auto wind = std::make_shared<ConstantForce>(Vector3D(1, 0, 0));  // Constant wind force
    
    world.addForce(gravity);
    world.addForce(drag);
    world.addForce(wind);
    
    std::cout << "Created 3 bodies with different masses and initial conditions" << std::endl;
    std::cout << "Applied forces: Gravity, Drag, and constant Wind" << std::endl;
    
    // Simulate for 5 seconds
    double simulationTime = 5.0;
    
    std::cout << "\nSimulation progress:" << std::endl;
    world.simulate(simulationTime, [](double time, int steps) {
        if (static_cast<int>(time * 2) % 2 == 0) {  // Every 0.5 seconds
            std::cout << "t=" << std::fixed << std::setprecision(1) << time << "s" << std::endl;
        }
    });
    
    // Print final states
    std::cout << "\nFinal system state:" << std::endl;
    world.printState();
}

/**
 * Test Case 5: Performance and Accuracy Test
 * Test with different time steps to analyze accuracy vs performance trade-off
 */
void testAccuracyVsPerformance() {
    std::cout << "\n=== TEST 5: ACCURACY VS PERFORMANCE ===" << std::endl;
    
    std::vector<double> timeSteps = {0.1, 0.05, 0.01, 0.005, 0.001};
    double totalTime = 2.0;  // Simulate for 2 seconds
    
    std::cout << "Testing free fall accuracy with different time steps:" << std::endl;
    std::cout << std::setw(10) << "dt(s)" << std::setw(12) << "Final Y(m)" 
              << std::setw(12) << "Error(m)" << std::setw(10) << "Steps" << std::endl;
    std::cout << std::string(44, '-') << std::endl;
    
    // Analytical solution for comparison
    double analyticalY = 50.0 - 0.5 * 9.81 * totalTime * totalTime;
    
    for (double dt : timeSteps) {
        PhysicsWorld world(dt, false);  // No logging for performance test
        
        auto ball = world.addBody(1.0, Vector3D(0, 50, 0), Vector3D(0, 0, 0));
        auto gravity = std::make_shared<GravityForce>(9.81);
        world.addForce(gravity);
        
        // Time the simulation (simple timing)
        auto start = std::chrono::high_resolution_clock::now();
        world.simulate(totalTime);
        auto end = std::chrono::high_resolution_clock::now();
        
        double finalY = ball->getPosition().y;
        double error = std::abs(finalY - analyticalY);
        
        std::cout << std::fixed << std::setprecision(3)
                  << std::setw(10) << dt 
                  << std::setw(12) << finalY
                  << std::setw(12) << error
                  << std::setw(10) << world.getStepCount() << std::endl;
    }
    
    std::cout << "Analytical result: " << std::fixed << std::setprecision(3) 
              << analyticalY << "m" << std::endl;
}

/**
 * Interactive menu for running different test cases
 */
void runInteractiveMenu() {
    while (true) {
        std::cout << "\n====== PHYSICS SIMULATION MENU ======" << std::endl;
        std::cout << "1. Free Fall Test" << std::endl;
        std::cout << "2. Projectile Motion Test" << std::endl;
        std::cout << "3. Spring-Mass System Test" << std::endl;
        std::cout << "4. Multi-Body System Test" << std::endl;
        std::cout << "5. Accuracy vs Performance Test" << std::endl;
        std::cout << "6. Run All Tests" << std::endl;
        std::cout << "0. Exit" << std::endl;
        std::cout << "=======================================" << std::endl;
        std::cout << "Choose option: ";
        
        int choice;
        std::cin >> choice;
        
        switch (choice) {
            case 1:
                testFreeFall();
                break;
            case 2:
                testProjectileMotion();
                break;
            case 3:
                testSpringMassSystem();
                break;
            case 4:
                testMultiBodySystem();
                break;
            case 5:
                testAccuracyVsPerformance();
                break;
            case 6:
                testFreeFall();
                testProjectileMotion();
                testSpringMassSystem();
                testMultiBodySystem();
                testAccuracyVsPerformance();
                break;
            case 0:
                std::cout << "Exiting simulation..." << std::endl;
                return;
            default:
                std::cout << "Invalid option. Please try again." << std::endl;
                break;
        }
        
        std::cout << "\nPress Enter to continue...";
        std::cin.ignore();
        std::cin.get();
    }
}

int main() {
    try {
        std::cout << "====================================================" << std::endl;
        std::cout << "    PHYSICS SIMULATION ENGINE - ITERATION 1" << std::endl;
        std::cout << "    Rigid Body Dynamics with Newtonian Physics" << std::endl;
        std::cout << "====================================================" << std::endl;
        
        runInteractiveMenu();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}