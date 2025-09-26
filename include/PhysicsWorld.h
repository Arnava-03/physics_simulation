#pragma once

#include "RigidBody.h"
#include "Forces.h"
#include "Vector3D.h"
#include <vector>
#include <memory>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

/**
 * PhysicsWorld class manages the entire physics simulation
 * Contains all rigid bodies and forces, handles integration and time stepping
 */
class PhysicsWorld {
private:
    std::vector<std::shared_ptr<RigidBody>> bodies;    // All rigid bodies in the world
    std::vector<std::shared_ptr<Force>> forces;        // All force generators
    
    double currentTime;                                 // Current simulation time (s)
    double timeStep;                                   // Integration time step (s)
    int stepCount;                                     // Number of simulation steps taken
    
    // Simulation statistics
    double totalEnergy;                                // Total system energy
    Vector3D totalMomentum;                           // Total system momentum
    
    // Logging
    bool loggingEnabled;                              // Whether to log simulation data
    std::string logFileName;                          // Output log file name
    std::ofstream logFile;                           // File stream for logging
    int logFrequency;                                // Log every N steps (1 = every step)

public:
    /**
     * Constructor
     * @param dt Time step for integration (seconds)
     * @param enableLogging Whether to enable data logging
     * @param logFile Name of log file (default: "simulation.log")
     */
    PhysicsWorld(double dt = 0.01, bool enableLogging = false, 
                 const std::string& logFile = "simulation.log")
        : currentTime(0.0), timeStep(dt), stepCount(0), totalEnergy(0.0), 
          totalMomentum(0, 0, 0), loggingEnabled(enableLogging), 
          logFileName(logFile), logFrequency(1) {
        
        if (timeStep <= 0.0) {
            throw std::invalid_argument("Time step must be positive");
        }
        
        if (loggingEnabled) {
            initializeLogging();
        }
    }

    /**
     * Destructor - closes log file if open
     */
    ~PhysicsWorld() {
        if (logFile.is_open()) {
            logFile.close();
        }
    }

    // Getters
    double getCurrentTime() const { return currentTime; }
    double getTimeStep() const { return timeStep; }
    int getStepCount() const { return stepCount; }
    size_t getBodyCount() const { return bodies.size(); }
    size_t getForceCount() const { return forces.size(); }
    double getTotalEnergy() const { return totalEnergy; }
    const Vector3D& getTotalMomentum() const { return totalMomentum; }

    // Setters
    void setTimeStep(double dt) { 
        if (dt <= 0.0) {
            throw std::invalid_argument("Time step must be positive");
        }
        timeStep = dt; 
    }
    void setLogFrequency(int freq) { logFrequency = std::max(1, freq); }

    /**
     * Add a rigid body to the simulation
     * @param body Shared pointer to the rigid body
     * @return Reference to the added body
     */
    std::shared_ptr<RigidBody> addBody(std::shared_ptr<RigidBody> body) {
        if (body) {
            bodies.push_back(body);
        }
        return body;
    }

    /**
     * Create and add a rigid body to the simulation
     * @param mass Body mass (kg)
     * @param position Initial position
     * @param velocity Initial velocity
     * @param name Optional name for the body
     * @return Shared pointer to the created body
     */
    std::shared_ptr<RigidBody> addBody(double mass, const Vector3D& position = Vector3D(0, 0, 0),
                                       const Vector3D& velocity = Vector3D(0, 0, 0),
                                       const std::string& name = "") {
        auto body = std::make_shared<RigidBody>(mass, position, velocity);
        if (!name.empty()) {
            body->setName(name);
        }
        return addBody(body);
    }

    /**
     * Add a force generator to the simulation
     * @param force Shared pointer to the force generator
     */
    void addForce(std::shared_ptr<Force> force) {
        if (force) {
            forces.push_back(force);
        }
    }

    /**
     * Remove a body from the simulation
     * @param body Shared pointer to the body to remove
     * @return True if body was found and removed
     */
    bool removeBody(std::shared_ptr<RigidBody> body) {
        auto it = std::find(bodies.begin(), bodies.end(), body);
        if (it != bodies.end()) {
            bodies.erase(it);
            return true;
        }
        return false;
    }

    /**
     * Remove a force from the simulation
     * @param force Shared pointer to the force to remove
     * @return True if force was found and removed
     */
    bool removeForce(std::shared_ptr<Force> force) {
        auto it = std::find(forces.begin(), forces.end(), force);
        if (it != forces.end()) {
            forces.erase(it);
            return true;
        }
        return false;
    }

    /**
     * Clear all bodies from the simulation
     */
    void clearBodies() {
        bodies.clear();
    }

    /**
     * Clear all forces from the simulation
     */
    void clearForces() {
        forces.clear();
    }

    /**
     * Get body by index
     * @param index Index of the body
     * @return Shared pointer to the body (null if index out of range)
     */
    std::shared_ptr<RigidBody> getBody(size_t index) const {
        if (index < bodies.size()) {
            return bodies[index];
        }
        return nullptr;
    }

    /**
     * Perform one simulation step
     * Applies all forces to all bodies and integrates motion
     */
    void step() {
        // Clear all accumulated forces from previous step
        for (auto& body : bodies) {
            body->clearForces();
        }

        // Apply all forces to all bodies
        for (auto& force : forces) {
            if (force->isActive()) {
                for (auto& body : bodies) {
                    Vector3D forceVector = force->computeForce(*body);
                    body->applyForce(forceVector);
                }
            }
        }

        // Update all bodies using numerical integration
        for (auto& body : bodies) {
            body->update(timeStep);
        }

        // Update simulation time and step counter
        currentTime += timeStep;
        stepCount++;

        // Calculate system properties
        calculateSystemProperties();

        // Log data if enabled
        if (loggingEnabled && (stepCount % logFrequency == 0)) {
            logSimulationData();
        }
    }

    /**
     * Run simulation for a specified duration
     * @param duration Total time to simulate (seconds)
     * @param progressCallback Optional callback for progress updates
     */
    void simulate(double duration, 
                  std::function<void(double, int)> progressCallback = nullptr) {
        if (duration <= 0.0) {
            return;
        }

        int totalSteps = static_cast<int>(duration / timeStep);
        int progressInterval = std::max(1, totalSteps / 100);  // Update progress every 1%

        for (int i = 0; i < totalSteps; ++i) {
            step();
            
            if (progressCallback && (i % progressInterval == 0 || i == totalSteps - 1)) {
                progressCallback(currentTime, stepCount);
            }
        }
    }

    /**
     * Reset simulation to initial state
     */
    void reset() {
        currentTime = 0.0;
        stepCount = 0;
        
        for (auto& body : bodies) {
            body->reset();
        }
        
        if (loggingEnabled && logFile.is_open()) {
            logFile.close();
            initializeLogging();
        }
    }

    /**
     * Get simulation state as string
     * @return String representation of current simulation state
     */
    std::string getStateString() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        oss << "=== Physics World State ===" << std::endl;
        oss << "Time: " << currentTime << "s, Steps: " << stepCount << std::endl;
        oss << "Bodies: " << bodies.size() << ", Forces: " << forces.size() << std::endl;
        oss << "Total Energy: " << totalEnergy << "J" << std::endl;
        oss << "Total Momentum: " << totalMomentum << " kg⋅m/s" << std::endl;
        
        for (size_t i = 0; i < bodies.size(); ++i) {
            oss << "Body " << i << ": " << bodies[i]->getStateString() << std::endl;
        }
        
        return oss.str();
    }

    /**
     * Print current simulation state to console
     */
    void printState() const {
        std::cout << getStateString() << std::endl;
    }

private:
    /**
     * Initialize logging system
     */
    void initializeLogging() {
        logFile.open(logFileName);
        if (logFile.is_open()) {
            // Write header
            logFile << "# Physics Simulation Log" << std::endl;
            logFile << "# Time(s), Step, Body_ID, Name, Pos_X, Pos_Y, Pos_Z, "
                    << "Vel_X, Vel_Y, Vel_Z, KE(J), Mass(kg)" << std::endl;
        } else {
            std::cerr << "Warning: Could not open log file: " << logFileName << std::endl;
            loggingEnabled = false;
        }
    }

    /**
     * Log current simulation data to file
     */
    void logSimulationData() {
        if (!logFile.is_open()) return;

        for (const auto& body : bodies) {
            logFile << std::fixed << std::setprecision(6)
                    << currentTime << ", "
                    << stepCount << ", "
                    << body->getId() << ", "
                    << body->getName() << ", "
                    << body->getPosition().x << ", "
                    << body->getPosition().y << ", "
                    << body->getPosition().z << ", "
                    << body->getVelocity().x << ", "
                    << body->getVelocity().y << ", "
                    << body->getVelocity().z << ", "
                    << body->getKineticEnergy() << ", "
                    << body->getMass() << std::endl;
        }
    }

    /**
     * Calculate total system energy and momentum
     */
    void calculateSystemProperties() {
        totalEnergy = 0.0;
        totalMomentum.setZero();

        for (const auto& body : bodies) {
            totalEnergy += body->getKineticEnergy();
            totalMomentum += body->getMomentum();
        }
    }
};