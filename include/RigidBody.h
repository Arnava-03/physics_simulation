#pragma once

#include "Vector3D.h"
#include <vector>
#include <string>

/**
 * RigidBody class representing a physical object in the simulation
 * Contains all necessary properties for Newtonian dynamics: mass, position, velocity, forces
 * Uses Euler integration for motion updates
 */
class RigidBody {
private:
    static int nextId;  // For unique body identification
    int id;            // Unique identifier for this body
    
public:
    // Physical properties
    double mass;              // Mass of the body (kg)
    Vector3D position;        // Current position (m)
    Vector3D velocity;        // Current velocity (m/s)
    Vector3D acceleration;    // Current acceleration (m/s²)
    Vector3D netForce;        // Sum of all forces acting on the body (N)
    
    // Additional properties
    std::string name;         // Optional name for identification
    bool isStatic;           // If true, body doesn't move (infinite mass effectively)
    double restitution;      // Coefficient of restitution (for future collision handling)
    
    // Constructors
    RigidBody() 
        : id(++nextId), mass(1.0), position(0, 0, 0), velocity(0, 0, 0), 
          acceleration(0, 0, 0), netForce(0, 0, 0), name("Body_" + std::to_string(id)), 
          isStatic(false), restitution(0.8) {}
    
    RigidBody(double m, const Vector3D& pos = Vector3D(0, 0, 0), 
              const Vector3D& vel = Vector3D(0, 0, 0))
        : id(++nextId), mass(m), position(pos), velocity(vel), 
          acceleration(0, 0, 0), netForce(0, 0, 0), name("Body_" + std::to_string(id)), 
          isStatic(false), restitution(0.8) {}
    
    RigidBody(double m, const Vector3D& pos, const Vector3D& vel, const std::string& bodyName)
        : id(++nextId), mass(m), position(pos), velocity(vel), 
          acceleration(0, 0, 0), netForce(0, 0, 0), name(bodyName), 
          isStatic(false), restitution(0.8) {}

    // Destructor
    ~RigidBody() = default;

    // Getters
    int getId() const { return id; }
    double getMass() const { return mass; }
    const Vector3D& getPosition() const { return position; }
    const Vector3D& getVelocity() const { return velocity; }
    const Vector3D& getAcceleration() const { return acceleration; }
    const Vector3D& getNetForce() const { return netForce; }
    const std::string& getName() const { return name; }
    bool getIsStatic() const { return isStatic; }
    double getRestitution() const { return restitution; }

    // Setters
    void setMass(double m) { 
        if (m <= 0.0) {
            throw std::invalid_argument("Mass must be positive");
        }
        mass = m; 
    }
    
    void setPosition(const Vector3D& pos) { position = pos; }
    void setVelocity(const Vector3D& vel) { velocity = vel; }
    void setName(const std::string& bodyName) { name = bodyName; }
    void setStatic(bool staticState) { 
        isStatic = staticState;
        if (isStatic) {
            velocity.setZero();
            acceleration.setZero();
        }
    }
    void setRestitution(double e) { 
        restitution = std::max(0.0, std::min(1.0, e)); // Clamp between 0 and 1
    }

    /**
     * Apply a force to the rigid body
     * Forces are accumulated in netForce and will be used in the next integration step
     * @param force The force vector to apply (N)
     */
    void applyForce(const Vector3D& force) {
        if (!isStatic) {
            netForce += force;
        }
    }

    /**
     * Apply an impulse (instantaneous change in momentum) to the body
     * @param impulse The impulse vector (N⋅s or kg⋅m/s)
     */
    void applyImpulse(const Vector3D& impulse) {
        if (!isStatic && mass > 0.0) {
            velocity += impulse / mass;
        }
    }

    /**
     * Clear all accumulated forces
     * Should be called after each integration step
     */
    void clearForces() {
        netForce.setZero();
    }

    /**
     * Update the body's state using Euler integration
     * Integrates the equations of motion: F = ma, v = v + a*dt, x = x + v*dt
     * @param deltaTime Time step for integration (s)
     */
    void update(double deltaTime) {
        if (isStatic || deltaTime <= 0.0) {
            return;  // Static bodies don't move
        }

        // Newton's second law: a = F/m
        if (mass > 0.0) {
            acceleration = netForce / mass;
        } else {
            acceleration.setZero();
        }

        // Euler integration for velocity: v = v + a*dt
        velocity += acceleration * deltaTime;

        // Euler integration for position: x = x + v*dt
        position += velocity * deltaTime;
    }

    /**
     * Get kinetic energy of the body
     * KE = 0.5 * m * v²
     * @return Kinetic energy in Joules
     */
    double getKineticEnergy() const {
        if (isStatic) return 0.0;
        return 0.5 * mass * velocity.magnitudeSquared();
    }

    /**
     * Get momentum of the body
     * p = m * v
     * @return Momentum vector (kg⋅m/s)
     */
    Vector3D getMomentum() const {
        if (isStatic) return Vector3D(0, 0, 0);
        return mass * velocity;
    }

    /**
     * Set the body to have zero velocity and acceleration
     */
    void stop() {
        velocity.setZero();
        acceleration.setZero();
    }

    /**
     * Check if the body is at rest (very low velocity)
     * @param tolerance Velocity threshold below which body is considered at rest
     * @return True if body is effectively at rest
     */
    bool isAtRest(double tolerance = 1e-6) const {
        return isStatic || velocity.magnitude() < tolerance;
    }

    /**
     * Get string representation of the body's current state
     * Useful for debugging and logging
     * @return String containing position, velocity, and force information
     */
    std::string getStateString() const {
        return name + " - Pos: " + position.toString() + 
               ", Vel: " + velocity.toString() + 
               ", Force: " + netForce.toString() + 
               ", Mass: " + std::to_string(mass) + "kg";
    }

    /**
     * Reset the body to initial state
     * @param pos Initial position
     * @param vel Initial velocity
     */
    void reset(const Vector3D& pos = Vector3D(0, 0, 0), 
               const Vector3D& vel = Vector3D(0, 0, 0)) {
        position = pos;
        velocity = vel;
        acceleration.setZero();
        clearForces();
    }
};

// Static member initialization
int RigidBody::nextId = 0;