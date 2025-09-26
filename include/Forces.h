#pragma once

#include "Vector3D.h"
#include "RigidBody.h"
#include <memory>

/**
 * Abstract base class for all force generators in the physics simulation
 * Forces can be applied to rigid bodies and compute force vectors based on body state
 */
class Force {
public:
    virtual ~Force() = default;

    /**
     * Compute the force vector to be applied to a given rigid body
     * @param body The rigid body to compute force for
     * @return Force vector in Newtons
     */
    virtual Vector3D computeForce(const RigidBody& body) const = 0;

    /**
     * Get a descriptive name for this force type
     * @return String description of the force
     */
    virtual std::string getName() const = 0;

    /**
     * Check if this force is active/enabled
     * @return True if force should be applied
     */
    virtual bool isActive() const { return true; }
};

/**
 * Gravitational force generator
 * Applies constant downward force F = m * g
 */
class GravityForce : public Force {
private:
    Vector3D gravity;  // Gravitational acceleration vector (m/s²)

public:
    /**
     * Constructor with default Earth gravity
     * @param g Gravitational acceleration (default: -9.81 m/s² in Y direction)
     */
    explicit GravityForce(const Vector3D& g = Vector3D(0, -9.81, 0)) 
        : gravity(g) {}

    /**
     * Constructor for magnitude-only gravity (assumes downward Y direction)
     * @param magnitude Gravitational acceleration magnitude (m/s²)
     */
    explicit GravityForce(double magnitude) 
        : gravity(0, -magnitude, 0) {}

    Vector3D computeForce(const RigidBody& body) const override {
        if (body.getIsStatic()) {
            return Vector3D(0, 0, 0);  // Static bodies don't experience gravity
        }
        return body.getMass() * gravity;  // F = m * g
    }

    std::string getName() const override {
        return "Gravity Force";
    }

    // Getters and setters
    const Vector3D& getGravity() const { return gravity; }
    void setGravity(const Vector3D& g) { gravity = g; }
    void setGravity(double magnitude) { gravity = Vector3D(0, -magnitude, 0); }
};

/**
 * Air resistance/drag force generator
 * Applies force proportional to velocity: F = -k * v * |v|
 * This gives quadratic drag which is realistic for medium to high speeds
 */
class DragForce : public Force {
private:
    double dragCoefficient;  // Drag coefficient k

public:
    /**
     * Constructor
     * @param k Drag coefficient (dimensionless, typically 0.1 to 2.0)
     */
    explicit DragForce(double k = 0.1) 
        : dragCoefficient(k) {}

    Vector3D computeForce(const RigidBody& body) const override {
        if (body.getIsStatic()) {
            return Vector3D(0, 0, 0);
        }

        Vector3D velocity = body.getVelocity();
        if (velocity.isZero()) {
            return Vector3D(0, 0, 0);  // No drag if no movement
        }

        // Quadratic drag: F = -k * v * |v|
        double speed = velocity.magnitude();
        Vector3D dragDirection = -velocity.normalized();  // Opposite to velocity
        
        return dragDirection * (dragCoefficient * speed * speed);
    }

    std::string getName() const override {
        return "Drag Force";
    }

    // Getters and setters
    double getDragCoefficient() const { return dragCoefficient; }
    void setDragCoefficient(double k) { dragCoefficient = k; }
};

/**
 * Constant force generator
 * Applies a constant force vector regardless of body state
 * Useful for testing and simple scenarios like wind
 */
class ConstantForce : public Force {
private:
    Vector3D force;    // Constant force vector
    bool active;       // Whether this force is currently active

public:
    /**
     * Constructor
     * @param f Constant force vector (N)
     */
    explicit ConstantForce(const Vector3D& f) 
        : force(f), active(true) {}

    /**
     * Constructor with component specification
     * @param fx Force in X direction (N)
     * @param fy Force in Y direction (N)
     * @param fz Force in Z direction (N)
     */
    ConstantForce(double fx, double fy, double fz) 
        : force(fx, fy, fz), active(true) {}

    Vector3D computeForce(const RigidBody& body) const override {
        if (!active || body.getIsStatic()) {
            return Vector3D(0, 0, 0);
        }
        return force;
    }

    std::string getName() const override {
        return "Constant Force";
    }

    bool isActive() const override {
        return active;
    }

    // Getters and setters
    const Vector3D& getForce() const { return force; }
    void setForce(const Vector3D& f) { force = f; }
    void setActive(bool isActive) { active = isActive; }
};

/**
 * Spring force generator
 * Applies Hooke's law force: F = -k * (x - x0)
 * Can be used for springs, elastic connections, etc.
 */
class SpringForce : public Force {
private:
    Vector3D anchorPoint;      // Fixed point the spring is attached to
    double springConstant;     // Spring stiffness k (N/m)
    double restLength;         // Natural/rest length of spring (m)
    bool active;               // Whether this force is active

public:
    /**
     * Constructor
     * @param anchor Fixed anchor point for the spring
     * @param k Spring constant (N/m)
     * @param restLen Rest length of spring (m)
     */
    SpringForce(const Vector3D& anchor, double k, double restLen = 0.0)
        : anchorPoint(anchor), springConstant(k), restLength(restLen), active(true) {}

    Vector3D computeForce(const RigidBody& body) const override {
        if (!active || body.getIsStatic()) {
            return Vector3D(0, 0, 0);
        }

        // Calculate displacement from anchor
        Vector3D displacement = body.getPosition() - anchorPoint;
        double currentLength = displacement.magnitude();
        
        if (currentLength < 1e-10) {
            return Vector3D(0, 0, 0);  // Avoid division by zero
        }

        // Hooke's law: F = -k * (current_length - rest_length) * direction
        double extension = currentLength - restLength;
        Vector3D direction = displacement.normalized();
        
        return -springConstant * extension * direction;
    }

    std::string getName() const override {
        return "Spring Force";
    }

    bool isActive() const override {
        return active;
    }

    // Getters and setters
    const Vector3D& getAnchorPoint() const { return anchorPoint; }
    void setAnchorPoint(const Vector3D& anchor) { anchorPoint = anchor; }
    double getSpringConstant() const { return springConstant; }
    void setSpringConstant(double k) { springConstant = k; }
    double getRestLength() const { return restLength; }
    void setRestLength(double length) { restLength = length; }
    void setActive(bool isActive) { active = isActive; }
};

// Type aliases for easier usage
using ForcePtr = std::shared_ptr<Force>;
using GravityForcePtr = std::shared_ptr<GravityForce>;
using DragForcePtr = std::shared_ptr<DragForce>;
using ConstantForcePtr = std::shared_ptr<ConstantForce>;
using SpringForcePtr = std::shared_ptr<SpringForce>;