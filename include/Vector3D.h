#pragma once

#include <cmath>
#include <iostream>

/**
 * Vector3D class for 3D mathematical operations in physics simulation
 * Supports basic vector arithmetic, dot/cross products, and utility functions
 */
class Vector3D {
public:
    double x, y, z;

    // Constructors
    Vector3D() : x(0.0), y(0.0), z(0.0) {}
    Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}
    Vector3D(const Vector3D& other) : x(other.x), y(other.y), z(other.z) {}

    // Assignment operator
    Vector3D& operator=(const Vector3D& other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
            z = other.z;
        }
        return *this;
    }

    // Vector addition
    Vector3D operator+(const Vector3D& other) const {
        return Vector3D(x + other.x, y + other.y, z + other.z);
    }

    // Vector subtraction
    Vector3D operator-(const Vector3D& other) const {
        return Vector3D(x - other.x, y - other.y, z - other.z);
    }

    // Unary minus (negation)
    Vector3D operator-() const {
        return Vector3D(-x, -y, -z);
    }

    // Scalar multiplication
    Vector3D operator*(double scalar) const {
        return Vector3D(x * scalar, y * scalar, z * scalar);
    }

    // Scalar division
    Vector3D operator/(double scalar) const {
        if (std::abs(scalar) < 1e-10) {
            throw std::runtime_error("Division by zero in Vector3D");
        }
        return Vector3D(x / scalar, y / scalar, z / scalar);
    }

    // Compound assignment operators
    Vector3D& operator+=(const Vector3D& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3D& operator-=(const Vector3D& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3D& operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vector3D& operator/=(double scalar) {
        if (std::abs(scalar) < 1e-10) {
            throw std::runtime_error("Division by zero in Vector3D");
        }
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    // Dot product
    double dot(const Vector3D& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Cross product
    Vector3D cross(const Vector3D& other) const {
        return Vector3D(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    // Magnitude (length) of the vector
    double magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    // Squared magnitude (useful for optimization)
    double magnitudeSquared() const {
        return x * x + y * y + z * z;
    }

    // Normalize the vector to unit length
    Vector3D normalized() const {
        double mag = magnitude();
        if (mag < 1e-10) {
            return Vector3D(0, 0, 0);  // Return zero vector if magnitude is too small
        }
        return *this / mag;
    }

    // Normalize the vector in place
    void normalize() {
        double mag = magnitude();
        if (mag > 1e-10) {
            *this /= mag;
        } else {
            x = y = z = 0.0;
        }
    }

    // Check if vector is approximately zero
    bool isZero(double tolerance = 1e-10) const {
        return magnitude() < tolerance;
    }

    // Distance between two points represented as vectors
    double distance(const Vector3D& other) const {
        return (*this - other).magnitude();
    }

    // Set all components to zero
    void setZero() {
        x = y = z = 0.0;
    }

    // Set vector components
    void set(double newX, double newY, double newZ) {
        x = newX;
        y = newY;
        z = newZ;
    }

    // String representation for debugging
    std::string toString() const {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
    }
};

// Global operators for scalar multiplication (scalar * vector)
inline Vector3D operator*(double scalar, const Vector3D& vector) {
    return vector * scalar;
}

// Stream output operator
inline std::ostream& operator<<(std::ostream& os, const Vector3D& vector) {
    os << "(" << vector.x << ", " << vector.y << ", " << vector.z << ")";
    return os;
}

// Commonly used vectors as static constants
namespace VectorConstants {
    static const Vector3D ZERO(0.0, 0.0, 0.0);
    static const Vector3D UNIT_X(1.0, 0.0, 0.0);
    static const Vector3D UNIT_Y(0.0, 1.0, 0.0);
    static const Vector3D UNIT_Z(0.0, 0.0, 1.0);
    static const Vector3D GRAVITY(0.0, -9.81, 0.0);  // Standard Earth gravity
}