#include "distance_alg.hpp"

DistanceAlgorithm::DistanceAlgorithm() {
    reset();
}

void DistanceAlgorithm::reset() {
    velocity = 0.0f;
    distance = 0.0f;
}

void DistanceAlgorithm::update(float accel, float dtSeconds) {
    // Convert acceleration to downward reference frame
    float accelDown = -accel;
    
    // First integration: acceleration to velocity
    // v = v0 + a*dt
    velocity += accelDown * dtSeconds;
    
    // Second integration: velocity to distance
    // Using trapezoidal method for better accuracy:
    // d = d0 + v*dt + 0.5*a*dt²
    distance += velocity * dtSeconds + 0.5f * accelDown * dtSeconds * dtSeconds;
    
    // Clamp distance to non-negative 
    if (distance < 0.0f) {
        distance = 0.0f;
    }
}

float DistanceAlgorithm::getDistance() const {
    return distance;
}

float DistanceAlgorithm::getVelocity() const {
    return velocity;
}
