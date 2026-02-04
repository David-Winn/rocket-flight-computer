#pragma once

class DistanceAlgorithm {
public:
    DistanceAlgorithm();
    
    // Initialize/reset the integrator (call at apogee)
    void reset();
    
    // Update with new acceleration reading and time change
    // accel: acceleration in m/s² (positive = upward)
    // dtSeconds: time change since last update in seconds
    void update(float accel, float dtSeconds);
    
    // Get current distance fallen (positive = downward from apogee)
    float getDistance() const;
    
    // Get current velocity (positive = downward)
    float getVelocity() const;

private:
    float velocity;    // Current velocity (m/s)
    float distance;    // Current distance from apogee (m)
};