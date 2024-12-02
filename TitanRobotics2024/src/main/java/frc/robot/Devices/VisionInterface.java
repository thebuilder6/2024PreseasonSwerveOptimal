package frc.robot.Devices;

import java.util.Optional;

/**
 * Interface for vision systems to standardize access to vision measurements
 */
public interface VisionInterface {
    /**
     * Represents a complete measurement from the vision system
     */
    class VisionMeasurement {
        public final double x;          // X position (m)
        public final double y;          // Y position (m)
        public final double rotation;    // Rotation (radians)
        public final double confidence;  // Confidence level of the measurement
        public final double latencySeconds; // Latency of the measurement (seconds)
        public final double distanceToTarget; // Distance to the target (m)

        public VisionMeasurement(double x, double y, double rotation, double confidence, double latencySeconds, double distanceToTarget) {
            this.x = x;
            this.y = y;
            this.rotation = rotation;
            this.confidence = confidence;
            this.latencySeconds = latencySeconds;
            this.distanceToTarget = distanceToTarget;
        }
    }

    /**
     * Gets the latest vision measurement
     * @return Optional containing the measurement, or empty if measurement failed
     */
    Optional<VisionMeasurement> getMeasurement();

    /**
     * Validates the given vision measurement
     * @param measurement The measurement to validate
     * @return true if valid, false otherwise
     */
    boolean validateMeasurement(VisionMeasurement measurement);
}