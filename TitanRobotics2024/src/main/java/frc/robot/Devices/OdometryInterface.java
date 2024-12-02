package frc.robot.Devices;

import java.util.Optional;

/**
 * Interface for odometry systems to standardize access to odometry measurements
 */
public interface OdometryInterface {
    /**
     * Represents a complete measurement from the odometry system
     */
    class OdometryMeasurement {
        public final double x;      // estimated X position (m)
        public final double y;      // estimated Y position (m)
        public final double vx;     // X velocity (m/s)
        public final double vy;     // Y velocity (m/s)
        public final double omega;  // angular velocity (rad/s)
        public final double timestamp;

        public OdometryMeasurement(double x, double y, double vx, double vy, double omega, double timestamp) {
            this.x = x;
            this.y = y;
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
            this.timestamp = timestamp;
        }
    }

    /**
     * Gets the latest odometry measurement
     * @return Optional containing the measurement, or empty if measurement failed
     */
    Optional<OdometryMeasurement> getMeasurement();

    /**
     * Validates the given odometry measurement
     * @param measurement The measurement to validate
     * @return true if valid, false otherwise
     */
    boolean validateMeasurement(OdometryMeasurement measurement);
}
