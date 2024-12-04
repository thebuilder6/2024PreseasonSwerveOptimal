package frc.robot.Devices;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;


import java.util.Optional;

/**
 * Interface for IMU devices to standardize access to inertial measurements
 */
public interface IMUInterface {
    /**
     * Represents a complete set of IMU measurements
     */
    public class IMUMeasurement {
        // Raw measurements
        public final Measure<Velocity<Velocity<Distance>>> rawAccelX;    // m/s^2
        public final Measure<Velocity<Velocity<Distance>>> rawAccelY;    // m/s^2
        public final Measure<Velocity<Velocity<Distance>>> rawAccelZ;    // m/s^2
        public final Measure<Angle> gyroPitch;     
        public final Measure<Angle> gyroRoll;     
        public final Measure<Angle> gyroYaw;
        public final Measure<Velocity<Angle>> yawRate;

        // Processed measurements
        public final Rotation2d heading;   // Current heading
        public final double timestamp;     // When measurement was taken
        
        public IMUMeasurement(
                Measure<Velocity<Velocity<Distance>>> rawAccelX, 
                Measure<Velocity<Velocity<Distance>>> rawAccelY, 
                Measure<Velocity<Velocity<Distance>>> rawAccelZ,
                Measure<Angle> gyroPitch, 
                Measure<Angle> gyroRoll, 
                Measure<Angle> gyroYaw,
                Measure<Velocity<Angle>> yawRate,
                Rotation2d heading, double timestamp) {
            this.rawAccelX = rawAccelX;
            this.rawAccelY = rawAccelY;
            this.rawAccelZ = rawAccelZ;
            this.gyroPitch = gyroPitch;
            this.gyroRoll = gyroRoll;
            this.gyroYaw = gyroYaw;
            this.yawRate = yawRate;
            this.heading = heading;
            this.timestamp = timestamp;
        }
    }

    /**
     * Get the latest measurement from the IMU
     * @return Optional containing the measurement, or empty if measurement failed
     */
    Optional<IMUMeasurement> getMeasurement();

    /**
     * Check if the IMU is connected and functioning
     */
    boolean isOperational();

    /**
     * Reset the IMU's heading to zero
     */
    void zeroHeading();

    /**
     * Set the IMU's heading to a specific angle
     */
    void setHeading(Rotation2d heading);

    /**
     * Get the IMU's mounting configuration
     */
    MountConfiguration getMountConfiguration();

    /**
     * Configure how the IMU is mounted on the robot
     */
    public class MountConfiguration {
        public final Rotation2d rollOffset;    // Roll offset from level
        public final Rotation2d pitchOffset;   // Pitch offset from level
        public final Rotation2d yawOffset;     // Yaw offset from robot forward
        
        public MountConfiguration(
                Rotation2d rollOffset, 
                Rotation2d pitchOffset, 
                Rotation2d yawOffset) {
            this.rollOffset = rollOffset;
            this.pitchOffset = pitchOffset;
            this.yawOffset = yawOffset;
        }
    }
}
