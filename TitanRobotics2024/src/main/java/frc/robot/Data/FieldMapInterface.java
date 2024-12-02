// File: TitanRobotics2024/src/main/java/frc/robot/Data/FieldMapInterface.java
package frc.robot.Data;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.Optional;

/**
 * Interface for field map systems to standardize access to field boundaries and features.
 */
public interface FieldMapInterface {
    /**
     * Checks if a given pose is within the field boundaries.
     * 
     * @param pose The pose to check.
     * @return true if the pose is within bounds, false otherwise.
     */
    boolean isPoseWithinBounds(Pose2d pose);

    /**
     * Gets the nearest valid pose to a given pose.
     * 
     * @param pose The pose to check against.
     * @return An Optional containing the nearest valid pose, or empty if none found.
     */
    Optional<Pose2d> getNearestValidPose(Pose2d pose);

    /**
     * Gets the nearest feature to a given pose.
     * 
     * @param pose The pose to check against.
     * @return An Optional containing the nearest feature, or empty if none found.
     */
    //Optional<FieldFeature> getNearestFeature(Pose2d pose);

}