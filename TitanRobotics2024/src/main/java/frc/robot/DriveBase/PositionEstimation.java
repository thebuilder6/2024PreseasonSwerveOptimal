package frc.robot.DriveBase;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Devices.IMUInterface;
import frc.robot.Devices.IMUInterface.IMUMeasurement;
import frc.robot.Devices.OdometryInterface;
import frc.robot.Devices.OdometryInterface.OdometryMeasurement;
import frc.robot.Data.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

import java.util.Optional;
import frc.robot.Devices.VisionInterface;

/**
 * Position estimation system that fuses data from swerve drive odometry, IMU,
 * and vision
 * to maintain an accurate estimate of the robot's position on the field.
 */
public class PositionEstimation {
    // Core components and constants
    private final FieldMapInterface fieldMap;
    private final Timer timer;
    private final IMUInterface imu;
    private final NoiseParameters noiseParams;
    private final OdometryInterface odometry;
    private final VisionInterface vision;

    // State estimation matrices
    private Matrix<N7,N1> stateEstimate; // Current state estimate // [x, y, theta, vx, vy, omega, bias_omega]
    private Matrix<N7,N7>stateCovariance; // State uncertainty
    private Pose2d lastValidPose;

    // Measurement tracking
    private double lastUpdateTime;
    private ControlInput lastControlInput;
    private OdometryInterface.OdometryMeasurement lastOdometryMeasurement;
    private Optional<IMUInterface.IMUMeasurement> lastValidIMU = Optional.empty();

    // Constants
    private static final int STATE_SIZE = 7;
    private static final double SLIP_VELOCITY_THRESHOLD = 0.5; // m/s
    private static final double SLIP_ACCEL_THRESHOLD = 2.0; // m/s^2
    private static final double HEADING_DIFF_THRESHOLD = 0.1; // rad
    private static final double CONTROL_TRUST_FACTOR = 0.8;
    private static final double COVARIANCE_INCREASE_FACTOR = 2.0;
    private static final double MAX_ACCEL = 39.24; // m/s^2 (4G)
    private static final double MAX_ROTATION_RATE = Math.toRadians(1080.0); // rad/s
    private static final double VISION_MAX_JUMP = 1.0; // m
    private static final double FEATURE_CORRECTION_THRESHOLD = 0.5; // m

    private static final int IMU_MEASUREMENT_SIZE = 6;
    private static final int ODOMETRY_MEASUREMENT_SIZE = 3;
    private static final int VISION_MEASUREMENT_SIZE = 6;

    private double lastIMUTimestamp = 0.0;

    // Tunable noise parameters
    private static class NoiseParameters {
        // Process noise
        public double positionNoise = 0.01;
        public double velocityNoise = 0.1;
        public double headingNoise = 0.01;
        public double biasNoise = 0.001;

        // Measurement noise
        public double odometryPositionNoise = 0.05;
        public double odometryVelocityNoise = 0.1;
        public double imuHeadingNoise = 0.01;
        public double imuAccelNoise = 0.1;
        public double visionPositionNoise = 0.1;
        public double visionHeadingNoise = 0.05;
    }

    /**
     * Constructor.
     */
    public PositionEstimation(FieldMapInterface fieldMap, IMUInterface imu,
            OdometryInterface odometry, VisionInterface vision) {
        this.fieldMap = fieldMap;
        this.imu = imu;
        this.odometry = odometry;
        this.vision = vision;
        this.timer = new Timer();
        this.noiseParams = new NoiseParameters();
        // Initialize state estimation
        stateEstimate = new Matrix<>(Nat.N7(), Nat.N1());
        stateCovariance = Matrix.eye(Nat.N7());

        timer.start();
        lastUpdateTime = timer.get();
        lastValidPose = new Pose2d();
    }

    /**
     * Main update loop
     */
    public void update() {
        double currentTime = timer.get();
        double dt = currentTime - lastUpdateTime;

        // 1. Prediction step
        predict(dt);

        // 2. Process available measurements
        processMeasurements();

        lastUpdateTime = currentTime;
    }

    /**
     * Performs the prediction step of the Kalman filter
     * Updates state estimate (x_hat) and covariance (P) based on motion model
     */
    private void predict(double dt) {
        // Create state transition matrix F
        var F = Matrix.eye(Nat.N7());

        // Position update: p = p + v*dt
        F.set(0, 0, 1.0); // x
        F.set(0, 3, dt); // vx contribution to x
        F.set(1, 1, 1.0); // y
        F.set(1, 4, dt); // vy contribution to y

        // Heading update: theta = theta + omega*dt - bias*dt
        F.set(2, 2, 1.0); // theta
        F.set(2, 5, dt); // omega contribution
        F.set(2, 6, -dt); // bias contribution

        // Velocities (assumed constant in prediction)
        F.set(3, 3, 1.0); // vx
        F.set(4, 4, 1.0); // vy
        F.set(5, 5, 1.0); // omega

        // Bias (assumed constant with slow drift)
        F.set(6, 6, 1.0); // bias_omega

        // Create process noise matrix Q
        var Q = new Matrix<>(Nat.N7(), Nat.N7());

        // Position noise increases with velocity and time
        double pos_noise = Math.pow(dt, 2) * noiseParams.positionNoise;
        Q.set(0, 0, pos_noise);
        Q.set(1, 1, pos_noise);

        // Heading noise increases with angular velocity and time
        double heading_noise = Math.pow(dt, 2) * noiseParams.headingNoise;
        Q.set(2, 2, heading_noise);

        // Velocity noise
        double vel_noise = dt * noiseParams.velocityNoise;
        Q.set(3, 3, vel_noise);
        Q.set(4, 4, vel_noise);
        Q.set(5, 5, vel_noise);

        // Bias noise (very small as bias changes slowly)
        Q.set(6, 6, dt * noiseParams.biasNoise);

        // Predict state: x = Fx
        stateEstimate = F.times(stateEstimate);

        // Predict covariance: P = FPF' + Q
        stateCovariance = F.times(stateCovariance)
                .times(F.transpose())
                .plus(Q);
    }

    /**
     * Performs Kalman update step for any measurement
     */
    private void updateKalman(Matrix<?, N1> z, Matrix<?, N7> H, Matrix<?, ?> R) {
        // Innovation: y = z - Hx
        var innovation = z.minus(H.times(this.stateEstimate));
        
        // Innovation covariance: S = HPH' + R
        var S = H.times(stateCovariance)
            .times(H.transpose())
            .plus(R);
        
        // Kalman gain: K = PH'S^(-1)
        var K = stateCovariance.times(H.transpose())
            .times(S.inverse());
        
        // Update state: x = x + Ky
        stateEstimate = stateEstimate.plus(K.times(innovation));
        
        // Update covariance: P = (I - KH)P
        var I = Matrix.eye(Nat.N7());
        stateCovariance = I.minus(K.times(H))
            .times(stateCovariance);

        validateAndCorrectState();
    }

    /**
     * Process all available measurements in the correct order
     */
    private void processMeasurements() {
        // Process in order of update rate (fastest to slowest):
        // 1. IMU (high rate, good for quick motion changes)
        processIMUMeasurement();
        // 2. Odometry (high rate, good for relative motion)
        processOdometryMeasurement();
        // 3. Vision (low rate, good for absolute position)
        processVisionMeasurement();
    }

    /**
     * Process IMU measurements into the state estimate
     */
    private void processIMUMeasurement() {
        Optional<IMUInterface.IMUMeasurement> imuMeasurementOptional = getLatestIMUMeasurement();

        if (imuMeasurementOptional.isEmpty()) {
            return;
        }

        IMUInterface.IMUMeasurement measurement = imuMeasurementOptional.get();

        // Get mount configuration for transformations
        IMUInterface.MountConfiguration mountConfig = imu.getMountConfiguration();

        // Transform accelerations to robot frame
        double robotHeading = stateEstimate.get(2, 0);
        double cosHeading = Math.cos(robotHeading + mountConfig.yawOffset.getRadians());
        double sinHeading = Math.sin(robotHeading + mountConfig.yawOffset.getRadians());

        // Apply coordinate transformation
        double robotAccelX = measurement.rawAccelX * cosHeading - measurement.rawAccelY * sinHeading;
        double robotAccelY = measurement.rawAccelX * sinHeading + measurement.rawAccelY * cosHeading;

        // Create measurement matrix for IMU
        var H = new Matrix<>(Nat.N4(), Nat.N7());

        // IMU measures: [theta, ax, ay, omega]
        H.set(0, 2, 1.0); // theta measurement
        H.set(1, 3, 1.0); // ax relates to vx
        H.set(2, 4, 1.0); // ay relates to vy
        H.set(3, 5, 1.0); // omega measurement

        // Create measurement noise matrix
        var R = Matrix.eye(Nat.N4());
        R.set(0, 0, noiseParams.imuHeadingNoise);
        R.set(1, 1, noiseParams.imuAccelNoise);
        R.set(2, 2, noiseParams.imuAccelNoise);
        R.set(3, 3, noiseParams.imuHeadingNoise);

        // Create measurement vector
        var z = new Matrix<>(Nat.N4(), Nat.N1());
        z.set(0, 0, measurement.heading.getRadians());
        z.set(1, 0, robotAccelX);
        z.set(2, 0, robotAccelY);
        z.set(3, 0, measurement.rawGyroZ);

        // Check for significant acceleration or rotation
        double accelMagnitude = Math.sqrt(robotAccelX * robotAccelX + robotAccelY * robotAccelY);
        if (accelMagnitude > MAX_ACCEL / 2) {
            // Increase acceleration measurement noise during high acceleration
            R.set(1, 1, R.get(1, 1) * 2.0);
            R.set(2, 2, R.get(2, 2) * 2.0);
        }

        if (Math.abs(measurement.rawGyroZ) > MAX_ROTATION_RATE / 2) {
            // Increase rotation measurement noise during high rotation
            R.set(3, 3, R.get(3, 3) * 2.0);
        }

        // Update gyro bias estimate
        updateGyroBias(measurement.rawGyroZ, stateEstimate.get(5, 0));

        // Perform update
        updateKalman(z, H, R);
    }

    /**
     * Process odometry measurements into the state estimate
     */
    private void processOdometryMeasurement() {
        Optional<OdometryInterface.OdometryMeasurement> odometryMeasurementOptional = odometry.getMeasurement();
        if (odometryMeasurementOptional.isPresent()) {
            return;
        }

        OdometryInterface.OdometryMeasurement measurement = odometryMeasurementOptional.get();

        lastOdometryMeasurement = measurement;

        // Create measurement matrix for odometry
        var H = new Matrix<>(Nat.N5(), Nat.N7());

        // Odometry measures: [vx, vy, omega, x, y]
        H.set(0, 3, 1.0); // vx measurement
        H.set(1, 4, 1.0); // vy measurement
        H.set(2, 5, 1.0); // omega measurement
        H.set(3, 0, 1.0); // x measurement
        H.set(4, 1, 1.0); // y measurement

        // Create measurement noise matrix
        var R = Matrix.eye(Nat.N5());
        R.set(0, 0, noiseParams.odometryVelocityNoise);
        R.set(1, 1, noiseParams.odometryVelocityNoise);
        R.set(2, 2, noiseParams.odometryVelocityNoise);
        R.set(3, 3, noiseParams.odometryPositionNoise);
        R.set(4, 4, noiseParams.odometryPositionNoise);

        // Create measurement vector
        var z = new Matrix<>(Nat.N5(), Nat.N1());
        z.set(0, 0, measurement.vx);
        z.set(1, 0, measurement.vy);
        z.set(2, 0, measurement.omega);
        z.set(3, 0, measurement.x);
        z.set(4, 0, measurement.y);

        if (lastControlInput != null) { // Ensure you have a valid control input
            SlipDetectionResult slipResult = detectSlip((IMUMeasurement)getLatestIMUMeasurement().get(), lastOdometryMeasurement, lastControlInput);
     
     
            if (slipResult.isSlipping() && slipResult.confidence() > 0.5) { // Example threshold
                System.out.println("Slip detected! Confidence: " + slipResult.confidence());
                // Implement slip handling logic here, e.g.,
                // - Reduce trust in odometry measurements by increasing the odometry noise in the R matrix
                // - Correct the state estimate using the slip velocity
                // - Trigger other actions based on slip detection
            }
        }

        // Perform update
        updateKalman(z, H, R);
    }

    /**
     * Process vision measurements, accounting for latency and confidence
     */
    private void processVisionMeasurement() {
        Optional<VisionInterface.VisionMeasurement> visionMeasurementOptional = vision.getMeasurement();
        if (visionMeasurementOptional.isEmpty()) {
            return;
        }
        VisionInterface.VisionMeasurement measurement = visionMeasurementOptional.get();

        // Validate the measurement
        if (!vision.validateMeasurement(measurement)) {
            System.err.println("Invalid vision measurement.");
            return;
        }

        // Create measurement vector
        var z = new Matrix<>(Nat.N3(), Nat.N1());
        z.set(0, 0, measurement.x);
        z.set(1, 0, measurement.y);
        z.set(2, 0, measurement.rotation);

        // Create measurement matrix
        var H = new Matrix<>(Nat.N3(), Nat.N7());
        H.set(0, 0, 1.0); // x measurement
        H.set(1, 1, 1.0); // y measurement
        H.set(2, 2, 1.0); // rotation measurement

        // Create measurement noise matrix
        double visionNoiseScale = 1.0 / measurement.confidence;
        var R = new Matrix<>(Nat.N3(), Nat.N3());
        R.set(0, 0, noiseParams.visionPositionNoise * visionNoiseScale);
        R.set(1, 1, noiseParams.visionPositionNoise * visionNoiseScale);
        R.set(2, 2, noiseParams.visionHeadingNoise * visionNoiseScale);

        // Perform update
        updateKalman(z, H, R);
    }

    /**
     * Gets the latest IMU measurement with validation
     */
    private Optional<IMUInterface.IMUMeasurement> getLatestIMUMeasurement() {
        // Get raw measurement from IMU
        Optional<IMUInterface.IMUMeasurement> measurement = imu.getMeasurement();

        if (measurement.isEmpty()) {
            return Optional.empty();
        }
        IMUInterface.IMUMeasurement imuData = measurement.get();
        // Removed minimum update rate check as IMU should be as fast as possible
        // Validate measurements
        if (!validateIMUMeasurements(imuData)) {
            return Optional.empty();
        }
        // Update tracking
        lastIMUTimestamp = imuData.timestamp;
        return measurement;
    }

    /**
     * Validates IMU measurements for reasonableness
     */
    private boolean validateIMUMeasurements(IMUInterface.IMUMeasurement measurement) {
        if (!imu.isOperational()) {
            return false;
        }

        double accelMagnitude = Math.sqrt(measurement.rawAccelX * measurement.rawAccelX +
                measurement.rawAccelY * measurement.rawAccelY);
        if (Double.isNaN(accelMagnitude) || Double.isInfinite(accelMagnitude) || accelMagnitude > MAX_ACCEL) {
            return false;
        }

        if (Double.isNaN(measurement.rawGyroZ) || Double.isInfinite(measurement.rawGyroZ) ||
                Math.abs(measurement.rawGyroZ) > MAX_ROTATION_RATE) {
            return false;
        }

        return true;
    }

    

    /**
     * Gets the current estimated pose of the robot.
     * 
     * @return The estimated Pose2d
     */
    public Pose2d getEstimatedPose() {
        return new Pose2d(
                stateEstimate.get(0, 0),
                stateEstimate.get(1, 0),
                Rotation2d.fromRadians(stateEstimate.get(2, 0)));
    }

    /**
     * Resets the position estimator to a known pose.
     * 
     * @param pose The pose to reset to
     */
    public void reset(Pose2d pose) {
        stateEstimate.set(0, 0, pose.getX());
        stateEstimate.set(1, 0, pose.getY());
        stateEstimate.set(2, 0, pose.getRotation().getRadians());
        // Reset velocities and bias to zero
        for (int i = 3; i < STATE_SIZE; i++) {
            stateEstimate.set(i, 0, 0.0);
        }
        // Reset covariance to initial values
        stateCovariance = Matrix.eye(Nat.N7()).times(1.0);
        lastUpdateTime = timer.get();
        lastValidPose = pose;
    }

    /**
     * Update the current control inputs
     */
    public void updateControlInput(double vxCommand, double vyCommand, double omegaCommand) {
        lastControlInput = new ControlInput(vxCommand, vyCommand, omegaCommand, timer.get());
    }

    /**
     * Validates and corrects state estimate using field map
     */
    private void validateAndCorrectState() {
        Pose2d currentPose = getEstimatedPose();

        // Check if pose is within field bounds
        if (!fieldMap.isPoseWithinBounds(currentPose)) {
            // If outside bounds, increase uncertainty and project to nearest valid pose
            increaseStateUncertainty();

            Optional<Pose2d> validPose = fieldMap.getNearestValidPose(currentPose);
            validPose.ifPresent(this::correctStateEstimate); // Use method reference
        }

        // Check for nearby features for improved estimation
        // Optional<FieldFeature> nearestFeature = fieldMap.getNearestFeature(currentPose);
        // if (nearestFeature.isPresent() &&
        //         nearestFeature.get().getDistance(currentPose) < FEATURE_CORRECTION_THRESHOLD) {
        //     improveEstimateUsingFeature(nearestFeature.get());
        // }
    }

    /**
     * Increases uncertainty in the state estimate
     */
    private void increaseStateUncertainty() {
        // Increase position and heading uncertainty (using COVARIANCE_INCREASE_FACTOR)
        for (int i = 0; i < 3; i++) {
            stateCovariance.set(i, i, stateCovariance.get(i, i) * COVARIANCE_INCREASE_FACTOR);
        }
    }

    /**
     * Corrects state estimate to a valid pose
     */
    private void correctStateEstimate(Pose2d validPose) {
        // Correct position and heading
        stateEstimate.set(0, 0, validPose.getX());
        stateEstimate.set(1, 0, validPose.getY());
        stateEstimate.set(2, 0, validPose.getRotation().getRadians());

        // Increase velocity uncertainty after correction
        for (int i = 3; i < 6; i++) {
            stateCovariance.set(i, i, stateCovariance.get(i, i) * COVARIANCE_INCREASE_FACTOR);
        }

        lastValidPose = validPose;
    }

    /**
     * Improves state estimate using nearby field features
     */
    // private void improveEstimateUsingFeature(FieldFeature feature) {
    //     // Create measurement based on feature
    //     var z = new Matrix<>(Nat.N2(), Nat.N1());
    //     z.set(0, 0, feature.getPosition().getX());
    //     z.set(1, 0, feature.getPosition().getY());

    //     // Create measurement matrix (only affects position)
    //     var H = new Matrix<>(Nat.N2(), Nat.N7());
    //     H.set(0, 0, 1.0);
    //     H.set(1, 1, 1.0);

    //     // Create measurement noise based on distance to feature
    //     double featureNoise = feature.getDistance(getEstimatedPose()) * 0.1;
    //     var R = Matrix.eye(Nat.N2()).times(featureNoise);

    //     // Perform update
    //     updateKalman(z, H, R);
    // }

    private record ValidationResult(boolean isValid, Optional<Pose2d> correctedPose, double confidence) {
    }

    /**
     * Represents the control inputs sent to the drive base
     */
    private class ControlInput {
        public final double vxCommand; // Commanded X velocity (m/s)
        public final double vyCommand; // Commanded Y velocity (m/s)
        public final double omegaCommand; // Commanded angular velocity (rad/s)
        public final double timestamp;

        public ControlInput(double vxCommand, double vyCommand, double omegaCommand, double timestamp) {
            this.vxCommand = vxCommand;
            this.vyCommand = vyCommand;
            this.omegaCommand = omegaCommand;
            this.timestamp = timestamp;
        }
    }

    /**
     * Detects slip using IMU and control input data.
     *
     * @param imuMeasurement      The latest IMU measurement.
     * @param odometryMeasurement The latest odometry measurement.
     * @param controlInput        The latest control input.
     * @return A SlipDetectionResult containing slip information.
     */

    private SlipDetectionResult detectSlip(IMUInterface.IMUMeasurement imuMeasurement,
            OdometryInterface.OdometryMeasurement odometryMeasurement, ControlInput controlInput) {

        double currentTime = timer.get();
        double dt = currentTime - lastUpdateTime;

        if (dt <= 0 || lastOdometryMeasurement == null) {
            return new SlipDetectionResult(false, 0.0, 0.0 ,0.0 ); // Handle edge cases
        }

        // 1. IMU-Based Slip Detection (comparing expected vs. measured acceleration and
        // heading)
        Pose2d odometryPose2d = new Pose2d(odometryMeasurement.x,odometryMeasurement.y,Rotation2d.fromRadians(odometryMeasurement.omega));
        Pose2d lastodometryPose2d = new Pose2d(lastOdometryMeasurement.x,lastOdometryMeasurement.y,Rotation2d.fromRadians(lastOdometryMeasurement.omega));
        
        Twist2d odometryTwist = odometryPose2d.log(lastodometryPose2d); // Use proper twist
                                                                                            // calculation
        double expectedAccelX = odometryTwist.dx / dt;
        double expectedAccelY = odometryTwist.dy / dt;

        // Use appropriate rotation calculation
        double expectedAngularVelocity = odometryTwist.dtheta / dt;

        double accelDiffX = Math.abs(imuMeasurement.rawAccelX - expectedAccelX);
        double accelDiffY = Math.abs(imuMeasurement.rawAccelY - expectedAccelY);

        double gyroDiff = Math.abs(imuMeasurement.rawGyroZ - expectedAngularVelocity);

        boolean imuSlip = accelDiffX > SLIP_ACCEL_THRESHOLD || accelDiffY > SLIP_ACCEL_THRESHOLD
                || gyroDiff > HEADING_DIFF_THRESHOLD;

        // 2. Control-Based Slip Detection (comparing commanded vs. measured velocity)
        double velDiffX = Math.abs(odometryMeasurement.vx - controlInput.vxCommand);
        double velDiffY = Math.abs(odometryMeasurement.vy - controlInput.vyCommand);
        boolean controlSlip = velDiffX > SLIP_VELOCITY_THRESHOLD || velDiffY > SLIP_VELOCITY_THRESHOLD;

        // 3. Combine Slip Detection Results and Calculate Confidence
        boolean isSlipping = imuSlip || controlSlip;
        double slipConfidence = calculateSlipConfidence(imuSlip, controlSlip, accelDiffX, accelDiffY, gyroDiff,
                velDiffX, velDiffY);

        // 4. Calculate Slip Velocity (if slipping)
        double slipVelocityX = 0;
        double slipVelocityY = 0;

        if (isSlipping) {
            slipVelocityX = odometryMeasurement.vx - controlInput.vxCommand;
            slipVelocityY = odometryMeasurement.vy - controlInput.vyCommand;
        }

        return new SlipDetectionResult(isSlipping, slipConfidence, slipVelocityX, slipVelocityY);
    }
    
    /**
     * Calculates slip confidence based on various slip indicators.
     */
    private double calculateSlipConfidence(boolean imuSlip, boolean controlSlip, double accelDiffX, double accelDiffY,
            double gyroDiff, double velDiffX, double velDiffY) {
        // Example implementation - tune weights as needed
        double imuConfidence = 0.0;
        if (imuSlip) {
            imuConfidence = 0.5 + 0.2 * (accelDiffX / SLIP_ACCEL_THRESHOLD) + 0.2 * (accelDiffY / SLIP_ACCEL_THRESHOLD)
                    + 0.1 * (gyroDiff / HEADING_DIFF_THRESHOLD);

        }

        double controlConfidence = 0.0;
        if (controlSlip) {
            controlConfidence = 0.5 + 0.3 * (velDiffX / SLIP_VELOCITY_THRESHOLD)
                    + 0.2 * (velDiffY / SLIP_VELOCITY_THRESHOLD);
        }

        return Math.max(imuConfidence, controlConfidence);
    }

    private record SlipDetectionResult(boolean isSlipping, double confidence, double slipVelocityX, double slipVelocityY){}

    /**
     * Updates the gyro bias estimate based on the latest gyro measurement and the current estimated bias.
     */
    private void updateGyroBias(double rawGyroZ, double currentBias) {
        // Example implementation: simple bias correction
        double biasUpdateRate = 0.1; // Adjust this rate as necessary
        double newBias = currentBias + biasUpdateRate * (rawGyroZ - currentBias);
        stateEstimate.set(6, 0, newBias); // Update the bias in the state estimate
    }

}
