package frc.robot.DeviceImpls;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Devices.*;
import frc.robot.Data.Settings;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.LinearSystemLoop;

//https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/ProfiledPIDController.html

/**
 * Represents a Swerve Drive Feedback Motor, which is a motor that can be controlled to achieve a specific velocity or position.
 * It uses a Linear Quadratic Regulator (LQR) to control the motor's voltage output based on the current position and velocity.
 * 
 * @author Titan Robotics
 */
public class SwerveDriveFeedbackMotor implements FeedbackMotor {
    private String id;
    private BasicMotor motor;
    private RelativeEncoder encoder;
    private boolean isEnabled = false;
    private double voltage;

    // Single control system for both position and velocity
    private final LinearSystem<N2, N1, N2> drivetrainPlant;
    private final LinearQuadraticRegulator<N2, N1, N2> driveLQR;
    private final LinearSystemLoop<N2, N1, N2> driveLoop;
    private static final double MAX_VOLTAGE = 12.0; // Maximum voltage that can be applied to the motor
    private static final double DT = 0.020;  // 20ms control loop time step

    /**
     * Constructs a new SwerveDriveFeedbackMotor instance.
     * 
     * @param id The unique identifier for this motor.
     * @param motor The BasicMotor instance associated with this SwerveDriveFeedbackMotor.
     * @param encoder The RelativeEncoder instance associated with this SwerveDriveFeedbackMotor.
     */
    public SwerveDriveFeedbackMotor(String id, BasicMotor motor, RelativeEncoder encoder) {
        this.id = id;
        this.motor = motor;
        this.encoder = encoder;

        if (motor == null || !motor.isEnabled()) {
            System.out.println("Motor for " + id + " not found");
            isEnabled = false;
        } else {
            isEnabled = true;
        }

        if (encoder == null || !encoder.isEnabled()) {
            System.out.println("Encoder for " + id + " not found");
            //TODO: attempt to use sparkmax/neo encoder
            //TODO: else move to emergency no encoder mode
        }

        // Create plant and controller
        drivetrainPlant = LinearSystemId.createDCMotorSystem(
            Settings.SwerveDriveConstants.Kv,
            Settings.SwerveDriveConstants.Ka
        );

        driveLQR = new LinearQuadraticRegulator<>(
            drivetrainPlant,
            VecBuilder.fill(0.2, 1.0),  // Position and velocity error costs
            VecBuilder.fill(12.0),      // Control effort cost
            DT
        );

        driveLoop = new LinearSystemLoop<>(
            drivetrainPlant,
            driveLQR,
            null,
            MAX_VOLTAGE,
            DT
        );
    }

    /**
     * Sets the target velocity for the motor.
     * 
     * @param targetVelocity The target velocity in meters per second.
     */
    @Override
    public void setVelocity(Measure<Velocity<Distance>> targetVelocity) {
        double targetVelocityMPS = targetVelocity.in(Units.MetersPerSecond);
        SmartDashboard.putNumber(id + "/target_velocity_mps", targetVelocityMPS);
        if (!isEnabled) return;

        driveLoop.setNextR(VecBuilder.fill(
            getPosition().in(Units.Meters),
            targetVelocityMPS
        ));
        
        updateController();
    }

    /**
     * Sets the target position for the motor.
     * 
     * @param targetPosition The target position in meters.
     */
    public void setPosition(Measure<Distance> targetPosition) {
        SmartDashboard.putNumber(id + "/target_position_meters", targetPosition.in(Units.Meters));
        
        if (!isEnabled) return;
        driveLoop.setNextR(VecBuilder.fill(
            targetPosition.in(Units.Meters),
            0.0
        ));
        
        updateController();
    }

    /**
     * Updates the controller with the current state and calculates the voltage to apply to the motor.
     */
    private void updateController() {
        driveLoop.correct(VecBuilder.fill(
            getPosition().in(Units.Meters),
            getVelocity().in(Units.MetersPerSecond)
        ));
        
        driveLoop.predict(0.020);
        voltage = driveLoop.getU(0);
        motor.setVoltage(voltage);
        log();
    }

    /**
     * Returns the current position of the motor in meters.
     * 
     * @return The current position in meters.
     */
    @Override
    public Measure<Distance> getPosition() {
        return Units.Meters.of(
            encoder.getPosition().in(Units.Rotations) 
            * Settings.WHEEL_CIRCUMFERENCE.in(Units.Meters) 
            / Settings.SWERVE_DRIVE_GEAR_RATIO
        );
    }

    /**
     * Returns the current velocity of the motor in meters per second.
     * 
     * @return The current velocity in meters per second.
     */
    @Override
    public Measure<Velocity<Distance>> getVelocity() {
        return Units.MetersPerSecond.of(
            encoder.getAngularVelocity().in(Units.RotationsPerSecond) 
            * Settings.WHEEL_CIRCUMFERENCE.in(Units.Meters) 
            / Settings.SWERVE_DRIVE_GEAR_RATIO
        );
    }

    /**
     * Returns the current angle of the motor.
     * 
     * @return The current angle.
     */
    @Override
    public Measure<Angle> getAngle() {
        return encoder.getPosition();
    }

    /**
     * Logs the current state of the motor to the SmartDashboard.
     */
    private void log() {
        SmartDashboard.putNumber(id + "/position_meters", getPosition().in(Units.Meters));
        SmartDashboard.putNumber(id + "/velocity_mps", getVelocity().in(Units.MetersPerSecond));
        SmartDashboard.putNumber(id + "/voltage", voltage);
    }

    @Override
    public void setPositionAngle(Measure<Angle> position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPositionAngle'");
    }

    @Override
    public void setAngularVelocity(Measure<Velocity<Angle>> velocity) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAngularVelocity'");
    }

    @Override
    public void setAccelerationRadPerSecSquared(double accelerationRadPerSecSquared) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAccelerationRadPerSecSquared'");
    }

    @Override
    public Measure<Velocity<Angle>> getAngularVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAngularVelocity'");
    }
    
}
