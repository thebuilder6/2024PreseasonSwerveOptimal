package frc.robot.DeviceImpls;

import java.util.concurrent.atomic.AtomicLongFieldUpdater;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Settings;
import frc.robot.Devices.*;

//https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/ProfiledPIDController.html

/**
 * A class for a motor with an absolute encoder that can be
 * profile-controlled to a position using a PID controller. The
 * motor is assumed to be a rev brushless motor.
 * <p>
 * Requires a motor class that implements the FeedbackMotor interface
 * and an encoder class that implements the AbsoluteEncoder interface.
 * <p>
 * The motor is controlled by setting the target position in radians.
 * The PID controller is tuned to control the motor to the target
 * position. The motor's position is checked every loop and the
 * PID controller is updated with the current position. The motor's
 * velocity and acceleration are calculated by the PID controller.
 * <p>
 * The motor's position, velocity, and target position are published
 * to the SmartDashboard.
 * <p>
 * The motor's enable status is published to the SmartDashboard.
 * <p>
 * The motor's target position is set using the setPositionRad method.
 * <p>
 * The motor's velocity and acceleration are calculated by the PID
 * controller and are not set directly.
 * <p>
 * The motor's target position is set in radians. The motor's position
 * is read in radians. The motor's velocity is calculated in
 * radians per second. The motor's acceleration is calculated in
 * radians per second squared.
 * <p>
 * The motor is enabled when the constructor is called. The motor is
 * disabled when the disable method is called.
 * <p>
 * The motor's enable status is published to the SmartDashboard.
 * <p>
 * The motor's target position is published to the SmartDashboard.
 * <p>
 * The motor's actual position is published to the SmartDashboard.
 * <p>
 * The motor's actual velocity is published to the SmartDashboard.
 * <p>
 * The motor's actual acceleration is not published to the
 * SmartDashboard.
 * <p>
 * The motor's target velocity is published to the SmartDashboard.
 * <p>
 * The motor's target acceleration is published to the SmartDashboard.
 * <p>
 * The motor's actual position, velocity, and acceleration are
 * published to the SmartDashboard.
 */
public class SwerveSteerFeedbackMotor implements FeedbackMotor {
    private String id;
    private BasicMotor motor;

    private AbsoluteEncoder encoder;
    public boolean isEnabled = false;
    public double targetPositionRad = 0.0;
    public double voltage;
    
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Settings.SwerveSteerConstants.Ks,
            Settings.SwerveSteerConstants.Kv, Settings.SwerveSteerConstants.Ka);
    private ProfiledPIDController pidController = new ProfiledPIDController(
            Settings.SwerveSteerConstants.Kp,
            Settings.SwerveSteerConstants.Ki,
            Settings.SwerveSteerConstants.Kd,
            new TrapezoidProfile.Constraints(
                    Settings.SwerveSteerConstants.maxVelocityConstraint,
                    Settings.SwerveSteerConstants.maxAccelerationConstraint));
    // TODO Implement tuning live in dashboard 

    public SwerveSteerFeedbackMotor(String id, BasicMotor motor, AbsoluteEncoder encoder) {
        this.id = id;
        motor.setInverted(true);
        this.motor = motor;
        this.encoder = encoder;

        if (motor == null || motor.isEnabled() == false) {
            System.out.println("Motor for " + id + " not found");
            isEnabled = false;
        } else {
            isEnabled = true;
        }
        if (encoder == null || encoder.isEnabled() == false) {
            System.out.println("Encoder for " + id + " not found");
            //TODO: attempt to use sparkmax/neo encoder
            //TODO: else move to emergency no encoder mode
        }
        pidController.enableContinuousInput(-Math.PI, Math.PI); //allows for full circle
    }

    public void updateConstants() {
        feedforward = new SimpleMotorFeedforward(Settings.SwerveSteerConstants.Ks, Settings.SwerveSteerConstants.Kv,
                Settings.SwerveSteerConstants.Ka);
        pidController.setPID(Settings.SwerveSteerConstants.Kp, Settings.SwerveSteerConstants.Ki,
                Settings.SwerveSteerConstants.Kd);
        pidController
                .setConstraints(new TrapezoidProfile.Constraints(Settings.SwerveSteerConstants.maxVelocityConstraint,
                        Settings.SwerveSteerConstants.maxAccelerationConstraint));
    }

    public void setPositionAngle(Measure<Angle> targetPosition) {
        double targetPositionRad = targetPosition.in(Units.Radians);
        updateConstants(); // maybe problem 

        SmartDashboard.putNumber(id + "/targetPositionRad", targetPositionRad);
        this.targetPositionRad = targetPositionRad;
        if (isEnabled) {
            voltage = pidController.calculate(encoder.getAbsolutePosition().in(Units.Radians), targetPositionRad)
                    + feedforward.calculate(pidController.getSetpoint().velocity);
            motor.setVoltage(voltage);
        }
        SmartDashboard.putNumber(id + "/actualPositionRad", encoder.getAbsolutePosition().in(Units.Radians));
        SmartDashboard.putNumber(id + "/actualVelocityRadPerSec",
                encoder.getAngularVelocity().in(Units.RadiansPerSecond));
        SmartDashboard.putNumber(id + "/targetVelocityRadPerSec", pidController.getSetpoint().velocity);
        SmartDashboard.putNumber(id + "/voltage", voltage);
        // could add acceleration controls using timer
    }

    public void setAngularVelocity(Measure<Velocity<Angle>> velocity) {
        double velocityRad = velocity.in(Units.RadiansPerSecond);
        SmartDashboard.putNumber(id + "/targetVelocityRadPerSec", velocityRad);

        // could add control for velocity would need separate controller 
        //or might be better to modify the trapezoid mid stream
        if (isEnabled) {
            voltage = feedforward.calculate(velocityRad);
            motor.setVoltage(voltage);
        }
        //SmartDashboard.putNumber(id + "/actualPositionRad", encoder.getAbsolutePositionRad());
        SmartDashboard.putNumber(id + "/actualVelocityRadPerSec",
                encoder.getAngularVelocity().in(Units.RadiansPerSecond));
        SmartDashboard.putNumber(id + "/voltage", voltage);
    }

    public void setAccelerationRadPerSecSquared(double accelerationRadPerSecSquared) {
        SmartDashboard.putNumber(id + "/targetAccelerationRadPerSecSquared", accelerationRadPerSecSquared);
        // could add control for acceleration dont know why you would need it
    }

    public Measure<Angle> getAngle() {
        return encoder.getAbsolutePosition();
    }

    public Measure<Velocity<Angle>> getAngularVelocity() {
        return encoder.getAngularVelocity();
    }

    public void log() {
        SmartDashboard.putNumber(id + "/targetPositionRad", targetPositionRad);
        SmartDashboard.putNumber(id + "/actualPositionRad", encoder.getAbsolutePosition().in(Units.Radians));
        SmartDashboard.putNumber(id + "/actualVelocityRadPerSec",
                encoder.getAngularVelocity().in(Units.RadiansPerSecond));
        SmartDashboard.putNumber(id + "/targetVelocityRadPerSec", pidController.getSetpoint().velocity);
    }

    @Override
    public Measure<Distance> getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public Measure<Velocity<Distance>> getVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
    }

    @Override
    public void setVelocity(Measure<Velocity<Distance>> velocity) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVelocityDistance'");
    }

}
