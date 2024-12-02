package frc.robot.DeviceImpls;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import frc.robot.Devices.BasicMotor;
//https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax

/**
 * A class that wraps a CANSparkMaxNeoMotor and implements the basic motor interface as well as the relative encoder interface.
 * <p>
 * This class is intended to be used as a drop in replacement for any other motor object that is 
 * used in the robot codebase. It provides a simple interface for setting the motor speed and reading the motor position and velocity.
 * <p>
 * The motor is assumed to be a CANSparkMaxNeoMotor, and the internal encoder is assumed to be a Rev Through Bore Encoder.
 * <p>
 * The motor is set to brake mode by default, and the internal encoder is set to be the position and velocity source.
 */
public class CANSparkMaxNeoMotor implements frc.robot.Devices.RelativeEncoder, BasicMotor {
    private int motorID;
    private CANSparkMax motor;
    private boolean isEnabled = false;
    //TODO: Add feedback motor
    //TODO: Add non-feedback motor
    //TODO: Allow for Encoder passthrough
    public boolean internalEncoder = false;

    /**
     * Creates a new instance of this class with the given motor ID.
     * @param motorID The CAN ID of the motor to control.
     */
    public CANSparkMaxNeoMotor(int motorID) {
        this.motorID = motorID;
        try {
            motor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushless);
            motor.getDeviceId();//just to make sure it works
            isEnabled = true;
        } catch (Exception e) {
            System.out.println("CANSparkMax " + motorID + " not found");
            isEnabled = false;
        }
        if (isEnabled && motor.getEncoder() != null) {
            internalEncoder = true;
        } else {
            internalEncoder = false;
        }

    }

    /**
     * Sets the motor speed to the given value.
     * @param speed The motor speed to set, where 0 is stop and 1 is full speed.
     */
    @Override
    public void set(double speed) {
        if (isEnabled) {
            motor.set(speed);
        }
    }

    /**
     * Returns whether the motor is enabled or not.
     * @return True if the motor is enabled, false otherwise.
     */
    public boolean isEnabled() {
        return isEnabled;
    }

    /**
     * Sets the motor to the given break mode.
     * @param mode The break mode to set, one of {@link CANSparkMax.IdleMode#kBrake} or {@link CANSparkMax.IdleMode#kCoast}.
     */
    public void setBreakMode(CANSparkMax.IdleMode mode) {
        if (isEnabled) {
            motor.setIdleMode(mode);
        }
    }

    /**
     * Sets the motor to be inverted or not.
     * @param invert True if the motor should be inverted, false otherwise.
     */
    public void setInverted(boolean invert) {
        if (isEnabled) {
            motor.setInverted(invert);
        }
    }

    /**
     * Stops the motor.
     */
    public void stop() {
        if (isEnabled) {
            motor.set(0);
        }
    }

    /**
     * Returns the current position of the motor in radians.
     * @return The current position of the motor in radians.
     */
    @Override
    public Measure<Angle> getPosition() {
        if (internalEncoder) {
            return Units.Rotations.of(motor.getEncoder().getPosition());
        } else {
            return Units.Rotations.of(0);
        }
    }

    /**
     * Returns the current velocity of the motor in radians per second.
     * @return The current velocity of the motor in radians per second.
     */
    @Override
    public Measure<Velocity<Angle>> getAngularVelocity() {
        if (internalEncoder) {
            return Units.RotationsPerSecond.of(motor.getEncoder().getVelocity());
        } else {
            return Units.RotationsPerSecond.of(0);
        }
    }

    /**
     * Disables the motor and stops it from moving.
     */
    @Override
    public void disable() {
        isEnabled = false;
        motor.disable();
    }

    /**
     * Returns whether the motor is inverted or not.
     * @return True if the motor is inverted, false otherwise.
     */
    @Override
    public boolean getInverted() {
        if (isEnabled) {
            return motor.getInverted();
        } else {
            return false;
        }
    }

    /**
     * Returns the current motor output.
     * @return The current motor output.
     */
    @Override
    public double get() {
        if (isEnabled) {
            return motor.get();
        } else {
            return 0;
        }
    }

    /**
     * Stops the motor immediately.
     */
    @Override
    public void stopMotor() {
        if (isEnabled) {
            motor.stopMotor();
        }
    }

    /**
     * Sets the motor output voltage directly.
     * @param voltage The motor output voltage to set.
     */
    @Override
    public void setVoltage(double voltage) {
        if (isEnabled) {
            motor.setVoltage(voltage);
        }
    }

}
