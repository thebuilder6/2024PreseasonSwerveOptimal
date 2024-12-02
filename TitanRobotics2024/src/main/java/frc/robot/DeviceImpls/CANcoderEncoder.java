package frc.robot.DeviceImpls;

import frc.robot.Devices.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;
//https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/CANcoder.html
//it's a CANcoder i think is a AbsoluteEncoder and RelativeEncoder or it switches between them

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

/**
 * A CANcoder is a position and velocity sensor that can be used to sense the position and velocity of a motor.
 * This class implements the RelativeEncoder and AbsoluteEncoder interfaces.
 * The CANcoder is used in the SwerveDrive module to control the movement of the
 * robot. It is used in the SwerveSteerFeedbackMotor to control the steering of the robot.
 */
public class CANcoderEncoder implements RelativeEncoder, frc.robot.Devices.AbsoluteEncoder {
    private CANcoder cancoder;
    private final int ID;
    public boolean isEnabled = true;

    /**
     * Constructor for CANcoderEncoder.
     * @param ID The ID of the CANCoder.
     */
    public CANcoderEncoder(Integer ID) {
        this.ID = ID;
        try {
            cancoder = new CANcoder(ID);
            cancoder.getVersion();//just to make sure it works
        } catch (Exception e) {
            System.out.println("CANcoder " + ID + " not found");
            cancoder = null;
            isEnabled = false;
        }
    }

    /**
     * Gets whether the CANCoder is enabled.
     * @return Whether the CANCoder is enabled.
     */
    @Override
    public boolean isEnabled() {
        return isEnabled;
    }

    /**
     * Gets the current position of the motor in radians.
     * @return The current position of the motor in radians.
     */
    @Override
    public Measure<Angle> getPosition() {
        return Units.Rotations.of(cancoder.getPosition().getValue());
    }

    /**
     * Gets the current velocity of the motor in radians per second.
     * @return The current velocity of the motor in radians per second.
     */
    @Override
    public Measure<Velocity<Angle>> getAngularVelocity() {
        return Units.RotationsPerSecond.of(cancoder.getVelocity().getValue());
    }

    /**
     * Gets the current absolute position of the motor in radians.
     * @return The current absolute position of the motor in radians.
     */
    @Override
    public Measure<Angle> getAbsolutePosition() {
        return Units.Rotations.of(cancoder.getAbsolutePosition().getValue());
    }

    /**
     * Sets the position of the motor to zero.
     */
    public void zeroPosition() {
        cancoder.setPosition(0);
    }

}
