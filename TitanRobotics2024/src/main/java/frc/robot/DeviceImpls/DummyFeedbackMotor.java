package frc.robot.DeviceImpls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Devices.*;

public class DummyFeedbackMotor implements FeedbackMotor {
    private String id;

    public DummyFeedbackMotor(String id) {
        this.id = id;
    }

    public void setPosition(double positionRad) {
        SmartDashboard.putNumber(id + "/positionRad", positionRad);
    }

    public void setAngluarVelocity(double velocityRad) {
        SmartDashboard.putNumber(id + "/velocityRadPerSec", velocityRad);
    }

    public void setAccelerationRadPerSecSquared(double accelerationRadPerSecSquared) {

    }

    @Override
    public Measure<Angle> getAngle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPositionRad'");
    }

    @Override
    public Measure<Velocity<Angle>> getAngularVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAngularVelocity'");
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
    public void setVelocity(Measure<Velocity<Distance>> velocity) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVelocity'");
    }

}