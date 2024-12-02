package frc.robot.Devices;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Angle;

public interface FeedbackMotor {
  public void setPositionAngle(Measure<Angle> position);

  public void setAngularVelocity(Measure<Velocity<Angle>> velocity);

  public void setVelocity(Measure<Velocity<Distance>> velocity);

  public void setAccelerationRadPerSecSquared(double accelerationRadPerSecSquared);

  public Measure<Angle> getAngle();

  public Measure<Velocity<Angle>> getAngularVelocity();

  public Measure<Distance> getPosition();

  public Measure<Velocity<Distance>> getVelocity();
}
