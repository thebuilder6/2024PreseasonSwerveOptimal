package frc.robot.Devices;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface AbsoluteEncoder extends Device {
  public Measure<Angle> getAbsolutePosition();

  public Measure<Velocity<Angle>> getAngularVelocity();
}
