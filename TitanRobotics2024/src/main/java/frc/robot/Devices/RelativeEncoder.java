package frc.robot.Devices;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface RelativeEncoder extends Device {
  public Measure<Angle> getPosition();

  public Measure<Velocity<Angle>> getAngularVelocity();
}
