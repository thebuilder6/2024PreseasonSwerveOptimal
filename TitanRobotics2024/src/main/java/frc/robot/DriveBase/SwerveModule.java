package frc.robot.DriveBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import frc.robot.Devices.*;

public class SwerveModule {
  public FeedbackMotor steer;
  public FeedbackMotor drive;

  public SwerveModule(FeedbackMotor steer, FeedbackMotor drive) {
    this.steer = steer;
    this.drive = drive;
  }

  private double magnitude(double[] point) {
    return Math.sqrt(Math.pow(point[0], 2) + Math.pow(point[1], 2));
  }

  private double angle(double[] point) {
    return Math.atan2(point[1], point[0]);
  }

  public void setVelVec(double[] vec) {
    this.setAngle(this.angle(vec));
    this.setSpeed(this.magnitude(vec));
  }

  public void setAngle(double angle) {
    this.steer.setPositionAngle(Units.Radians.of(angle));
    //this.rotation.setVelocityRadPerSec(angle);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(this.steer.getAngle().in(Units.Radians));
  }

  public double getWheelVelMPS() {
    return this.drive.getVelocity().in(Units.MetersPerSecond);
  }

  public void setSpeed(double speed) {
    this.drive.setVelocity(Units.MetersPerSecond.of(speed));
  }
}
