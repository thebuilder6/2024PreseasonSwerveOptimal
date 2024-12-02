package frc.robot.Devices;

public interface NonFeedbackMotor extends Device {
  public double getSpeedMaxRadPerSec();

  public double getSpeedMinNormalized();

  public void setSpeedNormalized(double speedNormalized);
}
