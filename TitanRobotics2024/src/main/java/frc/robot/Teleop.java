package frc.robot;
import frc.robot.DriveBase.*;
import frc.robot.Devices.*;
public class Teleop {
  public Controller controller;
  private DriveBase drive_base;
  public Teleop(Controller controller, DriveBase drive_base) {
    this.controller = controller;
    this.drive_base = drive_base;
  }
  private double MAX_SPEED = 10; // meters per second
  public void update() {
    double x = this.controller.getRightX();
    double y = this.controller.getRightY();
    double theta = this.controller.getLeftX();
    x = Math.abs(x)>0.1 ? x : 0.0;
    y = Math.abs(y)>0.1 ? y : 0.0;
    theta = Math.abs(theta)>0.1 ? theta : 0.0;
    double[] vector = new double[] { x * MAX_SPEED, y * MAX_SPEED};
    this.drive_base.robotOrientedDriveVelocity(vector, theta);
  }
}
