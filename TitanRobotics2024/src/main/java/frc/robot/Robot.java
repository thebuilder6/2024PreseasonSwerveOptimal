// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.PortMap;
import frc.robot.Data.Settings;
import frc.robot.Teleop;
import frc.robot.DeviceImpls.CANSparkMaxNeoMotor;
import frc.robot.DeviceImpls.CANcoderEncoder;
import frc.robot.DeviceImpls.DummyFeedbackMotor;
import frc.robot.DeviceImpls.SwerveSteerFeedbackMotor;
import frc.robot.DeviceImpls.SwerveDriveFeedbackMotor;
import frc.robot.Devices.*;
import frc.robot.DriveBase.DriveBase;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private DriveBase driveBase;
  private Teleop teleop;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Settings.init();
    ArrayList<FeedbackMotor> rotateMotors = new ArrayList<>(Arrays.asList(
        new SwerveSteerFeedbackMotor("rotate0", new CANSparkMaxNeoMotor(PortMap.RFSteerSwerveMotor),
            new CANcoderEncoder(PortMap.RFSteerSwerveEncoder)),
        new SwerveSteerFeedbackMotor("rotate1", new CANSparkMaxNeoMotor(PortMap.LFSteerSwerveMotor),
            new CANcoderEncoder(PortMap.LFSteerSwerveEncoder)),
        new SwerveSteerFeedbackMotor("rotate2", new CANSparkMaxNeoMotor(PortMap.LBSteerSwerveMotor),
            new CANcoderEncoder(PortMap.LBSteerSwerveEncoder)),
        new SwerveSteerFeedbackMotor("rotate3", new CANSparkMaxNeoMotor(PortMap.RBSteerSwerveMotor),
            new CANcoderEncoder(PortMap.RBSteerSwerveEncoder))));
    ArrayList<FeedbackMotor> spinMotors = new ArrayList<>(Arrays.asList(
        new SwerveDriveFeedbackMotor("spin0", new CANSparkMaxNeoMotor(PortMap.RFDriveSwerveMotor),
            new CANcoderEncoder(PortMap.RFSteerSwerveEncoder)),
        new SwerveDriveFeedbackMotor("spin1", new CANSparkMaxNeoMotor(PortMap.LFDriveSwerveMotor),
            new CANcoderEncoder(PortMap.LFSteerSwerveEncoder)),
        new SwerveDriveFeedbackMotor("spin2", new CANSparkMaxNeoMotor(PortMap.LBDriveSwerveMotor),
            new CANcoderEncoder(PortMap.LBSteerSwerveEncoder)),
        new SwerveDriveFeedbackMotor("spin3", new CANSparkMaxNeoMotor(PortMap.RBDriveSwerveMotor),
            new CANcoderEncoder(PortMap.RBSteerSwerveEncoder))));
    ArrayList<double[]> locations = new ArrayList<>(Arrays.asList(
        new double[] { 1.0, 1.0 },
        new double[] { -1.0, 1.0 },
        new double[] { -1.0, -1.0 },
        new double[] { 1.0, -1.0 }));
    this.driveBase = new DriveBase(rotateMotors, spinMotors, locations);
    
    Controller controller = new Controller(PortMap.CONTROLLER);
    this.teleop = new Teleop(controller, this.driveBase);
  }

  /**
   * This function is called every 20 ms, no matter the mission. Use this for items
   * like diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   * <p>
   * This runs after the mission specific periodic functions, but before LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Settings.update();
    //System.out.println("Hello world");
    SmartDashboard.putString("hello", "world");
    this.teleop.update();
    this.driveBase.update();
  }

  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
