package frc.robot.DriveBase;

import frc.robot.Data.Settings;
import frc.robot.Devices.*;
import java.util.ArrayList;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class DriveBase {
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator poseEstimator;
  private ArrayList<SwerveModule> wheels;
  private SwerveModuleState[] states;
  public double[] straightVec;
  public double rotSpeed;
  private ChassisSpeeds speeds;
  private IMUInterface imu;
  private VisionInterface vision;


  public DriveBase(ArrayList<FeedbackMotor> rotateMotors, ArrayList<FeedbackMotor> spinMotors,
      ArrayList<double[]> locations, IMUInterface imu, VisionInterface vision) {
    ArrayList<SwerveModule> wheels = new ArrayList<SwerveModule>();
    for (int i = 0; i < rotateMotors.size(); i++) {
      wheels.add(new SwerveModule(rotateMotors.get(i), spinMotors.get(i)));
    }
    this.wheels = wheels;
    this.straightVec = new double[] { 0.0, 0.0 };
    this.rotSpeed = 0.0;
    Translation2d m_frontLeftLocation = new Translation2d(locations.get(0)[0], locations.get(0)[1]);
    Translation2d m_frontRightLocation = new Translation2d(locations.get(1)[0], locations.get(1)[1]);
    Translation2d m_backLeftLocation = new Translation2d(locations.get(2)[0], locations.get(2)[1]);
    Translation2d m_backRightLocation = new Translation2d(locations.get(3)[0], locations.get(3)[1]);
    this.kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
        m_backRightLocation);
    this.imu = imu;
    this.vision = vision;

    // this.odometry = new SwerveDriveOdometry(this.kinematic);
  }

  public void robotOrientedDriveVelocity(double[] straightVec, double rotSpeed) {
    speeds = new ChassisSpeeds(straightVec[0], straightVec[1], rotSpeed);
    
  }

  public void fieldOrientedDriveVelocity(double[] straightVec, double rotSpeed, Rotation2d robotOrientaion) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(straightVec[0], straightVec[1], rotSpeed, robotOrientaion);
  }

  public ChassisSpeeds getSpeeds(){
     return kinematics.toChassisSpeeds( new SwerveModuleState(wheels.get(0).getWheelVelMPS(), wheels.get(0).getAngle()),
                                        new SwerveModuleState(wheels.get(1).getWheelVelMPS(), wheels.get(1).getAngle()),
                                        new SwerveModuleState(wheels.get(2).getWheelVelMPS(), wheels.get(2).getAngle()),
                                        new SwerveModuleState(wheels.get(3).getWheelVelMPS(), wheels.get(3).getAngle()));
  }

  public void update() {
    states = this.kinematics.toSwerveModuleStates(speeds);
    for (int i = 0; i < this.wheels.size(); i++) {
      SwerveModuleState tempState = SwerveModuleState.optimize(states[i], wheels.get(i).getAngle());
      // cos compensation
      // tempState.speedMetersPerSecond *=
      // tempState.angle.minus(wheels.get(i).getAngle()).getCos();
      this.wheels.get(i).setSpeed(tempState.speedMetersPerSecond);
      this.wheels.get(i).setAngle(tempState.angle.getRadians());
    }
  }
  private boolean poseEstimatorInitialized = false;
  public void poseEstimator(){
    if (Settings.poseEstimatorIsEnabled){
      if (!poseEstimatorInitialized){
        if (!imu.isOperational()){
          System.err.println();
          return; 
        }
        SwerveModulePosition[] positions;
        for (int i = 0; i < this.wheels.size(); i++) {
          positions[i] = new SwerveModulePosition(wheels.get(i).getPosition(), wheels.get(i).getAngle());
        }
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, imu, positions ,Settings.initialPoseMeters)
      }

    }
  }
}
