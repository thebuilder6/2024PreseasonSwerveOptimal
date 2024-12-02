package frc.robot.Data;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Settings {

    // Swerve Module Settings
    public static final double STEER_ROTATIONS_TO_ENCODER_RADS = 2 * Math.PI;


    public static final Measure<Distance> WHEEL_CIRCUMFERENCE = Units.Meters.of(0.318); // A wheel with a 4 inch outer diameter (OD) has a circumference of approximately 0.318 meters
    public static final double SWERVE_DRIVE_GEAR_RATIO = 6.75; // Example ratio, adjust to your actual gear ratio

    // Swerve Drive Motor Constants
    // TODO: Tune these
    public static class SwerveDriveConstants {

        public static final String ID = "SwerveDriveConstants";
        public static double Ks = 0.0;
        public static double Kv = 0.0;
        public static double Ka = 0.0;
        public static double Kp = 1.0;
        public static double Ki = 0.0;
        public static double Kd = 0.0;
        public static double maxVelocityConstraint = 30.0;
        public static double maxAccelerationConstraint = 10000.0;

        public static void dashboardEditorConstantsInit() {
            SmartDashboard.putNumber(ID + "/Kp", Kp);
            SmartDashboard.putNumber(ID + "/Ki", Ki);
            SmartDashboard.putNumber(ID + "/Kd", Kd);
            SmartDashboard.putNumber(ID + "/Ks", Ks);
            SmartDashboard.putNumber(ID + "/Kv", Kv);
            SmartDashboard.putNumber(ID + "/MaxVelocityConstraint", maxVelocityConstraint);
            SmartDashboard.putNumber(ID + "/MaxAccelerationConstraint", maxAccelerationConstraint);
        }

        public static void dashboardEditorConstantsChange() {
            Kp = SmartDashboard.getNumber(ID + "/Kp", Kp);
            Ki = SmartDashboard.getNumber(ID + "/Ki", Ki);
            Kd = SmartDashboard.getNumber(ID + "/Kd", Kd);
            Ks = SmartDashboard.getNumber(ID + "/Ks", Ks);
            Kv = SmartDashboard.getNumber(ID + "/Kv", Kv);
            maxVelocityConstraint = SmartDashboard.getNumber(ID + "/MaxVelocityConstraint", maxVelocityConstraint);
            maxAccelerationConstraint = SmartDashboard.getNumber(ID + "/MaxAccelerationConstraint",
                    maxAccelerationConstraint);
        }
    }

    public static class SwerveSteerConstants {

        public static final String ID = "SwerveSteerConstants";
        public static double Ks = 0.2;
        public static double Kv = 0.415;
        public static double Ka = 0.0;
        public static double Kp = 5.8;
        public static double Ki = 0.0;
        public static double Kd = 0.0;
        public static double maxVelocityConstraint = 30.0;
        public static double maxAccelerationConstraint = 10000.0;

        public static void dashboardEditorConstantsInit() {
            SmartDashboard.putNumber(ID + "/Kp", Kp);
            SmartDashboard.putNumber(ID + "/Ki", Ki);
            SmartDashboard.putNumber(ID + "/Kd", Kd);
            SmartDashboard.putNumber(ID + "/Ks", Ks);
            SmartDashboard.putNumber(ID + "/Kv", Kv);
            SmartDashboard.putNumber(ID + "/MaxVelocityConstraint", maxVelocityConstraint);
            SmartDashboard.putNumber(ID + "/MaxAccelerationConstraint", maxAccelerationConstraint);
        }

        public static void dashboardEditorConstantsChange() {
            Kp = SmartDashboard.getNumber(ID + "/Kp", Kp);
            Ki = SmartDashboard.getNumber(ID + "/Ki", Ki);
            Kd = SmartDashboard.getNumber(ID + "/Kd", Kd);
            Ks = SmartDashboard.getNumber(ID + "/Ks", Ks);
            Kv = SmartDashboard.getNumber(ID + "/Kv", Kv);
            maxVelocityConstraint = SmartDashboard.getNumber(ID + "/MaxVelocityConstraint", maxVelocityConstraint);
            maxAccelerationConstraint = SmartDashboard.getNumber(ID + "/MaxAccelerationConstraint",
                    maxAccelerationConstraint);
        }
    }

    private static final SendableChooser<String> editableConstantsChooser = new SendableChooser<>();
    private static String constantSet;

    public static void dashboardEditorSelector() {
        constantSet = editableConstantsChooser.getSelected();
        if (constantSet.equals(SwerveDriveConstants.ID)) {
            SwerveDriveConstants.dashboardEditorConstantsInit();
        } else if (constantSet.equals(SwerveSteerConstants.ID)) {
            SwerveSteerConstants.dashboardEditorConstantsInit();
        } else {
            System.err.println("ERROR: invalid option selected on dashboard");
        }
    }

    public static void init() {
        editableConstantsChooser.setDefaultOption("SwerveDriveConstants", SwerveDriveConstants.ID);
        editableConstantsChooser.addOption("SwerveSteerConstants", SwerveSteerConstants.ID);
        SmartDashboard.putData("Editable Constants", editableConstantsChooser);
    }

    public static void update() {
        dashboardEditorSelector();
        if (constantSet.equals(SwerveDriveConstants.ID)) {
            SwerveDriveConstants.dashboardEditorConstantsChange();
        } else if (constantSet.equals(SwerveSteerConstants.ID)) {
            SwerveSteerConstants.dashboardEditorConstantsChange();
        } else {
            System.err.println("ERROR: invalid option selected on dashboard");
        }
    }
}
