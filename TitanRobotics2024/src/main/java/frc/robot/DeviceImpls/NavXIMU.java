package frc.robot.DeviceImpls;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Devices.IMUInterface;

import java.util.Optional;

public class NavXIMU implements IMUInterface {
    private final AHRS navX;
    private final MountConfiguration mountConfig;
    
    public NavXIMU(SPI.Port port, MountConfiguration mountConfig) {
        this.navX = new AHRS(port);
        this.mountConfig = mountConfig;
    }

    @Override
    public Optional<IMUMeasurement> getMeasurement() {
        try {
            if (!isOperational()) {
                return Optional.empty();
            }

            // Convert accelerations from G's to m/s^2
            Measure<Velocity<Velocity<Distance>>> accelX = Units.MetersPerSecondPerSecond.of(navX.getRawAccelX() * 9.81);
            Measure<Velocity<Velocity<Distance>>> accelY = Units.MetersPerSecondPerSecond.of(navX.getRawAccelY() * 9.81);
            Measure<Velocity<Velocity<Distance>>> accelZ = Units.MetersPerSecondPerSecond.of(navX.getRawAccelZ() * 9.81);

            // Get gyro yaw pitch roll
            Measure<Angle> gyroPitch = Units.Degrees.of(navX.getPitch());
            Measure<Angle> gyroRoll = Units.Degrees.of(navX.getRoll());
            Measure<Angle> gyroYaw = Units.Degrees.of(navX.getYaw());
            Measure<Velocity<Angle>> yawRate = Units.DegreesPerSecond.of(navX.getRate());

            // Apply mount corrections
            IMUMeasurement measurement = new IMUMeasurement(
                accelX, accelY, accelZ,
                gyroPitch, gyroRoll, gyroYaw,
                yawRate, 
                getAdjustedHeading(),
                navX.getLastSensorTimestamp() * 1e-6  // Convert microseconds to seconds
            );

            return Optional.of(measurement);
        } catch (Exception e) {
            return Optional.empty();
        }
    }

    @Override
    public boolean isOperational() {
        return navX.isConnected() && !navX.isCalibrating();
    }

    @Override
    public void zeroHeading() {
        navX.zeroYaw();
    }

    @Override
    public void setHeading(Rotation2d heading) {
        navX.setAngleAdjustment(heading.getDegrees());
    }

    @Override
    public MountConfiguration getMountConfiguration() {
        return mountConfig;
    }

    private Rotation2d getAdjustedHeading() {
        return Rotation2d.fromDegrees(navX.getFusedHeading())
            .plus(mountConfig.yawOffset);
    }
}
