package frc.robot.DeviceImpls;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
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
            double accelX = navX.getRawAccelX() * 9.81;
            double accelY = navX.getRawAccelY() * 9.81;
            double accelZ = navX.getRawAccelZ() * 9.81;

            // Get raw gyro rates in rad/s
            double gyroX = Math.toRadians(navX.getRawGyroX());
            double gyroY = Math.toRadians(navX.getRawGyroY());
            double gyroZ = Math.toRadians(navX.getRawGyroZ());

            // Apply mount corrections
            IMUMeasurement measurement = new IMUMeasurement(
                accelX, accelY, accelZ,
                gyroX, gyroY, gyroZ,
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
        return Rotation2d.fromDegrees(navX.getYaw())
            .plus(mountConfig.yawOffset);
    }
}
