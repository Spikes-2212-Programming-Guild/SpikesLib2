package com.spikes2212.command.drivetrains;

import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.function.Supplier;

public class SwerveDrivetrain extends DashboardedSubsystem {

    private final GenericSwerveModule frontLeft;
    private final GenericSwerveModule frontRight;
    private final GenericSwerveModule backLeft;
    private final GenericSwerveModule backRight;
    private final Supplier<Double> robotRotation;
    private final double distanceBetweenModulesX;
    private final double distanceBetweenModulesY;
    private final Translation2d frontLeftWheelPosition;
    private final Translation2d frontRightWheelPosition;
    private final Translation2d backLeftWheelPosition;
    private final Translation2d backRightWheelPosition;
    private final Translation2d centerOfRobot;
    private final double maxSpeedMetersPerSecond;

    private final SwerveDriveKinematics kinematics;

    public SwerveDrivetrain(String namespaceName, GenericSwerveModule frontLeft, GenericSwerveModule frontRight,
                            GenericSwerveModule backLeft, GenericSwerveModule backRight, Supplier<Double> robotRotation,
                            double distanceBetweenModulesX, double distanceBetweenModulesY,
                            double maxSpeedMetersPerSecond) {
        super(namespaceName);
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.robotRotation = robotRotation;
        this.distanceBetweenModulesX = distanceBetweenModulesX;
        this.distanceBetweenModulesY = distanceBetweenModulesY;
        this.frontLeftWheelPosition = new Translation2d(distanceBetweenModulesX / 2, distanceBetweenModulesY / 2);
        this.frontRightWheelPosition = new Translation2d(distanceBetweenModulesX / 2, -distanceBetweenModulesY / 2);
        this.backLeftWheelPosition = new Translation2d(-distanceBetweenModulesX / 2, distanceBetweenModulesY / 2);
        this.backRightWheelPosition = new Translation2d(-distanceBetweenModulesX / 2, -distanceBetweenModulesY / 2);
        this.centerOfRobot =
                new Translation2d((frontLeftWheelPosition.getX() + backRightWheelPosition.getX()) / 2,
                        (frontLeftWheelPosition.getY() + backRightWheelPosition.getY()) / 2);
        this.kinematics = new SwerveDriveKinematics(frontLeftWheelPosition, frontRightWheelPosition,
                backLeftWheelPosition, backRightWheelPosition);
        this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative,
                      boolean usePID) {
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getRotation2d());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, centerOfRobot);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMetersPerSecond);
        frontLeft.set(states[0], usePID);
        frontRight.set(states[1], usePID);
        backLeft.set(states[2], usePID);
        backRight.set(states[3], usePID);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public double getAngle() {
        return robotRotation.get();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    @Override
    public void configureDashboard() {
    }
}
