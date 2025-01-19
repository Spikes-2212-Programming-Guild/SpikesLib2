package com.spikes2212.command.drivetrains.swervedrivetrains.drivetrains;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.command.drivetrains.swervedrivetrains.modules.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.function.Supplier;

public class SwerveDrivetrain extends DashboardedSubsystem {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final Supplier<Double> robotAngle;
    private final Translation2d centerOfRobot;
    private final double maxSpeed;

    private final SwerveDriveKinematics kinematics;

    public SwerveDrivetrain(String namespaceName, SwerveModule frontLeft, SwerveModule frontRight,
                            SwerveModule backLeft, SwerveModule backRight, Supplier<Double> angle,
                            Translation2d centerOfRobot, double maxSpeed) {
        super(namespaceName);
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.robotAngle = angle;
        this.centerOfRobot = centerOfRobot;
        this.maxSpeed = maxSpeed;
        kinematics = new SwerveDriveKinematics(frontLeft.getModulePosition(), frontRight.getModulePosition(),
                backLeft.getModulePosition(), backRight.getModulePosition());
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative, boolean usePID,
                      boolean limitSpeed) {
        ChassisSpeeds speeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed,
                Rotation2d.fromDegrees(robotAngle.get())) : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, centerOfRobot);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);
        frontLeft.set(states[0], usePID, limitSpeed);
        frontRight.set(states[1], usePID, limitSpeed);
        backLeft.set(states[2], usePID, limitSpeed);
        backRight.set(states[3], usePID, limitSpeed);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void resetRelativeEncoders() {
        frontLeft.resetRelativeEncoder();
        frontRight.resetRelativeEncoder();
        backLeft.resetRelativeEncoder();
        backRight.resetRelativeEncoder();
    }

    public double getRobotAngle() {
        return robotAngle.get();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getRobotAngle());
    }

    @Override
    public void configureDashboard() {
    }
}
