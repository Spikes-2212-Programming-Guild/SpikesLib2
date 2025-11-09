package com.spikes2212.command.drivetrains.swerve;

import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveDrivetrain extends DashboardedSubsystem {

    private static final String DEFAULT_NAMESPACE_NAME = "drivetrain";

    protected final SwerveModule frontLeftModule;
    protected final SwerveModule frontRightModule;
    protected final SwerveModule backLeftModule;
    protected final SwerveModule backRightModule;

    protected final double drivetrainTrackWidth;
    protected final double drivetrainTrackLength;
    protected final double maxPossibleVelocity;
    protected final SwerveDriveKinematics kinematics;

    public SwerveDrivetrain(String namespaceName, SwerveModule frontLeftModule, SwerveModule frontRightModule,
                            SwerveModule backLeftModule, SwerveModule backRightModule, double drivetrainTrackWidth,
                            double drivetrainTrackLength, double maxPossibleVelocity) {
        super(namespaceName);
        this.frontLeftModule = frontLeftModule;
        this.frontRightModule = frontRightModule;
        this.backLeftModule = backLeftModule;
        this.backRightModule = backRightModule;
        this.drivetrainTrackWidth = drivetrainTrackWidth;
        this.drivetrainTrackLength = drivetrainTrackLength;
        this.maxPossibleVelocity = maxPossibleVelocity;

        Translation2d frontLeftWheelDistanceFromCenter =
                new Translation2d(drivetrainTrackWidth / 2, drivetrainTrackLength / 2);
        Translation2d frontRightWheelDistanceFromCenter =
                new Translation2d(drivetrainTrackWidth / 2, -drivetrainTrackLength / 2);
        Translation2d backLeftWheelDistanceFromCenter =
                new Translation2d(-drivetrainTrackWidth / 2, drivetrainTrackLength / 2);
        Translation2d backRightWheelDistanceFromCenter =
                new Translation2d(-drivetrainTrackWidth / 2, -drivetrainTrackLength / 2);

        kinematics = new SwerveDriveKinematics(frontLeftWheelDistanceFromCenter,
                frontRightWheelDistanceFromCenter, backLeftWheelDistanceFromCenter,
                backRightWheelDistanceFromCenter);

        resetRelativeEncoders();
    }

    public SwerveDrivetrain(SwerveModule frontLeftModule, SwerveModule frontRightModule, SwerveModule backLeftModule,
                            SwerveModule backRightModule, double drivetrainTrackWidth, double drivetrainTrackLength,
                            double maxPossibleVelocity) {
        this(getClassName(DEFAULT_NAMESPACE_NAME), frontLeftModule, frontRightModule, backLeftModule, backRightModule,
                drivetrainTrackWidth, drivetrainTrackLength, maxPossibleVelocity);
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative,
                      double timeStep, boolean usePIDVelocity) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(getChassisSpeeds(isFieldRelative, xSpeed,
                ySpeed, rotationSpeed, timeStep));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxPossibleVelocity);
        setTargetModuleStates(states, usePIDVelocity);
    }

    protected void setTargetModuleStates(SwerveModuleState[] targetModuleStates, boolean usePIDVelocity) {
        frontLeftModule.setTargetState(targetModuleStates[0], maxPossibleVelocity, usePIDVelocity);
        frontRightModule.setTargetState(targetModuleStates[1], maxPossibleVelocity, usePIDVelocity);
        backLeftModule.setTargetState(targetModuleStates[2], maxPossibleVelocity, usePIDVelocity);
        backRightModule.setTargetState(targetModuleStates[3], maxPossibleVelocity, usePIDVelocity);
    }

    protected abstract Rotation2d getCurrentRobotAngle();

    protected ChassisSpeeds getChassisSpeeds(boolean fieldRelative, double xSpeed, double ySpeed,
                                             double rotationSpeed, double timeStep) {
        if (fieldRelative) {
            return ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed,
                    getCurrentRobotAngle()), timeStep);
        } else {
            return ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed), timeStep);
        }
    }

    public void resetRelativeEncoders() {
        frontLeftModule.resetRelativeEncoder();
        frontRightModule.resetRelativeEncoder();
        backLeftModule.resetRelativeEncoder();
        backRightModule.resetRelativeEncoder();
    }

    public void stop() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }
}
