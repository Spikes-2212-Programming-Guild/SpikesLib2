package com.spikes2212.command.drivetrains.swerve;

import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveDrivetrain extends DashboardedSubsystem {

    private static final String DEFAULT_NAMESPACE_NAME = "drivetrain";

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final double trackWidth;
    private final double trackLength;
    private final double maxVelocity;
    private final SwerveDriveKinematics kinematics;

    private Rotation2d currentRobotAngle;

    public SwerveDrivetrain(String namespaceName, SwerveModule frontLeft, SwerveModule frontRight,
                            SwerveModule backLeft, SwerveModule backRight, double trackWidth, double trackLength,
                            double maxVelocity) {
        super(namespaceName);
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.trackWidth = trackWidth;
        this.trackLength = trackLength;
        this.maxVelocity = maxVelocity;

        Translation2d frontLeftWheelDistanceFromCenter = new Translation2d(trackWidth / 2, trackLength / 2);
        Translation2d frontRightWheelDistanceFromCenter = new Translation2d(trackWidth / 2, -trackLength / 2);
        Translation2d backLeftWheelDistanceFromCenter = new Translation2d(-trackWidth / 2, trackLength / 2);
        Translation2d backRightWheelDistanceFromCenter = new Translation2d(-trackWidth / 2, -trackLength / 2);

        kinematics = new SwerveDriveKinematics(frontLeftWheelDistanceFromCenter,
                frontRightWheelDistanceFromCenter, backLeftWheelDistanceFromCenter,
                backRightWheelDistanceFromCenter);
    }

    public SwerveDrivetrain(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft,
                            SwerveModule backRight, double trackWidth, double trackLength, double maxVelocity) {
        this(getClassName(DEFAULT_NAMESPACE_NAME), frontLeft, frontRight, backLeft, backRight, trackWidth, trackLength,
                maxVelocity);
    }

    public abstract void setCurrentRobotAngle();

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative, double timeStep) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(getChassisSpeeds(isFieldRelative, xSpeed, ySpeed,
                rotationSpeed, timeStep));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);
        setTargetModuleStates(states);
    }

    public void setTargetModuleStates(SwerveModuleState[] targetModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleStates, maxVelocity);
        setCurrentRobotAngle();
        frontLeft.setTargetState(targetModuleStates[0]);
        frontRight.setTargetState(targetModuleStates[1]);
        backLeft.setTargetState(targetModuleStates[2]);
        backRight.setTargetState(targetModuleStates[3]);
    }

    public ChassisSpeeds getChassisSpeeds(boolean fieldRelative, double xSpeed, double ySpeed, double rotationSpeed, double timeStep) {
        if (fieldRelative) {
            setCurrentRobotAngle();
            return ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, currentRobotAngle), timeStep);
        } else {
            return ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed), timeStep);
        }
    }

    public void resetRelativeEncoders() {
        frontLeft.resetRelativeEncoder();
        frontRight.resetRelativeEncoder();
        backLeft.resetRelativeEncoder();
        backRight.resetRelativeEncoder();
    }

    public void stopModules() {
        frontLeft.stopModule();
        frontRight.stopModule();
        backLeft.stopModule();
        backRight.stopModule();
    }
}
