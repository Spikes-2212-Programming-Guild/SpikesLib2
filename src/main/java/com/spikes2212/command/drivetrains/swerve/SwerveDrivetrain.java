package com.spikes2212.command.drivetrains.swerve;

import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveDrivetrain extends DashboardedSubsystem {

    private static final String DEFAULT_NAMESPACE_NAME = "drivetrain";

    protected final SwerveModule frontLeft;
    protected final SwerveModule frontRight;
    protected final SwerveModule backLeft;
    protected final SwerveModule backRight;

    protected final double trackWidth;
    protected final double trackLength;
    protected final double maxVelocity;
    protected final SwerveDriveKinematics kinematics;

    protected Rotation2d currentRobotAngle;

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
        this(getClassName(DEFAULT_NAMESPACE_NAME), frontLeft, frontRight, backLeft, backRight, trackWidth,
                trackLength, maxVelocity);
    }

    public abstract void updateRobotAngle();

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative, double timeStep) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(getChassisSpeeds(isFieldRelative, xSpeed, ySpeed,
                rotationSpeed, timeStep));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);
        setTargetModuleStates(states);
    }

    protected void setTargetModuleStates(SwerveModuleState[] targetModuleStates) {
        frontLeft.setTargetState(targetModuleStates[0], maxVelocity);
        frontRight.setTargetState(targetModuleStates[1], maxVelocity);
        backLeft.setTargetState(targetModuleStates[2], maxVelocity);
        backRight.setTargetState(targetModuleStates[3], maxVelocity);
    }

    protected ChassisSpeeds getChassisSpeeds(boolean fieldRelative, double xSpeed, double ySpeed, double rotationSpeed,
                                             double timeStep) {
        if (fieldRelative) {
            updateRobotAngle();
            return ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed,
                    currentRobotAngle), timeStep);
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

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}
