package com.spikes2212.command.drivetrains.swerve;

import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * This class represents a swerve drivetrain that uses vector calculations to combine the robot’s desired translation
 * and rotation, determining each wheel’s speed and direction for smooth, omnidirectional movement.*
 *
 * @author Gil Ein-Gar
 * @see DashboardedSubsystem
 */
public abstract class SwerveDrivetrain extends DashboardedSubsystem {

    private static final String DEFAULT_NAMESPACE_NAME = "drivetrain";

    protected final SwerveModule frontLeftModule;
    protected final SwerveModule frontRightModule;
    protected final SwerveModule backLeftModule;
    protected final SwerveModule backRightModule;

    protected final double halfDrivetrainTrackWidth;
    protected final double halfDrivetrainTrackLength;
    protected final double maxPossibleVelocity;
    protected final SwerveDriveKinematics kinematics;

    /**
     * Constructs a new instance of {@link SwerveDrivetrain}.
     *
     * @param namespaceName             the namespace name for the drivetrain
     * @param frontLeftModule           the front left module using {@link SwerveModule}
     * @param frontRightModule          the front right module using {@link SwerveModule}
     * @param backLeftModule            the back left module using {@link SwerveModule}
     * @param backRightModule           the back right module using{@link SwerveModule}
     * @param halfDrivetrainTrackWidth  half of the width of the drivetrain
     * @param halfDrivetrainTrackLength half of the length of the drivetrain
     * @param maxPossibleVelocity       the maximum possible velocity of the drive motors
     */
    public SwerveDrivetrain(String namespaceName, SwerveModule frontLeftModule, SwerveModule frontRightModule,
                            SwerveModule backLeftModule, SwerveModule backRightModule, double halfDrivetrainTrackWidth,
                            double halfDrivetrainTrackLength, double maxPossibleVelocity) {
        super(namespaceName);
        this.frontLeftModule = frontLeftModule;
        this.frontRightModule = frontRightModule;
        this.backLeftModule = backLeftModule;
        this.backRightModule = backRightModule;
        this.halfDrivetrainTrackWidth = halfDrivetrainTrackWidth;
        this.halfDrivetrainTrackLength = halfDrivetrainTrackLength;
        this.maxPossibleVelocity = maxPossibleVelocity;
        kinematics = calculateKinematics();

        resetRelativeEncoders();
    }

    /**
     * Constructs a new instance of {@link SwerveDrivetrain} with the default name of "drivetrain".
     *
     * @param frontLeftModule           the front left module using {@link SwerveModule}
     * @param frontRightModule          the front right module using {@link SwerveModule}
     * @param backLeftModule            the back left module using {@link SwerveModule}
     * @param backRightModule           the back right module using {@link SwerveModule}
     * @param halfDrivetrainTrackWidth  half the width of the drivetrain
     * @param halfDrivetrainTrackLength half the length of the drivetrain
     * @param maxPossibleVelocity       the maximum possible velocity of the drive motors
     */
    public SwerveDrivetrain(SwerveModule frontLeftModule, SwerveModule frontRightModule, SwerveModule backLeftModule,
                            SwerveModule backRightModule, double halfDrivetrainTrackWidth, double halfDrivetrainTrackLength,
                            double maxPossibleVelocity) {
        this(getClassName(DEFAULT_NAMESPACE_NAME), frontLeftModule, frontRightModule, backLeftModule, backRightModule,
                halfDrivetrainTrackWidth, halfDrivetrainTrackLength, maxPossibleVelocity);
    }

    /**
     * Function that moves the robot in a swerve motion.
     *
     * @param xSpeed          the desired speed on the x-axis.
     * @param ySpeed          the desired speed on the y-axis.
     * @param rotationSpeed   the desired speed for the drivetrain rotation.
     * @param isFieldRelative whether the drive should be relative to the field or to the robot.
     * @param timeStep        the amount of time between calculations, representing how long the speed value is applied
     *                        before updating again.
     * @param usePIDVelocity  whether the robot velocity will be applied using P.I.D or not.
     */
    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative,
                      double timeStep, boolean usePIDVelocity) {
        SwerveModuleState[] states;
        if (isFieldRelative) {
            states = getFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, timeStep);
        } else {
            states = getRobotRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, timeStep);
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxPossibleVelocity);
        setTargetModuleStates(states, usePIDVelocity);
    }

    /**
     * Sets the desired module states.
     *
     * @param targetModuleStates an array containing the desired speed and angle for each swerve module
     * @param usePIDVelocity     whether the modules will drive with P.I.D for the velocity
     */
    protected void setTargetModuleStates(SwerveModuleState[] targetModuleStates, boolean usePIDVelocity) {
        frontLeftModule.setTargetState(targetModuleStates[0], maxPossibleVelocity, usePIDVelocity);
        frontRightModule.setTargetState(targetModuleStates[1], maxPossibleVelocity, usePIDVelocity);
        backLeftModule.setTargetState(targetModuleStates[2], maxPossibleVelocity, usePIDVelocity);
        backRightModule.setTargetState(targetModuleStates[3], maxPossibleVelocity, usePIDVelocity);
    }

    /**
     * Updates the robot angle using an external sensor.
     * This resets the field relativity to be the same as the robot relativity.
     *
     * @return the current angle of the robot
     */
    protected abstract Rotation2d getCurrentRobotAngle();

    /**
     * Set the field relative speeds
     *
     * @param xSpeed        the desired speed on the x-axis.
     * @param ySpeed        the desired speed on the y-axis.
     * @param rotationSpeed the desired speed for the drivetrain rotation.
     * @param timeStep      the derivation of time the speed should be applied.
     * @return the necessary {@link SwerveModuleState[]} for field relative movement
     */
    protected SwerveModuleState[] getFieldRelativeSpeeds(double xSpeed, double ySpeed, double rotationSpeed,
                                                         double timeStep) {
        return kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,
                ySpeed, rotationSpeed, getCurrentRobotAngle()), timeStep));
    }

    /**
     * Set the robot relative speeds
     *
     * @param xSpeed        the desired speed on the x-axis.
     * @param ySpeed        the desired speed on the y-axis.
     * @param rotationSpeed the desired speed for the drivetrain rotation.
     * @param timeStep      the derivation of time the speed should be applied.
     * @return the necessary {@link SwerveModuleState[]} for robot relative movement
     */
    protected SwerveModuleState[] getRobotRelativeSpeeds(double xSpeed, double ySpeed, double rotationSpeed,
                                                         double timeStep) {
        return kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed,
                rotationSpeed), timeStep));
    }

    /**
     * Calculates the 2d positions of each wheel in relation to the center of the robot.
     *
     * @return a {@link SwerveDriveKinematics} according to the wheels positions in relation to the center
     */
    protected SwerveDriveKinematics calculateKinematics() {
        Translation2d frontLeftWheelDistanceFromCenter =
                new Translation2d(halfDrivetrainTrackWidth, halfDrivetrainTrackLength);
        Translation2d frontRightWheelDistanceFromCenter =
                new Translation2d(halfDrivetrainTrackWidth, -halfDrivetrainTrackLength);
        Translation2d backLeftWheelDistanceFromCenter =
                new Translation2d(-halfDrivetrainTrackWidth, halfDrivetrainTrackLength);
        Translation2d backRightWheelDistanceFromCenter =
                new Translation2d(-halfDrivetrainTrackWidth, -halfDrivetrainTrackLength);

        return new SwerveDriveKinematics(frontLeftWheelDistanceFromCenter,
                frontRightWheelDistanceFromCenter, backLeftWheelDistanceFromCenter,
                backRightWheelDistanceFromCenter);
    }

    /**
     * @return the robots current {@link ChassisSpeeds}
     */
    protected ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(frontLeftModule.getModuleState(), frontRightModule.getModuleState(),
                backLeftModule.getModuleState(), backRightModule.getModuleState());
    }

    /**
     * @return returns the front left {@link SwerveModule}
     */
    public SwerveModule getFrontLeftModule() {
        return frontLeftModule;
    }

    /**
     * @return returns the front right {@link SwerveModule}
     */
    public SwerveModule getFrontRightModule() {
        return frontRightModule;
    }

    /**
     * @return returns the back left {@link SwerveModule}
     */
    public SwerveModule getBackLeftModule() {
        return backLeftModule;
    }

    /**
     * @return returns the back right {@link SwerveModule}
     */
    public SwerveModule getBackRightModule() {
        return backRightModule;
    }

    /**
     * @return the robot's swerve kinematics
     */
    protected SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Resets the relative encoders of every module according to their absolute encoder.
     */
    public void resetRelativeEncoders() {
        frontLeftModule.resetRelativeEncoder();
        frontRightModule.resetRelativeEncoder();
        backLeftModule.resetRelativeEncoder();
        backRightModule.resetRelativeEncoder();
    }

    /**
     * Stops all modules.
     */
    public void stop() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }
}
