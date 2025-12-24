package com.spikes2212.command.drivetrains.swerve;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Represents a single swerve module, serving as the abstraction for one wheel’s steering and driving
 * mechanisms within a swerve drivetrain.
 *
 * @author Gil Ein-Gar
 * @see DashboardedSubsystem
 */
public abstract class SwerveModule extends DashboardedSubsystem {

    protected final SmartMotorController driveMotor;
    protected final SmartMotorController turnMotor;

    protected final boolean driveMotorInverted;
    protected final boolean turnMotorInverted;
    protected final double absoluteEncoderOffset;

    protected final PIDSettings driveMotorPIDSettings;
    protected final PIDSettings turnMotorPIDSettings;
    protected final FeedForwardSettings driveMotorFeedForwardSettings;
    protected final FeedForwardSettings turnMotorFeedForwardSettings;

    /**
     * Constructs a new instance of  {@link SwerveModule}.
     *
     * @param namespaceName                 the namespace name used for the module
     * @param driveMotor                    the motor that is used for turning the wheel in the pitch axis
     * @param turnMotor                     the motor that is used fot turning the wheel in the yaw axis
     * @param driveMotorInverted            whether the drive motor is inverted or not
     * @param turnMotorInverted             whether the turn motor is inverted ot not
     * @param absoluteEncoderOffset         absolute encoder offset while wheel is parallel to the track
     * @param driveMotorPIDSettings         the drive motor pid settings
     * @param turnMotorPIDSettings          the turn motor pid settings
     * @param driveMotorFeedForwardSettings the drive motor feed forward settings
     * @param turnMotorFeedForwardSettings  the turn motor feed forward settings
     */
    public SwerveModule(String namespaceName, SmartMotorController driveMotor, SmartMotorController turnMotor,
                        boolean driveMotorInverted, boolean turnMotorInverted, double absoluteEncoderOffset,
                        PIDSettings driveMotorPIDSettings, PIDSettings turnMotorPIDSettings,
                        FeedForwardSettings driveMotorFeedForwardSettings,
                        FeedForwardSettings turnMotorFeedForwardSettings) {
        super(namespaceName);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnMotorInverted = turnMotorInverted;
        this.driveMotorInverted = driveMotorInverted;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.driveMotorPIDSettings = driveMotorPIDSettings;
        this.turnMotorPIDSettings = turnMotorPIDSettings;
        this.driveMotorFeedForwardSettings = driveMotorFeedForwardSettings;
        this.turnMotorFeedForwardSettings = turnMotorFeedForwardSettings;
        driveMotor.setInverted(driveMotorInverted);
        turnMotor.setInverted(turnMotorInverted);
        configureTurnController();
        configureDriveController();
        configureAbsoluteEncoder();
    }

    /**
     * Configures drive motor conversion factors that translate encoder rotations
     * into real-world distance and speed measurements.
     */
    protected abstract void configureDriveController();

    /**
     * Configures turn motor conversion factors that convert motor rotations
     * into angular position and velocity values in degrees.
     */
    protected abstract void configureTurnController();

    /**
     * Defines the contract for configuring an absolute encoder.
     * Implementations should set direction, offset, and calibration parameters
     * to align encoder readings with the system’s mechanics.
     */
    protected abstract void configureAbsoluteEncoder();

    /**
     * Sets the desired module state.
     *
     * @param targetState         the desired state of the module
     * @param maxPossibleVelocity the maximum possible velocity of the drive motor
     * @param useVelocityPID      whether the module will drive with P.I.D for the velocity
     */
    public void setTargetState(SwerveModuleState targetState, double maxPossibleVelocity, boolean useVelocityPID) {
        targetState.optimize(Rotation2d.fromDegrees(getRelativeModuleAngle()));
        setTargetAngle(targetState.angle);
        setTargetVelocity(targetState.speedMetersPerSecond, maxPossibleVelocity, useVelocityPID);
    }

    /**
     * Sets the desired module angle.
     *
     * @param targetAngle the desired angle of the module
     */
    public void setTargetAngle(Rotation2d targetAngle) {
        turnMotor.pidSet(UnifiedControlMode.POSITION, targetAngle.getDegrees(), turnMotorPIDSettings,
                turnMotorFeedForwardSettings, false);
    }

    /**
     * Sets the desired module velocity.
     *
     * @param targetVelocity      the desired velocity of the module
     * @param maxPossibleVelocity the maximum possible velocity of the drive motor
     * @param useVelocityPID      whether the module will drive with P.I.D for the velocity
     */
    public void setTargetVelocity(double targetVelocity, double maxPossibleVelocity, boolean useVelocityPID) {
        if (useVelocityPID) {
            driveMotor.pidSet(UnifiedControlMode.VELOCITY, targetVelocity, driveMotorPIDSettings,
                    driveMotorFeedForwardSettings, false);
        } else {
            driveMotor.set(targetVelocity / maxPossibleVelocity);
        }
    }

    /**
     * @return the current state of the module
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(driveMotor.getVelocity(), getAbsoluteModuleAngle());
    }

    /**
     * @return the current relative module angle
     */
    public double getRelativeModuleAngle() {
        return turnMotor.getPosition();
    }

    /**
     * @return the current module velocity
     */
    public double getModuleVelocity() {
        return driveMotor.getVelocity();
    }

    /**
     * @return the {@link SwerveModulePosition} of this module
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveMotor.getPosition(), getAbsoluteModuleAngle());
    }

    /**
     * Resets the relative encoder according to the absolute encoder.
     */
    public void resetRelativeEncoder() {
        turnMotor.setPosition(getAbsoluteModuleAngle().getDegrees());
    }

    /**
     * @return the absolute module angle using an absolute encoder.
     */
    protected abstract Rotation2d getAbsoluteModuleAngle();

    /**
     * stops the module
     */
    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }
}
