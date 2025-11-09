package com.spikes2212.command.drivetrains.swerve;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
     * into angular position and velocity values.
     */
    protected abstract void configureTurnController();

    /**
     * Defines the contract for configuring an absolute encoder.
     * Implementations should set direction, offset, and calibration parameters
     * to align encoder readings with the system’s mechanics.
     */
    protected abstract void configureAbsoluteEncoder();

    public void setTargetState(SwerveModuleState targetState, double maxVelocity, boolean usePIDVelocity) {
        targetState.optimize(Rotation2d.fromDegrees(turnMotor.getPosition()));
        setTargetAngle(targetState.angle);
        setTargetVelocity(targetState.speedMetersPerSecond, maxVelocity, usePIDVelocity);
    }

    public void setTargetAngle(Rotation2d targetAngle) {
        turnMotor.pidSet(UnifiedControlMode.POSITION, targetAngle.getDegrees(), turnMotorPIDSettings,
                turnMotorFeedForwardSettings, false);
    }

    public void setTargetVelocity(double targetVelocity, double maxVelocity, boolean usePIDVelocity) {
        if (usePIDVelocity) {
            driveMotor.pidSet(UnifiedControlMode.VELOCITY, targetVelocity, driveMotorPIDSettings,
                    driveMotorFeedForwardSettings, false);
        } else {
            driveMotor.set(targetVelocity / maxVelocity);
        }
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(driveMotor.getVelocity(), getAbsoluteModuleAngle());
    }

    public double getRelativeModuleAngle() {
        return turnMotor.getPosition();
    }

    public double getModuleVelocity() {
        return driveMotor.getVelocity();
    }

    /**
     * resets the relative encoder according to the absolute encoder
     */
    public void resetRelativeEncoder() {
        turnMotor.setPosition(getAbsoluteModuleAngle().getDegrees());
    }

    /**
     * @return the absolute module angle using an absolute encoder
     */
    protected abstract Rotation2d getAbsoluteModuleAngle();

    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }
}
