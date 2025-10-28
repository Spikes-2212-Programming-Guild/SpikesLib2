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

    protected final boolean driveInverted;
    protected final boolean turnInverted;
    protected final double offset;

    protected final PIDSettings drivePIDSettings;
    protected final PIDSettings turnPIDSettings;
    protected final FeedForwardSettings driveFeedForwardSettings;
    protected final FeedForwardSettings turnFeedForwardSettings;

    public SwerveModule(String namespaceName, SmartMotorController driveMotor, SmartMotorController turnMotor,
                        boolean driveInverted, boolean turnInverted, double offset, PIDSettings drivePIDSettings,
                        PIDSettings turnPIDSettings, FeedForwardSettings driveFeedForwardSettings,
                        FeedForwardSettings turnFeedForwardSettings) {
        super(namespaceName);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnInverted = turnInverted;
        this.driveInverted = driveInverted;
        this.offset = offset;
        this.drivePIDSettings = drivePIDSettings;
        this.turnPIDSettings = turnPIDSettings;
        this.driveFeedForwardSettings = driveFeedForwardSettings;
        this.turnFeedForwardSettings = turnFeedForwardSettings;
        driveMotor.setInverted(driveInverted);
        turnMotor.setInverted(turnInverted);
        configureTurnController();
        configureDriveController();
        configureAbsoluteEncoder();
    }

    public void setTargetState(SwerveModuleState targetState, double maxVelocity, boolean usePIDVelocity) {
        targetState.optimize(Rotation2d.fromDegrees(turnMotor.getPosition()));
        setTargetAngle(targetState.angle);
        setTargetVelocity(targetState.speedMetersPerSecond, maxVelocity, usePIDVelocity);
    }

    public void setTargetAngle(Rotation2d targetAngle) {
        turnMotor.pidSet(UnifiedControlMode.POSITION, targetAngle.getDegrees(), turnPIDSettings,
                turnFeedForwardSettings, false);
    }


    public void setTargetVelocity(double targetVelocity, double maxVelocity, boolean usePIDVelocity) {
        if (usePIDVelocity) {
            driveMotor.pidSet(UnifiedControlMode.VELOCITY, targetVelocity, drivePIDSettings,
                    driveFeedForwardSettings, false);
        } else {
            driveMotor.set(targetVelocity / maxVelocity);
        }
    }

    protected abstract void configureDriveController();

    protected abstract void configureTurnController();

    protected abstract void configureAbsoluteEncoder();

    protected abstract Rotation2d getAbsoluteModuleAngle();

    public void resetRelativeEncoder() {
        turnMotor.setPosition(getAbsoluteModuleAngle().getDegrees());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity(),
                getAbsoluteModuleAngle());
    }

    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }
}
