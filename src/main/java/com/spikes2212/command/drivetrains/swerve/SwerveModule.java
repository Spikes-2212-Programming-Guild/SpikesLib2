package com.spikes2212.command.drivetrains.swerve;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.function.Supplier;

public abstract class SwerveModule extends DashboardedSubsystem {

    private static final double ABSOLUTE_POSITION_DISCONTINUITY_POINT = 1;
    private static final double DEGREES_IN_ROTATIONS = 360;

    protected final SmartMotorController driveMotor;
    protected final SmartMotorController turnMotor;
    protected final Supplier<Double> absoluteModuleAngle;

    protected final boolean driveInverted;
    protected final boolean turnInverted;
    protected final boolean usePIDVelocity;
    protected final double offset;

    protected final PIDSettings drivePIDSettings;
    protected final PIDSettings turnPIDSettings;
    protected final FeedForwardSettings driveFeedForwardSettings;
    protected final FeedForwardSettings turnFeedForwardSettings;

    public SwerveModule(String namespaceName, SmartMotorController driveMotor, SmartMotorController turnMotor,
                        Supplier<Double> absoluteModuleAngle, boolean driveInverted, boolean turnInverted,
                        boolean usePIDVelocity, double offset, PIDSettings drivePIDSettings,
                        PIDSettings turnPIDSettings, FeedForwardSettings driveFeedForwardSettings,
                        FeedForwardSettings turnFeedForwardSettings) {
        super(namespaceName);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.absoluteModuleAngle = absoluteModuleAngle;
        this.turnInverted = turnInverted;
        this.driveInverted = driveInverted;
        this.usePIDVelocity = usePIDVelocity;
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

    public void setTargetState(SwerveModuleState targetState, double maxVelocity) {
        targetState.optimize(Rotation2d.fromDegrees(turnMotor.getPosition()));
        setTargetAngle(targetState.angle);
        setTargetVelocity(targetState.speedMetersPerSecond, maxVelocity);
    }

    public void setTargetAngle(Rotation2d targetAngle) {
        turnMotor.pidSet(UnifiedControlMode.POSITION, targetAngle.getDegrees(), turnPIDSettings,
                turnFeedForwardSettings, false);
    }


    public void setTargetVelocity(double targetVelocity, double maxVelocity) {
        if (usePIDVelocity) {
            driveMotor.pidSet(UnifiedControlMode.VELOCITY, targetVelocity, drivePIDSettings, driveFeedForwardSettings,
                    false);
        } else {
            driveMotor.set(targetVelocity / maxVelocity);
        }
    }

    public abstract void configureDriveController();

    public abstract void configureTurnController();

    public abstract void configureAbsoluteEncoder();

    public double getAbsoluteAngle() {
        return absoluteModuleAngle.get();
    }

    public void resetRelativeEncoder() {
        turnMotor.setPosition(getAbsoluteAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity(),
                Rotation2d.fromDegrees(getAbsoluteAngle()));
    }

    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }
}
