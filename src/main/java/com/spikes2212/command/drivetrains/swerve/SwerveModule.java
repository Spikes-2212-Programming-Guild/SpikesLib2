package com.spikes2212.command.drivetrains.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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

    private final SmartMotorController driveMotor;
    private final SmartMotorController turnMotor;
    private final Supplier<Double> absoluteEncoder;

    private final boolean turnInverted;
    private final boolean driveInverted;
    private final boolean usePIDVelocity;
    private final double offset;

    private final PIDSettings drivePIDSettings;
    private final PIDSettings turnPIDSettings;
    private final FeedForwardSettings driveFeedForwardSettings;
    private final FeedForwardSettings turnFeedForwardSettings;

    public SwerveModule(String namespaceName, SmartMotorController driveMotor, SmartMotorController turnMotor,
                        Supplier<Double> absoluteEncoder, boolean turnInverted, boolean driveInverted,
                        boolean usePIDVelocity, double offset, PIDSettings drivePIDSettings,
                        PIDSettings turnPIDSettings, FeedForwardSettings driveFeedForwardSettings,
                        FeedForwardSettings turnFeedForwardSettings) {
        super(namespaceName);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.absoluteEncoder = absoluteEncoder;
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

    public void setTargetState(SwerveModuleState targetState) {
        targetState.optimize(Rotation2d.fromDegrees(turnMotor.getPosition()));
        setTargetAngle(targetState.angle);
        setTargetVelocity(targetState.speedMetersPerSecond);

    }

    public void setTargetAngle(Rotation2d targetAngle) {
        turnMotor.pidSet(UnifiedControlMode.POSITION, targetAngle.getDegrees(), turnPIDSettings,
                turnFeedForwardSettings, false);
    }


    public void setTargetVelocity(double targetVelocity) {
        if (usePIDVelocity) {
            driveMotor.pidSet(UnifiedControlMode.VELOCITY, targetVelocity, drivePIDSettings, driveFeedForwardSettings,
                    false);
        } else {
            driveMotor.set(targetVelocity);
        }
    }

    public abstract void configureDriveController();

    public abstract void configureTurnController();

    public abstract void configureAbsoluteEncoder();

    public double getAbsoluteAngle() {
        return absoluteEncoder.get();
    }

    public void resetRelativeEncoder() {
        turnMotor.setPosition(getAbsoluteAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity(),
                Rotation2d.fromDegrees(getAbsoluteAngle()));
    }

    public void stopModule() {
        turnMotor.stopMotor();
        driveMotor.stopMotor();
    }
}
