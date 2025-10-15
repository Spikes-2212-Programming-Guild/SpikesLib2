package com.spikes2212.command.drivetrains.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule extends DashboardedSubsystem {

    private static final double ABSOLUTE_POSITION_DISCONTINUITY_POINT = 1;
    private static final double DEGREES_IN_ROTATIONS = 360;

    private final SmartMotorController driveMotor;
    private final SmartMotorController turnMotor;
    private final CANcoder absoluteEncoder;

    private final boolean cancoderInverted;
    private final boolean driveInverted;
    private final boolean usePIDAngle;
    private final boolean usePIDVelocity;
    private final double offset;

    private final PIDSettings drivePIDSettings;
    private final PIDSettings turnPIDSettings;
    private final FeedForwardSettings driveFeedForwardSettings;
    private final FeedForwardSettings turnFeedForwardSettings;

    public SwerveModule(String namespaceName, SmartMotorController driveMotor, SmartMotorController turnMotor,
                        CANcoder absoluteEncoder, boolean cancoderInverted, boolean driveInverted, boolean turnInverted,
                        boolean usePIDAngle, boolean usePIDVelocity, double offset, PIDSettings drivePIDSettings,
                        PIDSettings turnPIDSettings, FeedForwardSettings driveFeedForwardSettings,
                        FeedForwardSettings turnFeedForwardSettings) {
        super(namespaceName);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.absoluteEncoder = absoluteEncoder;
        this.cancoderInverted = cancoderInverted;
        this.driveInverted = driveInverted;
        this.usePIDAngle = usePIDAngle;
        this.usePIDVelocity = usePIDVelocity;
        this.offset = offset;
        this.drivePIDSettings = drivePIDSettings;
        this.turnPIDSettings = turnPIDSettings;
        this.driveFeedForwardSettings = driveFeedForwardSettings;
        this.turnFeedForwardSettings = turnFeedForwardSettings;
        driveMotor.setInverted(driveInverted);
        turnMotor.setInverted(cancoderInverted);
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
        if (usePIDAngle) {
            turnMotor.pidSet(UnifiedControlMode.POSITION, targetAngle.getDegrees(), turnPIDSettings,
                    turnFeedForwardSettings, false);
        } else {
            turnMotor.set(targetAngle.getDegrees());
        }
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

    public void configureAbsoluteEncoder() {
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(ABSOLUTE_POSITION_DISCONTINUITY_POINT)
                .withSensorDirection(cancoderInverted ? SensorDirectionValue.Clockwise_Positive :
                        SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(offset);
        absoluteEncoder.getConfigurator().apply(magnetConfigs);
    }

    public double getAbsoluteAngle() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * DEGREES_IN_ROTATIONS;
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
