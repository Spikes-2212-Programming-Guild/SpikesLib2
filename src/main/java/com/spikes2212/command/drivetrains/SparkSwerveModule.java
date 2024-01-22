package com.spikes2212.command.drivetrains;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;

import java.util.function.Supplier;

public class SparkSwerveModule extends GenericSwerveModule {

    private static final int PID_SLOT = 0;

    private final CANSparkBase driveController;
    private final CANSparkBase turnController;
    private final RelativeEncoder driveEncoder;

    private final double maxSpeedMetersPerSecond;

    private final PIDSettings turnPIDSettings;
    private final PIDSettings drivePIDSettings;
    private final FeedForwardSettings driveFeedForwardSettings;
    private final FeedForwardController driveFeedForwardController;

    public SparkSwerveModule(String namespaceName, CANSparkBase driveController, CANSparkBase turnController,
                             Supplier<Double> moduleRotation, boolean driveInverted, double minSpeedMetersPerSecond,
                             double maxSpeedMetersPerSecond, FeedForwardSettings driveFeedForwardSettings,
                             PIDSettings turnPIDSettings, PIDSettings drivePIDSettings) {
        super(namespaceName, driveController, turnController, moduleRotation, driveInverted, minSpeedMetersPerSecond);
        this.driveController = driveController;
        this.turnController = turnController;
        driveEncoder = driveController.getEncoder();
        this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
        this.turnPIDSettings = turnPIDSettings;
        this.drivePIDSettings = drivePIDSettings;
        this.driveFeedForwardSettings = driveFeedForwardSettings;
        driveFeedForwardController = new FeedForwardController(driveFeedForwardSettings,
                FeedForwardController.DEFAULT_PERIOD);
        configureDriveEncoder();
        configureTurnController();
    }

    public void configureDriveEncoder() {
        driveController.getPIDController().setP(drivePIDSettings.getkP());
        driveController.getPIDController().setI(drivePIDSettings.getkI());
        driveController.getPIDController().setD(drivePIDSettings.getkD());
    }

    public void configureTurnController() {
        turnController.getPIDController().setP(turnPIDSettings.getkP());
        turnController.getPIDController().setI(turnPIDSettings.getkI());
        turnController.getPIDController().setD(turnPIDSettings.getkD());
    }

    @Override
    public double getSpeed() {
        return driveEncoder.getVelocity();
    }

    @Override
    public void setAngle(double angle) {
        configureTurnController();
        turnController.getPIDController().setReference(angle, CANSparkBase.ControlType.kPosition, PID_SLOT);
    }

    @Override
    public void setSpeed(double speedMetersPerSecond, boolean usePID) {
        if (usePID) {
            configureDriveEncoder();
            driveFeedForwardController.setGains(driveFeedForwardSettings);
            double feedForward = driveFeedForwardController.calculate(speedMetersPerSecond);
            driveController.getPIDController().setReference(speedMetersPerSecond, CANSparkBase.ControlType.kVelocity,
                    PID_SLOT, feedForward);
        } else driveController.set(speedMetersPerSecond / maxSpeedMetersPerSecond);
    }

    @Override
    public void configureDashboard() {
    }
}
