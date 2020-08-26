package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * A command that moves a drivetrain with a set speed forward and with a PID loop to a certain angle.
 */
public class DriveArcadeWithPID extends CommandBase {
    /**
     * The drivetrain this command operates on.
     */
    private final TankDrivetrain drivetrain;

    /**
     * The PID Settings for the turning PID loop.
     */
    private PIDFSettings PIDFSettings;

    /**
     * The PID Controller for the turning PID loop.
     */
    private PIDController pidController;

    private FeedForwardSettings feedForwardSettings;
    private FeedForwardController feedForwardController;

    /**
     * The angle of the drivetrain.
     */
    private Supplier<Double> source;

    /**
     * The last time the drivetrain's angle wasn't within the target range.
     */
    private double lastTimeNotOnTarget;

    /**
     * The angle the drivetrain should reach.
     */
    private Supplier<Double> setpoint;

    /**
     * The speed at which to move the drivetrain forward.
     */
    private Supplier<Double> moveValue;

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, Supplier<Double> setpoint,
                              Supplier<Double> moveValue, PIDFSettings PIDFSettings,
                              FeedForwardSettings feedForwardSettings) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
        this.PIDFSettings = PIDFSettings;
        this.feedForwardSettings = feedForwardSettings;
        this.source = source;
        this.moveValue = moveValue;
        this.pidController = new PIDController(PIDFSettings.getkP(), PIDFSettings.getkI(), PIDFSettings.getkD());
        this.pidController.setSetpoint(setpoint.get());
        this.feedForwardController = new FeedForwardController(feedForwardSettings.getkS(), feedForwardSettings.getkV(),
                feedForwardSettings.getkA(), feedForwardSettings.getkG());
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, double setpoint, double moveValue,
                              PIDFSettings PIDFSettings, FeedForwardSettings feedForwardSettings) {
        this(drivetrain, source, () -> setpoint, () -> moveValue, PIDFSettings, feedForwardSettings);
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, Supplier<Double> setpoint,
                              Supplier<Double> moveValue, PIDFSettings PIDFSettings) {
        this(drivetrain, source, setpoint, moveValue, PIDFSettings, FeedForwardSettings.EMPTY_FFSETTINGS);
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, double setpoint, double moveValue,
                              PIDFSettings PIDFSettings) {
        this(drivetrain, source, () -> setpoint, () -> moveValue, PIDFSettings);
    }

    /**
     * updates the PIDLoop's setpoint.
     */
    @Override
    public void execute() {
        pidController.setTolerance(PIDFSettings.getTolerance());
        pidController.setPID(PIDFSettings.getkP(), PIDFSettings.getkI(), PIDFSettings.getkD());

        feedForwardController.setGains(feedForwardSettings.getkS(), feedForwardSettings.getkV(),
                feedForwardSettings.getkA(), feedForwardSettings.getkG());

        drivetrain.arcadeDrive(moveValue.get(), pidController.calculate(source.get(), setpoint.get()) +
                feedForwardController.calculate(setpoint.get()));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        double currentTime = Timer.getFPGATimestamp();

        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return currentTime - lastTimeNotOnTarget >= PIDFSettings.getWaitTime();
    }
}
