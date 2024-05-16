package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.SwerveDrivetrain;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class RotateSwerveWithPID extends Command {

    private final SwerveDrivetrain drivetrain;
    private final Supplier<Double> setpoint;
    private final PIDSettings pidSettings;
    private final PIDController pidController;
    private final FeedForwardSettings feedForwardSettings;
    private final FeedForwardController feedForwardController;
    private double lastTimeNotOnTarget;

    public RotateSwerveWithPID(SwerveDrivetrain drivetrain, Supplier<Double> setpoint, PIDSettings pidSettings,
                               FeedForwardSettings feedForwardSettings) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
        this.pidSettings = pidSettings;
        pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setTolerance(pidSettings.getTolerance());
        this.feedForwardSettings = feedForwardSettings;
        feedForwardController = new FeedForwardController(feedForwardSettings, FeedForwardController.DEFAULT_PERIOD);
    }

    @Override
    public void execute() {
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setTolerance(pidSettings.getTolerance());
        feedForwardController.setGains(feedForwardSettings);

        drivetrain.drive(0, 0, pidController.calculate(drivetrain.getAngle(), setpoint.get()) +
                feedForwardController.calculate(setpoint.get()), false, false);
    }

    @Override
    public boolean isFinished() {
        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= pidSettings.getWaitTime();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
