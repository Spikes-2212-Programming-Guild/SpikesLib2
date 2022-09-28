package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.Supplier;

/**
 * A command that moves a {@link TankDrivetrain} with a set speed forward and with a PID loop to a certain angle.
 *
 * @author Yuval Levy
 * @see TankDrivetrain
 */
public class DriveArcadeWithPID extends CommandBase {

    /**
     * The drivetrain this command operates on.
     */
    protected final TankDrivetrain drivetrain;

    /**
     * The PID settings for the turning PID loop.
     */
    protected final PIDSettings pidSettings;

    /**
     * The PID controller for the turning PID loop.
     */
    protected PIDController pidController;

    /**
     * The FeedForward settings for the turning FeedForwards loop.
     */
    protected final FeedForwardSettings feedForwardSettings;

    /**
     * The FeedForward controller for the turning FeedForwards loop.
     */
    protected FeedForwardController feedForwardController;

    /**
     * The angle of the drivetrain.
     */
    protected Supplier<Double> source;

    /**
     * The last time the drivetrain's angle wasn't within the target range.
     */
    private double lastTimeNotOnTarget;

    /**
     * The angle the drivetrain should reach.
     */
    protected Supplier<Double> setpoint;

    /**
     * The speed at which to move the drivetrain forward.
     */
    protected Supplier<Double> moveValue;

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, Supplier<Double> setpoint,
                              Supplier<Double> moveValue, PIDSettings pidSettings,
                              FeedForwardSettings feedForwardSettings) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
        this.pidSettings = pidSettings;
        this.feedForwardSettings = feedForwardSettings;
        this.source = source;
        this.moveValue = moveValue;
        this.pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        this.pidController.setSetpoint(setpoint.get());
        this.feedForwardController = new FeedForwardController(feedForwardSettings.getkS(), feedForwardSettings.getkV(),
                feedForwardSettings.getkA(), feedForwardSettings.getkG(), FeedForwardController.DEFAULT_PERIOD);
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, double setpoint, double moveValue,
                              PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        this(drivetrain, source, () -> setpoint, () -> moveValue, pidSettings, feedForwardSettings);
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, Supplier<Double> setpoint,
                              Supplier<Double> moveValue, PIDSettings pidSettings) {
        this(drivetrain, source, setpoint, moveValue, pidSettings, FeedForwardSettings.EMPTY_FFSETTINGS);
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, double setpoint, double moveValue,
                              PIDSettings pidSettings) {
        this(drivetrain, source, () -> setpoint, () -> moveValue, pidSettings);
    }

    @Override
    public void execute() {
        pidController.setTolerance(pidSettings.getTolerance());
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());

        feedForwardController.setGains(feedForwardSettings.getkS(), feedForwardSettings.getkV(),
                feedForwardSettings.getkA(), feedForwardSettings.getkG());

        drivetrain.arcadeDrive(moveValue.get(), pidController.calculate(source.get(), setpoint.get()) +
                feedForwardController.calculate(setpoint.get()));
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
