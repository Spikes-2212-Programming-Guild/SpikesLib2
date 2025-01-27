package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/**
 * A command that moves a {@link TankDrivetrain} with a set speed forward and with a PID loop to a certain angle.
 *
 * @author Yuval Levy
 * @see TankDrivetrain
 */
public class DriveArcadeWithPID extends Command {

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
    protected final PIDController pidController;

    /**
     * The FeedForward settings for the turning FeedForwards loop.
     */
    protected final FeedForwardSettings feedForwardSettings;

    /**
     * The FeedForward controller for the turning FeedForwards loop.
     */
    protected final FeedForwardController feedForwardController;

    /**
     * The angle of the drivetrain.
     */
    protected final Supplier<Double> source;

    /**
     * The angle the drivetrain should reach.
     */
    protected final Supplier<Double> setpoint;

    /**
     * The acceleration the drivetrain should be at.
     */
    protected final Supplier<Double> acceleration;

    /**
     * The speed at which to move the drivetrain forward.
     */
    protected final Supplier<Double> moveValue;

    /**
     * The last time the drivetrain's angle wasn't within the target range.
     */
    private double lastTimeNotOnTarget;

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, Supplier<Double> setpoint,
                              Supplier<Double> acceleration, Supplier<Double> moveValue, PIDSettings pidSettings,
                              FeedForwardSettings feedForwardSettings) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
        this.acceleration = acceleration;
        this.pidSettings = pidSettings;
        this.feedForwardSettings = feedForwardSettings;
        this.source = source;
        this.moveValue = moveValue;
        this.pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        this.pidController.setSetpoint(setpoint.get());
        this.feedForwardController = new FeedForwardController(feedForwardSettings.getkS(), feedForwardSettings.getkV(),
                feedForwardSettings.getkA(), feedForwardSettings.getkG(), feedForwardSettings.getControlMode());
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, double setpoint,
                              Supplier<Double> acceleration, double moveValue, PIDSettings pidSettings,
                              FeedForwardSettings feedForwardSettings) {
        this(drivetrain, source, () -> setpoint, acceleration, () -> moveValue, pidSettings, feedForwardSettings);
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, Supplier<Double> setpoint,
                              Supplier<Double> moveValue, PIDSettings pidSettings,
                              FeedForwardSettings feedForwardSettings) {
        this(drivetrain, source, setpoint, () -> 0.0, moveValue, pidSettings, feedForwardSettings);
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, double setpoint,
                              double moveValue, PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        this(drivetrain, source, () -> setpoint, () -> 0.0, () -> moveValue, pidSettings, feedForwardSettings);
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, Supplier<Double> setpoint,
                              Supplier<Double> moveValue, PIDSettings pidSettings) {
        this(drivetrain, source, setpoint, () -> 0.0, moveValue, pidSettings, FeedForwardSettings.EMPTY_FF_SETTINGS);
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, double setpoint,
                              double moveValue, PIDSettings pidSettings) {
        this(drivetrain, source, () -> setpoint, () -> moveValue, pidSettings);
    }

    @Override
    public void execute() {
        pidController.setTolerance(pidSettings.getTolerance());
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setIZone(pidSettings.getIZone());

        feedForwardController.setGains(feedForwardSettings);

        drivetrain.arcadeDrive(moveValue.get(), pidController.calculate(source.get(), setpoint.get()) +
                feedForwardController.calculate(source.get(), setpoint.get(), acceleration.get()));
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
