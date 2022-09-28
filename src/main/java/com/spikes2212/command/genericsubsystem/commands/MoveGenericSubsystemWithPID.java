package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * This command moves a {@link GenericSubsystem} according to a {@link Supplier}
 * or a constant speed until it reaches its target or can't move anymore.
 *
 * @author Yuval Levy
 * @see GenericSubsystem
 */
public class MoveGenericSubsystemWithPID extends CommandBase {

    /**
     * the subsystem the command moves.
     */
    protected final GenericSubsystem subsystem;

    /**
     * A supplier that returns the subsystem's current location.
     */
    protected final Supplier<Double> source;

    /**
     * The PID Settings for the PID control loop.
     */
    protected final PIDSettings pidSettings;

    /**
     * The Feed Forward Settings for the feed forward control loop.
     */
    protected final FeedForwardSettings feedForwardSettings;

    /**
     * the setpoint for the subsystem.
     */
    protected final Supplier<Double> setpoint;

    /**
     * An object that makes the necessary calculations for the PID control loop.
     */
    protected final PIDController pidController;

    /**
     * An object that makes the necessary calculations for the feed forward control loop.
     */
    protected final FeedForwardController feedForwardController;

    /**
     * The last time the subsystem didn't reach target.
     */
    private double lastTimeNotOnTarget;

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, Supplier<Double> setpoint, Supplier<Double> source,
                                       PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.pidSettings = pidSettings;
        this.feedForwardSettings = feedForwardSettings;
        this.setpoint = setpoint;
        this.source = source;
        this.feedForwardController = new FeedForwardController(feedForwardSettings.getkS(), feedForwardSettings.getkV(),
                feedForwardSettings.getkA(), feedForwardSettings.getkG(), FeedForwardController.DEFAULT_PERIOD);
        this.pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
    }

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, double setpoint, double source,
                                       PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        this(subsystem, () -> setpoint, () -> source, pidSettings, feedForwardSettings);
    }

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, Supplier<Double> setpoint, Supplier<Double> source,
                                       PIDSettings pidSettings) {
        this(subsystem, setpoint, source, pidSettings, FeedForwardSettings.EMPTY_FFSETTINGS);
    }

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, double setpoint, double source,
                                       PIDSettings pidSettings) {
        this(subsystem, () -> setpoint, () -> source, pidSettings, FeedForwardSettings.EMPTY_FFSETTINGS);
    }

    protected double calculatePIDAndFFValues() {
        pidController.setTolerance(pidSettings.getTolerance());
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        feedForwardController.setGains(feedForwardSettings.getkS(), feedForwardSettings.getkV(),
                feedForwardSettings.getkA(), feedForwardSettings.getkG());

        double pidValue = pidController.calculate(source.get(), setpoint.get());
        double svagValue = feedForwardController.calculate(setpoint.get());
        return pidValue + svagValue;
    }

    @Override
    public void execute() {
        subsystem.move(calculatePIDAndFFValues());
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= pidSettings.getWaitTime();
    }
}
