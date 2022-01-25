package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * A command that moves a {@link GenericSubsystem} according to a {@link Supplier}
 * or a constant speed until it can't move anymore.
 *
 * @author Yuval Levy
 * @see GenericSubsystem
 */
public class MoveGenericSubsystemWithPID extends CommandBase {

    private final GenericSubsystem subsystem;

    /**
     * A supplier that returns the subsystem's current location.
     */
    private Supplier<Double> source;

    private PIDSettings pidSettings;

    private FeedForwardSettings feedForwardSettings;

    /**
     * the setpoint for the subsystem.
     */
    private Supplier<Double> setpoint;

    private PIDController pidController;

    private FeedForwardController feedForwardController;

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
                feedForwardSettings.getkA(), feedForwardSettings.getkG(), 0.02);
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

    @Override
    public void execute() {
        pidController.setTolerance(pidSettings.getTolerance());
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        feedForwardController.setGains(feedForwardSettings.getkS(), feedForwardController.getkV(),
                feedForwardController.getkA(), feedForwardController.getkG());

        double pidValue = pidController.calculate(source.get(), setpoint.get());
        double svagValue = feedForwardController.calculate(setpoint.get());
        subsystem.move(pidValue + svagValue);
    }


    @Override
    public boolean isFinished() {
        if(!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= pidSettings.getWaitTime();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }
}
