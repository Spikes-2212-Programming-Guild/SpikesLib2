package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;

import java.util.function.Supplier;

/**
 * This command moves a {@link GenericSubsystem} according to a {@link Supplier} so it will reach a certain speed.
 *
 * @author Ofri Rosenbaum
 * @see MoveGenericSubsystemWithPID
 */
public class MoveGenericSubsystemWithPIDForSpeed extends MoveGenericSubsystemWithPID {

    public MoveGenericSubsystemWithPIDForSpeed(GenericSubsystem subsystem, Supplier<Double> targetVelocity,
                                               Supplier<Double> source, PIDSettings pidSettings,
                                               FeedForwardSettings feedForwardSettings) {
        super(subsystem, targetVelocity, source, pidSettings, feedForwardSettings);
    }

    public MoveGenericSubsystemWithPIDForSpeed(GenericSubsystem subsystem, double targetVelocity, double source,
                                               PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        super(subsystem, targetVelocity, source, pidSettings, feedForwardSettings);
    }

    public MoveGenericSubsystemWithPIDForSpeed(GenericSubsystem subsystem, Supplier<Double> targetVelocity,
                                               Supplier<Double> source, PIDSettings pidSettings) {
        super(subsystem, targetVelocity, source, pidSettings);
    }

    public MoveGenericSubsystemWithPIDForSpeed(GenericSubsystem subsystem, double targetVelocity, double source,
                                               PIDSettings pidSettings) {
        super(subsystem, targetVelocity, source, pidSettings);
    }

    @Override
    public void execute() {
        subsystem.move(subsystem.getSpeed() + calculatePIDAndFFValues());
    }
}
