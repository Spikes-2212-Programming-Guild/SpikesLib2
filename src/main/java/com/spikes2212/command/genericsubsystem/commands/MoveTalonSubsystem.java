package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.TalonSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Move a {@link TalonSubsystem} using its Talon's control loops.
 */
public class MoveTalonSubsystem extends CommandBase {
    /**
     * The {@link TalonSubsystem} this command will run on.
     */
    private final TalonSubsystem subsystem;

    /**
     * The setpoint this command should bring the {@link TalonSubsystem} to.
     */
    private final double setpoint;

    public MoveTalonSubsystem(TalonSubsystem subsystem, double setpoint) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        subsystem.configureLoop();
    }

    @Override
    public void execute() {
        subsystem.pidSet(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.finish();
    }

    @Override
    public boolean isFinished() {
        return subsystem.onTarget(setpoint);
    }
}
