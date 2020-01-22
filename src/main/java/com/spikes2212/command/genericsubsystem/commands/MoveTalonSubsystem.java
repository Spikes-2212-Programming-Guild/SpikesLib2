package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.TalonSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveTalonSubsystem extends CommandBase {
    private final TalonSubsystem subsystem;
    private final double setpoint;

    public MoveTalonSubsystem(TalonSubsystem subsystem, double setpoint) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        subsystem.config();
    }

    @Override
    public void execute() {
        subsystem.update(setpoint);
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
