package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.PIDLoop;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveGenericSubsystemWithPID extends CommandBase {

    private final GenericSubsystem subsystem;
    private final PIDLoop pidLoop;

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, PIDLoop pidLoop) {
        super();
        this.subsystem = subsystem;
        this.pidLoop = pidLoop;
        this.addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        pidLoop.enable();
    }

    @Override
    public void execute() {
        pidLoop.update();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
        pidLoop.disable();
    }

    @Override
    public boolean isFinished() {
        return pidLoop.onTarget();
    }
}
