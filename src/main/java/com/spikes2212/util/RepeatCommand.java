package com.spikes2212.util;

import edu.wpi.first.wpilibj2.command.*;

import java.util.Set;

/**
 * This {@link Command} repeats a sequence of commands indefinitely
 */

public class RepeatCommand extends CommandBase {

    private Command command;

    public RepeatCommand(Command... command) {
        this.command = new SequentialCommandGroup(command);
    }

    @Override
    public void initialize() {
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
        if(command.isFinished()) {
            command.initialize();
        }
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return command.getRequirements();
    }
}
