package com.spikes2212.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

/**
 * A command that repeats a sequence of commands indefinitely.
 *
 * @author Eran Goldstein
 */
@Deprecated(since = "2024", forRemoval = true)
public class RepeatCommand extends Command {

    private final Command command;

    /**
     * Constructs a RepeatCommand that will repeat a SequentialCommandGroup that will include the commands given.
     *
     * @param command the commands to be repeated
     */
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
        if (command.isFinished())
            command.initialize();
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
