package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.PneumaticSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClosePneumaticSubsystem extends InstantCommand {
    public ClosePneumaticSubsystem(PneumaticSubsystem subsystem) {
        super(subsystem::close, subsystem);
    }
}
