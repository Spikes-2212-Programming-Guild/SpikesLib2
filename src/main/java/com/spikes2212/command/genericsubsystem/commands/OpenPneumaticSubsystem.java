package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.PneumaticSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class OpenPneumaticSubsystem extends InstantCommand {
    public OpenPneumaticSubsystem(PneumaticSubsystem subsystem) {
        super(subsystem::open, subsystem);
    }
}