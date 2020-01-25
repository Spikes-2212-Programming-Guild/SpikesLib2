package com.spikes2212.command.genericsubsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface PneumaticSubsystem extends Subsystem {
    void close();

    void open();
}
