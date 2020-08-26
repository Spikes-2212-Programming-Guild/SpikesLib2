package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDFSettings;

import java.util.function.Supplier;

public class OrientWithPID extends DriveArcadeWithPID {

    public OrientWithPID(TankDrivetrain drivetrain, Supplier<Double> source, Supplier<Double> setpoint,
                         PIDFSettings PIDFSettings, FeedForwardSettings feedForwardSettings) {
        super(drivetrain, source, setpoint, () -> 0.0, PIDFSettings, feedForwardSettings);
    }

    public OrientWithPID(TankDrivetrain drivetrain, Supplier<Double> source, double setpoint, PIDFSettings PIDFSettings,
                         FeedForwardSettings feedForwardSettings) {
        this(drivetrain, source, () -> setpoint, PIDFSettings, feedForwardSettings);
    }

    public OrientWithPID(TankDrivetrain drivetrain, Supplier<Double> source, Supplier<Double> setpoint,
                         PIDFSettings PIDFSettings) {
        this(drivetrain, source, setpoint, PIDFSettings, FeedForwardSettings.EMPTY_FFSETTINGS);
    }

    public OrientWithPID(TankDrivetrain drivetrain, Supplier<Double> source, double setpoint, PIDFSettings PIDFSettings) {
        this(drivetrain, source, setpoint, PIDFSettings, FeedForwardSettings.EMPTY_FFSETTINGS);
    }

}
