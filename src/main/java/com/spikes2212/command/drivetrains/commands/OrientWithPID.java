package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;

import java.util.function.Supplier;

public class OrientWithPID extends DriveArcadeWithPID {
    public OrientWithPID(TankDrivetrain drivetrain, PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                         Supplier<Double> source, Supplier<Double> setpoint) {
        super(drivetrain, pidSettings, feedForwardSettings, source, setpoint, () -> 0.0);
    }

    public OrientWithPID(TankDrivetrain drivetrain, PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                         Supplier<Double> source, double setpoint) {
        this(drivetrain, pidSettings, feedForwardSettings, source, () -> setpoint);
    }

    public OrientWithPID(TankDrivetrain drivetrain, PIDSettings pidSettings, Supplier<Double> source,
                         Supplier<Double> setpoint) {
        this(drivetrain, pidSettings, FeedForwardSettings.EMPTY_FFSETTINGS, source, setpoint);
    }

    public OrientWithPID(TankDrivetrain drivetrain, PIDSettings pidSettings, Supplier<Double> source, double setpoint) {
        this(drivetrain, pidSettings, FeedForwardSettings.EMPTY_FFSETTINGS, source, setpoint);
    }
}
