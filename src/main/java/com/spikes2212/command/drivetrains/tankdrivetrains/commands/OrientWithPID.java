package com.spikes2212.command.drivetrains.tankdrivetrains.commands;

import com.spikes2212.command.drivetrains.tankdrivetrains.TankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;

import java.util.function.Supplier;

@Deprecated(forRemoval = true, since = "2025")
public class OrientWithPID extends DriveArcadeWithPID {

    public OrientWithPID(TankDrivetrain drivetrain, Supplier<Double> source, Supplier<Double> setpoint,
                         PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        super(drivetrain, source, setpoint, () -> 0.0, pidSettings, feedForwardSettings);
    }

    public OrientWithPID(TankDrivetrain drivetrain, Supplier<Double> source, double setpoint, PIDSettings pidSettings,
                         FeedForwardSettings feedForwardSettings) {
        this(drivetrain, source, () -> setpoint, pidSettings, feedForwardSettings);
    }

    public OrientWithPID(TankDrivetrain drivetrain, Supplier<Double> source, Supplier<Double> setpoint,
                         PIDSettings pidSettings) {
        this(drivetrain, source, setpoint, pidSettings, FeedForwardSettings.EMPTY_FF_SETTINGS);
    }

    public OrientWithPID(TankDrivetrain drivetrain, Supplier<Double> source, double setpoint, PIDSettings pidSettings) {
        this(drivetrain, source, setpoint, pidSettings, FeedForwardSettings.EMPTY_FF_SETTINGS);
    }
}
