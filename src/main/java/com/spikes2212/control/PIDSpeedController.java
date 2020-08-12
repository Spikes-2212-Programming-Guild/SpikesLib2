package com.spikes2212.control;

import java.util.function.Supplier;

public interface PIDSpeedController {
    void configureLoop(Supplier<Double> maxSpeed, Supplier<Double> minSpeed, int timeout);

    void pidSet(double setpoint, int timeout);

    boolean onTarget(double setpoint);

    void finish();
}
