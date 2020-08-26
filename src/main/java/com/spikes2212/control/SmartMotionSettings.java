package com.spikes2212.control;

import java.util.function.Supplier;

public class SmartMotionSettings {
    private Supplier<Double> minVelocity;
    private Supplier<Double> maxVelocity;
    private Supplier<Double> maxAcceleration;

    public SmartMotionSettings(Supplier<Double> minVelocity, Supplier<Double> maxVelocity, Supplier<Double> maxAcceleration) {
        this.minVelocity = minVelocity;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public SmartMotionSettings(Supplier<Double> maxVelocity, Supplier<Double> maxAcceleration) {
        this(() -> 0.0, maxVelocity, maxAcceleration);
    }

    public Supplier<Double> getMinVelocity() {
        return minVelocity;
    }

    public void setMinVelocity(Supplier<Double> minVelocity) {
        this.minVelocity = minVelocity;
    }

    public Supplier<Double> getMaxVelocity() {
        return maxVelocity;
    }

    public void setMaxVelocity(Supplier<Double> maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    public Supplier<Double> getMaxAcceleration() {
        return maxAcceleration;
    }

    public void setMaxAcceleration(Supplier<Double> maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }
}
