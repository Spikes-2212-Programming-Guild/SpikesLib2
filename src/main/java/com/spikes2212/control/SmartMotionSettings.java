package com.spikes2212.control;

import java.util.function.Supplier;

/**
 * The smart motion constants used in a smart motion loop, in addition to the regular PID constants.
 */
public class SmartMotionSettings {

    /**
     * The minimum velocity of the motor during this smart motion loop.
     */
    private Supplier<Double> minVelocity;

    /**
     * The maximum velocity of the motor during this smart motion loop.
     */
    private Supplier<Double> maxVelocity;

    /**
     * The maximum acceleration of the motor during this smart motion loop.
     */
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
