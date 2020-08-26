package com.spikes2212.control;

import java.util.function.Supplier;

public class MotionMagicSettings {
    /**
     * The maximum acceleration.
     */
    private Supplier<Double> targetAcceleration;

    /**
     * The maximum velocity.
     */
    private Supplier<Double> maxVelocity;

    /**
     * The smoothness of the acceleration graph (between 0-8).
     */
    private Supplier<Double> smoothing;

    public MotionMagicSettings(Supplier<Double> targetAcceleration, Supplier<Double> maxVelocity, Supplier<Double> smoothing) {
        this.targetAcceleration = targetAcceleration;
        this.maxVelocity = maxVelocity;
        this.smoothing = smoothing;
    }

    public Supplier<Double> getTargetAcceleration() {
        return targetAcceleration;
    }

    public void setTargetAcceleration(Supplier<Double> targetAcceleration) {
        this.targetAcceleration = targetAcceleration;
    }

    public Supplier<Double> getMaxVelocity() {
        return maxVelocity;
    }

    public void setMaxVelocity(Supplier<Double> maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    public Supplier<Double> getSmoothing() {
        return smoothing;
    }

    public void setSmoothing(Supplier<Double> smoothing) {
        this.smoothing = smoothing;
    }
}