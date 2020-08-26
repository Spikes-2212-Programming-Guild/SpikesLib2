package com.spikes2212.control;

import java.util.function.Supplier;

public class MotionMagicSettings {
    /**
     * The maximum acceleration.
     */
    private Supplier<Integer> targetAcceleration;

    /**
     * The maximum velocity.
     */
    private Supplier<Integer> maxVelocity;

    /**
     * The smoothness of the acceleration graph (between 0-8).
     */
    private Supplier<Integer> smoothing;

    public MotionMagicSettings(Supplier<Integer> targetAcceleration, Supplier<Integer> maxVelocity, Supplier<Integer> smoothing) {
        this.targetAcceleration = targetAcceleration;
        this.maxVelocity = maxVelocity;
        this.smoothing = smoothing;
    }

    public int getTargetAcceleration() {
        return targetAcceleration.get();
    }

    public void setTargetAcceleration(Supplier<Integer> targetAcceleration) {
        this.targetAcceleration = targetAcceleration;
    }

    public int getMaxVelocity() {
        return maxVelocity.get();
    }

    public void setMaxVelocity(Supplier<Integer> maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    public int getSmoothing() {
        return smoothing.get();
    }

    public void setSmoothing(Supplier<Integer> smoothing) {
        this.smoothing = smoothing;
    }
}