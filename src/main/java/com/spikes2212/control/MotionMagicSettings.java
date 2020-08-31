package com.spikes2212.control;

import java.util.function.Supplier;

/**
 * The MotionMagic constants used in a PID loop.
 */

public class MotionMagicSettings {
    /**
     * The maximum acceleration.
     */
    private Supplier<Integer> maximumAcceleration;

    /**
     * The maximum velocity.
     */
    private Supplier<Integer> maxVelocity;

    /**
     * The smoothness of the acceleration graph (between 0-8).
     */
    private Supplier<Integer> smoothing;

    public MotionMagicSettings(Supplier<Integer> maximumAcceleration, Supplier<Integer> maxVelocity, Supplier<Integer> smoothing) {
        this.maximumAcceleration = maximumAcceleration;
        this.maxVelocity = maxVelocity;
        this.smoothing = smoothing;
    }

    public int getMaximumAcceleration() {
        return maximumAcceleration.get();
    }

    public void setMaximumAcceleration(Supplier<Integer> maximumAcceleration) {
        this.maximumAcceleration = maximumAcceleration;
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
