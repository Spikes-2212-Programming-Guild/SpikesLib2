package com.spikes2212.path;

/**
 * This class is used to limit the rate of change of robot velocities,
 * it is used in {@link PurePursuitController} in order to achieve smoother acceleration.
 */
public class RateLimiter {

    private double maxAcceleration;
    private double period;
    private double lastVelocity = 0;

    public RateLimiter(double maxAcceleration, double period) {
        this.maxAcceleration = maxAcceleration;
        this.period = period;
    }

    public double getPeriod() {
        return period;
    }

    public void setPeriod(double period) {
        this.period = period;
    }

    public double getMaxChange() {
        return maxAcceleration * period;
    }

    private double clampRate(double rate) {
        double maxRate = getMaxChange();
        return Math.max(-maxRate, Math.min(maxRate, rate));
    }

    public double calculate(double targetVelocity) {
        lastVelocity = lastVelocity + clampRate(targetVelocity - lastVelocity);
        return lastVelocity;
    }
}
