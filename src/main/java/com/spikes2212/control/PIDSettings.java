package com.spikes2212.control;

import java.util.function.Supplier;

/**
 * The PID constants used in a {@link PIDLoop}, and the error tolerance and loop wait time.
 */
public class PIDSettings {
    /**
     * The proportional component of the PID settings.
     */
    private Supplier<Double> kP;

    /**
     * The integral component of the PID settings.
     */
    private Supplier<Double> kI;

    /**
     * The derivative component of the PID settings.
     */
    private Supplier<Double> kD;

    /**
     * The acceptable distance from the target.
     */
    private Supplier<Double> tolerance;

    /**
     * The time required to stay on target.
     */
    private Supplier<Double> waitTime;

    public PIDSettings(double kP, double kI, double kD) {
        this(kP, kI, kD, 0);
    }

    public PIDSettings(double kP, double kI, double kD, double tolerance) {
        this(kP, kI, kD, tolerance, 0);
    }

    public PIDSettings(double kP, double kI, double kD, double tolerance, double waitTime) {
        this(() -> kP, () -> kI, () -> kD, () -> tolerance, () -> waitTime);
    }

    public PIDSettings(Supplier<Double> kP, Supplier<Double> kI, Supplier<Double> kD) {
        this(kP, kI, kD, () -> 0.0);
    }

    public PIDSettings(Supplier<Double> kP, Supplier<Double> kI, Supplier<Double> kD, Supplier<Double> tolerance) {
        this(kP, kI, kD, tolerance, () -> 0.0);
    }

    public PIDSettings(Supplier<Double> kP, Supplier<Double> kI, Supplier<Double> kD, Supplier<Double> tolerance, Supplier<Double> waitTime) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.tolerance = tolerance;
        this.waitTime = waitTime;
    }

    public Supplier<Double> getkP() {
        return kP;
    }

    public void setkP(Supplier<Double> kP) {
        this.kP = kP;
    }

    public Supplier<Double> getkI() {
        return kI;
    }

    public void setkI(Supplier<Double> kI) {
        this.kI = kI;
    }

    public Supplier<Double> getkD() {
        return kD;
    }

    public void setkD(Supplier<Double> kD) {
        this.kD = kD;
    }

    public Supplier<Double> getTolerance() {
        return tolerance;
    }

    public void setTolerance(Supplier<Double> tolerance) {
        this.tolerance = tolerance;
    }

    public Supplier<Double> getWaitTime() {
        return waitTime;
    }

    public void setWaitTime(Supplier<Double> waitTime) {
        this.waitTime = waitTime;
    }
}
