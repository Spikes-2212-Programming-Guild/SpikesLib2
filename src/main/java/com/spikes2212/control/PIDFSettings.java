package com.spikes2212.control;

import java.util.function.Supplier;

/**
 * The PID constants used in a PID loop, and the error tolerance and loop wait time.
 */
public class PIDFSettings {
    /**
     * The proportional component of the PIDF settings.
     */
    private Supplier<Double> kP;

    /**
     * The integral component of the PIDF settings.
     */
    private Supplier<Double> kI;

    /**
     * The derivative component of the PIDF settings.
     */
    private Supplier<Double> kD;

    /**
     * The feed forward component of the PIDF settings required to stay on target.
     */
    private Supplier<Double> kF;

    /**
     * The acceptable distance from the setpoint.
     */
    private Supplier<Double> tolerance;

    /**
     * The time required to stay on target.
     */
    private Supplier<Double> waitTime;

    public PIDFSettings(Supplier<Double> kP, Supplier<Double> kI, Supplier<Double> kD, Supplier<Double> kF, Supplier<Double> tolerance,
                        Supplier<Double> waitTime) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.tolerance = tolerance;
        this.waitTime = waitTime;
    }

    public PIDFSettings(double kP, double tolerance, double waitTime) {
        this(kP, 0.0, 0.0, tolerance, waitTime);
    }

    public PIDFSettings(double kP, double kI, double kD, double tolerance, double waitTime) {
        this(() -> kP, () -> kI, () -> kD, () -> 0.0, () -> tolerance, () -> waitTime);
    }

    public PIDFSettings(double kP, double kI, double kD, double kF, double tolerance, double waitTime) {
        this(() -> kP, () -> kI, () -> kD, () -> kF, () -> tolerance, () -> waitTime);
    }

    public PIDFSettings(Supplier<Double> kP, Supplier<Double> tolerance, Supplier<Double> waitTime) {
        this(kP, () -> 0.0, () -> 0.0, tolerance, waitTime);
    }

    public PIDFSettings(Supplier<Double> kP, Supplier<Double> kI, Supplier<Double> kD, Supplier<Double> tolerance,
                        Supplier<Double> waitTime) {
        this(kP, kI, kD, () -> 0.0, tolerance, waitTime);
    }

    public double getkP() {
        return kP.get();
    }

    public void setkP(Supplier<Double> kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI.get();
    }

    public void setkI(Supplier<Double> kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD.get();
    }

    public void setkD(Supplier<Double> kD) {
        this.kD = kD;
    }

    public double getkF() {
        return kF.get();
    }

    public void setkF(Supplier<Double> kF) {
        this.kF = kF;
    }

    public double getTolerance() {
        return tolerance.get();
    }

    public void setTolerance(Supplier<Double> tolerance) {
        this.tolerance = tolerance;
    }

    public double getWaitTime() {
        return waitTime.get();
    }

    public void setWaitTime(Supplier<Double> waitTime) {
        this.waitTime = waitTime;
    }
}
