package com.spikes2212.control;

import java.util.function.Supplier;

public class PIDSettings {
    private Supplier<Double> kP;
    private Supplier<Double> kI;
    private Supplier<Double> kD;
    private Supplier<Double> tolerance;
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
