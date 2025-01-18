package com.spikes2212.control;

import java.util.function.Supplier;

/**
 * The constants used in a PID calculation, as well as tolerance and loop wait time.
 *
 * @author Eran Goldshtein
 */
public class PIDSettings {

    public static final PIDSettings EMPTY_PID_SETTINGS = new PIDSettings(0, 0, 0, 0, 0,
            0);

    /**
     * the proportional component of the PID settings
     */
    private final Supplier<Double> kP;

    /**
     * the integral component of the PID settings
     */
    private final Supplier<Double> kI;

    /**
     * the derivative component of the PID settings
     */
    private final Supplier<Double> kD;

    private final Supplier<Double> IZone;

    /**
     * the acceptable distance from the target
     */
    private final Supplier<Double> tolerance;

    /**
     * the time required to stay on target
     */
    private final Supplier<Double> waitTime;

    public PIDSettings(Supplier<Double> kP, Supplier<Double> kI, Supplier<Double> kD, Supplier<Double> IZone,
                       Supplier<Double> tolerance, Supplier<Double> waitTime) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.IZone = IZone;
        this.tolerance = tolerance;
        this.waitTime = waitTime;
    }

    public PIDSettings(double kP, double tolerance, double waitTime) {
        this(kP, 0.0, 0.0, tolerance, 0.0, waitTime);
    }

    public PIDSettings(double kP, double kI, double kD, double IZone, double tolerance, double waitTime) {
        this(() -> kP, () -> kI, () -> kD, () -> IZone, () -> tolerance, () -> waitTime);
    }

    public PIDSettings(Supplier<Double> kP, Supplier<Double> tolerance, Supplier<Double> waitTime) {
        this(kP, () -> 0.0, () -> 0.0, () -> 0.0, tolerance, waitTime);
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

    public double getTolerance() {
        return tolerance.get();
    }

    public void setTolerance(Supplier<Double> tolerance) {
        this.tolerance = tolerance;
    }

    public double getIZone() {return IZone.get(); }

    public void setIZone(Supplier<Double> IZone) {
        this.IZone = IZone;
    }

    public double getWaitTime() {
        return waitTime.get();
    }

    public void setWaitTime(Supplier<Double> waitTime) {
        this.waitTime = waitTime;
    }
}
