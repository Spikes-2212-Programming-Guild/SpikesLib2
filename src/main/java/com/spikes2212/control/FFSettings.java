package com.spikes2212.control;

import java.util.function.Supplier;

/**
 * The SVAG constants used in a feed forward control loop.
 *
 * @author Tal Sitton
 */
public class FFSettings {
    /**
     * The static constant
     */
    private Supplier<Double> kS;

    /**
     * The velocity constant
     */
    private Supplier<Double> kV;

    /**
     * The acceleration constant
     */
    private Supplier<Double> kA;

    /**
     * The gravity constant
     */
    private Supplier<Double> kG;

    public FFSettings(Supplier<Double> kS, Supplier<Double> kV, Supplier<Double> kA) {
        this(kS, kV, kA, () -> 0.0);
    }

    public FFSettings(Supplier<Double> kV, Supplier<Double> kA) {
        this(() -> 0.0, kV, kA, () -> 0.0);
    }

    public FFSettings(double kS, double kV, double kA) {
        this(() -> kS, () -> kV, () -> kA, () -> 0.0);
    }

    public FFSettings(double kV, double kA) {
        this(() -> 0.0, () -> kV, () -> kA, () -> 0.0);
    }

    public FFSettings(double kS, double kV, double kA, double kG) {
        this(() -> kS, () -> kV, () -> kA, () -> kG);
    }

    public FFSettings(Supplier<Double> kS, Supplier<Double> kV, Supplier<Double> kA, Supplier<Double> kG) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }

    public double getkS() {
        return kS.get();
    }

    public void setkS(Supplier<Double> kS) {
        this.kS = kS;
    }

    public double getkV() {
        return kV.get();
    }

    public void setkV(Supplier<Double> kV) {
        this.kV = kV;
    }

    public double getkA() {
        return kA.get();
    }

    public void setkA(Supplier<Double> kA) {
        this.kA = kA;
    }

    public double getkG() {
        return kG.get();
    }

    public void setkG(Supplier<Double> kG) {
        this.kG = kG;
    }
}
