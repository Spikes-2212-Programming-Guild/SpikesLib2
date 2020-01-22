package com.spikes2212.control;

import java.util.function.Supplier;

public class SVAGSettings {
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

    public SVAGSettings(Supplier<Double> kS, Supplier<Double> kV, Supplier<Double> kA) {
        this(kS, kV, kA, () -> 0.0);
    }

    public SVAGSettings(Supplier<Double> kV, Supplier<Double> kA) {
        this(() -> 0.0, kV, kA, () -> 0.0);
    }

    public SVAGSettings(double kS, double kV, double kA) {
        this(() -> kS, () -> kV, () -> kA, () -> 0.0);
    }

    public SVAGSettings(double kV, double kA) {
        this(() -> 0.0, () -> kV, () -> kA, () -> 0.0);
    }

    public SVAGSettings(double kS, double kV, double kA, double kG) {
        this(() -> kS, () -> kV, () -> kA, () -> kG);
    }

    public SVAGSettings(Supplier<Double> kS, Supplier<Double> kV, Supplier<Double> kA, Supplier<Double> kG) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }

    public Supplier<Double> getkS() {
        return kS;
    }

    public void setkS(Supplier<Double> kS) {
        this.kS = kS;
    }

    public Supplier<Double> getkV() {
        return kV;
    }

    public void setkV(Supplier<Double> kV) {
        this.kV = kV;
    }

    public Supplier<Double> getkA() {
        return kA;
    }

    public void setkA(Supplier<Double> kA) {
        this.kA = kA;
    }

    public Supplier<Double> getkG() {
        return kG;
    }

    public void setkG(Supplier<Double> kG) {
        this.kG = kG;
    }
}
