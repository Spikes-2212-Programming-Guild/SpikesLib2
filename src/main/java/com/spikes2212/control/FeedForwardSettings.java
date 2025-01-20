package com.spikes2212.control;

import java.util.function.Supplier;

/**
 * The constants used in a FeedForward calculation.
 *
 * @author Tal Sitton
 */
public class FeedForwardSettings {

    /**
     * Empty FeedForwardSettings, which effectively make the FeedForwardController do nothing.
     */
    public static final FeedForwardSettings EMPTY_FF_SETTINGS = new FeedForwardSettings(
            FeedForwardController.ControlMode.LINEAR_POSITION);

    /**
     * The applied control mode
     */
    private final FeedForwardController.ControlMode controlMode;

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

    public FeedForwardSettings(Supplier<Double> kS, Supplier<Double> kV, Supplier<Double> kA, Supplier<Double> kG,
                               FeedForwardController.ControlMode controlMode) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
        this.controlMode = controlMode;
    }

    public FeedForwardSettings(Supplier<Double> kS, Supplier<Double> kV, Supplier<Double> kA,
                               FeedForwardController.ControlMode controlMode) {
        this(kS, kV, kA, () -> 0.0, controlMode);
    }

    public FeedForwardSettings(Supplier<Double> kV, Supplier<Double> kA,
                               FeedForwardController.ControlMode controlMode) {
        this(() -> 0.0, kV, kA, () -> 0.0, controlMode);
    }

    public FeedForwardSettings(double kS, double kV, double kA, FeedForwardController.ControlMode controlMode) {
        this(() -> kS, () -> kV, () -> kA, () -> 0.0, controlMode);
    }

    public FeedForwardSettings(double kV, double kA, FeedForwardController.ControlMode controlMode) {
        this(() -> 0.0, () -> kV, () -> kA, () -> 0.0, controlMode);
    }

    public FeedForwardSettings(double kS, double kV, double kA, double kG,
                               FeedForwardController.ControlMode controlMode) {
        this(() -> kS, () -> kV, () -> kA, () -> kG, controlMode);
    }

    public FeedForwardSettings(FeedForwardController.ControlMode controlMode) {
        this(() -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0, controlMode);
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

    public FeedForwardController.ControlMode getControlMode() {
        return controlMode;
    }
}
