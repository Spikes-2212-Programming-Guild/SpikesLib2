package com.spikes2212.control;

/**
 * @author Tuval Rivkinind Barlev
 */
public class FeedForwardController {

    public static final double DEFAULT_PERIOD = 0.02;

    /**
     * The static constant
     */
    private double kS;

    /**
     * The velocity constant
     */
    private double kV;

    /**
     * The acceleration constant
     */
    private double kA;

    /**
     * The gravity constant
     */
    private double kG;

    /**
     * The previous target used for the derivative.
     */
    private double previousTarget;

    /**
     * The calling period for the calculate function.
     */
    private final double period;

    public FeedForwardController(double kS, double kV, double kA, double kG, double period) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
        this.period = period;
        this.previousTarget = 0;
    }

    public FeedForwardController(double kV, double kA, double period) {
        this(0, kV, kA, 0, period);
    }

    public FeedForwardController(double kS, double kV, double kA, double period) {
        this(kS, kV, kA, 0, period);
    }

    public FeedForwardController(FeedForwardSettings settings, double period) {
        this(settings.getkS(), settings.getkV(), settings.getkA(), settings.getkG(), period);
    }

    public void setGains(double kV, double kA) {
        this.kV = kV;
        this.kA = kA;
    }

    public void setGains(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public void setGains(double kS, double kV, double kA, double kG) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }

    public double getkS() {
        return kS;
    }

    public void setkS(double kS) {
        this.kS = kS;
    }

    public double getkV() {
        return kV;
    }

    public void setkV(double kV) {
        this.kV = kV;
    }

    public double getkA() {
        return kA;
    }

    public void setkA(double kA) {
        this.kA = kA;
    }

    public double getkG() {
        return kG;
    }

    public void setkG(double kG) {
        this.kG = kG;
    }

    public double getPeriod() {
        return period;
    }

    public void reset() {
        this.previousTarget = 0;
    }

    /**
     * Calculates the desired output using a simple feed forward method.
     * This method should be called with the period given in the constructor.
     *
     * @param setpoint the target velocity
     * @return the desired output
     */
    public double calculate(double setpoint) {
        double targetDerivative = (setpoint - previousTarget) / period;
        previousTarget = setpoint;
        return kG + kS * Math.signum(setpoint) + kV * setpoint + kA * targetDerivative;
    }
}
