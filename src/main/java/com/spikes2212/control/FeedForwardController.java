package com.spikes2212.control;

/**
 * This class represents a FeedForwardController for use with the path package (mostly).
 */
public class FeedForwardController {

    /**
     * The velocity constant
     */
    private double kV;

    /**
     * The acceleration constant
     */
    private double kA;

    /**
     * The feedback constant
     */
    private double kB;

    private double previousTarget;

    public FeedForwardController(double kV, double kA, double kB) {
        this(kV, kA, kB, 0);
    }

    public FeedForwardController(double kV, double kA, double kB, double initialTarget) {
        this.kV = kV;
        this.kA = kA;
        this.kB = kB;
        this.previousTarget = initialTarget;
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

    public double getkB() {
        return kB;
    }

    public void setkB(double kB) {
        this.kB = kB;
    }

    /**
     * Calculates the desired output using a simple feed forward-feed back.
     * This method should be called frequently or it will be very inaccurate.
     * @param target the target velocity
     * @param measurement the measured velocity
     * @return the desired output
     */
    public double calculate(double target, double measurement) {
        double targetDerivative = target - previousTarget;
        previousTarget = target;
        return kV * target + kA * targetDerivative + kB * (target - measurement);
    }
}
