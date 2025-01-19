package com.spikes2212.control;

/**
 * @author Tuval Rivkinind Barlev
 */
public class FeedForwardController {

    public enum ControlMode {
        LINEAR_POSITION, LINEAR_VELOCITY, ANGULAR_POSITION, ANGULAR_VELOCITY;
    }

    /**
     * The type of feed forward control to use
     */
    private final ControlMode controlMode;

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

    public FeedForwardController(double kS, double kV, double kA, double kG, ControlMode controlMode) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
        this.controlMode = controlMode;
    }

    public FeedForwardController(double kV, double kA, ControlMode controlMode) {
        this(0, kV, kA, 0, controlMode);
    }

    public FeedForwardController(double kS, double kV, double kA, ControlMode controlMode) {
        this(kS, kV, kA, 0, controlMode);
    }

    public FeedForwardController(FeedForwardSettings settings, ControlMode controlMode) {
        this(settings.getkS(), settings.getkV(), settings.getkA(), settings.getkG(), controlMode);
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

    public void setGains(FeedForwardSettings feedForwardSettings) {
        setGains(feedForwardSettings.getkS(), feedForwardSettings.getkV(), feedForwardSettings.getkA(),
                feedForwardSettings.getkG());
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

    public ControlMode getControlMode() {
        return controlMode;
    }

    /**
     * Calculates the desired output using a simple feed forward method.
     *
     * @param source       the current state
     * @param setpoint     the desired state
     * @param acceleration the desired acceleration
     * @return the desired output
     */
    public double calculate(double source, double setpoint, double acceleration) {
        double kSValue = 0;
        double kGValue = 0;
        switch (controlMode) {
            case LINEAR_POSITION -> {
                kSValue = kS * Math.signum(setpoint - source);
                kGValue = kG;
            }
            case LINEAR_VELOCITY -> {
                kSValue = kS * Math.signum(setpoint);
                kGValue = kG;
            }
            case ANGULAR_POSITION -> {
                kSValue = kS * Math.signum(setpoint - source);
                kGValue = kG * Math.cos(source);
            }
            case ANGULAR_VELOCITY -> {
                kSValue = kS * Math.signum(setpoint);
                kGValue = kG * Math.cos(source);
            }
        }
        return kSValue + kV * setpoint + kA * acceleration + kGValue;
    }

    public double calculate(double source, double setpoint) {
        return calculate(source, setpoint, 0);
    }
}
