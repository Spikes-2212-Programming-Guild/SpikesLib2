package com.spikes2212.control;

public class PIDSettings {
    private double kP;
    private double kI;
    private double kD;
    private double setpoint;
    private double tolerance;
    private double waitTime;

    public PIDSettings(double kP, double kI, double kD, double setpoint) {
        this(kP, kI, kD, setpoint, 0);
    }

    public PIDSettings(double kP, double kI, double kD, double setpoint, double tolerance) {
        this(kP, kI, kD, setpoint, tolerance, 0);
    }

    public PIDSettings(double kP, double kI, double kD, double setpoint, double tolerance, double waitTime) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.setpoint = setpoint;
        this.tolerance = tolerance;
        this.waitTime = waitTime;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getTolerance() {
        return tolerance;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double getWaitTime() {
        return waitTime;
    }

    public void setWaitTime(double waitTime) {
        this.waitTime = waitTime;
    }
}
