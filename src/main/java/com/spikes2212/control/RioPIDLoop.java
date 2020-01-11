package com.spikes2212.control;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;

import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A PID loop that runs on the Roborio in a separated thread.
 *
 * @author Yuval Levy
 */

public class RioPIDLoop implements PIDLoop {

    /**
     * An enum that represents the frequency the PID loop runs in.
     */
    public enum Frequency {
        LOW(0.05),
        DEFAULT(0.02),
        HIGH(0.01);

        private double period;

        Frequency(double period) {
            this.period = period;
        }
    }

    /**
     * The notifier creates a separated thread for the PID loop.
     */
    private Notifier notifier;

    /**
     * The PIDController calculates the wanted speed to reach target.
     */
    private PIDController controller;

    /**
     * Contains the setting for the pid loop: Kp, Ki, Kd, tolerance and waitTime.
     */
    private PIDSettings pidSettings;

    /**
     * The setpoint for the loop.
     */
    private double setpoint;

    /**
     * The frequency the PID loop runs in.
     */
    private Frequency frequency;

    /**
     * Returns my current location.
     */
    private Supplier<Double> source;

    /**
     * The last time the subsystem didn't reached target.
     */
    private double lastTimeNotOnTarget;

    /**
     * The output of RioPIDLoop.
     */
    private Consumer<Double> output;


    public RioPIDLoop(PIDSettings pidSettings, double setpoint, Supplier<Double> source, Consumer<Double> output,
                      Frequency frequency, boolean continuous, double minContinuousValue, double maxContinuousValue) {
        this.pidSettings = pidSettings;
        this.setpoint = setpoint;
        this.frequency = frequency;
        this.source = source;
        this.lastTimeNotOnTarget = Timer.getFPGATimestamp();
        this.output = output;
        notifier = new Notifier(this::periodic);
        setContinuousMode(continuous, minContinuousValue, maxContinuousValue);
    }

    public RioPIDLoop(PIDSettings pidSettings, double setpoint, Supplier<Double> source, Consumer<Double> output,
                      Frequency frequency) {
        this(pidSettings, setpoint, source, output, frequency, false, 0.0, 0.0);
    }

    public RioPIDLoop(PIDSettings pidSettings, double setpoint, Supplier<Double> source, Consumer<Double> output) {
        this(pidSettings, setpoint, source, output, Frequency.DEFAULT, false, 0, 0);
    }

    public RioPIDLoop(PIDSettings pidSettings, double setpoint, Supplier<Double> source, Consumer<Double> output,
                      boolean continuous, double minContinuousValue, double maxContinuousValue) {
        this(pidSettings, setpoint, source, output, Frequency.DEFAULT, continuous, minContinuousValue, maxContinuousValue);
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setPidSettings(PIDSettings pidSettings) {
        this.pidSettings = pidSettings;
    }


    public PIDSettings getPidSettings() {
        return pidSettings;
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public Supplier<Double> getSource() {
        return source;
    }

    public Consumer<Double> getOutput() {
        return output;
    }

    @Override
    public void enable() {
        controller = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD(), frequency.period);
        notifier.startPeriodic(frequency.period);
    }

    private void periodic() {
        output.accept(controller.calculate(source.get()));
    }

    @Override
    public void disable() {
        notifier.stop();
    }

    private void setContinuousMode(boolean mode, double maxValue, double minValue) {
        if (mode)
            this.controller.enableContinuousInput(maxValue, minValue);
        else
            this.controller.disableContinuousInput();

    }

    @Override
    public void update() {
        controller.setSetpoint(setpoint);
        controller.setTolerance(pidSettings.getTolerance());
    }

    @Override
    public boolean onTarget() {
        if (!controller.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= pidSettings.getWaitTime();
    }
}
