package com.spikes2212.control;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A PID loop that runs on a separated thread.
 *
 * @author Yuval Levy
 */

public class RioPIDLoop implements PIDLoop {

    /**
     * An enum that represents the frequency the PID loop runs in.
     */
    private enum Frequency {
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
     * The proportional component of the loop.
     */
    private Supplier<Double> kp;

    /**
     * The integral component of the loop.
     */
    private Supplier<Double> ki;

    /**
     * The derivative component of the loop.
     */
    private Supplier<Double> kd;

    /**
     * The acceptable distance from target.
     */
    private Supplier<Double> tolerance;

    /**
     * The setpoint for the loop.
     */
    private Supplier<Double> setpoint;

    /**
     * The time required to stay on target.
     */
    private Supplier<Double> waitTime;

    /**
     * The frequency the PID loop runs in.
     */
    private Frequency frequency;

    /**
     * Returns my current location
     */
    private Supplier<Double> PIDSource;

    /**
     * The calculation returned from the pid.
     */
    private Supplier<Double> PIDCalculations;

    /**
     * The last time the subsystem didn't reached target
     */
    private double lastTimeNotOnTarget;

    /**
     * The output of RioPIDLoop
     */
    private Consumer<Double> speedConsumer;

    public RioPIDLoop(Supplier<Double> kp, Supplier<Double> ki, Supplier<Double> kd, Supplier<Double> tolerance,
                      Supplier<Double> setpoint, Supplier<Double> PIDSource, Consumer<Double> speedConsumer, Supplier<Double> waitTime, Frequency frequency) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.tolerance = tolerance;
        this.setpoint = setpoint;
        this.waitTime = waitTime;
        this.frequency = frequency;
        this.PIDSource = PIDSource;
        this.lastTimeNotOnTarget = Timer.getFPGATimestamp();
        this.speedConsumer = speedConsumer;
    }

    public RioPIDLoop(Supplier<Double> kp, Supplier<Double> ki, Supplier<Double> kd, Supplier<Double> tolerance,
                      Supplier<Double> setpoint, Supplier<Double> PIDSource, Consumer<Double> speedConsumer, Supplier<Double> waitTime) {
        this(kp, ki, kd, tolerance, setpoint, PIDSource, speedConsumer, waitTime, Frequency.DEFAULT);
    }

    public RioPIDLoop(Supplier<Double> kp, Supplier<Double> ki, Supplier<Double> kd, Supplier<Double> tolerance,
                      Supplier<Double> setpoint, Supplier<Double> PIDSource, Consumer<Double> speedConsumer, Frequency frequency) {
        this(kp, ki, kd, tolerance, setpoint, PIDSource, speedConsumer, () -> 0.0, frequency);
    }

    public RioPIDLoop(Supplier<Double> kp, Supplier<Double> ki, Supplier<Double> kd, Supplier<Double> tolerance,
                      Supplier<Double> setpoint, Supplier<Double> PIDSource, Consumer<Double> speedConsumer) {
        this(kp, ki, kd, tolerance, setpoint, PIDSource, speedConsumer, () -> 0.0, Frequency.DEFAULT);
    }

    public RioPIDLoop(double kp, double ki, double kd, double tolerance,
                      double setpoint, Supplier<Double> PIDSource, Consumer<Double> speedConsumer, double waitTime, Frequency frequency) {
        this(() -> kp, () -> ki, () -> kd, () -> tolerance, () -> setpoint, PIDSource, speedConsumer, () -> waitTime, frequency);
    }

    public RioPIDLoop(double kp, double ki, double kd, double tolerance,
                      double setpoint, Supplier<Double> PIDSource, Consumer<Double> speedConsumer, double waitTime) {
        this(() -> kp, () -> ki, () -> kd, () -> tolerance, () -> setpoint, PIDSource, speedConsumer, () -> waitTime, Frequency.DEFAULT);
    }

    public RioPIDLoop(double kp, double ki, double kd, double tolerance,
                      double setpoint, Supplier<Double> PIDSource, Consumer<Double> speedConsumer, Frequency frequency) {
        this(() -> kp, () -> ki, () -> kd, () -> tolerance, () -> setpoint, PIDSource, speedConsumer, () -> 0.0, frequency);
    }

    public RioPIDLoop(double kp, double ki, double kd, double tolerance,
                      double setpoint, Supplier<Double> PIDSource, Consumer<Double> speedConsumer) {
        this(() -> kp, () -> ki, () -> kd, () -> tolerance, () -> setpoint, PIDSource, speedConsumer, () -> 0.0, Frequency.DEFAULT);
    }

    @Override
    public void enable() {
        controller = new PIDController(kp.get(), ki.get(), kd.get(), frequency.period);
        notifier = new Notifier(this::periodic);
        notifier.startPeriodic(frequency.period);
    }

    private void periodic() {
        PIDCalculations = () -> controller.calculate(PIDSource.get(), setpoint.get());
    }

    @Override
    public void disable() {
        notifier.stop();
        notifier.close();
    }


    public void enableContinuousInput(double minimumValue, double maximumValue) {
        this.controller.enableContinuousInput(minimumValue, maximumValue);
    }

    @Override
    public void update() {
        speedConsumer.accept(PIDCalculations.get());
    }

    @Override
    public boolean onTarget() {
        if (PIDSource.get() - setpoint.get() > tolerance.get()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= waitTime.get();
    }
}
