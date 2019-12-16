package com.spikes2212.control;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;

import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.concurrent.locks.ReentrantLock;

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
    private Supplier<Double> kP;

    /**
     * The integral component of the loop.
     */
    private Supplier<Double> kI;

    /**
     * The derivative component of the loop.
     */
    private Supplier<Double> kD;

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

    /**
     * A lock that synchronizes the different threads.
     */
    private ReentrantLock lock;

    public RioPIDLoop(Supplier<Double> kp, Supplier<Double> ki, Supplier<Double> kd, Supplier<Double> tolerance,
                      Supplier<Double> setpoint, Supplier<Double> source, Consumer<Double> output, Supplier<Double> waitTime, Frequency frequency) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
        this.tolerance = tolerance;
        this.setpoint = setpoint;
        this.waitTime = waitTime;
        this.frequency = frequency;
        this.source = source;
        this.lastTimeNotOnTarget = Timer.getFPGATimestamp();
        this.output = output;
        notifier = new Notifier(this::periodic);
        lock = new ReentrantLock();
    }

    public RioPIDLoop(Supplier<Double> kp, Supplier<Double> ki, Supplier<Double> kd, Supplier<Double> tolerance,
                      Supplier<Double> setpoint, Supplier<Double> source, Consumer<Double> output, Supplier<Double> waitTime) {
        this(kp, ki, kd, tolerance, setpoint, source, output, waitTime, Frequency.DEFAULT);
    }

    public RioPIDLoop(Supplier<Double> kp, Supplier<Double> ki, Supplier<Double> kd, Supplier<Double> tolerance,
                      Supplier<Double> setpoint, Supplier<Double> source, Consumer<Double> output, Frequency frequency) {
        this(kp, ki, kd, tolerance, setpoint, source, output, () -> 0.0, frequency);
    }

    public RioPIDLoop(Supplier<Double> kp, Supplier<Double> ki, Supplier<Double> kd, Supplier<Double> tolerance,
                      Supplier<Double> setpoint, Supplier<Double> source, Consumer<Double> output) {
        this(kp, ki, kd, tolerance, setpoint, source, output, () -> 0.0, Frequency.DEFAULT);
    }

    public RioPIDLoop(double kp, double ki, double kd, double tolerance,
                      double setpoint, Supplier<Double> source, Consumer<Double> output, double waitTime, Frequency frequency) {
        this(() -> kp, () -> ki, () -> kd, () -> tolerance, () -> setpoint, source, output, () -> waitTime, frequency);
    }

    public RioPIDLoop(double kp, double ki, double kd, double tolerance,
                      double setpoint, Supplier<Double> source, Consumer<Double> output, double waitTime) {
        this(() -> kp, () -> ki, () -> kd, () -> tolerance, () -> setpoint, source, output, () -> waitTime, Frequency.DEFAULT);
    }

    public RioPIDLoop(double kp, double ki, double kd, double tolerance,
                      double setpoint, Supplier<Double> source, Consumer<Double> output, Frequency frequency) {
        this(() -> kp, () -> ki, () -> kd, () -> tolerance, () -> setpoint, source, output, () -> 0.0, frequency);
    }

    public RioPIDLoop(double kp, double ki, double kd, double tolerance,
                      double setpoint, Supplier<Double> source, Consumer<Double> output) {
        this(() -> kp, () -> ki, () -> kd, () -> tolerance, () -> setpoint, source, output, () -> 0.0, Frequency.DEFAULT);
    }

    @Override
    public void enable() {
        controller = new PIDController(kP.get(), kI.get(), kD.get(), frequency.period);
        notifier.startPeriodic(frequency.period);
    }

    private void periodic() {
        lock.lock();
        output.accept(controller.calculate(source.get()));
        lock.unlock();
    }

    @Override
    public void disable() {
        notifier.stop();
    }


    public void enableContinuousInput(double minimumValue, double maximumValue) {
        this.controller.enableContinuousInput(minimumValue, maximumValue);
    }

    public void disableContinuousInput() {
        this.controller.disableContinuousInput();
    }

    @Override
    public void update() {
        lock.lock();
        controller.setSetpoint(setpoint.get());
        lock.unlock();
    }


    @Override
    public boolean onTarget() {
        if (source.get() - setpoint.get() > tolerance.get()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= waitTime.get();
    }
}
