package com.spikes2212.command.genericsubsystem;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

/**
 * This class represents a generic subsystem that moves within or without limits.
 *
 * @author Yuval Levy
 */
public abstract class GenericSubsystem extends SubsystemBase {
    /**
     * The current speed of this subsystem.
     */
    private double currentSpeed = 0;

    /**
     * The maximum speed this subsystem may reach.
     */
    private Supplier<Double> maxSpeed;

    /**
     * The minimum speed this subsystem may reach.
     */
    private Supplier<Double> minSpeed;

    /**
     * Constructs a new instance of GenericSubsystem.
     */
    public GenericSubsystem() {
        this(-1, 1);
    }

    /**
     * Constructs a new instance of GenericSubsystem with the given minSpeed and maxSpeed.
     *
     * @param minSpeed the minimum speed
     * @param maxSpeed the maximum speed
     */
    public GenericSubsystem(double minSpeed, double maxSpeed) {
        this(() -> minSpeed, () -> maxSpeed);
    }

    /**
     * Constructs a new instance of GenericSubsystem with the given minSpeed supplier and maxSpeed supplier.
     *
     * @param minSpeed the minimum speed
     * @param maxSpeed the maximum speed
     */
    public GenericSubsystem(Supplier<Double> minSpeed, Supplier<Double> maxSpeed) {
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
    }

    /**
     * Moves this {@link GenericSubsystem} with the given speed, as long as it is
     * within the limits specified when this {@link GenericSubsystem} was constructed.
     *
     * @param speed the speed to move the subsystem with.
     */
    public final void move(double speed) {
        if(speed < minSpeed.get()) speed = minSpeed.get();
        if(speed > maxSpeed.get()) speed = maxSpeed.get();
        if(canMove(speed)) {
            apply(speed);
            currentSpeed = speed;
        }
    }

    /**
     * Applies a given speed to the subsystem.
     *
     * @param speed the speed
     */
    public abstract void apply(double speed);

    /**
     * Returns whether the subsystem can move safely.
     *
     * @param speed the speed
     * @return whether the subsystem can move safely
     */
    public abstract boolean canMove(double speed);

    /**
     * Stops this subsystem's movement.
     */
    public abstract void stop();

    /**
     * Return the current speed of this {@link GenericSubsystem}.
     *
     * @return the current speed of this {@link GenericSubsystem}.
     */
    public double getSpeed() {
        return currentSpeed;
    }
}
