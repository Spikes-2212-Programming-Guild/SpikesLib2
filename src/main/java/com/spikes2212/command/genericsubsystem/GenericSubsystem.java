package com.spikes2212.command.genericsubsystem;


import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

/**
 * This class represents a generic subsystem that moves within a limitation, or without one.
 *
 * @author Yuval Levy
 */
public abstract class GenericSubsystem extends DashboardedSubsystem {

    private double currentSpeed = 0;
    protected Supplier<Double> maxSpeed;
    protected Supplier<Double> minSpeed;

    /**
     * Constructs a new instance of GenericSubsystem.
     *
     * @param namespaceName the name of the subsystem's namespace
     */
    public GenericSubsystem(String namespaceName) {
        this(namespaceName, -1, 1);
    }

    /**
     * Constructs a new instance of GenericSubsystem. <br>
     *
     * This constructor is deprecated. <br>
     * Please use {@link #GenericSubsystem(String namespaceName)}.
     */

    @Deprecated(since = "2022", forRemoval = true)
    public GenericSubsystem() {
        this("generic subsystem");
    }

    /**
     * Constructs a new instance of GenericSubsystem with the given minSpeed and maxSpeed.
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param minSpeed the minimum speed
     * @param maxSpeed the maximum speed
     */
    public GenericSubsystem(String namespaceName, double minSpeed, double maxSpeed) {
        this(namespaceName, () -> minSpeed, () -> maxSpeed);
    }

    /**
     * Constructs a new instance of GenericSubsystem with the given minSpeed and maxSpeed. <br><br>
     *
     * This constructor is deprecated. <br>
     * Please use {@link #GenericSubsystem(String namespaceName, double minSpeed, double maxSpeed)} instead. <br><br>
     *
     * @param minSpeed the minimum speed
     * @param maxSpeed the maximum speed
     *
     */
    @Deprecated(since = "2022", forRemoval = true)
    public GenericSubsystem(double minSpeed, double maxSpeed) {
        this("generic subsystem", minSpeed, maxSpeed);
    }

    /**
     * Constructs a new instance of GenericSubsystem with the given minSpeed supplier and maxSpeed supplier.
     *
     * @param minSpeed the minimum speed
     * @param maxSpeed the maximum speed
     */
    public GenericSubsystem(String namespaceName, Supplier<Double> minSpeed, Supplier<Double> maxSpeed) {
        super(namespaceName);
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
    }

    /**
     * Constructs a new instance of GenericSubsystem with the given minSpeed supplier and maxSpeed supplier. <br>
     *
     * This constructor is deprecated. <br>
     * Please use {@link #GenericSubsystem(String namespaceName, double minSpeed, double maxSpeed)} instead. <br><br>
     *
     * @param minSpeed the minimum speed
     * @param maxSpeed the maximum speed
     */
    @Deprecated(since = "2022", forRemoval = true)
    public GenericSubsystem(Supplier<Double> minSpeed, Supplier<Double> maxSpeed) {
        this("generic subsystem", minSpeed, maxSpeed);
    }

    /**
     * Moves this {@link GenericSubsystem} with the given speed, as long as it is
     * within the limits specified when this {@link GenericSubsystem} was constructed.
     *
     * @param speed the speed to move the subsystem with.
     */
    public final void move(double speed) {
        if (speed < minSpeed.get()) speed = minSpeed.get();
        if (speed > maxSpeed.get()) speed = maxSpeed.get();
        if (canMove(speed)) {
            apply(speed);
            currentSpeed = speed;
        }
    }

    /**
     * This method applies a given speed to the subsystem.
     *
     * @param speed the speed
     */
    protected abstract void apply(double speed);

    /**
     * This method returns whether the subsystem can move safely.
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
