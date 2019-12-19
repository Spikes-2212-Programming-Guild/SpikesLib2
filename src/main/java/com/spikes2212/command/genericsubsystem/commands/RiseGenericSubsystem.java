package com.spikes2212.command.genericsubsystem.commands;

import java.util.function.Supplier;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;

import edu.wpi.first.wpilibj.Timer;

/**
 * This command rise the speed of a {@link GenericSubsystem} linearly so it will
 * reach a wanted speed after a given time.
 *
 * @author Yuval Levy
 */
public class RiseGenericSubsystem extends MoveGenericSubsystem {
    protected final double time;
    private double acceleration;
    private double currentSpeed;
    private double startTime;
    private boolean finishWhenReachingSpeed;

    /**
     * This constructs a new {@link RiseGenericSubsystem} command using the
     * {@link GenericSubsystem} this command operates on and a supplier supplying the
     * wanted speed the {@link GenericSubsystem} should move with after the given
     * time.
     *
     * @param subsystem   the {@link GenericSubsystem} this command should move.
     * @param wantedSpeed the speed the subsystem should move after the time.
     * @param time        the time it takes for the subsystem to get to the speed.
     */
    public RiseGenericSubsystem(GenericSubsystem subsystem, Supplier<Double> wantedSpeed, double time, boolean finishWhenReachingSpeed) {
        super(subsystem, wantedSpeed);
        if (time <= 1) {
            time = 1;
        }
        this.time = time;
        this.finishWhenReachingSpeed = finishWhenReachingSpeed;
    }

    /**
     * This constructs a new {@link RiseGenericSubsystem} command using the
     * {@link GenericSubsystem} this command operates on and a supplier supplying the
     * wanted speed the {@link GenericSubsystem} should move with after the given
     * time.
     *
     * @param subsystem   the {@link GenericSubsystem} this command should move.
     * @param wantedSpeed the speed the subsystem should move after the time.
     * @param time        the time it takes for the subsystem to get to the speed.
     */
    public RiseGenericSubsystem(GenericSubsystem subsystem, double wantedSpeed, double time, boolean finishWhenReachingSpeed) {
        this(subsystem, () -> wantedSpeed, time, finishWhenReachingSpeed);
    }

    /**
     * Reset the timer.
     */
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        currentSpeed = 0;
        acceleration = speedSupplier.get() / time;
    }

    /**
     * Calculate the speed and moves the subsystem.
     */
    @Override
    public void execute() {
        currentSpeed = (Timer.getFPGATimestamp() - startTime) * acceleration;
        if (Math.abs(currentSpeed) > Math.abs(speedSupplier.get()))
            currentSpeed = speedSupplier.get();
        subsystem.move(currentSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || (finishWhenReachingSpeed && currentSpeed == speedSupplier.get());
    }

}