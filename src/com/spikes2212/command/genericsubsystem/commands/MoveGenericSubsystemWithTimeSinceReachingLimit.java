package com.spikes2212.command.genericsubsystem.commands;

import java.util.function.Supplier;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;

import edu.wpi.first.wpilibj.Timer;

/**
 * This command moves a {@link GenericSubsystem} according to a {@link Supplier}
 * or a constant speed until it reaches its limit, and then continue moving for
 * a given number of seconds.
 *
 * @author Omri "Riki" Cohen
 *
 * @see MoveGenericSubsystem
 */
public class MoveGenericSubsystemWithTimeSinceReachingLimit extends MoveGenericSubsystem {

    /**
     * The time the {@link GenericSubsystem} should keep trying to move after reaching
     * its end point.
     */
    protected double waitTime;

    /**
     * The last time the {@link GenericSubsystem} has been on the wanted endState.
     */
    protected double lastTimeAtEndState = 0;

    /**
     * This constructs a new {@link MoveGenericSubsystemWithTimeSinceReachingLimit}
     * command using the {@link GenericSubsystem} this command runs on, a supplier
     * supplying the speed the {@link GenericSubsystem} should move with, and a wait
     * time.
     *
     * @param subsystem
     *            the {@link GenericSubsystem} this command should move.
     * @param speedSupplier
     *            a {@link Double} {@link Supplier} supplying the speed this
     *            subsystem should be moved with. Must only supply values between -1
     *            and 1.
     * @param waitTime
     *            the time the {@link GenericSubsystem} should keep trying to move
     *            after reaching its end point.
     */
    public MoveGenericSubsystemWithTimeSinceReachingLimit(GenericSubsystem subsystem, Supplier<Double> speedSupplier,
                                                          double waitTime) {
        super(subsystem, speedSupplier);
        this.waitTime = waitTime;
    }

    /**
     * This constructs a new {@link MoveGenericSubsystemWithTimeSinceReachingLimit}
     * command using the {@link GenericSubsystem} this command runs on, a constant
     * speed the {@link GenericSubsystem} should move with, and a wait time.
     *
     * @param subsystem
     *            the {@link GenericSubsystem} this command should move.
     * @param speed
     *            a speed this subsystem should be moved with. Must only be a value
     *            between -1 and 1.
     * @param waitTime
     *            the time the {@link GenericSubsystem} should keep trying to move
     *            after reaching its end point.
     */
    public MoveGenericSubsystemWithTimeSinceReachingLimit(GenericSubsystem subsystem, double speed, double waitTime) {
        super(subsystem, speed);
        this.waitTime = waitTime;
    }

    /**
     * Checks if the subsystem has reached the limits given, then waits for the
     * given amount time.
     */
    @Override
    public boolean isFinished() {
        double currentTime = Timer.getFPGATimestamp();
        if (!super.isFinished()) { /*
         * if not in the ending position reset the timer.
         */
            lastTimeAtEndState = currentTime;
        }
        if (currentTime - lastTimeAtEndState > waitTime) { /*
         * if the subsystem was on limit the wanted time the command
         * is finished.
         */
            return true;
        }
        return false;
    }

}