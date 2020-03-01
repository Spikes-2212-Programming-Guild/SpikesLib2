package com.spikes2212.command.genericsubsystem.commands;


import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;


/**
 * This command moves a {@link GenericSubsystem} according to a {@link com.spikes2212.control.PIDLoop}
 * until it reaches its target or until it can't move anymore .
 *
 * @author Yuval Levy
 * @see GenericSubsystem
 */
public class MoveGenericSubsystem extends CommandBase {

    protected final GenericSubsystem subsystem;
    protected final Supplier<Double> speedSupplier;

    /**
     * This constructs a new {@link MoveGenericSubsystem} command using the
     * {@link GenericSubsystem} this command operates on and a supplier supplying the
     * speed the {@link GenericSubsystem} should move with.
     *
     * @param subsystem     the {@link GenericSubsystem} this command should move.
     * @param speedSupplier a Double {@link Supplier} supplying the speed this subsystem
     *                      should be moved with. Must only supply values between -1 and 1.
     */
    public MoveGenericSubsystem(GenericSubsystem subsystem, Supplier<Double> speedSupplier) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.speedSupplier = speedSupplier;
    }

    public MoveGenericSubsystem(GenericSubsystem subsystem, double speedSupplier) {
        this(subsystem, () -> speedSupplier);
    }

    /**
     * moves the subsystem at the given speed
     */
    @Override
    public void execute() {
        subsystem.move(speedSupplier.get());
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return !subsystem.canMove(speedSupplier.get());
    }

}