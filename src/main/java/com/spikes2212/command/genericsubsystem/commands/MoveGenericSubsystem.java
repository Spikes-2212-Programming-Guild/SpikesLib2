package com.spikes2212.command.genericsubsystem.commands;


import java.util.function.Supplier;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * This command moves a {@link GenericSubsystem} according to a {@link Supplier}
 * or a constant speed until it cannot move any more.
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
        super();
        this.subsystem = subsystem;
        this.speedSupplier = speedSupplier;
        this.addRequirements(subsystem);
    }

    public MoveGenericSubsystem(GenericSubsystem subsystem, double speedSupplier) {
        this(subsystem, () -> speedSupplier);
    }


    @Override
    public void initialize() {

    }

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