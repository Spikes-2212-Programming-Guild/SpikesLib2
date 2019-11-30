package com.spikes2212.command.genericsubsystem.commands;

import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command moves a {@link GenericSubsystem} according to a {@link Supplier}
 * or a constant speed until it cannot move any more.
 *
 * @author Omri "Riki" Cohen and Itamar Rivkind
 * @see GenericSubsystem
 */
public class MoveGenericSubsystem implements Command {

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
        this.subsystem = subsystem;
        this.speedSupplier = speedSupplier;
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

    @Override
    public Set<Subsystem> getRequirements() {
        HashSet<Subsystem> requirements = new HashSet<Subsystem>(1);
        requirements.add(subsystem);
        return requirements;
    }
}