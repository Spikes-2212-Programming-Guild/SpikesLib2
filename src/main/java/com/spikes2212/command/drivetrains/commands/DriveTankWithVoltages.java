package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/**
 * A command that moves a {@link TankDrivetrain} using voltage supplied to the left and the right sides independently.
 *
 * @author Yorai Coval
 * @see TankDrivetrain
 */
public class DriveTankWithVoltages extends Command {

    protected final Supplier<Double> leftVoltageSupplier;
    protected final Supplier<Double> rightVoltageSupplier;
    protected final TankDrivetrain drivetrain;
    protected final Supplier<Boolean> isFinished;

    /**
     * This constructs a new {@link DriveTankWithVoltages} command that moves the given {@link TankDrivetrain} according to voltage
     * values from Double {@link Supplier}s for the left and right sides. <br>
     * Positive values move the drivetrain forward.
     *
     * @param drivetrain   the drivetrain this command requires and moves
     * @param leftVoltageSupplier  the double {@link Supplier} supplying the voltage for the left side (-12 to 12)
     * @param rightVoltageSupplier the double {@link Supplier} supplying the voltage for the right side (-12 to 12)
     * @param isFinished   when to finish the command
     */
    public DriveTankWithVoltages(TankDrivetrain drivetrain, Supplier<Double> leftVoltageSupplier,
                                 Supplier<Double> rightVoltageSupplier, Supplier<Boolean> isFinished) {
        this.drivetrain = drivetrain;
        this.leftVoltageSupplier = leftVoltageSupplier;
        this.rightVoltageSupplier = rightVoltageSupplier;
        this.isFinished = isFinished;
    }

    /**
     * This constructs a new {@link DriveTankWithVoltages} command that moves the given {@link TankDrivetrain} according to voltage
     * values from Double {@link Supplier}s for the left and right sides. <br>
     * Positive values move the drivetrain forward.
     *
     * @param drivetrain   the drivetrain this command requires and moves
     * @param leftVoltageSupplier  the double {@link Supplier} supplying the voltage for the left side (-12 to 12)
     * @param rightVoltageSupplier the double {@link Supplier} supplying the voltage for the right side (-12 to 12)
     */
    public DriveTankWithVoltages(TankDrivetrain drivetrain, Supplier<Double> leftVoltageSupplier,
                                 Supplier<Double> rightVoltageSupplier) {
        this(drivetrain, leftVoltageSupplier, rightVoltageSupplier, () -> false);
    }

    /**
     * This constructs a new {@link DriveTankWithVoltages} command that moves the given {@link TankDrivetrain} according to voltage
     * values for the left and right sides. <br>
     * Positive values move the drivetrain forward.
     *
     * @param drivetrain   the drivetrain this command requires and moves
     * @param leftVoltage  the double {@link Supplier} supplying the voltage for the left side (-12 to 12)
     * @param rightVoltage the double {@link Supplier} supplying the voltage for the right side (-12 to 12)
     * @param isFinished   when to finish the command
     */
    public DriveTankWithVoltages(TankDrivetrain drivetrain, double leftVoltage, double rightVoltage,
                                 Supplier<Boolean> isFinished) {
        this(drivetrain, () -> leftVoltage, () -> rightVoltage, isFinished);
    }

    /**
     * This constructs a new {@link DriveTankWithVoltages} command that moves the given {@link TankDrivetrain} according to voltage
     * values for the left and right sides. <br>
     * Positive values move the drivetrain forward.
     *
     * @param drivetrain   the drivetrain this command requires and moves
     * @param leftVoltage  the double {@link Supplier} supplying the voltage for the left side (-12 to 12)
     * @param rightVoltage the double {@link Supplier} supplying the voltage for the right side (-12 to 12)
     */
    public DriveTankWithVoltages(TankDrivetrain drivetrain, double leftVoltage, double rightVoltage) {
        this(drivetrain, () -> leftVoltage, () -> rightVoltage, () -> false);
    }

    @Override
    public void execute() {
        drivetrain.tankDriveVoltages(leftVoltageSupplier.get(), rightVoltageSupplier.get());
    }

    @Override
    public boolean isFinished() {
        return isFinished.get();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
