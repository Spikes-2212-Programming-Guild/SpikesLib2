package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * A command that moves a {@link TankDrivetrain} using speeds supplied to the left and the right sides independently.
 *
 * @author Yuval Levy
 * @see TankDrivetrain
 */
public class DriveTank extends CommandBase {

    protected final TankDrivetrain tankDrivetrain;
    protected final Supplier<Double> leftSpeedSupplier;
    protected final Supplier<Double> rightSpeedSupplier;
    protected final Supplier<Boolean> isFinished;
    protected final boolean squareInputs;

    /**
     * This constructs a new {@link DriveTank} command that moves the given
     * {@link TankDrivetrain} according to speed values from Double {@link Supplier}s for left and right sides. <br>
     * Positive values move the drivetrain forward.
     *
     * @param drivetrain         the tank drivetrain this command operates on
     * @param leftSpeedSupplier  the double {@link Supplier} supplying the left side's speed (-1 to 1).
     *                           Positive values go forwards
     * @param rightSpeedSupplier the double {@link Supplier} supplying the right side's speed (-1 to 1).
     *                           Positive values go forwards
     * @param isFinished         when to finish the command
     * @param squareInputs       whether to square the speed suppliers' values
     */
    public DriveTank(TankDrivetrain drivetrain, Supplier<Double> leftSpeedSupplier,
                     Supplier<Double> rightSpeedSupplier, Supplier<Boolean> isFinished, boolean squareInputs) {
        addRequirements(drivetrain);
        this.tankDrivetrain = drivetrain;
        this.leftSpeedSupplier = leftSpeedSupplier;
        this.rightSpeedSupplier = rightSpeedSupplier;
        this.isFinished = isFinished;
        this.squareInputs = squareInputs;
    }

    /**
     * This constructs a new {@link DriveTank} command that moves the given
     * {@link TankDrivetrain} according to speed values from Double {@link Supplier}s for left and right sides. <br>
     * Positive values move the drivetrain forward. Does not square the inputs.
     *
     * @param drivetrain         the tank drivetrain this command operates on
     * @param leftSpeedSupplier  the double {@link Supplier} supplying the left side's speed (-1 to 1).
     *                           Positive values go forwards
     * @param rightSpeedSupplier the double {@link Supplier} supplying the right side's speed (-1 to 1).
     *                           Positive values go forwards
     * @param isFinished         when to finish the command
     */
    public DriveTank(TankDrivetrain drivetrain, Supplier<Double> leftSpeedSupplier,
                     Supplier<Double> rightSpeedSupplier, Supplier<Boolean> isFinished) {
        this(drivetrain, leftSpeedSupplier, rightSpeedSupplier, isFinished, false);
    }

    /**
     * This constructs a new {@link DriveTank} command that moves the given
     * {@link TankDrivetrain} according to speed values from Double {@link Supplier}s for left and right sides. <br>
     * Positive values move the drivetrain forward.
     *
     * @param drivetrain         the tank drivetrain this command operates on
     * @param leftSpeedSupplier  the double {@link Supplier} supplying the left side's speed (-1 to 1).
     *                           Positive values go forwards
     * @param rightSpeedSupplier the double {@link Supplier} supplying the right side's speed (-1 to 1).
     *                           Positive values go forwards
     * @param squareInputs       whether to square the speed suppliers' values
     */
    public DriveTank(TankDrivetrain drivetrain, Supplier<Double> leftSpeedSupplier,
                     Supplier<Double> rightSpeedSupplier, boolean squareInputs) {
        this(drivetrain, leftSpeedSupplier, rightSpeedSupplier, () -> false, squareInputs);
    }

    /**
     * This constructs a new {@link DriveTank} command that moves the given
     * {@link TankDrivetrain} according to speed values from Double {@link Supplier}s for left and right sides. <br>
     * Positive values move the drivetrain forward. Does not square the inputs.
     *
     * @param drivetrain         the tank drivetrain this command operates on
     * @param leftSpeedSupplier  the double {@link Supplier} supplying the left side's speed (-1 to 1).
     *                           Positive values go forwards
     * @param rightSpeedSupplier the double {@link Supplier} supplying the right side's speed (-1 to 1).
     *                           Positive values go forwards
     */
    public DriveTank(TankDrivetrain drivetrain, Supplier<Double> leftSpeedSupplier,
                     Supplier<Double> rightSpeedSupplier) {
        this(drivetrain, leftSpeedSupplier, rightSpeedSupplier, () -> false, false);
    }

    /**
     * This constructs a new {@link DriveTank} command that moves the given
     * {@link TankDrivetrain} according to speed values for left and right sides. <br>
     * Positive values move the drivetrain forward.
     *
     * @param drivetrain   the tank drivetrain this command operates on
     * @param leftSpeed    the left side's speed (-1 to 1). Positive values go forwards
     * @param rightSpeed   the right side's speed (-1 to 1). Positive values go forwards
     * @param isFinished   when to finish the command
     * @param squareInputs whether to square the speed values
     */
    public DriveTank(TankDrivetrain drivetrain, double leftSpeed,
                     double rightSpeed, Supplier<Boolean> isFinished, boolean squareInputs) {
        this(drivetrain, () -> leftSpeed, () -> rightSpeed, isFinished, squareInputs);
    }

    /**
     * This constructs a new {@link DriveTank} command that moves the given
     * {@link TankDrivetrain} according to speed values for left and right sides. <br>
     * Positive values move the drivetrain forward. Does not square the inputs.
     *
     * @param drivetrain   the tank drivetrain this command operates on
     * @param leftSpeed    the left side's speed (-1 to 1). Positive values go forwards
     * @param rightSpeed   the right side's speed (-1 to 1). Positive values go forwards
     * @param isFinished   when to finish the command
     */
    public DriveTank(TankDrivetrain drivetrain, double leftSpeed, double rightSpeed, Supplier<Boolean> isFinished) {
        this(drivetrain, leftSpeed, rightSpeed, isFinished, false);
    }

    /**
     * This constructs a new {@link DriveTank} command that moves the given
     * {@link TankDrivetrain} according to speed values for left and right sides. <br>
     * Positive values move the drivetrain forward.
     *
     * @param drivetrain   the tank drivetrain this command operates on
     * @param leftSpeed    the left side's speed (-1 to 1). Positive values go forwards
     * @param rightSpeed   the right side's speed (-1 to 1). Positive values go forwards
     * @param squareInputs whether to square the speed values
     */
    public DriveTank(TankDrivetrain drivetrain, double leftSpeed, double rightSpeed, boolean squareInputs) {
        this(drivetrain, leftSpeed, rightSpeed, () -> false, squareInputs);
    }

    /**
     * This constructs a new {@link DriveTank} command that moves the given
     * {@link TankDrivetrain} according to speed values for left and right sides. <br>
     * Positive values move the drivetrain forward. Does not square the inputs.
     *
     * @param drivetrain   the tank drivetrain this command operates on
     * @param leftSpeed    the left side's speed (-1 to 1). Positive values go forwards
     * @param rightSpeed   the right side's speed (-1 to 1). Positive values go forwards
     */
    public DriveTank(TankDrivetrain drivetrain, double leftSpeed, double rightSpeed) {
        this(drivetrain, leftSpeed, rightSpeed, () -> false, false);
    }

    @Override
    public void execute() {
        tankDrivetrain.tankDrive(leftSpeedSupplier.get(), rightSpeedSupplier.get(), squareInputs);
    }

    @Override
    public boolean isFinished() {
        return isFinished.get();
    }

    @Override
    public void end(boolean interrupted) {
        tankDrivetrain.stop();
    }
}
