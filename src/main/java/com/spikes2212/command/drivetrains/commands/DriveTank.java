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
     * {@link TankDrivetrain} according to speed values from Double {@link Supplier}s for left and right sides.<br>
     * Positive values move the drivetrain forward.
     *
     * @param drivetrain         the drivetrain this command requires and moves.
     * @param leftSpeedSupplier  the double {@link Supplier} supplying the speed to move the left side with,
     *                           ranging from -1 to 1.
     * @param rightSpeedSupplier the double {@link Supplier} supplying the speed to move the right side with,
     *                           ranging from -1 to 1.
     * @param squareInputs       whether to square the speeds or not.
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

    public DriveTank(TankDrivetrain drivetrain, Supplier<Double> leftSpeedSupplier,
                     Supplier<Double> rightSpeedSupplier, Supplier<Boolean> isFinished) {
        this(drivetrain, leftSpeedSupplier, rightSpeedSupplier, isFinished, false);
    }

    public DriveTank(TankDrivetrain drivetrain, Supplier<Double> leftSpeedSupplier,
                     Supplier<Double> rightSpeedSupplier, boolean squareInputs) {
        this(drivetrain, leftSpeedSupplier, rightSpeedSupplier, () -> false, squareInputs);
    }

    public DriveTank(TankDrivetrain drivetrain, Supplier<Double> leftSpeedSupplier,
                     Supplier<Double> rightSpeedSupplier) {
        this(drivetrain, leftSpeedSupplier, rightSpeedSupplier, () -> false, false);
    }

    public DriveTank(TankDrivetrain drivetrain, double leftSpeedSupplier,
                     double rightSpeedSupplier, Supplier<Boolean> isFinished, boolean squareInputs) {
        this(drivetrain, () -> leftSpeedSupplier, () -> rightSpeedSupplier, isFinished, squareInputs);
    }

    public DriveTank(TankDrivetrain drivetrain, double leftSpeedSupplier,
                     double rightSpeedSupplier, Supplier<Boolean> isFinished) {
        this(drivetrain, leftSpeedSupplier, rightSpeedSupplier, isFinished, false);
    }

    public DriveTank(TankDrivetrain drivetrain, double leftSpeedSupplier,
                     double rightSpeedSupplier, boolean squareInputs) {
        this(drivetrain, leftSpeedSupplier, rightSpeedSupplier, () -> false, squareInputs);
    }

    public DriveTank(TankDrivetrain drivetrain, double leftSpeedSupplier,
                     double rightSpeedSupplier) {
        this(drivetrain, leftSpeedSupplier, rightSpeedSupplier, () -> false, false);
    }
    @Override
    public void execute() {
        tankDrivetrain.tankDrive(leftSpeedSupplier.get(), rightSpeedSupplier.get(), squareInputs);
    }

    @Override
    public boolean isFinished() {
        return this.isFinished.get();
    }

    @Override
    public void end(boolean interrupted) {
        tankDrivetrain.stop();
    }
}
