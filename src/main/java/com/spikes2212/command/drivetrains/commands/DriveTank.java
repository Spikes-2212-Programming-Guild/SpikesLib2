package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * This command moves a {@link TankDrivetrain} using speeds supplied to the left and the right sides independently.
 */
public class DriveTank extends CommandBase {
    /**
     * The {@link TankDrivetrain} this command will move.
     */
    protected final TankDrivetrain tankDrivetrain;

    /**
     * The speed the left side of the {@link TankDrivetrain} should move at.
     */
    protected final Supplier<Double> leftSpeedSupplier;

    /**
     * The speed the right side of the {@link TankDrivetrain} should move at.
     */
    protected final Supplier<Double> rightSpeedSupplier;

    /**
     * A supplier that returns `true` when the command should finished and `false` otherwise.
     */
    Supplier<Boolean> isFinished;

    /**
     * This constructs a new {@link DriveTank} command that moves the given
     * {@link TankDrivetrain} according to speed values from Double
     * {@link Supplier}s for left and right sides.<br>
     * Positive values move forwards.
     *
     * @param drivetrain         the drivetrain this command requires and moves.
     * @param leftSpeedSupplier  the double {@link Supplier} supplying the speed to move in the
     *                           left side at.
     * @param rightSpeedSupplier the double {@link Supplier} supplying the speed to move in the
     *                           right side at.
     */
    public DriveTank(TankDrivetrain drivetrain, Supplier<Double> leftSpeedSupplier,
                     Supplier<Double> rightSpeedSupplier, Supplier<Boolean> isFinished) {
        addRequirements(drivetrain);
        this.tankDrivetrain = drivetrain;
        this.leftSpeedSupplier = leftSpeedSupplier;
        this.rightSpeedSupplier = rightSpeedSupplier;
        this.isFinished = isFinished;
    }

    public DriveTank(TankDrivetrain drivetrain, Supplier<Double> leftSpeedSupplier,
                     Supplier<Double> rightSpeedSupplier) {
        this(drivetrain, leftSpeedSupplier, rightSpeedSupplier, () -> false);
    }

    public DriveTank(TankDrivetrain drivetrain, double leftSpeed, double rightSpeed, boolean isFinished) {
        this(drivetrain, () -> leftSpeed, () -> rightSpeed, () -> isFinished);
    }

    public DriveTank(TankDrivetrain drivetrain, double leftSpeed, double rightSpeed) {
        this(drivetrain, () -> leftSpeed, () -> rightSpeed, () -> false);
    }


    @Override
    public void initialize() {
    }

    /**
     * sets the values for both the leftSpeedSupplier and the rightSpeedSupplier.
     */
    @Override
    public void execute() {
        tankDrivetrain.tankDrive(leftSpeedSupplier.get(), rightSpeedSupplier.get());
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
