package com.spikes2212.command.drivetrains.commands;

import java.util.function.Supplier;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * this command moves a {@link TankDrivetrain} by linear and rotational speeds, using
 * the arcade control method written by WPILIB.
 *
 * @author yuval levy
 * @see TankDrivetrain
 */

public class DriveArcade extends CommandBase {
    protected final TankDrivetrain tankDrivetrain;
    protected final Supplier<Double> moveValueSupplier;
    protected final Supplier<Double> rotateValueSupplier;
    protected final Supplier<Boolean> isFinished;

    /**
     * This constructs a new {@link DriveArcade} command that moves the given
     * {@link TankDrivetrain} according to speed values from Double {@link Supplier}s
     * for linear and rotational movements.
     *
     * @param drivetrain          the tank drivetrain this command opperates on.
     * @param moveValueSupplier   the double {@link Supplier} supplying the speed to move forward
     *                            with. Positive values go forwards.
     * @param rotateValueSupplier the double {@link Supplier} supplying the speed to turn with.
     *                            Positive values go left.
     */
    public DriveArcade(TankDrivetrain drivetrain, Supplier<Double> moveValueSupplier,
                       Supplier<Double> rotateValueSupplier, Supplier<Boolean> isFinished) {
        addRequirements(drivetrain);
        this.tankDrivetrain = drivetrain;
        this.moveValueSupplier = moveValueSupplier;
        this.rotateValueSupplier = rotateValueSupplier;
        this.isFinished = isFinished;
    }

    public DriveArcade(TankDrivetrain drivetrain, Supplier<Double> moveValueSupplier,
                       Supplier<Double> rotateValueSupplier) {
        this(drivetrain, moveValueSupplier, rotateValueSupplier, () -> false);
    }

    public DriveArcade(TankDrivetrain drivetrain, double moveValue, double rotateValue) {
        this(drivetrain, () -> moveValue, () -> rotateValue, () -> false);
    }

    public DriveArcade(TankDrivetrain drivetrain, double moveValue, double rotateValue, boolean isFinished) {
        this(drivetrain, () -> moveValue, () -> rotateValue, () -> isFinished);
    }

    /**
     * sets the rotationValue and the forwardValue
     */
    @Override
    public void execute() {
        tankDrivetrain.arcadeDrive(moveValueSupplier.get(), rotateValueSupplier.get());
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