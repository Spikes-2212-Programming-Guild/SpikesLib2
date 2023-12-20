package com.spikes2212.command.drivetrains.commands;

import java.util.function.Supplier;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that moves a {@link TankDrivetrain} using linear and rotational voltages.
 *
 * @author Yorai Coval
 * @see TankDrivetrain
 */

public class DriveArcadeWithVoltages extends CommandBase {

    protected final TankDrivetrain tankDrivetrain;
    protected final Supplier<Double> moveValueSupplier;
    protected final Supplier<Double> rotateValueSupplier;
    protected final Supplier<Boolean> isFinished;

    /**
     * Constructs a new {@link DriveArcadeWithVoltages} command that moves the given
     * {@link TankDrivetrain} according to voltage values from Double {@link Supplier}s
     * for linear and rotational movements.
     *
     * @param drivetrain          the tank drivetrain this command operates on
     * @param moveValueSupplier   the Double {@link Supplier} supplying the linear voltage (-12 to 12).
     *                            Positive values go forwards
     * @param rotateValueSupplier the Double {@link Supplier} supplying the rotational voltage (-12 to 12).
     *                            Positive values go clockwise
     * @param isFinished          when to finish the command
     */
    public DriveArcadeWithVoltages(TankDrivetrain drivetrain, Supplier<Double> moveValueSupplier,
                                   Supplier<Double> rotateValueSupplier, Supplier<Boolean> isFinished) {
        addRequirements(drivetrain);
        this.tankDrivetrain = drivetrain;
        this.moveValueSupplier = moveValueSupplier;
        this.rotateValueSupplier = rotateValueSupplier;
        this.isFinished = isFinished;
    }

    /**
     * Constructs a new {@link DriveArcadeWithVoltages} command that moves the given
     * {@link TankDrivetrain} according to voltage values from Double {@link Supplier}s
     * for linear and rotational movements.
     *
     * @param drivetrain          the tank drivetrain this command operates on
     * @param moveValueSupplier   the Double {@link Supplier} supplying the linear voltage (-12 to 12).
     *                            Positive values go forwards
     * @param rotateValueSupplier the Double {@link Supplier} supplying the rotational voltage (-12 to 12).
     *                            Positive values go clockwise
     */
    public DriveArcadeWithVoltages(TankDrivetrain drivetrain, Supplier<Double> moveValueSupplier,
                                   Supplier<Double> rotateValueSupplier) {
        this(drivetrain, moveValueSupplier, rotateValueSupplier, () -> false);
    }

    /**
     * Constructs a new {@link DriveArcadeWithVoltages} command that moves the given
     * {@link TankDrivetrain} according to voltage values for linear and rotational movements.
     *
     * @param drivetrain  the tank drivetrain this command operates on
     * @param moveValue   the linear voltage (-12 to 12). Positive values go forwards
     * @param rotateValue the rotational voltage (-12 to 12). Positive values go clockwise
     * @param isFinished  when to finish the command
     */
    public DriveArcadeWithVoltages(TankDrivetrain drivetrain, double moveValue, double rotateValue,
                                   Supplier<Boolean> isFinished) {
        this(drivetrain, () -> moveValue, () -> rotateValue, isFinished);
    }

    /**
     * Constructs a new {@link DriveArcadeWithVoltages} command that moves the given
     * {@link TankDrivetrain} according to voltage values for linear and rotational movements.
     *
     * @param drivetrain  the tank drivetrain this command operates on
     * @param moveValue   the linear voltage (-12 to 12). Positive values go forwards
     * @param rotateValue the rotational voltage (-12 to 12). Positive values go clockwise
     */
    public DriveArcadeWithVoltages(TankDrivetrain drivetrain, double moveValue, double rotateValue) {
        this(drivetrain, () -> moveValue, () -> rotateValue, () -> false);
    }

    @Override
    public void execute() {
        tankDrivetrain.arcadeDriveVoltages(moveValueSupplier.get(), rotateValueSupplier.get());
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
