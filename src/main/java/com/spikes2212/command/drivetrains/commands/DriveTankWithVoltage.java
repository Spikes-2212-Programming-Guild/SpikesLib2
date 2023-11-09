package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * A command that moves a {@link TankDrivetrain} using voltage supplied to the left and the right sides independently.
 *
 * @author Yuval Levy
 * @see TankDrivetrain
 */
public class DriveTankWithVoltage extends CommandBase {
    protected final Supplier<Double> leftVoltage;
    protected final Supplier<Double> rightVoltage;
    protected final TankDrivetrain drivetrain;
    /**
     * This constructs a new {@link DriveTank} command that moves the given
     * {@link TankDrivetrain} according to speed values from Double {@link Supplier}s for left and right sides.<br>
     * Positive values move forwards.
     *
     * @param drivetrain         the drivetrain this command requires and moves.
     * @param rightVoltage  the double {@link Supplier} supplying the voltage to move on the left side with.
     * @param leftVoltage the double {@link Supplier} supplying the voltage to move on the right side with.
     */
    public DriveTankWithVoltage(TankDrivetrain drivetrain, Supplier<Double> leftVoltage, Supplier<Double> rightVoltage) {
        this.drivetrain =drivetrain;
        this.leftVoltage=leftVoltage;
        this.rightVoltage=rightVoltage;
    }
    @Override
    public void execute() {
        drivetrain.tankDriveVoltages(leftVoltage.get(), rightVoltage.get());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

}