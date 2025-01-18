package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/**
 * A command that moves a {@link TankDrivetrain} while controlling the curvature of its path,
 * using the curvature drive method written by WPILib.
 *
 * @author Simon Kharmatsky
 * @see TankDrivetrain
 */
public class DriveCurvature extends Command {

    protected final TankDrivetrain drivetrain;
    protected final Supplier<Double> speed;
    protected final Supplier<Double> rotation;

    public DriveCurvature(TankDrivetrain drivetrain, Supplier<Double> speed, Supplier<Double> rotation) {
        this.drivetrain = drivetrain;
        this.speed = speed;
        this.rotation = rotation;
    }

    public DriveCurvature(TankDrivetrain drivetrain, double speed, double rotation) {
        this(drivetrain, () -> speed, () -> rotation);
    }

    @Override
    public void execute() {
        drivetrain.curvatureDrive(speed.get(), rotation.get());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
