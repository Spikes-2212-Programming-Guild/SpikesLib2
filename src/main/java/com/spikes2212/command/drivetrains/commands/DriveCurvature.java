package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class DriveCurvature extends CommandBase {


    protected TankDrivetrain drivetrain;

    protected Supplier<Double> speed;
    protected Supplier<Double> rotation;

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
