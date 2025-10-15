package com.spikes2212.command.drivetrains.swerve.commands;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class DriveSwerve extends Command {

    private final SwerveDrivetrain drivetrain;

    private final Supplier<Double> xSpeed;
    private final Supplier<Double> ySpeed;
    private final Supplier<Double> rotationSpeed;

    private final boolean isFieldRelative;

    double timer;

    public DriveSwerve(SwerveDrivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                       Supplier<Double> rotationSpeed, boolean isFieldRelative) {
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotationSpeed = rotationSpeed;
        this.isFieldRelative = isFieldRelative;
    }

    @Override
    public void initialize(){
        timer = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        drivetrain.drive(xSpeed.get(), ySpeed.get(), rotationSpeed.get(), isFieldRelative,
                Timer.getFPGATimestamp() - timer);
        timer = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.stopModules();
    }
}
