package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.PIDLoop;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class DriveArcadeWithPID extends CommandBase {

    private TankDrivetrain drivetrain;
    private PIDLoop rotationPIDLoop;
    private Supplier<Double> moveForwardSpeed;

    public DriveArcadeWithPID(TankDrivetrain drivetrain, PIDLoop rotationPIDLoop, Supplier<Double> moveForwardSpeed) {
        super();
        this.drivetrain = drivetrain;
        this.rotationPIDLoop = rotationPIDLoop;
        this.moveForwardSpeed = moveForwardSpeed;
        this.addRequirements(drivetrain);
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, PIDLoop rotationPIDLoop, double moveForwardSpeed) {
        this(drivetrain, rotationPIDLoop, () -> moveForwardSpeed);
    }

    @Override
    public void initialize() {
        rotationPIDLoop.enable();
    }

    @Override
    public void execute() {
        rotationPIDLoop.update();
        drivetrain.stop();
    }

    @Override
    public void end(boolean interrupted) {
        rotationPIDLoop.disable();
    }

    @Override
    public boolean isFinished() {
        return rotationPIDLoop.onTarget();
    }


}
