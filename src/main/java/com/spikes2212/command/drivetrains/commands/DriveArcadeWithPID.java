package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.PIDLoop;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class DriveArcadeWithPID extends CommandBase {

    private final TankDrivetrain drivetrain;
    private final PIDLoop movementPIDLoop;
    private final Supplier<Double> moveForwardSpeed;

    public DriveArcadeWithPID(TankDrivetrain drivetrain, PIDLoop movementPIDLoop, Supplier<Double> moveForwardSpeed) {
        super();
        this.drivetrain = drivetrain;
        this.movementPIDLoop = movementPIDLoop;
        this.moveForwardSpeed = moveForwardSpeed;
        this.addRequirements(drivetrain);
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, PIDLoop movementPIDLoop, double moveForwardSpeed) {
        this(drivetrain, movementPIDLoop, () -> moveForwardSpeed);
    }

    @Override
    public void initialize() {
        movementPIDLoop.enable();
    }

    @Override
    public void execute() {
        movementPIDLoop.update();
        drivetrain.stop();
    }

    @Override
    public void end(boolean interrupted) {
        movementPIDLoop.disable();
    }

    @Override
    public boolean isFinished() {
        return movementPIDLoop.onTarget();
    }


}
