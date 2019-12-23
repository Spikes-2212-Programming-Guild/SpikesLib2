package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.PIDLoop;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class DriveWithPID extends CommandBase {

    private final TankDrivetrain drivetrain;
    private final PIDLoop movementPIDLoop;

    public DriveWithPID(TankDrivetrain drivetrain, PIDLoop movementPIDLoop) {
        super();
        this.drivetrain = drivetrain;
        this.movementPIDLoop = movementPIDLoop;
        this.addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        movementPIDLoop.enable();
    }

    @Override
    public void execute() {
        movementPIDLoop.update();
    }

    @Override
    public void end(boolean interrupted) {
        movementPIDLoop.disable();
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return movementPIDLoop.onTarget();
    }

}
