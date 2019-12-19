package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.PIDLoop;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Objects;

public class DriveTankWithPID extends CommandBase {

    /**
     * the drivetrain the command moves.
     */
    private final TankDrivetrain drivetrain;

    /**
     * the PIDloop that calculates and sets the values for the motors.
     */
    private final PIDLoop pidLoop;

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDLoop pidLoop){
        super();
        this.drivetrain = drivetrain;
        this.pidLoop = pidLoop;
        this.addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        pidLoop.enable();
    }

    @Override
    public void execute() {
        pidLoop.update();
    }

    @Override
    public void end(boolean interrupted) {
        pidLoop.disable();
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return pidLoop.onTarget();
    }
}
