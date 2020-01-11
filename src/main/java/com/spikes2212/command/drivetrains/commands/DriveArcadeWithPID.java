package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.PIDLoop;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveArcadeWithPID extends CommandBase {

    private final TankDrivetrain drivetrain;
    private final PIDLoop movementPIDLoop;
    private double setpoint;

    /**
     * @param drivetrain      is the {@link TankDrivetrain} the command moves.
     * @param movementPIDLoop is the {@link PIDLoop} that calculates and sets the speed to the drivetrain.
     */

    public DriveArcadeWithPID(TankDrivetrain drivetrain, PIDLoop movementPIDLoop, double setpoint) {
        super();
        this.drivetrain = drivetrain;
        this.movementPIDLoop = movementPIDLoop;
        this.setpoint = setpoint;
        this.addRequirements(drivetrain);
    }

    /**
     * starts the given PIDLoop.
     */
    @Override
    public void initialize() {
        movementPIDLoop.setSetpoint(setpoint);
        movementPIDLoop.enable();
    }

    /**
     * updates the PIDLoop's setpoint.
     */
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
