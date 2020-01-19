package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.PIDLoop;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;


public class DriveArcadeWithPID extends CommandBase {

    private final TankDrivetrain drivetrain;
    private final PIDLoop pidLoop;
    private Supplier<Double> setpoint;

    /**
     * @param drivetrain is the {@link TankDrivetrain} the command moves.
     * @param pidLoop is the {@link PIDLoop} that calculates and sets the speed to the drivetrain.
     */

    public DriveArcadeWithPID(TankDrivetrain drivetrain, PIDLoop pidLoop, Supplier<Double> setpoint) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
        this.pidLoop = pidLoop;
        this.pidLoop.setSetpoint(setpoint.get());
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, PIDLoop pidLoop, double setpoint){
        this(drivetrain, pidLoop, () -> setpoint);
    }

    /**
     * starts the given PIDLoop.
     */
    @Override
    public void initialize() {
        pidLoop.enable();
    }

    /**
     * updates the PIDLoop's setpoint.
     */
    @Override
    public void execute() {
        pidLoop.setSetpoint(setpoint.get());
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
