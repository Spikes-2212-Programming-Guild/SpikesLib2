package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.PIDLoop;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class DriveTankWithPID extends CommandBase {

    private final TankDrivetrain drivetrain;
    private final PIDLoop rightPIDLoop;
    private final PIDLoop leftPIDLoop;
    private Supplier<Double> rightSetpoint;
    private Supplier<Double> leftSetpoint;

    /**
     * @param drivetrain    is the {@link TankDrivetrain} the command moves.
     * @param rightPIDLoop  is the {@link PIDLoop} that calculates and sets the speed to the drivetrain.
     * @param rightSetpoint is the setpoint for the PIDLoop.
     */

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDLoop rightPIDLoop, PIDLoop leftPIDLoop, Supplier<Double> rightSetpoint, Supplier<Double> leftSetpoint) {
        super();
        this.drivetrain = drivetrain;
        this.rightPIDLoop = rightPIDLoop;
        this.leftPIDLoop = leftPIDLoop;
        this.rightSetpoint = rightSetpoint;
        this.leftSetpoint = leftSetpoint;
        this.addRequirements(drivetrain);
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDLoop rightPIDLoop, PIDLoop leftPIDLoop, double rightSetpoint, double leftSetpoint) {
        this(drivetrain, rightPIDLoop, leftPIDLoop, () -> rightSetpoint, () -> leftSetpoint);
    }

    /**
     * starts the given PIDLoop.
     */
    @Override
    public void initialize() {
        rightPIDLoop.enable();

        leftPIDLoop.enable();
    }

    /**
     * updates the PIDLoop's setpoint.
     */
    @Override
    public void execute() {
        rightPIDLoop.setSetpoint(leftSetpoint.get());
        leftPIDLoop.setSetpoint(rightSetpoint.get());
        rightPIDLoop.update();
        leftPIDLoop.update();
    }

    @Override
    public void end(boolean interrupted) {
        rightPIDLoop.disable();
        leftPIDLoop.disable();
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return leftPIDLoop.onTarget() && rightPIDLoop.onTarget();
    }
}
