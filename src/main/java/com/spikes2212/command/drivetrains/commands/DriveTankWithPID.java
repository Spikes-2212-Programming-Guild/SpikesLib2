package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.PIDLoop;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class DriveTankWithPID extends CommandBase {

    private final TankDrivetrain drivetrain;
    private final PIDLoop leftPIDLoop;
    private final PIDLoop rightPIDLoop;
    private Supplier<Double> leftSetpoint;
    private Supplier<Double> rightSetpoint;

    /**
     * @param drivetrain    is the {@link TankDrivetrain} the command moves.
     * @param rightPIDLoop  is the {@link PIDLoop} that calculates and sets the speed to the drivetrain.
     * @param rightSetpoint is the setpoint for the PIDLoop.
     */

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDLoop leftPIDLoop, PIDLoop rightPIDLoop,
                            Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.leftPIDLoop = leftPIDLoop;
        this.rightPIDLoop = rightPIDLoop;
        this.leftSetpoint = leftSetpoint;
        this.rightSetpoint = rightSetpoint;
        this.rightPIDLoop.setSetpoint(rightSetpoint.get());
        this.leftPIDLoop.setSetpoint(leftSetpoint.get());
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDLoop leftPIDLoop, PIDLoop rightPIDLoop,
                            double leftSetpoint, double rightSetpoint) {
        this(drivetrain, rightPIDLoop, leftPIDLoop, () -> rightSetpoint, () -> leftSetpoint);
    }

    /**
     * starts the given PIDLoop.
     */
    @Override
    public void initialize() {
        leftPIDLoop.enable();
        rightPIDLoop.enable();
    }

    /**
     * updates the PIDLoop's setpoint.
     */
    @Override
    public void execute() {
        leftPIDLoop.setSetpoint(rightSetpoint.get());
        rightPIDLoop.setSetpoint(leftSetpoint.get());
        leftPIDLoop.update();
        rightPIDLoop.update();
    }

    @Override
    public void end(boolean interrupted) {
        leftPIDLoop.disable();
        rightPIDLoop.disable();
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return leftPIDLoop.onTarget() && rightPIDLoop.onTarget();
    }
}
