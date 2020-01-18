package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.PIDLoop;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class MoveGenericSubsystemWithPID extends CommandBase {

    private final GenericSubsystem subsystem;
    private final PIDLoop pidLoop;
    private Supplier<Double> setpoint;

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, PIDLoop pidLoop, Supplier<Double> setpoint) {
        this.addRequirements(subsystem);
        this.subsystem = subsystem;
        this.pidLoop = pidLoop;
        this.setpoint = setpoint;
        this.pidLoop.setSetpoint(setpoint.get());
    }

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, PIDLoop pidLoop, double setpoint){
        this(subsystem, pidLoop, ()-> setpoint);
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
        subsystem.stop();
        pidLoop.disable();
    }

    @Override
    public boolean isFinished() {
        return pidLoop.onTarget();
    }
}
