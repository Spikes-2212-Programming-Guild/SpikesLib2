package com.spikes2212.command.genericsubsystem.commands;


import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class MoveMotorGenericSubsystemWithVoltage extends CommandBase {
    protected final MotoredGenericSubsystem subsystem;
    protected final Supplier<Double> voltageSupplier;

    public MoveMotorGenericSubsystemWithVoltage(MotoredGenericSubsystem subsystem, Supplier<Double> voltageSupplier) {
        this.subsystem = subsystem;
        this.voltageSupplier = voltageSupplier;
    }
    public MoveMotorGenericSubsystemWithVoltage(MotoredGenericSubsystem subsystem, double voltage) {
        this(subsystem, () -> voltage);
    }
    @Override
    public void execute(){
        subsystem.SetVoltage(this.voltageSupplier.get() / RobotController.getBatteryVoltage());
    }
    @Override
    public void end(boolean interrupted) {
        subsystem.stop();}
    @Override
    public boolean isFinished() {
        return !subsystem.canMove(this.voltageSupplier.get() / RobotController.getBatteryVoltage());
    }

}


