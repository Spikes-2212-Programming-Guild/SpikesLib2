package com.spikes2212.command.genericsubsystem.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.PIDTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Predicate;
import java.util.function.Supplier;

public class MoveTalonWithPID extends CommandBase {

    private PIDTalon talon;
    private Supplier<Double> maxSpeed;
    private Supplier<Double> minSpeed;
    private Supplier<Double> setpoint;
    private int timeout;
    private Predicate<Double> canMove;
    private double lastTimeNotOnTarget;

    public MoveTalonWithPID(Subsystem subsystem, PIDTalon talon, Supplier<Double> maxSpeed,
                            Supplier<Double> minSpeed, ControlMode mode, Supplier<Double> setpoint,
                            int timeout, Predicate<Double> canMove) {
        addRequirements(subsystem);
        this.talon = talon;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.setpoint = setpoint;
        this.timeout = timeout;
        this.canMove = canMove;
    }

    public MoveTalonWithPID(GenericSubsystem genericSubsystem, PIDTalon talon, ControlMode mode,
                            Supplier<Double> setpoint, int timeout) {
        addRequirements(genericSubsystem);
        this.talon = talon;
        this.maxSpeed = genericSubsystem.maxSpeed;
        this.minSpeed = genericSubsystem.minSpeed;
        this.setpoint = setpoint;
        this.timeout = timeout;
        this.canMove = genericSubsystem::canMove;
    }

    public MoveTalonWithPID(Subsystem subsystem, PIDTalon talon, Supplier<Double> maxSpeed,
                            Supplier<Double> minSpeed, ControlMode mode, Supplier<Double> setpoint, int timeout) {
        addRequirements(subsystem);
        this.talon = talon;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.setpoint = setpoint;
        this.timeout = timeout;
        this.canMove = v->true;
    }

    public MoveTalonWithPID(Subsystem subsystem, PIDTalon talon, Supplier<Double> maxSpeed,
                            Supplier<Double> minSpeed, Supplier<Double> setpoint, int timeout, Predicate<Double> canMove) {
        addRequirements(subsystem);
        this.talon = talon;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.setpoint = setpoint;
        this.timeout = timeout;
        this.canMove = canMove;
    }

    public MoveTalonWithPID(GenericSubsystem genericSubsystem, PIDTalon talon, Supplier<Double> setpoint, int timeout) {
        addRequirements(genericSubsystem);
        this.talon = talon;
        this.maxSpeed = genericSubsystem.maxSpeed;
        this.minSpeed = genericSubsystem.minSpeed;
        this.setpoint = setpoint;
        this.timeout = timeout;
        this.canMove = genericSubsystem::canMove;
    }

    public MoveTalonWithPID(Subsystem subsystem, PIDTalon talon, Supplier<Double> maxSpeed,
                            Supplier<Double> minSpeed, Supplier<Double> setpoint, int timeout) {
        addRequirements(subsystem);
        this.talon = talon;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.setpoint = setpoint;
        this.timeout = timeout;
        this.canMove = v->true;
    }

    @Override
    public void initialize() {
        talon.configureLoop(maxSpeed, minSpeed, timeout);
    }

    @Override
    public void execute() {
        talon.pidSet(setpoint.get(), timeout);
    }

    @Override
    public void end(boolean interrupted) {
        talon.finish();
    }

    @Override
    public boolean isFinished() {
        if(!talon.onTarget(setpoint.get())){
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return !canMove.test(talon.getTalon().getMotorOutputPercent()) ||
                Timer.getFPGATimestamp() - lastTimeNotOnTarget >= talon.getSettings().getWaitTime();
    }
}
