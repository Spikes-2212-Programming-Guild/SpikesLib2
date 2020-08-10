package com.spikes2212.command.genericsubsystem.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.PIDTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Predicate;
import java.util.function.Supplier;

/**
 * This command makes a Talon speed controller run a PID loop.
 */
public class MoveTalonWithPID extends CommandBase {

    /**
     * The {@link PIDTalon} with the necessary details.
     */
    private PIDTalon talon;
    /**
     * A {@link Supplier} that returns the maximum speed allowed for the PID loop.
     */
    private Supplier<Double> maxSpeed;
    /**
     * A {@link Supplier} that returns the minimum speed allowed for the PID loop.
     */
    private Supplier<Double> minSpeed;
    /**
     * A {@link Supplier} that returns the target setpoint for the PID loop.
     */
    private Supplier<Double> setpoint;
    /**
     * The Talon's timeout. Any process that changes the Talon's configurations will time out after that many ms.
     * Use 0 if you want to have no time limit on such processes.
     */
    private int timeout;
    /**
     * A {@link Predicate} that receives the speed of the motor and
     * returns whether that speed is valid for that motor.
     */
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
        this.canMove = v -> true;
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
        this.canMove = v -> true;
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
        if (!talon.onTarget(setpoint.get())) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return !canMove.test(talon.getTalon().getMotorOutputPercent()) ||
                Timer.getFPGATimestamp() - lastTimeNotOnTarget >= talon.getSettings().getWaitTime();
    }
}
