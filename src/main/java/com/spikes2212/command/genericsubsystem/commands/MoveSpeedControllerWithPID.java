package com.spikes2212.command.genericsubsystem.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.PIDSpeedController;
import com.spikes2212.control.PIDTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Predicate;
import java.util.function.Supplier;

/**
 * This command makes a {@link PIDSpeedController} run a PID loop.
 */
public class MoveSpeedControllerWithPID extends CommandBase {

    /**
     * The {@link PIDSpeedController} with the necessary details.
     */
    private PIDSpeedController speedController;

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
     * A {@link Predicate} that receives the speed of the motor and
     * returns whether that speed is valid for that motor.
     */
    private Predicate<Double> canMove;
    private double lastTimeNotOnTarget;

    public MoveSpeedControllerWithPID(Subsystem subsystem, PIDTalon speedController, Supplier<Double> maxSpeed,
                                      Supplier<Double> minSpeed, ControlMode mode, Supplier<Double> setpoint,
                                      Predicate<Double> canMove) {
        addRequirements(subsystem);
        this.speedController = speedController;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.setpoint = setpoint;
        this.canMove = canMove;
    }

    public MoveSpeedControllerWithPID(GenericSubsystem genericSubsystem, PIDTalon speedController, ControlMode mode,
                                      Supplier<Double> setpoint) {
        addRequirements(genericSubsystem);
        this.speedController = speedController;
        this.maxSpeed = genericSubsystem.maxSpeed;
        this.minSpeed = genericSubsystem.minSpeed;
        this.setpoint = setpoint;
        this.canMove = genericSubsystem::canMove;
    }

    public MoveSpeedControllerWithPID(Subsystem subsystem, PIDTalon speedController, Supplier<Double> maxSpeed,
                                      Supplier<Double> minSpeed, ControlMode mode, Supplier<Double> setpoint) {
        addRequirements(subsystem);
        this.speedController = speedController;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.setpoint = setpoint;
        this.canMove = v -> true;
    }

    public MoveSpeedControllerWithPID(Subsystem subsystem, PIDTalon speedController, Supplier<Double> maxSpeed,
                                      Supplier<Double> minSpeed, Supplier<Double> setpoint, Predicate<Double> canMove) {
        addRequirements(subsystem);
        this.speedController = speedController;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.setpoint = setpoint;
        this.canMove = canMove;
    }

    public MoveSpeedControllerWithPID(GenericSubsystem genericSubsystem, PIDTalon speedController, Supplier<Double> setpoint) {
        addRequirements(genericSubsystem);
        this.speedController = speedController;
        this.maxSpeed = genericSubsystem.maxSpeed;
        this.minSpeed = genericSubsystem.minSpeed;
        this.setpoint = setpoint;
        this.canMove = genericSubsystem::canMove;
    }

    public MoveSpeedControllerWithPID(Subsystem subsystem, PIDTalon speedController, Supplier<Double> maxSpeed,
                                      Supplier<Double> minSpeed, Supplier<Double> setpoint) {
        addRequirements(subsystem);
        this.speedController = speedController;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.setpoint = setpoint;
        this.canMove = v -> true;
    }

    @Override
    public void initialize() {
        speedController.configureLoop(maxSpeed, minSpeed);
    }

    @Override
    public void execute() {
        speedController.pidSet(setpoint.get());
    }

    @Override
    public void end(boolean interrupted) {
        speedController.finish();
    }

    @Override
    public boolean isFinished() {
        if (!speedController.onTarget(setpoint.get())) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return !canMove.test(speedController.getSpeed()) ||
                Timer.getFPGATimestamp() - lastTimeNotOnTarget >= speedController.getWaitTime();
    }
}
