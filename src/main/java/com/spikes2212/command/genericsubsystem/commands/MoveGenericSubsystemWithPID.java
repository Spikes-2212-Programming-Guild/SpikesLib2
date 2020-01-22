package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * This command moves a {@link GenericSubsystem} according to a {@link Supplier}
 * or a constant speed until it can't move anymore.
 *
 * @author Yuval Levy
 * @see GenericSubsystem
 */
public class MoveGenericSubsystemWithPID extends CommandBase {
    /**
     * the subsystem the command moves.
     */
    private final GenericSubsystem subsystem;

    /**
     * A supplier that returns the subsystem's current location.
     */
    private Supplier<Double> source;

    /**
     * The PID Settings for the PID control loop.
     */
    private PIDSettings pidSettings;

    /**
     * the setpoint for the subsystem.
     */
    private Supplier<Double> setpoint;

    /**
     * An object that makes the necessary calculations for the PID control loop.
     */
    private PIDController pidController;

    /**
     * The last time the subsystem didn't reach target.
     */
    private double lastTimeNotOnTarget;

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, PIDSettings pidSettings, Supplier<Double> setpoint) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.pidSettings = pidSettings;
        this.setpoint = setpoint;
        this.pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
    }

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, PIDSettings pidSettings, double setpoint) {
        this(subsystem, pidSettings, () -> setpoint);
    }

    @Override
    public void execute() {
        pidController.setTolerance(pidSettings.getTolerance());
        subsystem.move(pidController.calculate(source.get(), setpoint.get()));
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        if(!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= pidSettings.getWaitTime();
    }
}
