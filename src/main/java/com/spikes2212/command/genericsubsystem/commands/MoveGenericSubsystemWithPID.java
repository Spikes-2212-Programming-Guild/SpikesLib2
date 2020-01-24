package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * This command moves a {@link GenericSubsystem} according to a {@link com.spikes2212.control.PIDLoop}
 * until it reaches its target or until it can't move anymore.
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

    /*
     *
     */
    private FFSettings ffSettings;
    /**
     * the setpoint for the subsystem.
     */
    private Supplier<Double> setpoint;

    /**
     * An object that makes the necessary calculations for the PID control loop.
     */
    private PIDController pidController;

    /*
     *
     */
    private FeedForwardController feedForwardController;

    /**
     * The last time the subsystem didn't reach target.
     */
    private double lastTimeNotOnTarget;

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, PIDSettings pidSettings, Supplier<Double> setpoint, FFSettings ffSettings) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.pidSettings = pidSettings;
        this.ffSettings = ffSettings;
        this.setpoint = setpoint;
        this.feedForwardController = new FeedForwardController(ffSettings.getkS(), ffSettings.getkV(), ffSettings.getkA(), ffSettings.getkG(), 0.02);
        this.pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
    }

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, PIDSettings pidSettings, double setpoint, FFSettings ffSettings) {
        this(subsystem, pidSettings, () -> setpoint, ffSettings);
    }

    @Override
    public void initialize() {
        pidController.setTolerance(pidSettings.getTolerance());
        pidController.setPID(pidSettings.getkP(),pidSettings.getkI(),pidSettings.getkD());
    }

    @Override
    public void execute() {
        double svagValue=feedForwardController.calculate(setpoint.get());
        subsystem.move((pidController.calculate(source.get(), setpoint.get())+svagValue)/2);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= pidSettings.getWaitTime();
    }
}
