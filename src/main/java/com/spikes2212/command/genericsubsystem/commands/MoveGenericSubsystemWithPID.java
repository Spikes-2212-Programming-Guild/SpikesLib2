package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.PIDFSettings;
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
     * The PIDF Settings for the PID control loop.
     */
    private PIDFSettings PIDFSettings;

    /**
     * the setpoint for the subsystem.
     */
    private Supplier<Double> setpoint;

    /**
     * An object that makes the necessary calculations for the PID control loop.
     */
    private PIDController pidController;

    /**
     * The last time the subsystem didn't reach the target.
     */
    private double lastTimeNotOnTarget;

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, Supplier<Double> setpoint, Supplier<Double> source,
                                       PIDFSettings PIDFSettings) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.PIDFSettings = PIDFSettings;
        this.setpoint = setpoint;
        this.source = source;
        this.pidController = new PIDController(PIDFSettings.getkP(), PIDFSettings.getkI(), PIDFSettings.getkD());
    }

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, double setpoint, double source,
                                       PIDFSettings PIDFSettings) {
        this(subsystem, () -> setpoint, () -> source, PIDFSettings);
    }

    @Override
    public void execute() {
        pidController.setTolerance(PIDFSettings.getTolerance());
        pidController.setPID(PIDFSettings.getkP(), PIDFSettings.getkI(), PIDFSettings.getkD());

        double pidValue = pidController.calculate(source.get(), setpoint.get());
        double svagValue = PIDFSettings.getkF();
        subsystem.move(pidValue + svagValue);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        double currentTime = Timer.getFPGATimestamp();

        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return currentTime - lastTimeNotOnTarget >= PIDFSettings.getWaitTime();
    }
}
