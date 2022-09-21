package com.spikes2212.command.genericsubsystem.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * Move a {@link SmartMotorControllerSubsystem} using its motor controller's control loops.
 *
 * @author Eran Goldstein
 * @see SmartMotorControllerSubsystem
 */
public class MoveSmartMotorControllerSubsystem extends CommandBase {

    /**
     * The {@link SmartMotorControllerSubsystem} this command will run on.
     */
    private final SmartMotorControllerSubsystem subsystem;

    /**
     * The setpoint this command should bring the {@link SmartMotorControllerSubsystem} to.
     */
    private final Supplier<Double> setpoint;

    private final Supplier<Double> waitTime;

    private final ControlMode controlMode;

    private final int pidSlot;

    private double lastTimeNotOnTarget;

    public MoveSmartMotorControllerSubsystem(SmartMotorControllerSubsystem subsystem, int pidSlot,
                                             ControlMode controlMode, Supplier<Double> setpoint,
                                             Supplier<Double> waitTime) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.controlMode = controlMode;
        this.pidSlot = pidSlot;
        this.setpoint = setpoint;
        this.waitTime = waitTime;
        this.lastTimeNotOnTarget = 0;
    }

    public MoveSmartMotorControllerSubsystem(SmartMotorControllerSubsystem subsystem, int pidSlot,  double setpoint,
                                             Supplier<Double> waitTime) {
        this(subsystem, pidSlot, null, () -> setpoint, waitTime);
    }

    @Override
    public void initialize() {
        subsystem.configureLoop(pidSlot);
    }

    @Override
    public void execute() {
        subsystem.pidSet(pidSlot, controlMode, setpoint.get());
        subsystem.pidSet(pidSlot, setpoint.get());
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.finish();
    }

    @Override
    public boolean isFinished() {
        if (!subsystem.onTarget(setpoint.get())) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget > waitTime.get();
    }
}
