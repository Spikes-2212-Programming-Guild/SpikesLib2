package com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollersubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
    protected final SmartMotorControllerSubsystem subsystem;

    /**
     * The setpoint this command should bring the {@link SmartMotorControllerSubsystem} to.
     */
    private final Supplier<Double> setpoint;

    private final Supplier<Double> waitTime;

    private final ControlMode controlMode;
    private final CANSparkMax.ControlType controlType;

    protected final PIDSettings pidSettings;

    protected final FeedForwardSettings feedForwardSettings;

    private double lastTimeNotOnTarget;

    private MoveSmartMotorControllerSubsystem(SmartMotorControllerSubsystem subsystem, PIDSettings pidSettings,
                                             FeedForwardSettings feedForwardSettings,
                                             ControlMode controlMode, CANSparkMax.ControlType controlType,
                                             Supplier<Double> setpoint, Supplier<Double> waitTime) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.controlMode = controlMode;
        this.controlType = controlType;
        this.pidSettings = pidSettings;
        this.feedForwardSettings = feedForwardSettings;
        this.setpoint = setpoint;
        this.waitTime = waitTime;
        this.lastTimeNotOnTarget = 0;
    }

    public MoveSmartMotorControllerSubsystem(SmartMotorControllerSubsystem subsystem, PIDSettings pidSettings,
                                             FeedForwardSettings feedForwardSettings,
                                             ControlMode controlMode, Supplier<Double> setpoint,
                                             Supplier<Double> waitTime) {
        this(subsystem, pidSettings, feedForwardSettings, controlMode, null, setpoint, waitTime);
    }

    MoveSmartMotorControllerSubsystem(SmartMotorControllerSubsystem subsystem, PIDSettings pidSettings,
                                             FeedForwardSettings feedForwardSettings,
                                             CANSparkMax.ControlType controlType, Supplier<Double> setpoint,
                                             Supplier<Double> waitTime) {
        this(subsystem, pidSettings, feedForwardSettings, null, controlType, setpoint, waitTime);
    }

    @Override
    public void initialize() {
        subsystem.configureLoop(pidSettings, feedForwardSettings);
    }

    @Override
    public void execute() {
        subsystem.pidSet(controlMode, setpoint.get(), pidSettings, feedForwardSettings);
        subsystem.pidSet(controlType, setpoint.get(), pidSettings, feedForwardSettings);
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
