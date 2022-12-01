package com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollersubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;

import java.util.function.Supplier;


/**
 * A command that moves a {@link SmartMotorControllerSubsystem} by running a trapezoid profile on its
 * master's control loops.
 *
 * @author Yoel Perman Brilliant
 * @see SmartMotorControllerSubsystem
 * @see MoveSmartMotorControllerSubsystem
 */
public class MoveSmartMotorControllerSubsystemTrapezically extends MoveSmartMotorControllerSubsystem {

    protected final TrapezoidProfileSettings trapezoidProfileSettings;

    /**
     * Constructs a new instance of {@link MoveSmartMotorControllerSubsystemTrapezically}.
     *
     * @param subsystem the {@link SmartMotorControllerSubsystem} this command will run on
     * @param pidSettings the loop's PID constants
     * @param feedForwardSettings the loop's feed forward gains
     * @param setpoint the setpoint this command should bring the {@link SmartMotorControllerSubsystem} to
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    public MoveSmartMotorControllerSubsystemTrapezically(SmartMotorControllerSubsystem subsystem, PIDSettings pidSettings,
                                                         FeedForwardSettings feedForwardSettings,
                                                         Supplier<Double> setpoint,
                                                         TrapezoidProfileSettings trapezoidProfileSettings) {
        super(subsystem, pidSettings, feedForwardSettings, UnifiedControlMode.TRAPEZOID_PROFILE, setpoint);
        this.trapezoidProfileSettings = trapezoidProfileSettings;
    }

    @Override
    public void initialize() {
        subsystem.configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
    }

    @Override
    public void execute() {
        subsystem.pidSet(controlMode, setpoint.get(), pidSettings, feedForwardSettings, trapezoidProfileSettings);
    }
}
