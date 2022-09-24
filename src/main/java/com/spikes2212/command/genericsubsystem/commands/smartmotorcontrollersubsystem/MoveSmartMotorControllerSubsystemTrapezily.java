package com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollersubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class MoveSmartMotorControllerSubsystemTrapezily extends MoveSmartMotorControllerSubsystem {

    protected final TrapezoidProfileSettings trapezoidProfileSettings;

    public MoveSmartMotorControllerSubsystemTrapezily(SmartMotorControllerSubsystem subsystem, PIDSettings pidSettings,
                                               FeedForwardSettings feedForwardSettings,
                                               Supplier<Double> setpoint, Supplier<Double> waitTime,
                                               TrapezoidProfileSettings trapezoidProfileSettings) {
        super(subsystem, pidSettings, feedForwardSettings, ControlMode.MotionMagic, setpoint, waitTime);
        this.trapezoidProfileSettings = trapezoidProfileSettings;
    }

    public MoveSmartMotorControllerSubsystemTrapezily(SmartMotorControllerSubsystem subsystem, PIDSettings pidSettings,
                                                      FeedForwardSettings feedForwardSettings,
                                                      CANSparkMax.ControlType controlType,
                                                      Supplier<Double> setpoint, Supplier<Double> waitTime,
                                                      TrapezoidProfileSettings trapezoidProfileSettings) {
        super(subsystem, pidSettings, feedForwardSettings, CANSparkMax.ControlType.kSmartMotion, setpoint, waitTime);
        this.trapezoidProfileSettings = trapezoidProfileSettings;
    }

    @Override
    public void initialize() {
        subsystem.configureTrapezoid(trapezoidProfileSettings);
        super.initialize();
    }

    @Override
    public void execute() {
        subsystem.configureTrapezoid(trapezoidProfileSettings);
        super.execute();
    }
}
