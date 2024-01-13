package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A {@link Subsystem} which consists of a master {@link TalonFX} motor controller that runs control
 * loops and additional {@link TalonFX} motor controllers that follow it.
 *
 * @author
 * @see DashboardedSubsystem
 * @see SmartMotorControllerGenericSubsystem
 */
public class TalonFXGenericSubsystem extends DashboardedSubsystem implements SmartMotorControllerGenericSubsystem {


    @Override
    public void configureDashboard() {

    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings, FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {

    }

    @Override
    public boolean onTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        return false;
    }
}
