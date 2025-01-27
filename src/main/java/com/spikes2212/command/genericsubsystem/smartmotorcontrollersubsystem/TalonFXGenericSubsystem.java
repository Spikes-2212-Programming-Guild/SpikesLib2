package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A {@link Subsystem} which consists of a master {@link TalonFX} motor controller that runs control
 * loops and additional {@link TalonFX} motor controllers that follow it.
 *
 * @author Netta Halamish
 * @see DashboardedSubsystem
 * @see SmartMotorControllerGenericSubsystem
 */
@Deprecated(since = "2025", forRemoval = true)
public class TalonFXGenericSubsystem extends SmartMotorControllerGenericSubsystem {

    /**
     * Constructs a new instance of {@link TalonFXGenericSubsystem}.
     *
     * @param namespaceName the name of the subsystem's namespace
     */
    public TalonFXGenericSubsystem(String namespaceName, TalonFXWrapper... talonFXWrappers) {
        super(namespaceName, talonFXWrappers);
    }
}
